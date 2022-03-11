#include <Arduino.h>
#include <esp_task_wdt.h>
#include "lorawan.h"
#include "hardware_facts.h"
#include "oled.h"
#include "gp_button.h"
#include "power_management.h"
#include "gps.h"
#include "data_packet.h"
#include "env_sensor.h"
#include "wifi.h"
#include "bluetooth.h"
#include "supervisor.h"

static const char LOG_TAG[] = __FILE__;
static TaskHandle_t bluetooth_task, wifi_task, env_sensor_task,
    gps_task, lorawan_task, oled_task, gp_button_task, power_task, supervisor_task;

static unsigned long gps_chars_processed_reading = 0;
static int gps_consecutive_readings = 0;

static size_t lorawan_tx_counter_reading = 0;
static int lorawan_consecutive_readings = 0;

static unsigned long wifi_rounds_reading = 0;
static int wifi_consecutive_readings = 0;

static unsigned long bluetooth_rounds_reading = 0;
static int bluetooth_consecutive_readings = 0;

static double env_sensor_sum_temp_readings = 0;
static int env_sensor_consecutive_readings = 0;

static double power_sum_curr_draw_readings = 0;
static int power_consecutive_readings = 0;

void supervisor_setup()
{
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        ESP_LOGI(LOG_TAG, "wake-up reason is undefined");
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
        ESP_LOGI(LOG_TAG, "wake-up from RTC_IO (EXT0)");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        ESP_LOGI(LOG_TAG, "wake-up from RTC_CNT (EXT1)");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        ESP_LOGI(LOG_TAG, "wake-up from timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        ESP_LOGI(LOG_TAG, "wake-up from touch pad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        ESP_LOGI(LOG_TAG, "wake-up from ULP");
        break;
    case ESP_SLEEP_WAKEUP_GPIO:
        ESP_LOGI(LOG_TAG, "wake-up from GPIO");
        break;
    case ESP_SLEEP_WAKEUP_UART:
        ESP_LOGI(LOG_TAG, "wake-up from UART");
        break;
    default:
        ESP_LOGI(LOG_TAG, "wake-up reason is %d", wakeup_reason);
        break;
    }

    unsigned long priority = tskIDLE_PRIORITY;
    // ESP32 uses the first core (core 0) to handle wifi and BT radio, leaving the second core available for other tasks.
    // The higher the priority number, the higher the task priority.
    // The stack capacity is very generous - they are at least 3x the actual usage.
    xTaskCreatePinnedToCore(bluetooth_task_loop, "bluetooth_task_loop", 8 * 1024, NULL, priority++, &bluetooth_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(bluetooth_task));
    xTaskCreatePinnedToCore(wifi_task_loop, "wifi_task_loop", 8 * 1024, NULL, priority++, &wifi_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(wifi_task));
    xTaskCreatePinnedToCore(gps_task_loop, "gps_task_loop", 8 * 1024, NULL, priority++, &gps_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(gps_task));
    xTaskCreatePinnedToCore(env_sensor_task_loop, "env_sensor_task_loop", 8 * 1024, NULL, priority++, &env_sensor_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(env_sensor_task));
    xTaskCreatePinnedToCore(lorawan_task_loop, "lorawan_task_loop", 16 * 1024, NULL, priority++, &lorawan_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(lorawan_task));
    xTaskCreatePinnedToCore(oled_task_loop, "oled_task_loop", 16 * 1024, NULL, priority++, &oled_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(oled_task));
    xTaskCreatePinnedToCore(gp_button_task_loop, "gp_button_task_loop", 8 * 1024, NULL, priority++, &gp_button_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(gp_button_task));
    xTaskCreatePinnedToCore(power_task_loop, "power_task_loop", 8 * 1024, NULL, priority++, &power_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(power_task));
    xTaskCreatePinnedToCore(supervisor_task_loop, "supervisor_task_loop", 16 * 1024, NULL, priority++, &supervisor_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(supervisor_task));
    ESP_LOGI(LOG_TAG, "supervisor has successfully started all essential tasks");
}

void supervisor_reset()
{
    ESP_LOGE(LOG_TAG, "supervisor has detected a critical condition and will now reset the microcontroller");
    esp_restart();
}

void supervisor_check_task_stack()
{
    uint32_t heap_min_free_kb = ESP.getMinFreeHeap() / 1024;
    ESP_LOGI(TAG, "heap usage: free - %dKB, min.free - %dKB, capacity - %dKB, maxalloc: %dKB, min.free stack: %dKB",
             ESP.getFreeHeap() / 1024, heap_min_free_kb, ESP.getHeapSize() / 1024,
             ESP.getMaxAllocHeap() / 1024, uxTaskGetStackHighWaterMark(NULL) / 1024);
    UBaseType_t bluetooth_stack_free_kb = uxTaskGetStackHighWaterMark(bluetooth_task) / 1024,
                wifi_stack_free_kb = uxTaskGetStackHighWaterMark(wifi_task) / 1024,
                sensor_stack_free_kb = uxTaskGetStackHighWaterMark(env_sensor_task) / 1024,
                gps_stack_free_kb = uxTaskGetStackHighWaterMark(gps_task) / 1024,
                lorawan_stack_free_kb = uxTaskGetStackHighWaterMark(lorawan_task) / 1024,
                oled_stack_free_kb = uxTaskGetStackHighWaterMark(oled_task) / 1024,
                button_stack_free_kb = uxTaskGetStackHighWaterMark(gp_button_task) / 1024,
                power_stack_free_kb = uxTaskGetStackHighWaterMark(power_task) / 1024,
                supervisor_stack_free_kb = uxTaskGetStackHighWaterMark(supervisor_task) / 1024;
    if (heap_min_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        bluetooth_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        wifi_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB || sensor_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        gps_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB || lorawan_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        oled_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB || button_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        power_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB)
    {
        ESP_LOGE(TAG, "bluetooth task state: %d, min.free stack: %dKB", eTaskGetState(bluetooth_task), bluetooth_stack_free_kb);
        ESP_LOGE(TAG, "wifi task state: %d, min.free stack: %dKB", eTaskGetState(wifi_task), wifi_stack_free_kb);
        ESP_LOGE(TAG, "gps task state: %d, min.free stack: %dKB", eTaskGetState(gps_task), gps_stack_free_kb);
        ESP_LOGE(TAG, "sensor task state: %d, min.free stack: %dKB", eTaskGetState(env_sensor_task), sensor_stack_free_kb);
        ESP_LOGE(TAG, "lorawan task state: %d, min.free stack: %dKB", eTaskGetState(lorawan_task), lorawan_stack_free_kb);
        ESP_LOGE(TAG, "oled task state: %d, min.free stack: %dKB", eTaskGetState(oled_task), oled_stack_free_kb);
        ESP_LOGE(TAG, "GP button task state: %d, min.free stack: %dKB", eTaskGetState(gp_button_task), button_stack_free_kb);
        ESP_LOGE(TAG, "power task state: %d, min.free stack: %dKB", eTaskGetState(power_task), power_stack_free_kb);
        ESP_LOGE(TAG, "supervisor task state: %d, min.free stack: %dKB", eTaskGetState(supervisor_task), supervisor_stack_free_kb);
        supervisor_reset();
    }
}

void supervisor_check_gps()
{
    if (gps_get_chars_processed() == gps_chars_processed_reading)
    {
        if (++gps_consecutive_readings > SUPERVISOR_STUCK_PROGRESS_THRESHOLD / 2)
        {
            ESP_LOGE(TAG, "gps task does not appear to be making progress, chars processed reads %lu for %d times", gps_chars_processed_reading, gps_consecutive_readings);
        }
        if (gps_consecutive_readings >= SUPERVISOR_STUCK_PROGRESS_THRESHOLD)
        {
            supervisor_reset();
        }
    }
    else
    {
        gps_chars_processed_reading = gps_get_chars_processed();
        gps_consecutive_readings = 0;
    }
}

void supervisor_check_lorawan()
{
    if (power_get_lorawan_tx_counter() == lorawan_tx_counter_reading)
    {
        if (++lorawan_consecutive_readings > SUPERVISOR_STUCK_PROGRESS_THRESHOLD / 2)
        {
            ESP_LOGE(TAG, "lorawan task does not appear to be making progress, tx counter reads %d for %d times", lorawan_tx_counter_reading, lorawan_consecutive_readings);
            lorawan_debug_to_log();
            lorawan_reset();
        }
        if (lorawan_consecutive_readings >= SUPERVISOR_STUCK_PROGRESS_THRESHOLD)
        {
            supervisor_reset();
        }
    }
    else
    {
        lorawan_tx_counter_reading = power_get_lorawan_tx_counter();
        lorawan_consecutive_readings = 0;
    }
}

void supervisor_check_wifi()
{
    if (wifi_get_round_num() == wifi_rounds_reading)
    {
        if (++wifi_consecutive_readings > SUPERVISOR_STUCK_PROGRESS_THRESHOLD / 2)
        {
            ESP_LOGE(TAG, "wifi task does not appear to be making progress, total number of rounds reads %d for %d times", wifi_rounds_reading, wifi_consecutive_readings);
        }
        if (wifi_consecutive_readings >= SUPERVISOR_STUCK_PROGRESS_THRESHOLD)
        {
            supervisor_reset();
        }
    }
    else
    {
        wifi_rounds_reading = wifi_get_round_num();
        wifi_consecutive_readings = 0;
    }
}

void supervisor_check_bluetooth()
{
    if (bluetooth_get_round_num() == bluetooth_rounds_reading)
    {
        if (++bluetooth_consecutive_readings > SUPERVISOR_STUCK_PROGRESS_THRESHOLD / 2)
        {
            ESP_LOGE(TAG, "bluetooth task does not appear to be making progress, total number of rounds reads %d for %d times", bluetooth_rounds_reading, bluetooth_consecutive_readings);
        }
        if (bluetooth_consecutive_readings >= SUPERVISOR_STUCK_PROGRESS_THRESHOLD)
        {
            supervisor_reset();
        }
    }
    else
    {
        bluetooth_rounds_reading = wifi_get_round_num();
        bluetooth_consecutive_readings = 0;
    }
}

void supervisor_check_env_sensor()
{
    if (env_sensor_get_sum_temp_readings() == env_sensor_sum_temp_readings)
    {
        if (++env_sensor_consecutive_readings > SUPERVISOR_STUCK_PROGRESS_THRESHOLD / 2)
        {
            ESP_LOGE(TAG, "env sensor task does not appear to be making progress, temp sum reads %f for %d times", env_sensor_sum_temp_readings, env_sensor_consecutive_readings);
        }
        if (env_sensor_consecutive_readings >= SUPERVISOR_STUCK_PROGRESS_THRESHOLD)
        {
            supervisor_reset();
        }
    }
    else
    {
        env_sensor_sum_temp_readings = env_sensor_get_sum_temp_readings();
        env_sensor_consecutive_readings = 0;
    }
}

void supervisor_check_power()
{
    if (power_get_sum_curr_draw_readings() == power_sum_curr_draw_readings)
    {
        if (++power_consecutive_readings > SUPERVISOR_STUCK_PROGRESS_THRESHOLD / 2)
        {
            ESP_LOGE(TAG, "power management task does not appear to be making progress, total number of rounds reads %f for %d times", power_sum_curr_draw_readings, power_consecutive_readings);
        }
        if (power_consecutive_readings >= SUPERVISOR_STUCK_PROGRESS_THRESHOLD)
        {
            supervisor_reset();
        }
    }
    else
    {
        power_sum_curr_draw_readings = power_get_sum_curr_draw_readings();
        power_consecutive_readings = 0;
    }
}

void supervisor_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        supervisor_check_task_stack();
        supervisor_check_power();
        supervisor_check_lorawan();
        supervisor_check_env_sensor();
        supervisor_check_gps();
        supervisor_check_wifi();
        supervisor_check_bluetooth();
        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_TASK_LOOP_DELAY_MS));
    }
}