#include <Arduino.h>
#include <esp_task_wdt.h>
#include "lorawan.h"
#include "hardware_facts.h"
#include "oled.h"
#include "gp_button.h"
#include "power_management.h"
#include "gps.h"
#include "i2c.h"
#include "data_packet.h"
#include "env_sensor.h"
#include "wifi.h"
#include "supervisor.h"

static const char LOG_TAG[] = __FILE__;
static TaskHandle_t wifi_task, env_sensor_task, gps_task, lorawan_task, oled_task, gp_button_task, power_task, supervisor_task;

static unsigned long gps_chars_processed_reading = 0;
static int gps_consecutive_readings = 0;

static size_t lorawan_tx_bytes_reading = 0;
static int lorawan_consecutive_readings = 0;

static unsigned long wifi_rounds_reading = 0;
static int wifi_consecutive_readings = 0;

void supervisor_setup()
{
    unsigned long priority = tskIDLE_PRIORITY;
    // ESP32 uses the first core (core 0) to handle wifi and BT radio, leaving the second core available for other tasks.
    // The higher the priority number, the higher the task priority.
    // The stack capacity is very generous - they are more than 4x the actual usage.
    xTaskCreatePinnedToCore(wifi_task_loop, "wifi_task_loop", 8 * 1024, NULL, priority++, &wifi_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(wifi_task));
    xTaskCreatePinnedToCore(env_sensor_task_loop, "env_sensor_task_loop", 8 * 1024, NULL, priority++, &env_sensor_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(env_sensor_task));
    xTaskCreatePinnedToCore(gps_task_loop, "gps_task_loop", 8 * 1024, NULL, priority++, &gps_task, 1);
    ESP_ERROR_CHECK(esp_task_wdt_add(gps_task));
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
    LMIC_shutdown();
    esp_restart();
}

void supervisor_check_task_stack()
{
    uint32_t heap_min_free_kb = ESP.getMinFreeHeap() / 1024;
    ESP_LOGI(TAG, "heap usage: free - %dKB, min.free - %dKB, capacity - %dKB, maxalloc: %dKB, min.free stack: %dKB",
             ESP.getFreeHeap() / 1024, heap_min_free_kb, ESP.getHeapSize() / 1024,
             ESP.getMaxAllocHeap() / 1024, uxTaskGetStackHighWaterMark(NULL) / 1024);
    UBaseType_t wifi_stack_free_kb = uxTaskGetStackHighWaterMark(wifi_task) / 1024,
                sensor_stack_free_kb = uxTaskGetStackHighWaterMark(env_sensor_task) / 1024,
                gps_stack_free_kb = uxTaskGetStackHighWaterMark(gps_task) / 1024,
                lorawan_stack_free_kb = uxTaskGetStackHighWaterMark(lorawan_task) / 1024,
                oled_stack_free_kb = uxTaskGetStackHighWaterMark(oled_task) / 1024,
                button_stack_free_kb = uxTaskGetStackHighWaterMark(gp_button_task) / 1024,
                power_stack_free_kb = uxTaskGetStackHighWaterMark(power_task) / 1024,
                supervisor_stack_free_kb = uxTaskGetStackHighWaterMark(supervisor_task) / 1024;
    ESP_LOGI(TAG, "wifi task state: %d, min.free stack: %dKB", eTaskGetState(wifi_task), wifi_stack_free_kb);
    ESP_LOGI(TAG, "sensor task state: %d, min.free stack: %dKB", eTaskGetState(env_sensor_task), sensor_stack_free_kb);
    ESP_LOGI(TAG, "gps task state: %d, min.free stack: %dKB", eTaskGetState(gps_task), gps_stack_free_kb);
    ESP_LOGI(TAG, "lorawan task state: %d, min.free stack: %dKB", eTaskGetState(lorawan_task), lorawan_stack_free_kb);
    ESP_LOGI(TAG, "oled task state: %d, min.free stack: %dKB", eTaskGetState(oled_task), oled_stack_free_kb);
    ESP_LOGI(TAG, "GP button task state: %d, min.free stack: %dKB", eTaskGetState(gp_button_task), button_stack_free_kb);
    ESP_LOGI(TAG, "power task state: %d, min.free stack: %dKB", eTaskGetState(power_task), power_stack_free_kb);
    ESP_LOGI(TAG, "supervisor task state: %d, min.free stack: %dKB", eTaskGetState(supervisor_task), supervisor_stack_free_kb);
    if (ESP.getMinFreeHeap() / 1024 < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB)
    {
        supervisor_reset();
    }
    if (wifi_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB || sensor_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        gps_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB || lorawan_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        oled_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB || button_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB ||
        power_stack_free_kb < SUPERVISOR_FREE_MEM_RESET_THRESHOLD_KB)
    {
        supervisor_reset();
    }
}

void supervisor_check_gps()
{
    if (gps_get_chars_processed() == gps_chars_processed_reading)
    {
        gps_consecutive_readings++;
        ESP_LOGE(TAG, "gps task does not appear to be making progress, chars processed reads %lu for %d times", gps_chars_processed_reading, gps_consecutive_readings);
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
    if (lorawan_get_total_tx_bytes() == lorawan_tx_bytes_reading)
    {
        lorawan_consecutive_readings++;
        ESP_LOGE(TAG, "lorawan task does not appear to be making progress, total tx bytes reads %d for %d times", lorawan_tx_bytes_reading, lorawan_consecutive_readings);
        if (lorawan_consecutive_readings >= SUPERVISOR_STUCK_PROGRESS_THRESHOLD)
        {
            supervisor_reset();
        }
    }
    else
    {
        lorawan_tx_bytes_reading = lorawan_get_total_tx_bytes();
        lorawan_consecutive_readings = 0;
    }
}

void supervisor_check_wifi()
{
    if (wifi_get_round_num() == wifi_rounds_reading)
    {
        wifi_consecutive_readings++;
        ESP_LOGE(TAG, "wifi task does not appear to be making progress, total number of rounds reads %d for %d times", wifi_rounds_reading, wifi_consecutive_readings);
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

void supervisor_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_TASK_LOOP_DELAY_MS));
        supervisor_check_task_stack();
        supervisor_check_gps();
        supervisor_check_lorawan();
        supervisor_check_wifi();
    }
}