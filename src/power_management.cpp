#include <axp20x.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include "hardware_facts.h"
#include "oled.h"
#include "lorawan.h"
#include "power_management.h"
#include "wifi.h"
#include "bluetooth.h"
#include "gps.h"
#include "env_sensor.h"

static const char LOG_TAG[] = __FILE__;

static AXP20X_Class pmu;
static struct power_status status;
static bool is_conserving_power;
static power_config_t config = power_config_regular, config_before_conserving = power_config_regular;
static SemaphoreHandle_t i2c_mutex, pmu_mutex;
static unsigned long last_transmision_timestamp = 0;
static int last_cpu_freq_mhz = 240;
static int lorawan_tx_counter = 0;

static const int bt_prep_duration_ms = BLUETOOTH_SCAN_DURATION_SEC * 1000 + (BLUETOOTH_TASK_LOOP_DELAY_MS * 2) + 1000;
static const int wifi_prep_duration_ms = (WIFI_TASK_LOOP_DELAY_MS * WIFI_MAX_CHANNEL_NUM) + (WIFI_TASK_LOOP_DELAY_MS * 2) + 1000;
static const int env_sensor_prep_duration_ms = ENV_SENSOR_TASK_LOOP_DELAY_MS + 500;

void power_setup()
{
    memset(&status, 0, sizeof(status));
    i2c_mutex = xSemaphoreCreateMutex();
    pmu_mutex = xSemaphoreCreateMutex();
    power_set_cpu_freq_mhz(POWER_DEFAULT_CPU_FREQ_MHZ);
    if (!Wire.begin(I2C_SDA, I2C_SCL))
    {
        ESP_LOGW(LOG_TAG, "failed to initialise I2C");
    }
    power_i2c_lock();
    if (pmu.begin(Wire, AXP192_SLAVE_ADDRESS) != AXP_PASS)
    {
        ESP_LOGW(LOG_TAG, "failed to initialise AXP power management chip");
    }

    pmu.setDCDC1Voltage(3300); // OLED
    pmu.setDCDC2Voltage(0);    // Unused
    pmu.setLDO2Voltage(3300);  // LoRa
    pmu.setLDO3Voltage(3300);  // GPS

    pmu.setVWarningLevel1(3600);
    pmu.setVWarningLevel2(3800);
    pmu.setPowerDownVoltage(3300);

    pmu.setTimeOutShutdown(false);
    pmu.setTSmode(AXP_TS_PIN_MODE_DISABLE);
    pmu.setShutdownTime(AXP_POWER_OFF_TIME_4S);
    pmu.setStartupTime(AXP192_STARTUP_TIME_1S);

    // Turn on ADCs.
    pmu.adc1Enable(AXP202_BATT_VOL_ADC1, true);
    pmu.adc1Enable(AXP202_BATT_CUR_ADC1, true);
    pmu.adc1Enable(AXP202_VBUS_VOL_ADC1, true);
    pmu.adc1Enable(AXP202_VBUS_CUR_ADC1, true);

    // Handle power management events.
    pinMode(GPIO_NUM_35, INPUT_PULLUP);
    pmu.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_VBUS_OVER_VOL_IRQ | AXP202_BATT_REMOVED_IRQ |
                      AXP202_BATT_CONNECT_IRQ | AXP202_CHARGING_FINISHED_IRQ | AXP202_PEK_SHORTPRESS_IRQ,
                  1);
    pmu.clearIRQ();

    // Start charging the battery if it is installed.
    pmu.setChargeControlCur(AXP1XX_CHARGE_CUR_450MA);
    pmu.setChargingTargetVoltage(AXP202_TARGET_VOL_4_2V);
    pmu.enableChargeing(true);
    pmu.setChgLEDMode(AXP20X_LED_OFF);

    // Keep the on-board clock (& GPS) battery topped up.
    pmu.setBackupChargeCurrent(AXP202_BACKUP_CURRENT_100UA);
    pmu.setBackupChargeVoltage(AXP202_BACKUP_VOLTAGE_3V0);
    pmu.setBackupChargeControl(true);

    pmu.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // OLED
    pmu.setPowerOutPut(AXP192_DCDC2, AXP202_OFF); // Unused
    pmu.setPowerOutPut(AXP192_LDO2, AXP202_ON);   // LoRa
    pmu.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // GPS
    pmu.setPowerOutPut(AXP192_EXTEN, AXP202_OFF); // Unused
    power_i2c_unlock();
}

void power_set_power_output(uint8_t ch, bool on)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    pmu.setPowerOutPut(ch, on ? AXP202_ON : AXP202_OFF);
    xSemaphoreGive(i2c_mutex);
}

void power_i2c_lock()
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
}

void power_i2c_unlock()
{
    xSemaphoreGive(i2c_mutex);
}

unsigned long power_get_last_transmission_timestamp()
{
    return last_transmision_timestamp;
}

void power_set_last_transmission_timestamp()
{
    last_transmision_timestamp = millis();
}

bool power_get_may_transmit_lorawan()
{
    return last_transmision_timestamp == 0 || millis() - last_transmision_timestamp > config.tx_interval_sec * 1000;
}

void power_led_on()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    power_i2c_lock();
    pmu.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
    power_i2c_unlock();
    xSemaphoreGive(pmu_mutex);
}

void power_led_off()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    power_i2c_lock();
    pmu.setChgLEDMode(AXP20X_LED_OFF);
    power_i2c_unlock();
    xSemaphoreGive(pmu_mutex);
}

void power_led_blink()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    power_i2c_lock();
    pmu.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
    power_i2c_unlock();
    xSemaphoreGive(pmu_mutex);
}

int power_get_battery_millivolt()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    power_i2c_lock();
    int ret = (int)pmu.getBattVoltage();
    power_i2c_unlock();
    xSemaphoreGive(pmu_mutex);
    return ret;
}

void power_read_handle_lastest_irq()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    power_i2c_lock();
    pmu.readIRQ();
    power_i2c_unlock();
    if (pmu.isVbusOverVoltageIRQ())
    {
        power_i2c_lock();
        ESP_LOGW(TAG, "USB power supply voltage is too high (%f.3v)", pmu.getVbusVoltage() / 1000);
        power_i2c_unlock();
    }
    if (pmu.isBattPlugInIRQ())
    {
        ESP_LOGI(TAG, "battery inserted");
    }
    if (pmu.isBattRemoveIRQ())
    {
        ESP_LOGI(TAG, "battery removed");
    }
    if (pmu.isChargingDoneIRQ())
    {
        ESP_LOGI(TAG, "battery charging completed");
    }
    if (pmu.isPEKShortPressIRQ())
    {
        // If the button is clicked before OLED's timer deemed the display to have been inactive.
        // Otherwise, the user is simply clicking the button to wake the screen up.
        if (!oled_reset_last_input_timestamp())
        {
            ESP_LOGI(TAG, "turning to the next page");
            oled_go_to_next_page();
        }
    }
    if (pmu.isPEKLongtPressIRQ())
    {
        ESP_LOGW(TAG, "shutting down");
        power_i2c_lock();
        pmu.setChgLEDMode(AXP20X_LED_OFF);
        pmu.shutdown();
        power_i2c_unlock();
    }
    power_i2c_lock();
    pmu.clearIRQ();
    power_i2c_unlock();
    xSemaphoreGive(pmu_mutex);
}

void power_start_conserving()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    if (is_conserving_power)
    {
        xSemaphoreGive(pmu_mutex);
        return;
    }
    ESP_LOGW(TAG, "start conserving power by switching from mode %d to power save mode, battery current reads: %+.1f", config.mode_id, status.batt_milliamp);
    config_before_conserving = config;
    config = power_config_saver;
    is_conserving_power = true;
    xSemaphoreGive(pmu_mutex);
}

void power_stop_conserving()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    if (!is_conserving_power)
    {
        xSemaphoreGive(pmu_mutex);
        return;
    }
    ESP_LOGW(TAG, "stop conserving power and return to power mode %d", config_before_conserving.mode_id);
    config = config_before_conserving;
    is_conserving_power = false;
    xSemaphoreGive(pmu_mutex);
}

int power_get_uptime_sec()
{
    return (esp_timer_get_time() / 1000000);
}

struct power_status power_get_status()
{
    return status;
}

int power_get_todo()
{
    int ret = 0;
    // Calculate whether LoRaWAN RX/TX can/may be in progress.
    unsigned long ms_since_last_tx = millis() - last_transmision_timestamp;
    unsigned long sec_since_last_tx = ms_since_last_tx / 1000;
    unsigned long sec_until_next_tx = config.tx_interval_sec - sec_since_last_tx;
    // Allow transceiving for the period between -1 sec prior to the upcoming TX and 7 seconds after the upcoming TX.
    // 7 seconds should be long enough for both RX1 and RX2 windows.
    // Essentially: [-1 sec, +8 sec] + time_to_tx
    if (lorawan_tx_counter > 0 && (sec_since_last_tx < 8 || sec_until_next_tx < 1))
    {
        ret |= POWER_TODO_LORAWAN_TX_RX;
    }
    else if (lorawan_tx_counter == 0 && ms_since_last_tx > env_sensor_prep_duration_ms)
    {
        // The first transmission needs to wait until the environment sensor is read for the first time.
        ret |= POWER_TODO_LORAWAN_TX_RX;
    }

    // Ask bluetooth and wifi to be turned on shortly before the next TX.
    if (lorawan_tx_counter % LORAWAN_TX_KINDS == LORAWAN_TX_KIND_POS &&
        (ms_since_last_tx > config.tx_interval_sec * 1000 - bt_prep_duration_ms && ms_since_last_tx < config.tx_interval_sec * 1000))
    {
        ret |= POWER_TODO_TURN_ON_BLUETOOTH;
    }
    if (lorawan_tx_counter % LORAWAN_TX_KINDS == LORAWAN_TX_KIND_POS &&
        (ms_since_last_tx > config.tx_interval_sec * 1000 - wifi_prep_duration_ms && ms_since_last_tx < config.tx_interval_sec * 1000))
    {
        ret |= POWER_TODO_TURN_ON_WIFI;
    }
    // The sensor readings need to be taken for the first TX since boot as well as shortly before next TX.
    if (lorawan_tx_counter == 0 ||
        (lorawan_tx_counter % LORAWAN_TX_KINDS == LORAWAN_TX_KIND_ENV &&
         (ms_since_last_tx > config.tx_interval_sec * 1000 - env_sensor_prep_duration_ms && ms_since_last_tx < config.tx_interval_sec * 1000)))
    {
        ret |= POWER_TODO_READ_ENV_SENSOR;
    }

    // When in power saver config, GPS is turned on for 5 min and then off for 5 min.
    // Otherwise GPS is always on.
    if (power_get_config().intermittent_gps)
    {
        int sec = millis() / 1000;
        if (sec % (POWER_GPS_RUN_SLEEP_INTERVAL_SEC * 2) < POWER_GPS_RUN_SLEEP_INTERVAL_SEC)
        {
            ret |= POWER_TODO_TURN_ON_GPS;
        }
    }
    else
    {
        ret |= POWER_TODO_TURN_ON_GPS;
    }

    // If there is no power-related task to do and no user input, then ask caller to reduce CPU speed to conserve power.
    // Be aware that if user is looking at WiFi/BT scanner then CPU speed must not be reduced or WiFi/BT will cause a panic.
    if (ret == 0 && oled_get_ms_since_last_input() > wifi_prep_duration_ms + bt_prep_duration_ms && !wifi_get_state() && !bluetooth_get_state())
    {
        ret |= POWER_TODO_REDUCE_CPU_FREQ;
    }
    return ret;
}

int power_get_lorawan_tx_counter()
{
    return lorawan_tx_counter;
}

void power_inc_lorawan_tx_counter()
{
    lorawan_tx_counter++;
}

void power_read_status()
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    power_i2c_lock();
    status.is_batt_charging = pmu.isChargeing();
    status.batt_millivolt = pmu.getBattVoltage();
    status.usb_millivolt = pmu.getVbusVoltage();
    if (status.is_batt_charging)
    {
        status.batt_milliamp = pmu.getBattChargeCurrent();
    }
    else
    {
        status.batt_milliamp = -pmu.getBattDischargeCurrent();
    }
    status.power_draw_milliamp = pmu.getVbusCurrent();
    // The power management chip always draws power from USB when it is available.
    // Use battery discharging current as a condition too because the VBus current occasionally reads 0.
    status.is_usb_power_available = status.is_batt_charging || status.power_draw_milliamp > 5 || status.batt_milliamp > -5;
    if (!status.is_usb_power_available)
    {
        status.power_draw_milliamp = -status.batt_milliamp;
    }
    if (status.power_draw_milliamp < 0)
    {
        ESP_LOGI(TAG, "power draw reads negative (%.2f) - this should not have happened", status.power_draw_milliamp);
        status.power_draw_milliamp = 0;
    }
    power_i2c_unlock();
    xSemaphoreGive(pmu_mutex);
}

void power_log_status()
{
    ESP_LOGI(LOG_TAG, "mode: %d, is_batt_charging: %d, is_usb_power_available: %d, usb_millivolt: %d, batt_millivolt: %d, batt_milliamp: %.2f, power_draw_milliamp: %.2f",
             config.mode_id, status.is_batt_charging, status.is_usb_power_available, status.usb_millivolt, status.batt_millivolt, status.batt_milliamp, status.power_draw_milliamp);
}

void power_set_config(power_config_t val)
{
    ESP_LOGI(TAG, "setting power mode to %d", val.mode_id);
    config = val;
}

power_config_t power_get_config()
{
    return config;
}

void power_set_cpu_freq_mhz(int new_mhz)
{
    xSemaphoreTake(pmu_mutex, portMAX_DELAY);
    if (last_cpu_freq_mhz != new_mhz)
    {
        ESP_LOGI(TAG, "setting CPU frequency to %d MHz", new_mhz);
        if (!setCpuFrequencyMhz(new_mhz))
        {
            ESP_LOGW(LOG_TAG, "failed to set CPU frequency to %d MHz", new_mhz);
        }
        last_cpu_freq_mhz = new_mhz;
    }
    xSemaphoreGive(pmu_mutex);
}

void power_task_loop(void *_)
{
    unsigned long rounds = 0;
    while (true)
    {
        esp_task_wdt_reset();
        if (rounds % (POWER_TASK_READ_STATUS_DELAY_MS / POWER_TASK_LOOP_DELAY_MS) == 0)
        {
            power_read_status();
            // Act upon the latest power status readings.
            if (status.batt_milliamp < -10)
            {
                power_start_conserving();
            }
            else
            {
                power_stop_conserving();
            }
            if (power_get_todo() & POWER_TODO_REDUCE_CPU_FREQ)
            {
                power_set_cpu_freq_mhz(POWER_LOWEST_CPU_FREQ_MHZ);
            }
            else
            {
                power_set_cpu_freq_mhz(POWER_DEFAULT_CPU_FREQ_MHZ);
            }
        }
        if (rounds % (POWER_TASK_LOG_STATUS_DELAY_MS / POWER_TASK_LOOP_DELAY_MS) == 0)
        {
            power_log_status();
        }
        power_read_handle_lastest_irq();
        ++rounds;
        vTaskDelay(pdMS_TO_TICKS(POWER_TASK_LOOP_DELAY_MS));
    }
}
