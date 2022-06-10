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
static SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex(), wifi_bt_mutex = xSemaphoreCreateMutex();
static unsigned long last_transmision_timestamp = 0;
static int last_cpu_freq_mhz = 240;
static int lorawan_tx_counter = 0;
static double sum_curr_draw_readings = 0.0;

RTC_DATA_ATTR static struct power_status status;
RTC_DATA_ATTR static bool is_conserving_power;
RTC_DATA_ATTR static int config_mode_id = POWER_REGULAR;
RTC_DATA_ATTR static int config_mode_id_before_conserving = config_mode_id;
RTC_DATA_ATTR static unsigned long last_stop_conserve_power_timestamp = 0;

// The durations must be sufficient for taking a round of readings - preferrably with a generous amount of time buffer.
// They are largely magic numbers determined by inspecting the serial output.
static const int bt_prep_duration_ms = 3500;
static const int wifi_prep_duration_ms = 6000;
static const int env_sensor_prep_duration_ms = 2000;

void power_setup()
{
    memset(&status, 0, sizeof(status));
    power_set_cpu_freq_mhz(POWER_DEFAULT_CPU_FREQ_MHZ);
    power_i2c_lock();
    if (!Wire.begin(I2C_SDA, I2C_SCL))
    {
        ESP_LOGW(LOG_TAG, "failed to initialise I2C");
    }
    if (pmu.begin(Wire, AXP192_SLAVE_ADDRESS) != AXP_PASS)
    {
        ESP_LOGW(LOG_TAG, "failed to initialise AXP power management chip");
    }

    // https://www.solomon-systech.com/product/ssd1306/
    // "– VDD= 1.65V – 3.3V, <VBAT for IC Logic"
    // "– VBAT= 3.3V – 4.2V for charge pump regulator circuit"
    pmu.setDCDC1Voltage(3300); // OLED
    pmu.setDCDC2Voltage(0);    // Unused
    // https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf
    // "Supply voltage = 3.3 V"
    pmu.setLDO2Voltage(3300); // LoRa
    // https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_(GPS.G6-HW-09005).pdf
    // "NEO-6Q/M NEO-6P/V/T Min: 2.7, Typ: 3.0, Max: 3.6"
    pmu.setLDO3Voltage(3000); // GPS

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
    pmu.setChargeControlCur(AXP1XX_CHARGE_CUR_1000MA);
    pmu.setChargingTargetVoltage(AXP202_TARGET_VOL_4_2V);
    pmu.enableChargeing(true);
    pmu.setChgLEDMode(AXP20X_LED_OFF);

    // Keep the on-board clock (& GPS) battery topped up.
    pmu.setBackupChargeCurrent(AXP202_BACKUP_CURRENT_100UA);
    pmu.setBackupChargeVoltage(AXP202_BACKUP_VOLTAGE_3V0);
    pmu.setBackupChargeControl(true);

    pmu.setPowerOutPut(AXP192_DCDC1, AXP202_ON);      // OLED
    pmu.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);     // Unused
    pmu.setPowerOutPut(AXP192_LDO2, AXP202_ON);       // LoRa
    pmu.setPowerOutPut(GPS_POWER_CHANNEL, AXP202_ON); // GPS
    pmu.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);     // Unused
    power_i2c_unlock();
}
void power_i2c_lock()
{
    if (xSemaphoreTake(i2c_mutex, MUTEX_LOCK_TIMEOUT_MS) == pdFALSE)
    {
        ESP_LOGE(LOG_TAG, "failed to obtain i2c_mutex lock");
        assert(false);
    }
}

void power_i2c_unlock()
{
    xSemaphoreGive(i2c_mutex);
}

void power_wifi_bt_lock()
{
    if (xSemaphoreTake(wifi_bt_mutex, MUTEX_LOCK_TIMEOUT_MS) == pdFALSE)
    {
        ESP_LOGE(LOG_TAG, "failed to obtain wifi_bt_mutex lock");
        assert(false);
    }
}

void power_wifi_bt_unlock()
{
    xSemaphoreGive(wifi_bt_mutex);
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
    return last_transmision_timestamp == 0 || millis() - last_transmision_timestamp > power_get_config().tx_interval_sec * 1000;
}

void power_led_on()
{
    power_i2c_lock();
    pmu.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
    power_i2c_unlock();
}

void power_led_off()
{
    power_i2c_lock();
    pmu.setChgLEDMode(AXP20X_LED_OFF);
    power_i2c_unlock();
}

double power_get_sum_curr_draw_readings()
{
    return sum_curr_draw_readings;
}

void power_read_handle_lastest_irq()
{
    power_i2c_lock();
    pmu.readIRQ();
    if (pmu.isVbusOverVoltageIRQ())
    {
        ESP_LOGW(LOG_TAG, "USB power supply voltage is too high (%f.3v)", pmu.getVbusVoltage() / 1000);
    }
    if (pmu.isBattPlugInIRQ())
    {
        ESP_LOGI(LOG_TAG, "battery inserted");
    }
    if (pmu.isBattRemoveIRQ())
    {
        ESP_LOGI(LOG_TAG, "battery removed");
    }
    if (pmu.isChargingDoneIRQ())
    {
        ESP_LOGI(LOG_TAG, "battery charging completed");
    }
    if (pmu.isPEKShortPressIRQ())
    {
        // If the button is clicked before OLED's timer deemed the display to have been inactive.
        // Otherwise, the user is simply clicking the button to wake the screen up.
        if (!oled_reset_last_input_timestamp())
        {
            ESP_LOGI(LOG_TAG, "turning to the next page");
            oled_go_to_next_page();
        }
    }
    if (pmu.isPEKLongtPressIRQ())
    {
        ESP_LOGW(LOG_TAG, "shutting down");
        pmu.setChgLEDMode(AXP20X_LED_OFF);
        pmu.shutdown();
    }
    pmu.clearIRQ();
    power_i2c_unlock();
}

void power_start_conserving()
{
    power_i2c_lock();
    if (is_conserving_power)
    {
        power_i2c_unlock();
        return;
    }
    ESP_LOGW(LOG_TAG, "start conserving power by switching from mode %d to power save mode, battery current reads: %+.1f", config_mode_id, status.batt_milliamp);
    config_mode_id_before_conserving = config_mode_id;
    config_mode_id = POWER_SAVER;
    is_conserving_power = true;
    power_i2c_unlock();
}

void power_stop_conserving()
{
    power_i2c_lock();
    if (!is_conserving_power || ((millis() - last_stop_conserve_power_timestamp) / 1000) <= POWER_MIN_CONSERVATION_PERIOD_SEC)
    {
        power_i2c_unlock();
        return;
    }
    ESP_LOGW(LOG_TAG, "stop conserving power and return to power mode %d", config_mode_id_before_conserving);
    config_mode_id = config_mode_id_before_conserving;
    is_conserving_power = false;
    last_stop_conserve_power_timestamp = millis();
    power_i2c_unlock();
}

int power_get_uptime_sec()
{
    return (esp_timer_get_time() / 1000000);
}

struct power_status power_get_status()
{
    return status;
}

// The returned TODO flag bits will instruct peripherals' task loops to turn their power off (or stop collecting
// samples) when they are not in-use, hence conserving power.
// The general rule is to turn on peripherals required for the upcoming LoRaWAN transmission shortly before the transmission.
int power_get_todo()
{
    power_config_t config = power_get_config();
    int ret = 0;
    int uptime_sec = millis() / 1000;
    int deep_sleep_start_sec = power_get_config().deep_sleep_start_sec;
    // Do not enter deep sleep if user has made recent button inputs.
    if (deep_sleep_start_sec > 0 && uptime_sec >= deep_sleep_start_sec && oled_get_ms_since_last_input() > OLED_SLEEP_AFTER_INACTIVE_MS)
    {
        return POWER_TODO_ENTER_DEEP_SLEEP;
        // There's no need to ask caller to turn on anything else. The CPUs will be powered off entirely during deep sleep.
    }
    else
    {
        // Leave GPS turned on while not sleeping. It takes many minutes to obtain a GPS location fix.
        ret |= POWER_TODO_TURN_ON_GPS;
    }

    // Calculate whether LoRaWAN RX/TX can/may be in progress.
    unsigned long ms_since_last_tx = millis() - last_transmision_timestamp;
    unsigned long sec_since_last_tx = ms_since_last_tx / 1000;
    unsigned long sec_until_next_tx = config.tx_interval_sec - sec_since_last_tx;
    if (sec_since_last_tx > config.tx_interval_sec)
    {
        // Missed the previous LoRaWAN transmission according to the schedule, caller should transmit ASAP.
        sec_until_next_tx = 0;
    }
    // Allow transceiving for the period between -1 sec prior to the upcoming TX and 7 seconds after the upcoming TX.
    // 7 seconds should be long enough for both RX1 and RX2 windows.
    // Essentially: [-1 sec, +8 sec] + time_to_tx
    if (lorawan_tx_counter > 0 && (sec_since_last_tx < 8 || sec_until_next_tx < 1))
    {
        ret |= POWER_TODO_LORAWAN_TX_RX;
    }
    else if (lorawan_tx_counter == 0 && ms_since_last_tx > 5000)
    {
        // The first transmission needs to wait until the environment sensor is read for the first time.
        // The CPU is quite busy for the brief time period just after starting up, it takes a couple of seconds to get the first sensor readings.
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

    // If there is no power-related task to do and no user input, then ask caller to reduce CPU speed to conserve power.
    // Be aware that if user is looking at WiFi/BT scanner then CPU speed must not be reduced or WiFi/BT will cause a panic.
    // The lower CPU clock speed is sufficient for GPS though.
    if (ret == POWER_TODO_TURN_ON_GPS && oled_get_ms_since_last_input() > wifi_prep_duration_ms + bt_prep_duration_ms && !wifi_get_state() && !bluetooth_get_state())
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
    power_i2c_lock();
    status.is_batt_charging = pmu.isChargeing();
    status.batt_millivolt = pmu.getBattVoltage();
    if (status.batt_millivolt < 500)
    {
        // The AXP chip occasionally produces erranous and exceedingly low battery voltage readings even without a battery installed.
        status.batt_millivolt = 0;
    }
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
    status.is_usb_power_available = status.is_batt_charging || status.batt_milliamp > 3 || status.batt_millivolt < 3000 || status.power_draw_milliamp > 3 || pmu.getVbusVoltage() > 4000;
    if (!status.is_usb_power_available)
    {
        status.power_draw_milliamp = -status.batt_milliamp;
    }
    if (status.power_draw_milliamp < 0)
    {
        ESP_LOGI(LOG_TAG, "power draw reads negative (%.2f) - this should not have happened", status.power_draw_milliamp);
        status.power_draw_milliamp = 0;
    }
    sum_curr_draw_readings += status.power_draw_milliamp;
    power_i2c_unlock();
}

void power_log_status()
{
    ESP_LOGI(LOG_TAG, "mode: %d, wifi? %d, bt? %d, gps? %d, oled? %d, is_batt_charging: %d, is_usb_power_available: %d, usb_millivolt: %d, batt_millivolt: %d, batt_milliamp: %.2f, power_draw_milliamp: %.2f",
             config_mode_id,
             wifi_get_state(), bluetooth_get_state(), gps_get_state(), oled_get_state(),
             status.is_batt_charging, status.is_usb_power_available, status.usb_millivolt, status.batt_millivolt, status.batt_milliamp, status.power_draw_milliamp);
}

void power_set_config(int new_mode_id)
{
    ESP_LOGI(LOG_TAG, "setting power mode to %d", new_mode_id);
    config_mode_id = new_mode_id;
}

power_config_t power_get_config()
{
    switch (config_mode_id)
    {
    case POWER_BOOST:
        return power_config_boost;
        break;
    case POWER_REGULAR:
        return power_config_regular;
        break;
    case POWER_SAVER:
        return power_config_saver;
        break;
    default:
        return power_config_regular;
    }
}

void power_set_cpu_freq_mhz(int new_mhz)
{
    power_i2c_lock();
    if (last_cpu_freq_mhz != new_mhz)
    {
        ESP_LOGI(LOG_TAG, "setting CPU frequency to %d MHz", new_mhz);
        if (!setCpuFrequencyMhz(new_mhz))
        {
            ESP_LOGW(LOG_TAG, "failed to set CPU frequency to %d MHz", new_mhz);
        }
        last_cpu_freq_mhz = new_mhz;
    }
    power_i2c_unlock();
}

void power_enter_deep_sleep()
{
    ESP_LOGW(LOG_TAG, "preparing to enter deep sleep");
    bluetooth_off();
    wifi_off();
    gps_off();
    oled_off();
    power_led_off();
    esp_sleep_enable_timer_wakeup(power_get_config().deep_sleep_duration_sec * 1000 * 1000);
    ESP_LOGW(LOG_TAG, "entering deep sleep now");
    esp_deep_sleep_start();
}

void power_task_loop(void *_)
{
    unsigned long rounds = 0;
    while (true)
    {
        esp_task_wdt_reset();
        if (power_get_todo() & POWER_TODO_ENTER_DEEP_SLEEP)
        {
            power_enter_deep_sleep();
            return; // Not reachable, the CPUs shut down during deep sleep.
        }
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