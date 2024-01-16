#include <esp_log.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include "hardware_facts.h"
#include "oled.h"
#include "lorawan.h"
#include "power_management.h"
#include "wifi.h"
#include "bluetooth.h"
#include "gps.h"
#include "env_sensor.h"

static const char LOG_TAG[] = __FILE__;

static XPowersPMU *pmu;
static SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex(), wifi_bt_mutex = xSemaphoreCreateMutex();
static unsigned long last_transmision_timestamp = 0;
static int last_cpu_freq_mhz = 240;
static int lorawan_tx_counter = 0;
static double sum_curr_draw_readings = 0.0;
static bool pmu_irq_flag = false;

RTC_DATA_ATTR static struct power_status status;
RTC_DATA_ATTR static bool is_conserving_power;
RTC_DATA_ATTR static int config_mode_id = POWER_REGULAR;
RTC_DATA_ATTR static int config_mode_id_before_conserving = config_mode_id;
RTC_DATA_ATTR static unsigned long last_stop_conserve_power_timestamp = 0;

// The durations must be sufficient for taking two rounds of readings, the first round is often unreliable.
static const int bt_prep_duration_ms = (3000 * 2 + BLUETOOTH_TASK_LOOP_DELAY_MS * 3 + POWER_TASK_LOOP_DELAY_MS * 3); // typical: 3 seconds per bluetooth scan at 80MHz CPU frequency.
static const int bt_wifi_gap_ms = (2000 + POWER_TASK_LOOP_DELAY_MS * 3);                                             // typical: 2 seconds to shut down bluetooth and free up memory for wifi.
static const int wifi_prep_duration_ms = (4000 * 2 + WIFI_TASK_LOOP_DELAY_MS * 3 + POWER_TASK_LOOP_DELAY_MS * 3);    // typical: 4 seconds per wifi scan at 80MHz CPU frequency.
static const int env_sensor_prep_duration_ms = (ENV_SENSOR_TASK_LOOP_DELAY_MS * 3 + POWER_TASK_LOOP_DELAY_MS * 3);

void power_setup()
{
    ESP_LOGI(LOG_TAG, "setting up power management");
    memset(&status, 0, sizeof(status));
    power_set_cpu_freq_mhz(POWER_DEFAULT_CPU_FREQ_MHZ);
    power_i2c_lock();

    if (!Wire.begin(I2C_SDA, I2C_SCL, (uint32_t)I2C_FREQUENCY_HZ))
    {
        ESP_LOGW(LOG_TAG, "failed to initialise I2C");
    }

#ifdef AXP192
    pmu = new XPowersAXP192(Wire);
    // Both AXP192 and AXP2101 use the same I2C address - 0x34.
    if (!pmu->begin(Wire, AXP192_SLAVE_ADDRESS, I2C_SDA, I2C_SCL))
    {
        ESP_LOGW(LOG_TAG, "failed to initialise AXP power management chip");
    }
#endif
#ifdef AXP2101
    pmu = new XPowersAXP2101(Wire);
    // Both AXP192 and AXP2101 use the same I2C address - 0x34.
    if (!pmu->begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL))
    {
        ESP_LOGW(LOG_TAG, "failed to initialise AXP power management chip");
    }
#endif

    // Set USB power limits.
    if (pmu->getChipModel() == XPOWERS_AXP192)
    {
        ESP_LOGI(LOG_TAG, "setting up AXP192");
#ifdef AXP192
        // https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/AXP192_datasheet_en.pdf & AXP192%20Datasheet%20v1.13_cn..pdf
        pmu->setVbusVoltageLimit(XPOWERS_AXP192_VBUS_VOL_LIM_4V1);
        // There is no suitable current limit for 1.5A, the closest is only 0.5A.
        pmu->setVbusCurrentLimit(XPOWERS_AXP192_VBUS_CUR_LIM_OFF);
        // "wide input voltage range": 2.9V~6.3V
        // When using 3V for the shutdown voltage, the PMU actually shuts down at ~3.4V.
        pmu->setSysPowerDownVoltage(2800);

        pmu->enablePowerKeyLongPressPowerOff();
        pmu->setPowerKeyPressOffTime(XPOWERS_AXP192_POWEROFF_4S);
        pmu->setPowerKeyPressOnTime(XPOWERS_POWERON_2S);

        pmu->disableTSPinMeasure();
        pmu->setChargingLedMode(false);

        pmu->enableBattDetection();
        pmu->enableVbusVoltageMeasure();
        pmu->enableBattVoltageMeasure();
        pmu->enableSystemVoltageMeasure();

        // https://www.solomon-systech.com/product/ssd1306/
        // "– VDD= 1.65V – 3.3V, <VBAT for IC Logic"
        // "– VBAT= 3.3V – 4.2V for charge pump regulator circuit"
        pmu->setDC1Voltage(3300);
        pmu->enableDC1();
        // DC2 is unused.
        pmu->disableDC2();
        // https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf
        // "Supply voltage = 3.3 V"
        pmu->setLDO2Voltage(3300);
        pmu->enableLDO2();
        // https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_(GPS.G6-HW-09005).pdf
        // "NEO-6Q/M NEO-6P/V/T Min: 2.7, Typ: 3.0, Max: 3.6"
        pmu->setLDO3Voltage(3000);

        // Start charging the battery if it is installed.
        // The maximum supported charging current is 1.4A.
        pmu->setChargerConstantCurr(XPOWERS_AXP192_CHG_CUR_1000MA);
        pmu->setChargerTerminationCurr(XPOWERS_AXP192_CHG_ITERM_LESS_10_PERCENT);
        pmu->setChargeTargetVoltage(XPOWERS_AXP192_CHG_VOL_4V2);

        // Start charging the GPS memory backup battery.
        pmu->setBackupBattChargerVoltage(XPOWERS_AXP192_BACKUP_BAT_VOL_3V1);
        pmu->setBackupBattChargerCurr(XPOWERS_AXP192_BACKUP_BAT_CUR_100UA);

        // Conserve power by disabling temperature measurement.
        pmu->disableTemperatureMeasure();

        // Handle power management events.
        pinMode(POWER_PMU_IRQ, INPUT);
        attachInterrupt(POWER_PMU_IRQ, power_set_pmu_irq_flag, FALLING);
        pmu->disableIRQ(XPOWERS_AXP192_ALL_IRQ);
        pmu->clearIrqStatus();
        pmu->enableIRQ(
            XPOWERS_AXP192_BAT_INSERT_IRQ | XPOWERS_AXP192_BAT_REMOVE_IRQ |
            XPOWERS_AXP192_VBUS_INSERT_IRQ | XPOWERS_AXP192_VBUS_REMOVE_IRQ |
            XPOWERS_AXP192_PKEY_SHORT_IRQ |
            XPOWERS_AXP192_BAT_CHG_DONE_IRQ | XPOWERS_AXP192_BAT_CHG_START_IRQ);
        pmu->clearIrqStatus();
#endif
    }
    else if (pmu->getChipModel() == XPOWERS_AXP2101)
    {
        ESP_LOGI(LOG_TAG, "setting up AXP2101");
#ifdef AXP2101
        // https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf
        // VBUS min 3.9V, max 5.5V.
        pmu->setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V12);
        // VBUS max 2A both in and out.
        pmu->setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);
        // VBAT operating range is between 2.5V and 4.5V. The PMU shuts down just below 3.3V.
        // When using 3V for the shutdown voltage and 5% for the threshold, the PMU actually shuts down at ~3.1v.
        pmu->setSysPowerDownVoltage(3100);
        pmu->setLowBatShutdownThreshold(7);

        pmu->setLongPressPowerOFF();
        pmu->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
        pmu->setPowerKeyPressOnTime(XPOWERS_POWERON_2S);

        pmu->disableTSPinMeasure();
        pmu->setChargingLedMode(false);
        pmu->disableDC2();
        pmu->disableDC4();
        pmu->disableDC5();
        pmu->disableALDO1();
        pmu->disableALDO4();
        pmu->disableBLDO1();
        pmu->disableBLDO2();
        pmu->disableDLDO1();
        pmu->disableDLDO2();

        pmu->enableBattDetection();
        pmu->enableVbusVoltageMeasure();
        pmu->enableBattVoltageMeasure();
        pmu->enableSystemVoltageMeasure();

        // https://www.solomon-systech.com/product/ssd1306/
        // "– VDD= 1.65V – 3.3V, <VBAT for IC Logic"
        // "– VBAT= 3.3V – 4.2V for charge pump regulator circuit"
        pmu->setDC1Voltage(3300);
        pmu->enableDC1();
        // DC2 is unused.
        pmu->disableDC2();
        // https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf
        // "Supply voltage = 3.3 V"
        pmu->setALDO2Voltage(3300);
        pmu->enableALDO2();
        // https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_(GPS.G6-HW-09005).pdf
        // "NEO-6Q/M NEO-6P/V/T Min: 2.7, Typ: 3.0, Max: 3.6"
        pmu->setALDO3Voltage(3000);
        pmu->enableALDO3();

        // Conserve power by disabling temperature measurement.
        pmu->disableTemperatureMeasure();

        // Start charging the battery if it is installed.
        pmu->setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_100MA);
        pmu->setChargerConstantCurr(XPOWERS_AXP202_CHG_CUR_1000MA);
        pmu->setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_50MA);
        pmu->setChargeTargetVoltage(XPOWERS_AXP202_CHG_VOL_4V2);

        // Start charging the GPS memory backup battery.
        pmu->setButtonBatteryChargeVoltage(3100);
        pmu->enableButtonBatteryCharge();

        // Handle power management events.
        pinMode(POWER_PMU_IRQ, INPUT);
        attachInterrupt(POWER_PMU_IRQ, power_set_pmu_irq_flag, FALLING);
        pmu->disableIRQ(XPOWERS_AXP192_ALL_IRQ);
        pmu->clearIrqStatus();
        pmu->enableIRQ(
            XPOWERS_AXP202_BAT_INSERT_IRQ | XPOWERS_AXP202_BAT_REMOVE_IRQ |
            XPOWERS_AXP202_VBUS_INSERT_IRQ | XPOWERS_AXP202_VBUS_REMOVE_IRQ |
            XPOWERS_AXP202_PKEY_SHORT_IRQ |
            XPOWERS_AXP202_BAT_CHG_DONE_IRQ | XPOWERS_AXP202_BAT_CHG_START_IRQ);
        pmu->clearIrqStatus();
#endif
    }
    power_i2c_unlock();
    ESP_LOGI(LOG_TAG, "power management is ready");
    // When battery voltage drops low and waking up from deep sleep, enter super saver power mode.
    if (pmu->getVbusVoltage() < 4000 && pmu->getBattVoltage() > 2000 && pmu->getBattVoltage() < POWER_SUPER_SAVER_THRESHOLD_MILLIVOLT)
    {
        // Set the initial power mode according to battery voltage.
        power_set_config(POWER_SUPER_SAVER);
        ESP_LOGI(LOG_TAG, "entering super saver power mode due to low battery voltage");
    }
}

void power_set_pmu_irq_flag(void)
{
    pmu_irq_flag = true;
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
    // Unfortunately, AXP2101's blue LED won't stay on.
#ifdef AXP192
    power_i2c_lock();
    pmu->setChargingLedMode(true);
    power_i2c_unlock();
#endif
}

void power_led_off()
{
    power_i2c_lock();
    pmu->setChargingLedMode(false);
    power_i2c_unlock();
}

double power_get_sum_curr_draw_readings()
{
    return sum_curr_draw_readings;
}

void power_read_handle_lastest_irq()
{
    power_i2c_lock();
    if (pmu_irq_flag)
    {
        pmu_irq_flag = false;
        pmu->getIrqStatus();
        if (pmu->isBatInsertIrq())
        {
            ESP_LOGI(LOG_TAG, "battery inserted");
        }
        if (pmu->isBatRemoveIrq())
        {
            ESP_LOGI(LOG_TAG, "battery removed");
        }
        if (pmu->isBatChagerDoneIrq())
        {
            ESP_LOGI(LOG_TAG, "battery charging completed");
        }
        if (pmu->isPekeyShortPressIrq())
        {
            // If the button is clicked before OLED's timer deemed the display to have been inactive.
            // Otherwise, the user is simply clicking the button to wake the screen up.
            if (!oled_reset_last_input_timestamp())
            {
                ESP_LOGI(LOG_TAG, "turning to the next page");
                oled_go_to_next_page();
            }
        }
        if (pmu->isPekeyLongPressIrq())
        {
            ESP_LOGW(LOG_TAG, "shutting down");
            pmu->setChargingLedMode(false);
            pmu->shutdown();
        }
        pmu->clearIrqStatus();
    }
    power_i2c_unlock();
}

void power_start_conserving()
{
    power_i2c_lock();
    int new_mode = config_mode_id;
    if (status.batt_millivolt < POWER_SUPER_SAVER_THRESHOLD_MILLIVOLT)
    {
        new_mode = POWER_SUPER_SAVER;
    }
    else
    {
        new_mode = POWER_SAVER;
    }
    if (new_mode != config_mode_id)
    {
        config_mode_id_before_conserving = config_mode_id;
        config_mode_id = new_mode;
        ESP_LOGW(LOG_TAG, "start conserving power by switching from mode %d to power save mode %d, battery voltage: %+.2f, current: %.2f", config_mode_id_before_conserving, config_mode_id, status.batt_millivolt, status.batt_milliamp);
    }
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
    else if (config.mode_id != POWER_SUPER_SAVER)
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

    // Give Bluetooth and WiFi a turn at scanning prior to transmitting foxhunt info.
    if (config.mode_id != POWER_SUPER_SAVER &&
        lorawan_tx_counter % LORAWAN_TX_KINDS == LORAWAN_TX_KIND_POS &&
        // There is not enough memory to run bluetooth and wifi simultaneously.
        !(ret & POWER_TODO_TURN_ON_WIFI) && (!oled_get_state() || oled_get_page_number() != OLED_PAGE_WIFI_INFO) &&
        // Is it time to turn on bluetooth for routine scan?
        (ms_since_last_tx > config.tx_interval_sec * 1000 - bt_prep_duration_ms - wifi_prep_duration_ms - bt_wifi_gap_ms &&
         ms_since_last_tx < config.tx_interval_sec * 1000 - wifi_prep_duration_ms - bt_wifi_gap_ms))
    {
        ret |= POWER_TODO_TURN_ON_BLUETOOTH;
    }
    if (config.mode_id != POWER_SUPER_SAVER &&
        lorawan_tx_counter % LORAWAN_TX_KINDS == LORAWAN_TX_KIND_POS &&
        // There is not enough memory to run bluetooth and wifi simultaneously.
        !(ret & POWER_TODO_TURN_ON_BLUETOOTH) && (!oled_get_state() || oled_get_page_number() != OLED_PAGE_BT_INFO) &&
        // Is it time to turn on wifi for routine scan?
        (ms_since_last_tx > config.tx_interval_sec * 1000 - wifi_prep_duration_ms &&
         ms_since_last_tx < config.tx_interval_sec * 1000))
    {
        ret |= POWER_TODO_TURN_ON_WIFI;
    }

    // The sensor readings need to be taken for the first TX since boot as well as shortly before next TX.
    if (lorawan_tx_counter == 0 ||
        // The super saver power mode only transmits environment condition readings.
        (((lorawan_tx_counter % LORAWAN_TX_KINDS == LORAWAN_TX_KIND_ENV) || (config.mode_id == POWER_SUPER_SAVER)) &&
         (ms_since_last_tx > config.tx_interval_sec * 1000 - env_sensor_prep_duration_ms && ms_since_last_tx < config.tx_interval_sec * 1000)))
    {
        ret |= POWER_TODO_READ_ENV_SENSOR;
    }

    // If there is no power-related task to do and no user input, then ask caller to reduce CPU speed to conserve power.
    // Be aware that if user is looking at WiFi/BT scanner then CPU speed must not be reduced or WiFi/BT will cause a panic.
    // The lower CPU clock speed is sufficient for GPS though.
    if ((ret == 0 || ret == POWER_TODO_TURN_ON_GPS) &&                                  // lora / wifi / bt / sensor are not needed (flags unset)
        oled_get_ms_since_last_input() > wifi_prep_duration_ms + bt_prep_duration_ms && // leave CPU frequency high for recent button input
        !wifi_get_state() && !bluetooth_get_state())                                    // wifi and bt are powered off
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
    status.is_batt_charging = pmu->isCharging();
    status.batt_millivolt = pmu->getBattVoltage();
    if (status.batt_millivolt < 500)
    {
        // The AXP chip occasionally produces erranous and exceedingly low battery voltage readings even without a battery installed.
        status.batt_millivolt = 0;
    }
    status.usb_millivolt = pmu->getVbusVoltage();
#ifdef AXP192
    if (status.is_batt_charging)
    {
        status.batt_milliamp = pmu->getBatteryChargeCurrent();
    }
    else
    {
        status.batt_milliamp = -pmu->getBattDischargeCurrent();
    }
    status.power_draw_milliamp = pmu->getVbusCurrent();
#endif
#ifdef AXP2101
    // Unsure if the library is capable of reading the current consumptino: https://github.com/lewisxhe/XPowersLib/issues/12
    // Just default to typical readings.
    status.power_draw_milliamp = 80;
    status.batt_milliamp = 0;
    if (status.batt_millivolt > 2000 && !status.is_batt_charging && status.usb_millivolt < 4000)
    {
        status.batt_milliamp = -80;
    }
#endif
    // The power management chip always draws power from USB when it is available.
    // Use battery discharging current as a condition too because the VBus current occasionally reads 0.
    status.is_usb_power_available = status.is_batt_charging || status.batt_milliamp > 3 || status.batt_millivolt < 3000 || status.power_draw_milliamp > 3 || status.usb_millivolt > 4000;
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
    case POWER_SUPER_SAVER:
        return power_config_supersaver;
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
        bool success = setCpuFrequencyMhz(new_mhz);
        // Changing CPU frequency seems to always mess up the monitor baud rate.
        // See also: https://github.com/espressif/arduino-esp32/issues/6032 ("setCpuFrequencyMhz() changes Serial bauds if frequency<80Mhz")
        Serial.updateBaudRate(SERIAL_MONITOR_BAUD_RATE);
        if (!success)
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