#include <axp20x.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include "hardware_facts.h"
#include "i2c.h"
#include "oled.h"
#include "power_management.h"

static const char LOG_TAG[] = __FILE__;

static AXP20X_Class pmu;

void power_setup()
{
    i2c_lock();
    if (!pmu.begin(Wire, AXP192_SLAVE_ADDRESS))
    {
        ESP_LOGI(LOG_TAG, "successfully initialisex AXP power management chip");
    }
    else
    {
        ESP_LOGI(LOG_TAG, "failed to initialise AXP power management chip");
    }
    // The voltage levels took some inspiration from ESP32-paxcounter.
    pmu.setDCDC1Voltage(3300); // OLED
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
    pmu.setChargeControlCur(AXP1XX_CHARGE_CUR_700MA);
    pmu.setChargingTargetVoltage(AXP202_TARGET_VOL_4_15V);
    pmu.enableChargeing(true);
    pmu.setChgLEDMode(AXP20X_LED_OFF);

    pmu.setPowerOutPut(AXP192_LDO2, AXP202_ON);   // LoRa
    pmu.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // GPS
    pmu.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  // OLED
    pmu.setPowerOutPut(AXP192_DCDC2, AXP202_OFF); // unused
    pmu.setPowerOutPut(AXP192_EXTEN, AXP202_OFF); // unused
    i2c_unlock();
}

void power_led_on()
{
    i2c_lock();
    pmu.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
    i2c_unlock();
}

void power_led_off()
{
    i2c_lock();
    pmu.setChgLEDMode(AXP20X_LED_OFF);
    i2c_unlock();
}

int power_get_battery_millivolt()
{
    i2c_lock();
    int ret = (int)pmu.getBattVoltage();
    i2c_unlock();
    return ret;
}

void power_read_handle_lastest_irq()
{
    i2c_lock();
    pmu.readIRQ();
    i2c_unlock();
    if (pmu.isVbusOverVoltageIRQ())
    {
        i2c_lock();
        ESP_LOGI(TAG, "USB power supply voltage is too high (%f.3v)", pmu.getVbusVoltage() / 1000);
        i2c_unlock();
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
        ESP_LOGI(TAG, "turning to the next page");
        oled_go_to_next_page();
    }
    if (pmu.isPEKLongtPressIRQ())
    {
        ESP_LOGI(TAG, "shutting down");
        i2c_lock();
        pmu.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  // LoRa
        pmu.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // GPS
        pmu.setPowerOutPut(AXP192_DCDC1, AXP202_OFF); // OLED
        pmu.setPowerOutPut(AXP192_DCDC2, AXP202_OFF); // unused
        pmu.setPowerOutPut(AXP192_EXTEN, AXP202_OFF); // unused
        pmu.setChgLEDMode(AXP20X_LED_OFF);
        pmu.shutdown();
        i2c_unlock();
    }
    i2c_lock();
    pmu.clearIRQ();
    i2c_unlock();
}

bool power_is_batt_charging()
{
    i2c_lock();
    bool ret = pmu.isChargeing();
    i2c_unlock();
    return ret;
}

int power_get_uptime_sec()
{
    return (esp_timer_get_time() / 1000000);
}

void power_task_loop(void *_)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(POWER_TASK_LOOP_DELAY_MS));
        power_read_handle_lastest_irq();
        esp_task_wdt_reset();
    }
}
