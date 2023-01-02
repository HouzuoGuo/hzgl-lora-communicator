#include <Arduino.h>
#include <BLEDevice.h>
#include <esp_task_wdt.h>

#include "bluetooth.h"
#include "power_management.h"
#include "oled.h"

static const char LOG_TAG[] = __FILE__;

static bool is_powered_on = false;

static unsigned long round_num = 0;
static BLEScan *scanner = NULL;
static BLEAdvertisedDevice last_loudest_sender, loudest_sender;
static int last_loudest_rssi = BLUETOOTH_RSSI_FLOOR, loudest_rssi = BLUETOOTH_RSSI_FLOOR;
static int last_num_devices = 0, num_devices = 0;

void bluetooth_on()
{
    power_wifi_bt_lock();
    if (is_powered_on)
    {
        power_wifi_bt_unlock();
        return;
    }
    ESP_LOGI(LOG_TAG, "turning on Bluetooth");
    power_set_cpu_freq_mhz(POWER_DEFAULT_CPU_FREQ_MHZ);
    // The device name is not used because this scanner does not need to advertise itself.
    BLEDevice::init("hzgl-comm");
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    scanner = BLEDevice::getScan();
    scanner->setActiveScan(true);
    scanner->setInterval(BLUETOOTH_SCAN_DURATION_SEC * 100);
    scanner->setWindow(BLUETOOTH_SCAN_DURATION_SEC * BLUETOOTH_SCAN_DUTY_CYCLE_PCT);
    is_powered_on = true;
    power_wifi_bt_unlock();
}

void bluetooth_off()
{
    power_wifi_bt_lock();
    if (!is_powered_on)
    {
        power_wifi_bt_unlock();
        return;
    }
    ESP_LOGI(LOG_TAG, "turning off Bluetooth");
    btStop();
    BLEDevice::deinit();
    is_powered_on = false;
    power_wifi_bt_unlock();
}

bool bluetooth_get_state()
{
    return is_powered_on;
}

void bluetooth_scan()
{
    power_wifi_bt_lock();
    // Remember the results from the previous round of scan.
    last_loudest_sender = loudest_sender;
    last_loudest_rssi = loudest_rssi;
    last_num_devices = num_devices;
    loudest_rssi = BLUETOOTH_RSSI_FLOOR;
    loudest_sender = BLEAdvertisedDevice();
    num_devices = 0;
    scanner->clearResults();
    scanner->start(BLUETOOTH_SCAN_DURATION_SEC, false);
    BLEScanResults results = scanner->getResults();
    for (size_t i = 0; i < results.getCount(); ++i)
    {
        BLEAdvertisedDevice dev = results.getDevice(i);
        if (dev.getRSSI() > loudest_rssi)
        {
            loudest_sender = dev;
            loudest_rssi = dev.getRSSI();
        }
        num_devices++;
    }
    round_num++;
    power_wifi_bt_unlock();
    ESP_LOGI(LOG_TAG, "just finished a round of scan");
}

void bluetooth_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        if ((power_get_todo() & POWER_TODO_TURN_ON_BLUETOOTH) || (oled_get_state() && oled_get_page_number() == OLED_PAGE_BT_INFO))
        {
            bluetooth_on();
            bluetooth_scan();
        }
        else
        {
            bluetooth_off();
        }
        // The scanner runs for BLUETOOTH_SCAN_DURATION_SEC and then rests for a short period of time.
        vTaskDelay(pdMS_TO_TICKS(BLUETOOTH_TASK_LOOP_DELAY_MS));
    }
}

unsigned long bluetooth_get_round_num()
{
    return round_num;
}

BLEAdvertisedDevice bluetooth_get_loudest_sender()
{
    BLEAdvertisedDevice ret;
    power_wifi_bt_lock();
    ret = last_loudest_sender;
    power_wifi_bt_unlock();
    return ret;
}

int bluetooth_get_total_num_devices()
{
    return last_num_devices;
}