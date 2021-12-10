#include <Arduino.h>
#include <BLEDevice.h>
#include <esp_task_wdt.h>
#include "bluetooth.h"

static const char LOG_TAG[] = __FILE__;

static unsigned long round_num = 0;
static BLEScan *scanner = NULL;
static BLEAdvertisedDevice last_loudest_sender, loudest_sender;
static int last_loudest_rssi = BLUETOOTH_RSSI_FLOOR, loudest_rssi = BLUETOOTH_RSSI_FLOOR;
static int last_num_devices = 0, num_devices = 0;

void bluetooth_setup()
{
    // The device name is not used because this scanner does not need to advertise itself.
    BLEDevice::init("hzgl-comm");
    scanner = BLEDevice::getScan();
    ESP_LOGI(LOG_TAG, "successfully initialised bluetooth");
}

void BTDeviceDiscoveryCallBack::onResult(BLEAdvertisedDevice dev)
{
    if (dev.getRSSI() > loudest_rssi)
    {
        loudest_sender = dev;
        loudest_rssi = dev.getRSSI();
    }
    num_devices++;
}

void bluetooth_scan()
{
    // Remember previous scanning result.
    last_loudest_sender = loudest_sender;
    last_loudest_rssi = loudest_rssi;
    last_num_devices = num_devices;
    loudest_rssi = BLUETOOTH_RSSI_FLOOR;
    loudest_sender = BLEAdvertisedDevice();
    num_devices = 0;

    BLEDevice::setPower(ESP_PWR_LVL_P9);
    scanner->setAdvertisedDeviceCallbacks(new BTDeviceDiscoveryCallBack());
    scanner->setActiveScan(true);
    scanner->setInterval(100);
    scanner->setWindow(BLUETOOTH_SCAN_DUTY_CYCLE_PCT);
    BLEScanResults scan_results = scanner->start(BLUETOOTH_SCAN_DURATION_SEC, false);
    round_num++;
}

void bluetooth_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(BLUETOOTH_TASK_LOOP_DELAY_MS));
        bluetooth_scan();
    }
}

unsigned long bluetooth_get_round_num()
{
    return round_num;
}

BLEAdvertisedDevice bluetooth_get_loudest_sender()
{
    return last_loudest_sender;
}

int bluetooth_get_total_num_devices()
{
    return last_num_devices;
}