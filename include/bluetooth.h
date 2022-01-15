#pragma once

#include <BLEDevice.h>

#define BLUETOOTH_RSSI_FLOOR -100
#define BLUETOOTH_SCAN_DURATION_SEC 1
#define BLUETOOTH_SCAN_DUTY_CYCLE_PCT 50
#define BLUETOOTH_TASK_LOOP_DELAY_MS 500

void bluetooth_setup();
void bluetooth_on();
void bluetooth_off();
void bluetooth_scan();
void bluetooth_task_loop(void *_);
unsigned long bluetooth_get_round_num();
BLEAdvertisedDevice bluetooth_get_loudest_sender();
int bluetooth_get_total_num_devices();