#pragma once

// POWER_TASK_LOOP_DELAY_MS is the sleep interval of power management IRQ polling task loop.
#define POWER_TASK_LOOP_DELAY_MS 200

void power_setup();
void power_led_on();
void power_led_off();
int power_get_uptime_sec();
int power_get_battery_millivolt();
bool power_is_batt_charging();
void power_task_loop(void *);