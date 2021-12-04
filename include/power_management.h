#pragma once

// POWER_TASK_LOOP_DELAY_MS is the sleep interval of power management IRQ polling task loop.
#define POWER_TASK_LOOP_DELAY_MS 200

// POWER_TASK_READ_STATUS_DELAY_MS is the interval between two readings of the latest power status.
#define POWER_TASK_READ_STATUS_DELAY_MS (POWER_TASK_LOOP_DELAY_MS * 5)

struct power_status
{
    int batt_millivolt;
    bool is_batt_charging, is_usb_power_available;
    float batt_milliamp, power_draw_milliamp;
};

void power_setup();
void power_led_on();
void power_led_off();
int power_get_uptime_sec();
struct power_status power_get_status();
void power_read_status();
void power_log_status();
void power_task_loop(void *);