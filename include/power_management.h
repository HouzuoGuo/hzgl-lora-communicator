#pragma once

#include <lmic.h>

// POWER_TASK_LOOP_DELAY_MS is the sleep interval of power management IRQ polling task loop.
#define POWER_TASK_LOOP_DELAY_MS 100

// POWER_TASK_READ_STATUS_DELAY_MS is the interval between two readings of the latest power status.
#define POWER_TASK_READ_STATUS_DELAY_MS (POWER_TASK_LOOP_DELAY_MS * 5)

struct power_status
{
    int batt_millivolt, usb_millivolt;
    bool is_batt_charging, is_usb_power_available;
    // batt_milliamp is the rate of milliamps entering(+) in or discharged(-) from the battery.
    float batt_milliamp;
    // power_draw_milliamp is the rate of milliamps drawn from a power source. The number is always positive.
    float power_draw_milliamp;
};

const static int POWER_BOOST = 19909;
const static int POWER_REGULAR = 19905;
const static int POWER_SAVER = 19901;

typedef struct
{
    int mode_id;
    int power_dbm;
    int spreading_factor;
    int tx_interval_sec;
    String mode_name;
} power_config_t;

// The combo of ​​SF7​​ and bandwidth 125khz is often referred to as "DR5" (data rate 5): https://avbentem.github.io/airtime-calculator/ttn/eu868/
// Whereas the data rate drops to "3" when a transmission uses SF9.
const static power_config_t power_config_boost = {.mode_id = POWER_BOOST, .power_dbm = 22, .spreading_factor = DR_SF9, .tx_interval_sec = 20, .mode_name = "boost"};
const static power_config_t power_config_regular = {.mode_id = POWER_REGULAR, .power_dbm = 18, .spreading_factor = DR_SF7, .tx_interval_sec = 60, .mode_name = "regular"};
const static power_config_t power_config_saver = {.mode_id = POWER_SAVER, .power_dbm = 14, .spreading_factor = DR_SF7, .tx_interval_sec = 60, .mode_name = "saver"};

void power_set_config(power_config_t val);
power_config_t power_get_config();

void power_setup();
void power_i2c_lock();
void power_i2c_unlock();
void power_led_on();
void power_led_off();
void power_led_blink();
void power_start_conserving();
void power_stop_conserving();
int power_get_uptime_sec();
struct power_status power_get_status();
void power_set_config(power_config_t val);
power_config_t power_get_config();
void power_read_status();
void power_log_status();
void power_task_loop(void *);