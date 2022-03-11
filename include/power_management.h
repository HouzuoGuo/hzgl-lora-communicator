#pragma once

#include <lmic.h>
#include "lorawan.h"

// POWER_TASK_LOOP_DELAY_MS is the sleep interval of power management IRQ polling task loop.
#define POWER_TASK_LOOP_DELAY_MS 100

// POWER_TASK_READ_STATUS_DELAY_MS is the interval between two readings of the latest power status.
#define POWER_TASK_READ_STATUS_DELAY_MS (POWER_TASK_LOOP_DELAY_MS * 5)

// POWER_TASK_LOG_STATUS_DELAY_MS is the internal of logging the latest power consumption stats to serial.
#define POWER_TASK_LOG_STATUS_DELAY_MS (15 * 1000)

// POWER_DEFAULT_CPU_FREQ_MHZ is the regular CPU speed required for running this program.
// It is slower than the default CPU speed of ESP32, which is 240MHz.
#define POWER_DEFAULT_CPU_FREQ_MHZ 80
// POWER_LOWEST_CPU_FREQ_MHZ is the lowest CPU speed required to keep the program running, though insufficient for radio activities.
#define POWER_LOWEST_CPU_FREQ_MHZ 10

// POWER_TODO_LORAWAN_TX_RX bit field tells the caller of power_get_todo to proceed with LoRa RX and TX.
#define POWER_TODO_LORAWAN_TX_RX (1 << 1)
// POWER_TODO_REDUCE_CPU_FREQ bit field tells the caller of power_get_todo to reduce CPU frequency as a power saving measure.
#define POWER_TODO_REDUCE_CPU_FREQ (1 << 2)
// POWER_TODO_TURN_ON_BLUETOOTH bit field tells the caller of power_get_todo to turn on bluetooth and start scanning.
#define POWER_TODO_TURN_ON_BLUETOOTH (1 << 3)
// POWER_TODO_TURN_ON_WIFI bit field tells the caller of power_get_todo to turn on wifi and start scanning.
#define POWER_TODO_TURN_ON_WIFI (1 << 4)
// POWER_TODO_READ_ENV_SENSOR bit field tells the caller of power_get_todo to start reading environment sensor data.
#define POWER_TODO_READ_ENV_SENSOR (1 << 5)
// POWER_TODO_TURN_ON_GPS bit field tells the caller of power_get_todo to turn on GPS and start acquiring location.
#define POWER_TODO_TURN_ON_GPS (1 << 6)
// POWER_TODO_ENTER_DEEP_SLEEP bit field tells the caller of power_get_todo to enter deep sleep for the duration configured in power config.
#define POWER_TODO_ENTER_DEEP_SLEEP (1 << 7)

// POWER_SLOWEST_TX_INTERVAL_SEC is the slowest LoRaWAN transmission interval used in a power mode.
#define POWER_SLOWEST_TX_INTERVAL_SEC 90

// POWER_MIN_CONSERVATION_PERIOD_SEC is the minimum duration to wait before exiting from power conservation a second time.
// This interval helps to prevent rapidly switching between power saver config and reuglar config under unstable USB power.
// E.g. when USB power is supplied by solar panel.
#define POWER_MIN_CONSERVATION_PERIOD_SEC (10 * 60)

// MUTEX_LOCK_TIMEOUT_MS is the timeout in milliseconds used for obtaining a lock of bus/radio feature.
#define MUTEX_LOCK_TIMEOUT_MS 19876

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
    int deep_sleep_start_sec;
    int deep_sleep_duration_sec;
    String mode_name;
} power_config_t;

// Make two rounds of transmission before entering deep sleep (570 seconds =~ 10 minutes).
#define POWER_SAVER_WAKE_DURATION_SEC (2 * LORAWAN_TX_KINDS * (POWER_SLOWEST_TX_INTERVAL_SEC + 5))

// The combo of ​​SF7​​ and bandwidth 125khz is often referred to as "DR5" (data rate 5): https://avbentem.github.io/airtime-calculator/ttn/eu868/
// Whereas the data rate drops to "3" when a transmission uses SF9.
const static power_config_t power_config_boost = {
    .mode_id = POWER_BOOST,
    .power_dbm = 22,
    .spreading_factor = DR_SF9,
    .tx_interval_sec = 20,
    .deep_sleep_start_sec = 0,
    .deep_sleep_duration_sec = 0,
    .mode_name = "boost"};
const static power_config_t power_config_regular = {
    .mode_id = POWER_REGULAR,
    .power_dbm = 18,
    .spreading_factor = DR_SF7,
    .tx_interval_sec = 60,
    .deep_sleep_start_sec = 0,
    .deep_sleep_duration_sec = 0,
    .mode_name = "regular"};
const static power_config_t power_config_saver = {
    .mode_id = POWER_SAVER,
    .power_dbm = 14,
    .spreading_factor = DR_SF7,
    .tx_interval_sec = POWER_SLOWEST_TX_INTERVAL_SEC,
    .deep_sleep_start_sec = POWER_SAVER_WAKE_DURATION_SEC,
    // Enter deep sleep for 2/3rds of that duration.
    .deep_sleep_duration_sec = POWER_SAVER_WAKE_DURATION_SEC * 2 / 3,
    .mode_name = "saver"};

power_config_t power_get_config();

void power_setup();
void power_i2c_lock();
void power_i2c_unlock();
void power_wifi_bt_lock();
void power_wifi_bt_unlock();
void power_led_on();
void power_led_off();
void power_start_conserving();
void power_stop_conserving();
void power_set_cpu_freq_mhz(int);
int power_get_uptime_sec();
double power_get_sum_curr_draw_readings();
void power_inc_lorawan_tx_counter();
int power_get_lorawan_tx_counter();
unsigned long power_get_last_transmission_timestamp();
void power_set_last_transmission_timestamp();
bool power_get_may_transmit_lorawan();
int power_get_todo();
void power_enter_deep_sleep();
struct power_status power_get_status();
void power_set_config(int);
power_config_t power_get_config();
void power_read_status();
void power_log_status();
void power_task_loop(void *);