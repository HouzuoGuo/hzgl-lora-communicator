#pragma once

#include <lmic.h>

// POWER_TASK_LOOP_DELAY_MS is the sleep interval of power management IRQ polling task loop.
#define POWER_TASK_LOOP_DELAY_MS 100

// POWER_TASK_READ_STATUS_DELAY_MS is the interval between two readings of the latest power status.
#define POWER_TASK_READ_STATUS_DELAY_MS (POWER_TASK_LOOP_DELAY_MS * 5)

// POWER_TASK_LOG_STATUS_DELAY_MS is the internal of logging the latest power consumption stats to serial.
#define POWER_TASK_LOG_STATUS_DELAY_MS (30 * 1000)

// POWER_DEFAULT_CPU_FREQ_MHZ is the regular CPU speed required for running this program.
// It is slower than the default CPU speed of ESP32, which is 240MHz.
#define POWER_DEFAULT_CPU_FREQ_MHZ 80
// POWER_LOWEST_CPU_FREQ_MHZ is the lowest CPU speed required to keep the program running, though insufficient for radio activities.
#define POWER_LOWEST_CPU_FREQ_MHZ 20

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

// POWER_GPS_RUN_SLEEP_INTERVAL_SEC is the run & sleep duration of GPS in power saver configuration.
#define POWER_GPS_RUN_SLEEP_INTERVAL_SEC (5 * 60)

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
    bool intermittent_gps;
    String mode_name;
} power_config_t;

// The combo of ​​SF7​​ and bandwidth 125khz is often referred to as "DR5" (data rate 5): https://avbentem.github.io/airtime-calculator/ttn/eu868/
// Whereas the data rate drops to "3" when a transmission uses SF9.
const static power_config_t power_config_boost = {.mode_id = POWER_BOOST, .power_dbm = 22, .spreading_factor = DR_SF9, .tx_interval_sec = 20, .intermittent_gps = false, .mode_name = "boost"};
const static power_config_t power_config_regular = {.mode_id = POWER_REGULAR, .power_dbm = 18, .spreading_factor = DR_SF7, .tx_interval_sec = 60, .intermittent_gps = false, .mode_name = "regular"};
const static power_config_t power_config_saver = {.mode_id = POWER_SAVER, .power_dbm = 14, .spreading_factor = DR_SF7, .tx_interval_sec = 60, .intermittent_gps = true, .mode_name = "saver"};

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
void power_set_cpu_freq_mhz(int);
int power_get_uptime_sec();
void power_inc_lorawan_tx_counter();
int power_get_lorawan_tx_counter();
unsigned long power_get_last_transmission_timestamp();
void power_set_last_transmission_timestamp();
bool power_get_may_transmit_lorawan();
int power_get_todo();
void power_set_power_output(uint8_t ch, bool on);
struct power_status power_get_status();
void power_set_config(power_config_t val);
power_config_t power_get_config();
void power_read_status();
void power_log_status();
void power_task_loop(void *);