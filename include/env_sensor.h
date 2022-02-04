#pragma once

// ENV_SENSOR_TASK_LOOP_DELAY_MS is the sleep interval of the environment sensor task loop.
#define ENV_SENSOR_TASK_LOOP_DELAY_MS 1500

struct env_data
{
    double temp_celcius, humidity_pct, pressure_hpa, altitude_metre;
};

void env_sensor_setup();
void env_sensor_read_decode();
double env_sensor_get_sum_temp_readings();
struct env_data env_sensor_get_data();
void env_sensor_task_loop(void *_);