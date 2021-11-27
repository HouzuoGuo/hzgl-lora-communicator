#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <esp_task_wdt.h>
#include "env_sensor.h"
#include "hardware_facts.h"
#include "i2c.h"

static const char LOG_TAG[] = __FILE__;

Adafruit_BME280 bme;
struct env_data latest;

void env_sensor_setup()
{
    i2c_lock();
    if (bme.begin(BME280_I2C_ADDR))
    {
        ESP_LOGI(LOG_TAG, "successfully initialised BME280 sensor");
    }
    else
    {
        ESP_LOGI(LOG_TAG, "failed to initialise BME280 sensor");
    }
    i2c_unlock();
}

void env_sensor_read_decode()
{
    i2c_lock();
    latest.altitude_metre = bme.readAltitude(1013.25);
    latest.humidity_pct = bme.readHumidity();
    latest.pressure_hpa = bme.readPressure() / 100;
    latest.temp_celcius = bme.readTemperature();
    i2c_unlock();
}

struct env_data env_sensor_get_data()
{
    return latest;
}

void env_sensor_task_loop(void *_)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(ENV_SENSOR_TASK_LOOP_DELAY_MS));
        env_sensor_read_decode();
        esp_task_wdt_reset();
    }
}