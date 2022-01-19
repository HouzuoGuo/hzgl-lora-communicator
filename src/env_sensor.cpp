#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <esp_task_wdt.h>
#include "env_sensor.h"
#include "hardware_facts.h"
#include "power_management.h"
#include "lorawan.h"
#include "oled.h"

static const char LOG_TAG[] = __FILE__;

Adafruit_BME280 bme;
struct env_data latest;

void env_sensor_setup()
{
    memset(&latest, 0, sizeof(latest));
    power_i2c_lock();
    if (bme.begin(BME280_I2C_ADDR))
    {
        ESP_LOGI(LOG_TAG, "successfully initialised BME280 sensor");
    }
    else
    {
        ESP_LOGW(LOG_TAG, "failed to initialise BME280 sensor");
    }
    power_i2c_unlock();
}

void env_sensor_read_decode()
{
    power_i2c_lock();
    latest.altitude_metre = bme.readAltitude(1013.25);
    latest.humidity_pct = bme.readHumidity();
    latest.pressure_hpa = bme.readPressure() / 100;
    latest.temp_celcius = bme.readTemperature();
    power_i2c_unlock();
    if (latest.humidity_pct == 0 && latest.pressure_hpa == 0 && latest.temp_celcius == 0)
    {
        // Otherwise it will read 44330m.
        latest.altitude_metre = 0;
    }
    else
    {
        struct power_status power = power_get_status();
        if (power.is_usb_power_available)
        {
            latest.temp_celcius += TEMP_OFFSET_CELCIUS_USB;
        }
        else
        {
            latest.temp_celcius += TEMP_OFFSET_CELCIUS_BATT;
        }
    }
}

struct env_data env_sensor_get_data()
{
    return latest;
}

void env_sensor_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(ENV_SENSOR_TASK_LOOP_DELAY_MS));
        if (lorawan_is_warming_up() || (oled_is_awake() && oled_get_page_number() == OLED_PAGE_ENV_SENSOR_INFO))
        {
            env_sensor_read_decode();
        }
    }
}
