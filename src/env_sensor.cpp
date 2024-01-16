#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <esp_task_wdt.h>
#include "env_sensor.h"
#include "hardware_facts.h"
#include "power_management.h"
#include "oled.h"

static const char LOG_TAG[] = __FILE__;

static Adafruit_BME280 bme;
static struct env_data latest;
static double sum_temp_readings = 0.0;

void env_sensor_setup()
{
    ESP_LOGI(LOG_TAG, "setting up sensors");
    memset(&latest, 0, sizeof(latest));
    power_i2c_lock();
    if (!bme.begin(BME280_I2C_ADDR))
    {
        ESP_LOGW(LOG_TAG, "failed to initialise BME280 sensor");
    }
    power_i2c_unlock();

    ESP_LOGI(LOG_TAG, "sensors are ready");
}

void env_sensor_read_decode()
{
    static struct env_data new_readings;
    power_i2c_lock();
    new_readings.altitude_metre = bme.readAltitude(1013.25);
    new_readings.humidity_pct = bme.readHumidity();
    new_readings.pressure_hpa = bme.readPressure() / 100;
    new_readings.temp_celcius = bme.readTemperature();
    power_i2c_unlock();
    // Handle invalid sensor readings.
    if (new_readings.humidity_pct == 0 && new_readings.pressure_hpa == 0 && new_readings.temp_celcius == 0)
    {
        ESP_LOGI(LOG_TAG, "invalid bme humidity reading: %.2f", new_readings.pressure_hpa);
        return;
    }
    // -6000ft to 60000ft.
    if (new_readings.pressure_hpa > 1240 || new_readings.pressure_hpa < 70)
    {
        ESP_LOGI(LOG_TAG, "invalid bme pressure reading: %.2f", new_readings.pressure_hpa);
        return;
    }
    if (new_readings.altitude_metre > 19000 || new_readings.altitude_metre < -1900)
    {
        ESP_LOGI(LOG_TAG, "invalid bme altitude reading: %.2f", new_readings.altitude_metre);
        return;
    }
    // Not sure why humidity would ever be negative.
    if (new_readings.humidity_pct > 120 || new_readings.humidity_pct < -20)
    {
        ESP_LOGI(LOG_TAG, "invalid bme humidity reading: %.2f", new_readings.humidity_pct);
        return;
    }
    // +-20 celcius from esp32 storage temperature maximum
    if (new_readings.temp_celcius > 170 || new_readings.temp_celcius < -60)
    {
        ESP_LOGI(LOG_TAG, "invalid bme temperature reading: %.2f", new_readings.temp_celcius);
        return;
    }
    // Save the latest readings
    latest = new_readings;
    struct power_status power = power_get_status();
    if (power.is_usb_power_available)
    {
        latest.temp_celcius += TEMP_OFFSET_CELCIUS_USB;
    }
    else
    {
        latest.temp_celcius += TEMP_OFFSET_CELCIUS_BATT;
    }
    sum_temp_readings += latest.temp_celcius;
    ESP_LOGI(LOG_TAG, "just took a round of readings");
}

struct env_data env_sensor_get_data()
{
    return latest;
}

double env_sensor_get_sum_temp_readings()
{
    return sum_temp_readings;
}

void env_sensor_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        if ((power_get_todo() & POWER_TODO_READ_ENV_SENSOR) || (oled_get_state() && oled_get_page_number() == OLED_PAGE_ENV_SENSOR_INFO))
        {
            env_sensor_read_decode();
        }
        vTaskDelay(pdMS_TO_TICKS(ENV_SENSOR_TASK_LOOP_DELAY_MS));
    }
}
