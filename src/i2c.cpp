#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include "i2c.h"
#include "hardware_facts.h"

static SemaphoreHandle_t i2c_mutex;

void i2c_setup()
{
    i2c_mutex = xSemaphoreCreateMutex();
    if (Wire.begin(I2C_SDA, I2C_SCL))
    {
        ESP_LOGI(LOG_TAG, "successfully initialised I2C");
    }
    else
    {
        ESP_LOGI(LOG_TAG, "failed to initialise I2C");
    }
}

void i2c_lock()
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
}

void i2c_unlock()
{
    xSemaphoreGive(i2c_mutex);
}