#include <Arduino.h>
#include <esp_task_wdt.h>
#include "lorawan.h"
#include "hardware_facts.h"
#include "oled.h"
#include "gp_button.h"
#include "power_management.h"
#include "gps.h"
#include "data_packet.h"
#include "env_sensor.h"
#include "wifi.h"
#include "bluetooth.h"
#include "supervisor.h"

static const char LOG_TAG[] = __FILE__;

void setup()
{
  // Automatically panic and reset when any task becomes stuck.
  ESP_ERROR_CHECK(esp_task_wdt_init(SUPERVISOR_WATCHDOG_TIMEOUT_SEC, true));
  // Keey an eye on the setup itself too.
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  Serial.begin(SERIAL_MONITOR_BAUD_RATE);
  ESP_LOGI(LOG_TAG, "hzgl-lorawan-communicator is starting up");
  pinMode(GENERIC_PURPOSE_BUTTON, INPUT);
  power_setup();
  lorawan_setup();
  env_sensor_setup();
  // The supervisor starts all essential tasks.
  supervisor_setup();
  ESP_LOGI(LOG_TAG, "setup completed");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_TASK_LOOP_DELAY_MS));
  if (millis() / 1000 < SUPERVISOR_UNCONDITIONAL_RESET_INTERVAL_SEC)
  {
    // Reset watchdog timer prior to reaching the soft reset interval.
    esp_task_wdt_reset();
  }
  else
  {
    ESP_LOGW(LOG_TAG, "performing regular soft reset");
    // Always perform a soft reset at regular interval. This helps clear unpredictable faults.
    esp_restart();
  }
}