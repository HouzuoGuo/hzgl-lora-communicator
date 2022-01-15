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
  Serial.begin(115200);
  if (!setCpuFrequencyMhz(80))
  {
    ESP_LOGW(LOG_TAG, "failed to set CPU frequency for power savings");
  }

  pinMode(GENERIC_PURPOSE_BUTTON, INPUT);
  power_setup();
  gp_button_setup();
  oled_setup();
  lorawan_setup();
  gps_setup();
  env_sensor_setup();
  wifi_setup();
  bluetooth_setup();
  // Automatically panic and reset when a task gets stuck for over 30 seconds.
  ESP_ERROR_CHECK(esp_task_wdt_init(SUPERVISOR_WATCHDOG_TIMEOUT_SEC, true));
  // Keey an eye on the setup itself too.
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  // The supervisor starts all essential tasks.
  supervisor_setup();
  ESP_LOGI(LOG_TAG, "setup completed");
}

void loop()
{
  // The loop is not used at all. Just yield to all other tasks.
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(SUPERVISOR_TASK_LOOP_DELAY_MS));
  power_log_status();
}