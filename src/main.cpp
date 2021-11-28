#include <Arduino.h>
#include <esp_task_wdt.h>
#include "lorawan.h"
#include "hardware_facts.h"
#include "oled.h"
#include "gp_button.h"
#include "power_management.h"
#include "gps.h"
#include "i2c.h"
#include "data_packet.h"
#include "env_sensor.h"
#include "wifi.h"

static const char LOG_TAG[] = __FILE__;

void setup()
{
  pinMode(GENERIC_PURPOSE_BUTTON, INPUT);

  Serial.begin(115200);
  i2c_setup();
  power_setup();
  gp_button_setup();
  oled_setup();
  lorawan_setup();
  gps_setup();
  env_sensor_setup();
  wifi_setup();

  // Automatically panic and reset when a task gets stuck for over 30 seconds.
  esp_task_wdt_init(30, true);
  // Keey an eye on the setup itself too.
  esp_task_wdt_add(NULL);

  TaskHandle_t wifi_task, env_sensor_task, gps_task, lorawan_task, oled_task, gp_button_task, power_task;
  unsigned long priority = tskIDLE_PRIORITY;
  // ESP32 uses the first core (core 0) to handle wifi and BT radio, leaving the second core available for other tasks.
  // The higher the priority number, the higher the task priority.
  xTaskCreatePinnedToCore(wifi_task_loop, "wifi_task_loop", 16 * 1024, NULL, priority++, &wifi_task, 1);
  esp_task_wdt_add(wifi_task);
  xTaskCreatePinnedToCore(env_sensor_task_loop, "env_sensor_task_loop", 16 * 1024, NULL, priority++, &env_sensor_task, 1);
  esp_task_wdt_add(env_sensor_task);
  xTaskCreatePinnedToCore(gps_task_loop, "gps_task_loop", 16 * 1024, NULL, priority++, &gps_task, 1);
  esp_task_wdt_add(gps_task);
  xTaskCreatePinnedToCore(lorawan_task_loop, "lorawan_task_loop", 32 * 1024, NULL, priority++, &lorawan_task, 1);
  esp_task_wdt_add(lorawan_task);
  xTaskCreatePinnedToCore(oled_task_loop, "oled_task_loop", 32 * 1024, NULL, priority++, &oled_task, 1);
  esp_task_wdt_add(oled_task);
  xTaskCreatePinnedToCore(gp_button_task_loop, "gp_button_task_loop", 8 * 1024, NULL, priority++, &gp_button_task, 1);
  esp_task_wdt_add(gp_button_task);
  xTaskCreatePinnedToCore(power_task_loop, "power_task_loop", 16 * 1024, NULL, priority++, &power_task, 1);
  esp_task_wdt_add(power_task);
  ESP_LOGI(LOG_TAG, "setup completed");
}

void loop()
{
  // The loop simply yields to all other tasks.
  vTaskDelay(pdMS_TO_TICKS(10 * 1000));
  esp_task_wdt_reset();
}
