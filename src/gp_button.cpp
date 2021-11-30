#include <Arduino.h>
#include <esp_task_wdt.h>
#include "gp_button.h"
#include "hardware_facts.h"
#include "power_management.h"

static const char LOG_TAG[] = __FILE__;

static SemaphoreHandle_t mutex;
bool is_button_down = false;
unsigned long pushed_down_timestamp = 0;
unsigned long last_click_timestamp = 0;
String morse_signals_buf = "";
String morse_message_buf = "";
bool morse_space_inserted_after_word = false;

void gp_button_setup()
{
  mutex = xSemaphoreCreateMutex();
}

unsigned long gp_button_get_last_click_timestamp()
{
  return last_click_timestamp;
}

void gp_button_clear_morse_message_buf()
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  morse_message_buf.clear();
  xSemaphoreGive(mutex);
}

void gp_button_decode_morse_and_clear()
{
  char ch = gp_button_decode_morse(morse_signals_buf);
  ESP_LOGI(LOG_TAG, "morse decoded %c from %s", ch, morse_signals_buf.c_str());
  morse_message_buf += ch;
  morse_signals_buf.clear();
}

char gp_button_decode_morse(const String presses)
{
  if (presses == ".-")
    return 'a';
  else if (presses == "-...")
    return 'b';
  else if (presses == "-.-.")
    return 'c';
  else if (presses == "-..")
    return 'd';
  else if (presses == ".")
    return 'e';
  else if (presses == "..-.")
    return 'f';
  else if (presses == "--.")
    return 'g';
  else if (presses == "....")
    return 'h';
  else if (presses == "..")
    return 'i';
  else if (presses == ".---")
    return 'j';
  else if (presses == "-.-")
    return 'k';
  else if (presses == ".-..")
    return 'l';
  else if (presses == "--")
    return 'm';
  else if (presses == "-.")
    return 'n';
  else if (presses == "---")
    return 'o';
  else if (presses == ".--.")
    return 'p';
  else if (presses == "--.-")
    return 'q';
  else if (presses == ".-.")
    return 'r';
  else if (presses == "...")
    return 's';
  else if (presses == "-")
    return 't';
  else if (presses == "..-")
    return 'u';
  else if (presses == "...-")
    return 'v';
  else if (presses == ".--")
    return 'w';
  else if (presses == "-..-")
    return 'x';
  else if (presses == "-.--")
    return 'y';
  else if (presses == "--..")
    return 'z';
  else if (presses == ".----")
    return '1';
  else if (presses == "..---")
    return '2';
  else if (presses == "...--")
    return '3';
  else if (presses == "....-")
    return '4';
  else if (presses == ".....")
    return '5';
  else if (presses == "-....")
    return '6';
  else if (presses == "--...")
    return '7';
  else if (presses == "---..")
    return '8';
  else if (presses == "----.")
    return '9';
  else if (presses == "-----")
    return '0';
  else if (presses == ".-.-.-")
    return '.';
  else if (presses == "--..--")
    return ',';
  else if (presses == "..--..")
    return '?';
  else if (presses == ".----.")
    return '\'';
  else if (presses == "-.-.--")
    return '!';
  else if (presses == "-..-.")
    return '/';
  else if (presses == "-.--.")
    return '(';
  else if (presses == "-.--.-")
    return ')';
  else if (presses == ".-...")
    return '&';
  else if (presses == "---...")
    return ':';
  else if (presses == "-.-.-.")
    return ';';
  else if (presses == "-...-")
    return '=';
  else if (presses == ".-.-.")
    return '+';
  else if (presses == "-....-")
    return '-';
  else if (presses == "..--.-")
    return '_';
  else if (presses == ".-..-.")
    return '"';
  else if (presses == "...-..-")
    return '$';
  else if (presses == ".--.-.")
    return '@';
  else
    return '?';
}

void gp_button_read()
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (digitalRead(GENERIC_PURPOSE_BUTTON) == LOW)
  {
    power_led_on();
    if (!is_button_down)
    {
      is_button_down = true;
      pushed_down_timestamp = millis();
    }
  }
  else
  {
    if (is_button_down)
    {
      power_led_off();
      last_click_timestamp = millis();
      unsigned long duration = millis() - pushed_down_timestamp;
      ESP_LOGI(LOG_TAG, "pressed duration %d", duration);
      is_button_down = false;

      if (duration > MORSE_CLEAR_PRESS_DURATION_MS)
      {
        morse_signals_buf.clear();
        morse_message_buf.clear();
        ESP_LOGI(LOG_TAG, "buffers cleared after a long press");
      }
      else if (duration > MORSE_BACKSPACE_PRESS_DURATION_MS)
      {
        morse_signals_buf.clear();
        morse_message_buf.remove(morse_message_buf.length() - 1);
        ESP_LOGI(LOG_TAG, "deleted last character");
      }
      else if (duration > MORSE_DASH_PRESS_DURATION_MS)
      {
        morse_signals_buf += "-";
        ESP_LOGI(LOG_TAG, "latest_presses + dash: %s", morse_signals_buf.c_str());
      }
      else if (duration > MORSE_DOT_PRESS_DURATION_MS)
      {
        morse_signals_buf += ".";
        ESP_LOGI(LOG_TAG, "latest_presses + dot: %s", morse_signals_buf.c_str());
      }
    }

    unsigned long sinceLastPress = millis() - last_click_timestamp;
    if (sinceLastPress > MORSE_INTERVAL_BETWEEN_LETTERS_MS && morse_signals_buf.length() > 0)
    {
      ESP_LOGI(LOG_TAG, "%dms have elapsed since last press, decoding the character.", sinceLastPress);
      gp_button_decode_morse_and_clear();
      morse_space_inserted_after_word = false;
    }
    else if (sinceLastPress > MORSE_INTERVAL_BETWEEN_WORDS_MS && !morse_space_inserted_after_word && morse_message_buf.length() > 0)
    {
      ESP_LOGI(LOG_TAG, "%dms have elapsed since last press, inserting word boundary.", sinceLastPress);
      morse_message_buf += ' ';
      morse_space_inserted_after_word = true;
    }
  }
  xSemaphoreGive(mutex);
}

void gp_button_task_loop(void *_)
{
  while (true)
  {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(GP_BUTTON_TASK_LOOP_DELAY_MS));
    gp_button_read();
  }
}

String gp_button_get_latest_morse_signals()
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  String ret = morse_signals_buf;
  xSemaphoreGive(mutex);
  return ret;
}

String gp_button_get_morse_message_buf()
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  String ret = morse_message_buf;
  xSemaphoreGive(mutex);
  return ret;
}
