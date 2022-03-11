#include <Arduino.h>
#include <esp_task_wdt.h>
#include "gp_button.h"
#include "oled.h"
#include "hardware_facts.h"
#include "power_management.h"

static const char LOG_TAG[] = __FILE__;

static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
static bool is_button_down = false, is_lower_case = true;
static unsigned long pushed_down_timestamp = 0;
static unsigned long last_click_timestamp = 0;
static String morse_signals_buf = "";
static String morse_message_buf = "";
static String morse_edit_hint = "";
static bool morse_space_inserted_after_word = false;

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
  if (ch == '\0')
  {
    ESP_LOGI(LOG_TAG, "clear invalid morse input");
    morse_signals_buf.clear();
    return;
  }
  ESP_LOGI(LOG_TAG, "morse decoded %c from %s", ch, morse_signals_buf.c_str());
  morse_message_buf += ch;
  morse_signals_buf.clear();
}

char gp_button_decode_morse(const String presses)
{
  char ret = '\0';
  if (presses == ".-")
    ret = 'a';
  else if (presses == "-...")
    ret = 'b';
  else if (presses == "-.-.")
    ret = 'c';
  else if (presses == "-..")
    ret = 'd';
  else if (presses == ".")
    ret = 'e';
  else if (presses == "..-.")
    ret = 'f';
  else if (presses == "--.")
    ret = 'g';
  else if (presses == "....")
    ret = 'h';
  else if (presses == "..")
    ret = 'i';
  else if (presses == ".---")
    ret = 'j';
  else if (presses == "-.-")
    ret = 'k';
  else if (presses == ".-..")
    ret = 'l';
  else if (presses == "--")
    ret = 'm';
  else if (presses == "-.")
    ret = 'n';
  else if (presses == "---")
    ret = 'o';
  else if (presses == ".--.")
    ret = 'p';
  else if (presses == "--.-")
    ret = 'q';
  else if (presses == ".-.")
    ret = 'r';
  else if (presses == "...")
    ret = 's';
  else if (presses == "-")
    ret = 't';
  else if (presses == "..-")
    ret = 'u';
  else if (presses == "...-")
    ret = 'v';
  else if (presses == ".--")
    ret = 'w';
  else if (presses == "-..-")
    ret = 'x';
  else if (presses == "-.--")
    ret = 'y';
  else if (presses == "--..")
    ret = 'z';
  else if (presses == ".----")
    ret = '1';
  else if (presses == "..---")
    ret = '2';
  else if (presses == "...--")
    ret = '3';
  else if (presses == "....-")
    ret = '4';
  else if (presses == ".....")
    ret = '5';
  else if (presses == "-....")
    ret = '6';
  else if (presses == "--...")
    ret = '7';
  else if (presses == "---..")
    ret = '8';
  else if (presses == "----.")
    ret = '9';
  else if (presses == "-----")
    ret = '0';
  else if (presses == ".-.-.-")
    ret = '.';
  else if (presses == "--..--")
    ret = ',';
  else if (presses == "..--..")
    ret = '?';
  else if (presses == ".----.")
    ret = '\'';
  else if (presses == "-.-.--")
    ret = '!';
  else if (presses == "-..-.")
    ret = '/';
  else if (presses == "-.--.")
    ret = '(';
  else if (presses == "-.--.-")
    ret = ')';
  else if (presses == ".-...")
    ret = '&';
  else if (presses == "---...")
    ret = ':';
  else if (presses == "-.-.-.")
    ret = ';';
  else if (presses == "-...-")
    ret = '=';
  else if (presses == ".-.-.")
    ret = '+';
  else if (presses == "-....-")
    ret = '-';
  else if (presses == "..--.-")
    ret = '_';
  else if (presses == ".-..-.")
    ret = '"';
  else if (presses == "...-..-")
    ret = '$';
  else if (presses == ".--.-.")
    ret = '@';
  else
    morse_edit_hint = "Unknown morse input";

  if (!is_lower_case && ret >= 'a' && ret <= 'z')
  {
    ret = toupper(ret);
  }
  return ret;
}

void gp_button_read()
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  // LOW means the button is pressed down.
  if (digitalRead(GENERIC_PURPOSE_BUTTON) == LOW)
  {
    power_led_on();
    if (!is_button_down)
    {
      is_button_down = true;
      pushed_down_timestamp = millis();
      oled_reset_last_input_timestamp();
    }
    else if (oled_get_page_number() == OLED_PAGE_TX_MESSAGE || OLED_PAGE_TX_COMMAND)
    {
      // Show morse edit hint.
      unsigned long duration = millis() - pushed_down_timestamp;
      if (duration > MORSE_CLEAR_PRESS_DURATION_MS)
      {
        morse_edit_hint = "Release to clear";
      }
      else if (duration > MORSE_CHANGE_CASE_PRESS_DURATION_MS)
      {
        morse_edit_hint = "Release to switch aA";
      }
      else if (duration > MORSE_BACKSPACE_PRESS_DURATION_MS)
      {
        morse_edit_hint = "Release to backspace";
      }
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
      if (oled_get_page_number() == OLED_PAGE_TX_MESSAGE || oled_get_page_number() == OLED_PAGE_TX_COMMAND)
      {
        // Interpret the press as a morse input command.
        if (duration > MORSE_CLEAR_PRESS_DURATION_MS)
        {
          morse_signals_buf.clear();
          morse_message_buf.clear();
          morse_edit_hint.clear();
          ESP_LOGI(LOG_TAG, "buffers cleared after a long press");
        }
        else if (duration > MORSE_CHANGE_CASE_PRESS_DURATION_MS)
        {
          is_lower_case = !is_lower_case;
          morse_edit_hint.clear();
          ESP_LOGI(LOG_TAG, "changed case");
        }
        else if (duration > MORSE_BACKSPACE_PRESS_DURATION_MS)
        {
          morse_signals_buf.clear();
          morse_message_buf.remove(morse_message_buf.length() - 1);
          morse_edit_hint.clear();
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
      else if (oled_get_page_number() == OLED_PAGE_POWER_MGMT)
      {
        // Interpret the press as a click on the LoRaWAN page, which switches between the two power modes.
        if (duration > GP_BUTTON_CLICK_DURATION)
        {
          power_config_t conf = power_get_config();
          // regular (default) -> boost -> saver
          if (conf.mode_id == POWER_REGULAR)
          {
            power_set_config(POWER_BOOST);
          }
          else if (conf.mode_id == POWER_BOOST)
          {
            power_set_config(POWER_SAVER);
          }
          else
          {
            power_set_config(POWER_REGULAR);
          }
        }
      }
    }
    if (oled_get_page_number() == OLED_PAGE_TX_MESSAGE || oled_get_page_number() == OLED_PAGE_TX_COMMAND)
    {
      // Handle the idle after last morse key input.
      unsigned long since_last_press = millis() - last_click_timestamp;
      if (since_last_press > MORSE_INTERVAL_BETWEEN_LETTERS_MS && morse_signals_buf.length() > 0)
      {
        ESP_LOGI(LOG_TAG, "%dms have elapsed since last press, decoding the character.", since_last_press);
        gp_button_decode_morse_and_clear();
        morse_space_inserted_after_word = false;
      }
      else if (since_last_press > MORSE_INTERVAL_BETWEEN_WORDS_MS && !morse_space_inserted_after_word && morse_message_buf.length() > 0)
      {
        ESP_LOGI(LOG_TAG, "%dms have elapsed since last press, inserting word boundary.", since_last_press);
        morse_message_buf += ' ';
        morse_space_inserted_after_word = true;
      }
    }
  }
  xSemaphoreGive(mutex);
}

void gp_button_task_loop(void *_)
{
  while (true)
  {
    esp_task_wdt_reset();
    gp_button_read();
    vTaskDelay(pdMS_TO_TICKS(GP_BUTTON_TASK_LOOP_DELAY_MS));
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

String gp_button_get_edit_hint()
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  String ret = morse_edit_hint;
  xSemaphoreGive(mutex);
  return ret;
}

bool gp_button_is_input_lower_case()
{
  xSemaphoreTake(mutex, portMAX_DELAY);
  bool ret = is_lower_case;
  xSemaphoreGive(mutex);
  return ret;
}