#pragma once

#include <SSD1306Wire.h>
#include "hardware_facts.h"

// OLED_PAGE_RX_INFO is the page index number of the RX/TX info page, which is the first page displayed.
#define OLED_PAGE_RX_INFO 0
// OLED_PAGE_TX_MESSAGE is the page index number of the morse-code input page for typing the text message to be transmitted.
#define OLED_PAGE_TX_MESSAGE 1
// OLED_PAGE_TX_MESSAGE is the page index number of the morse-code input page for typing the app comand to be transmitted.
#define OLED_PAGE_TX_COMMAND 2
// OLED_PAGE_TX_MESSAGE is the page index number of the GPS date, time, and location information page.
#define OLED_PAGE_GPS_INFO 3
// OLED_PAGE_ENV_SENSOR_INFO is the page index number of the environment sensor information page.
#define OLED_PAGE_ENV_SENSOR_INFO 4
// OLED_PAGE_TX_MESSAGE is the page index number of the WiFi monitoring info page.
#define OLED_PAGE_WIFI_INFO 5
// OLED_PAGE_TX_MESSAGE is the page index number of the diagnosis information page.
#define OLED_PAGE_DIAGNOSIS 6

// OLED_TASK_LOOP_DELAY_MS is the sleep interval of the OLED display refresh task loop. The display refreshes at 20 FPS.
#define OLED_TASK_LOOP_DELAY_MS (1000 / 20)

// oled_setup initialises the OLED hardware and its user interface library.
void oled_setup();

// oled_display_line displays a string text on the specified line. Line number begins at 0.
void oled_draw_string_line(int line_number, String text);

int oled_get_page_number();
void oled_go_to_next_page();
unsigned long oled_get_last_page_nav_timestamp();
int oled_get_last_morse_input_page_num();
void oled_display_page_rx_info(char[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1]);
void oled_display_page_tx_message(char[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1]);
void oled_display_page_tx_command(char[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1]);
void oled_display_page_gps_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1]);
void oled_display_page_env_sensor_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1]);
void oled_display_page_env_wifi_sniffer_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1]);
void oled_display_page_diagnosis(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1]);
void oled_display_refresh();
void oled_task_loop(void *_);