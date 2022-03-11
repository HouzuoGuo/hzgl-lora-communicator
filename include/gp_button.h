#pragma once

#include <axp20x.h>
#include "lorawan.h"

// MORSE_DOT_PRESS_DURATION_MS is the duration the button needs to be held in order to register a dot morse signal.
#define MORSE_DOT_PRESS_DURATION_MS 100
// MORSE_DOT_PRESS_DURATION_MS is the duration the button needs to be held in order to register a dash morse signal.
#define MORSE_DASH_PRESS_DURATION_MS (3 * MORSE_DOT_PRESS_DURATION_MS)
// MORSE_DOT_PRESS_DURATION_MS is the duration the button needs to be held in order to delete the last morse signal.
#define MORSE_BACKSPACE_PRESS_DURATION_MS 1000
// MORSE_CHANGE_CASE_PRESS_DURATION_MS is the duration the button needs to be held in order to switch between upper and lower case.
#define MORSE_CHANGE_CASE_PRESS_DURATION_MS (2 * MORSE_BACKSPACE_PRESS_DURATION_MS)
// MORSE_CLEAR_PRESS_DURATION_MS is the duration the button needs to be held in order to clear the decoded morse message buffer.
#define MORSE_CLEAR_PRESS_DURATION_MS (3 * MORSE_BACKSPACE_PRESS_DURATION_MS)
// MORSE_INTERVAL_BETWEEN_LETTERS_MS is the duration to wait before decoding the buffered morse signals into a letter.
#define MORSE_INTERVAL_BETWEEN_LETTERS_MS (3 * MORSE_DOT_PRESS_DURATION_MS)
// MORSE_INTERVAL_BETWEEN_WORDS_MS is the duration to wait before automatically inserting a space into the morse message buffer.
#define MORSE_INTERVAL_BETWEEN_WORDS_MS 4000

// GP_BUTTON_CLICK_DURATION is the duration the button needs to be held and released in order to register a click.
#define GP_BUTTON_CLICK_DURATION MORSE_DOT_PRESS_DURATION_MS

// GP_BUTTON_TASK_LOOP_DELAY_MS is the sleep interval of the GP button task loop.
#define GP_BUTTON_TASK_LOOP_DELAY_MS (MORSE_DOT_PRESS_DURATION_MS / 5)

void gp_button_read();
char gp_button_decode_morse(String);
void gp_button_decode_morse_and_clear();
void gp_button_clear_morse_message_buf();
String gp_button_get_latest_morse_signals();
String gp_button_get_morse_message_buf();
unsigned long gp_button_get_last_click_timestamp();
void gp_button_task_loop(void *_);
String gp_button_get_edit_hint();
bool gp_button_is_input_lower_case();