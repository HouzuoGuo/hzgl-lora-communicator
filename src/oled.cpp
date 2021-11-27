#include <esp_task_wdt.h>
#include "env_sensor.h"
#include "gp_button.h"
#include "gps.h"
#include "hardware_facts.h"
#include "i2c.h"
#include "lorawan.h"
#include "oled.h"
#include "power_management.h"

static const char LOG_TAG[] = __FILE__;

static int curr_page_num = 0, last_morse_input_page_num = 0;
static unsigned long last_page_nav_timestamp = 0, last_gps_data_timestamp = 0;
static struct gps_data last_gps_data;

static SSD1306Wire oled(OLED_I2C_ADDR, I2C_SDA, I2C_SCL);

void oled_setup()
{
    i2c_lock();
    oled.init();
    oled.displayOn();
    oled.resetOrientation();
    oled.resetDisplay();
    oled.clear();

    oled.flipScreenVertically();
    oled.setBrightness(64);
    oled.setContrast(0xF1, 128, 0x40);
    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.setFont(ArialMT_Plain_10);
    i2c_unlock();
    ESP_LOGI(LOG_TAG, "successfully initialised OLED");
}

void oled_draw_string_line(int line_number, String text)
{
    oled.drawStringMaxWidth(0, line_number * OLED_FONT_HEIGHT_PX, 200, text);
}

int oled_get_page_number()
{
    return curr_page_num;
}

int oled_get_last_morse_input_page_num()
{
    return last_morse_input_page_num;
}

void oled_go_to_next_page()
{
    if (++curr_page_num == 6)
    {
        curr_page_num = OLED_PAGE_RX_INFO;
    }
    last_page_nav_timestamp = millis();
    ESP_LOGI(LOG_TAG, "page number is now %d", curr_page_num);
}

unsigned long oled_get_last_page_nav_timestamp()
{
    return last_page_nav_timestamp;
}

void oled_display_page_rx_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    lorawan_message_buf_t last_reception = lorawan_get_last_reception(), last_transmission = lorawan_get_transmission();
    unsigned long last_tx = (millis() - last_transmission.timestamp_millis) / 1000;
    String morse_signals = gp_button_get_latest_morse_signals(), morse_message = gp_button_get_morse_message_buf();
    if (last_transmission.timestamp_millis < 1)
    {
        last_tx = -1;
    }
    unsigned long last_rx = (millis() - last_reception.timestamp_millis) / 1000;
    if (last_reception.timestamp_millis < 1)
    {
        last_rx = -1;
    }
    if (last_transmission.timestamp_millis > 0)
    {
        if (morse_signals.length() == 0 && morse_message.length() == 0)
        {
            snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Last check in: %lus ago", last_tx);
        }
        else if (last_morse_input_page_num == OLED_PAGE_TX_COMMAND)
        {
            snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Transmit cmd: %lus ago", last_tx);
        }
        else if (last_morse_input_page_num == OLED_PAGE_TX_MESSAGE)
        {
            snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Transmit msg: %lus ago", last_tx);
        }
    }
    else
    {
        snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Last check in: ?s ago");
    }
    if (last_reception.timestamp_millis > 0)
    {
        char rx_info_display[OLED_MAX_LINE_LEN * 4 + 1] = {0};
        snprintf(rx_info_display, OLED_MAX_LINE_LEN * 4 + 1, "Received %lus ago: %s", last_rx, last_reception.buf);
        memcpy(lines[1], rx_info_display, OLED_MAX_LINE_LEN);
        memcpy(lines[2], &rx_info_display[OLED_MAX_LINE_LEN], OLED_MAX_LINE_LEN);
        memcpy(lines[3], &rx_info_display[OLED_MAX_LINE_LEN * 2], OLED_MAX_LINE_LEN);
        memcpy(lines[4], &rx_info_display[OLED_MAX_LINE_LEN * 3], OLED_MAX_LINE_LEN);
    }
    else
    {
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Nothing received yet.");
        snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Click PWR button to go");
        snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "to next page. Hold 4sec");
        snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "to shutdown.");
    }
}

void oled_display_page_tx_message(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    if (oled_get_last_page_nav_timestamp() < gp_button_get_last_click_timestamp() && last_morse_input_page_num != OLED_PAGE_TX_MESSAGE)
    {
        // Clear the message typed in the other page.
        gp_button_clear_morse_message_buf();
        last_morse_input_page_num = OLED_PAGE_TX_MESSAGE;
    }
    String morse_signals = gp_button_get_latest_morse_signals(), morse_message = gp_button_get_morse_message_buf();
    if ((morse_signals.length() == 0 && morse_message.length() == 0) || last_morse_input_page_num != OLED_PAGE_TX_MESSAGE)
    {
        snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Type a message in morse");
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "using the user button.");
        snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Hold 2sec to backspace.");
        snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Hold 4sec to clear.");
    }
    else
    {
        char tx_info_display[OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1] = {0};
        snprintf(tx_info_display, OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1, "TX text: %s %s", morse_message.c_str(), morse_signals.c_str());
        memcpy(lines[0], tx_info_display, OLED_MAX_LINE_LEN);
        memcpy(lines[1], &tx_info_display[OLED_MAX_LINE_LEN], OLED_MAX_LINE_LEN);
        memcpy(lines[2], &tx_info_display[OLED_MAX_LINE_LEN * 2], OLED_MAX_LINE_LEN);
        memcpy(lines[3], &tx_info_display[OLED_MAX_LINE_LEN * 3], OLED_MAX_LINE_LEN);
    }
}

void oled_display_page_tx_command(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    if (oled_get_last_page_nav_timestamp() < gp_button_get_last_click_timestamp() && last_morse_input_page_num != OLED_PAGE_TX_COMMAND)
    {
        // Clear the message typed in the other page.
        gp_button_clear_morse_message_buf();
        last_morse_input_page_num = OLED_PAGE_TX_COMMAND;
    }
    String morse_signals = gp_button_get_latest_morse_signals(), morse_message = gp_button_get_morse_message_buf();
    if ((morse_signals.length() == 0 && morse_message.length() == 0) || last_morse_input_page_num != OLED_PAGE_TX_COMMAND)
    {
        snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Type a command in morse");
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "using the user button.");
        snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Hold 2sec to backspace.");
        snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Hold 4sec to clear.");
    }
    else
    {
        char tx_info_display[OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1] = {0};
        snprintf(tx_info_display, OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1, "TX command: %s %s", morse_message.c_str(), morse_signals.c_str());
        memcpy(lines[0], tx_info_display, OLED_MAX_LINE_LEN);
        memcpy(lines[1], &tx_info_display[OLED_MAX_LINE_LEN], OLED_MAX_LINE_LEN);
        memcpy(lines[2], &tx_info_display[OLED_MAX_LINE_LEN * 2], OLED_MAX_LINE_LEN);
        memcpy(lines[3], &tx_info_display[OLED_MAX_LINE_LEN * 3], OLED_MAX_LINE_LEN);
    }
}

void oled_display_page_gps_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    if (millis() - last_gps_data_timestamp > 1000)
    {
        // Read the latest GPS data at most once each second.
        last_gps_data = gps_get_data();
        last_gps_data_timestamp = millis();
    }
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "GPS sats: %d HDOP: %f", last_gps_data.satellites, last_gps_data.hdop);
    snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "UTC%d-%02d-%02d %02d:%02d:%02d", last_gps_data.utc_year, last_gps_data.utc_month, last_gps_data.utc_day, last_gps_data.utc_hour, last_gps_data.utc_minute, last_gps_data.utc_second);
    if (!last_gps_data.valid_time)
    {
        snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "Acquiring time...");
    }
    if (last_gps_data.valid_pos)
    {
        double latitude = last_gps_data.latitude;
        char north_south = 'N';
        if (latitude < 0)
        {
            north_south = 'S';
            latitude = -latitude;
        }
        double longitude = last_gps_data.longitude;
        char east_west = 'E';
        if (longitude < 0)
        {
            east_west = 'W';
            longitude = -longitude;
        }
        // Convert coordinates from DD.DDDDDD to DD MM SS.
        double lat_d, lat_m, lat_s, throw_away;
        lat_m = modf(latitude, &lat_d) * 60;
        lat_s = modf(lat_m, &throw_away) * 60;
        snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "Lat: %c %dd %dm %ds", north_south, (int)lat_d, (int)lat_m, (int)lat_s);
        double lon_d, lon_m, lon_s;
        lon_m = modf(longitude, &lon_d) * 60;
        lon_s = modf(lon_m, &throw_away) * 60;
        snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Lon: %c %dd %dm %ds", east_west, (int)lon_d, (int)lon_m, (int)lon_s);
        snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Spd: %.0fkm/h Hdg: %.0f", last_gps_data.speed_kmh, last_gps_data.heading_deg);
        snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "Alt: %.0fm Age: %dsec ", last_gps_data.altitude_metre, last_gps_data.pos_age_sec);
    }
    else
    {
        snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Acquiring position...");
    }
}

void oled_display_page_env_sensor_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    struct env_data data = env_sensor_get_data();
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Ambient conditions");
    snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Temperature: %.2fc", data.temp_celcius);
    snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "Humidity: %.2f%%", data.humidity_pct);
    snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Pressure: %.2fhpa", data.pressure_hpa);
    snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Alt(rel.std): %.2fm", data.altitude_metre);
}

void oled_display_page_diagnosis(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Diagnosis info");
    snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Heap usage: %d/%dKB", (ESP.getHeapSize() - ESP.getFreeHeap()) / 1024, ESP.getHeapSize() / 1024);
    snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "Bat: %.3fv Charging: %s", float(power_get_battery_millivolt()) / 1000.0, power_is_batt_charging() ? "Y" : "N");
    snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "LoRa RSSI: %d SNR: %d", LMIC.rssi, LMIC.snr);
    snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Pkts: %d up %d dn", LMIC.seqnoUp, LMIC.seqnoDn);
    snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "GPS: read %luB", gps_get_chars_processed());
}

void oled_display_refresh()
{
    char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1];
    for (int i = 0; i < OLED_MAX_NUM_LINES; i++)
    {
        memset(lines[i], 0, OLED_MAX_LINE_LEN + 1);
    }
    switch (oled_get_page_number())
    {
    case OLED_PAGE_RX_INFO:
        oled_display_page_rx_info(lines);
        break;
    case OLED_PAGE_TX_MESSAGE:
        oled_display_page_tx_message(lines);
        break;
    case OLED_PAGE_TX_COMMAND:
        oled_display_page_tx_command(lines);
        break;
    case OLED_PAGE_GPS_INFO:
        oled_display_page_gps_info(lines);
        break;
    case OLED_PAGE_ENV_SENSOR_INFO:
        oled_display_page_env_sensor_info(lines);
        break;
    case OLED_PAGE_DIAGNOSIS:
        oled_display_page_diagnosis(lines);
        break;
    default:
        break;
    }
    oled.clear();
    for (int i = 0; i < OLED_MAX_NUM_LINES; i++)
    {
        oled_draw_string_line(i, lines[i]);
    }
    i2c_lock();
    oled.display();
    i2c_unlock();
}

void oled_task_loop(void *_)
{
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(OLED_TASK_LOOP_DELAY_MS));
        oled_display_refresh();
        esp_task_wdt_reset();
    }
}
