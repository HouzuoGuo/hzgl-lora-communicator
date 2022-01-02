#include <esp_task_wdt.h>
#include "env_sensor.h"
#include "gp_button.h"
#include "gps.h"
#include "hardware_facts.h"
#include "i2c.h"
#include "lorawan.h"
#include "oled.h"
#include "wifi.h"
#include "bluetooth.h"
#include "power_management.h"

static const char LOG_TAG[] = __FILE__;

static int curr_page_num = 0, last_morse_input_page_num = 0;
static unsigned long last_page_nav_timestamp = 0, last_gps_data_timestamp = 0;
static struct gps_data last_gps_data;
static bool is_oled_on = false;
static unsigned long last_input_timestamp = 0;

static SSD1306Wire oled(OLED_I2C_ADDR, I2C_SDA, I2C_SCL);

void oled_setup()
{
    i2c_lock();
    oled.init();
    i2c_unlock();
    oled_on();
    last_input_timestamp = millis();
    ESP_LOGI(LOG_TAG, "successfully initialised OLED");
}

bool oled_reset_last_input_timestamp()
{
    bool ret = millis() - last_input_timestamp > OLED_SLEEP_AFTER_INACTIVE_MS;
    last_input_timestamp = millis();
    return ret;
}

bool oled_is_awake()
{
    return is_oled_on;
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
    if (++curr_page_num == 9)
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
        if ((morse_signals.length() == 0 && morse_message.length() == 0) || last_morse_input_page_num == 0)
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
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "using the func button.");
        snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "Hold the button to:");
        snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "1sec - backspace");
        snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "2sec - switch case a/A");
        snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "3sec - clear");
    }
    else
    {
        char tx_info_display[OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1] = {0};
        snprintf(tx_info_display, OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1, "TX text: %s %s", morse_message.c_str(), morse_signals.c_str());
        memcpy(lines[0], tx_info_display, OLED_MAX_LINE_LEN);
        memcpy(lines[1], &tx_info_display[OLED_MAX_LINE_LEN], OLED_MAX_LINE_LEN);
        memcpy(lines[2], &tx_info_display[OLED_MAX_LINE_LEN * 2], OLED_MAX_LINE_LEN);
        memcpy(lines[3], &tx_info_display[OLED_MAX_LINE_LEN * 3], OLED_MAX_LINE_LEN);
        // Line 4 is intentionally left blank.
        snprintf(lines[5], OLED_MAX_LINE_LEN, "%c|%s", gp_button_is_input_lower_case() ? 'a' : 'A', gp_button_get_edit_hint().c_str());
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
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "using the func button.");
        snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "Hold the button to:");
        snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "1sec - backspace");
        snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "2sec - switch case a/A");
        snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "3sec - clear");
    }
    else
    {
        char tx_info_display[OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1] = {0};
        snprintf(tx_info_display, OLED_MAX_LINE_LEN * OLED_MAX_NUM_LINES + 1, "TX command: %s %s", morse_message.c_str(), morse_signals.c_str());
        memcpy(lines[0], tx_info_display, OLED_MAX_LINE_LEN);
        memcpy(lines[1], &tx_info_display[OLED_MAX_LINE_LEN], OLED_MAX_LINE_LEN);
        memcpy(lines[2], &tx_info_display[OLED_MAX_LINE_LEN * 2], OLED_MAX_LINE_LEN);
        memcpy(lines[3], &tx_info_display[OLED_MAX_LINE_LEN * 3], OLED_MAX_LINE_LEN);
        // Line 4 is intentionally left blank.
        snprintf(lines[5], OLED_MAX_LINE_LEN, "%c|%s", gp_button_is_input_lower_case() ? 'a' : 'A', gp_button_get_edit_hint().c_str());
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

void oled_display_page_env_wifi_sniffer_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "WiFi monitor chan#%d", wifi_get_channel_num());
    snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Loudest sender chan#%d:", wifi_get_last_loudest_sender_channel());
    uint8_t *loudest_sender_mac = wifi_get_last_loudest_sender_mac();
    snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "%02x:%02x:%02x:%02x:%02x:%02x", loudest_sender_mac[0], loudest_sender_mac[1], loudest_sender_mac[2], loudest_sender_mac[3], loudest_sender_mac[4], loudest_sender_mac[5]);
    snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Loudest RSSI: %d", wifi_get_last_loudest_sender_rssi());
    snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Pkts (all.chan.): %d", wifi_get_total_num_pkts());
    snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "Scan round: %lu", wifi_get_round_num());
}

void oled_display_page_env_bt_sniffer_info(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Bluetooth monitor");
    BLEAdvertisedDevice dev = bluetooth_get_loudest_sender();
    snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Loudest RSSI: %d", dev.getRSSI());
    snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "MAC: %s", dev.getAddress().toString().c_str());
    snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Name: %s", dev.getName().c_str());
    snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Num.devices: %d", bluetooth_get_total_num_devices());
    snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "Scan round: %lu", bluetooth_get_round_num());
}

void oled_display_page_lora(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "LoRaWAN DR#%d Ch#%d", LMIC.datarate, LMIC.txChnl);
    power_config_t conf = power_get_config();
    int sf_reading = 0;
    switch (conf.spreading_factor)
    {
    case DR_SF7:
        sf_reading = 7;
        break;
    case DR_SF9:
        sf_reading = 9;
        break;
    }
    snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "TX power: %ddBm SF: %d", conf.power_dbm, sf_reading);
    snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "TX interval: %d sec", conf.tx_internal_sec);
    snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Power mode: %s", conf.mode_name.c_str());
    snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Click the user button");
    snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "to change power mode.");
}

void oled_display_page_diagnosis(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "Heap usage: %d/%dKB", (ESP.getHeapSize() - ESP.getFreeHeap()) / 1024, ESP.getHeapSize() / 1024);
    struct power_status power = power_get_status();
    if (power.is_batt_charging || !power.is_usb_power_available)
    {
        // Battery is installed and it is charging/discharging.
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Batt: %.3fv %+.0fmA", float(power.batt_millivolt) / 1000.0, power.batt_milliamp);
    }
    else if (power.batt_millivolt < 100)
    {
        // Battery is not installed.
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "USB: %.3fv %.0fmA", float(power.usb_millivolt) / 1000.0, -power.power_draw_milliamp);
    }
    else
    {
        // Battery is installed and it is neither charging nor discharging.
        snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "Batt: %.3fv USB%.0fmA", float(power.batt_millivolt) / 1000.0, -power.power_draw_milliamp);
    }
    snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "LoRa: RSSI %d SNR %d", LMIC.rssi, LMIC.snr);
    snprintf(lines[3], OLED_MAX_LINE_LEN + 1, "Pkts: %d up %d dn", LMIC.seqnoUp, LMIC.seqnoDn);
    snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Data: %dB up %dB dn", lorawan_get_total_tx_bytes(), lorawan_get_total_rx_bytes());
    snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "GPS: read %luB", gps_get_chars_processed());
}

void oled_display_going_to_sleep(char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1])
{
    snprintf(lines[0], OLED_MAX_LINE_LEN + 1, "The screen is going to");
    snprintf(lines[1], OLED_MAX_LINE_LEN + 1, "sleep soon.");
    snprintf(lines[2], OLED_MAX_LINE_LEN + 1, "TX will continue.");
    snprintf(lines[4], OLED_MAX_LINE_LEN + 1, "Click PWR button to");
    snprintf(lines[5], OLED_MAX_LINE_LEN + 1, "wake screen up.");
}

unsigned int oled_get_ms_since_last_input()
{
    return millis() - last_input_timestamp;
}

void oled_on()
{
    if (is_oled_on)
    {
        return;
    }
    ESP_LOGI(LOG_TAG, "turning on OLED");
    i2c_lock();
    oled.displayOn();
    oled.clear();
    oled.setBrightness(64);
    oled.setContrast(0xF1, 128, 0x40);
    oled.resetOrientation();
    oled.flipScreenVertically();
    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.setFont(ArialMT_Plain_10);
    i2c_unlock();
    is_oled_on = true;
}

void oled_off()
{
    if (!is_oled_on)
    {
        return;
    }
    ESP_LOGI(LOG_TAG, "turning off OLED");
    i2c_lock();
    oled.clear();
    oled.setBrightness(0);
    oled.displayOff();
    i2c_unlock();
    is_oled_on = false;
}

void oled_display_refresh()
{
    // Conserve power when power management is in the saver mode.
    if (power_get_config().mode_id == POWER_SAVER)
    {
        if (oled_get_ms_since_last_input() < OLED_SLEEP_AFTER_INACTIVE_MS && !is_oled_on)
        {
            oled_on();
            power_led_off();
        }
        else if (oled_get_ms_since_last_input() > OLED_SLEEP_AFTER_INACTIVE_MS && is_oled_on)
        {
            oled_off();
            power_led_blink();
        }
    }
    if (is_oled_on)
    {
        char lines[OLED_MAX_NUM_LINES][OLED_MAX_LINE_LEN + 1];
        for (int i = 0; i < OLED_MAX_NUM_LINES; i++)
        {
            memset(lines[i], 0, OLED_MAX_LINE_LEN + 1);
        }
        if (power_get_config().mode_id == POWER_SAVER &&
            (oled_get_ms_since_last_input() > (OLED_SLEEP_AFTER_INACTIVE_MS - OLED_SLEEP_REMINDER_DURATION_MS) &&
             oled_get_ms_since_last_input() < OLED_SLEEP_AFTER_INACTIVE_MS))
        {
            // Reminder the user for the brief period before the screen goes to sleep.
            oled_display_going_to_sleep(lines);
        }
        else
        {
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
            case OLED_PAGE_WIFI_INFO:
                oled_display_page_env_wifi_sniffer_info(lines);
                break;
            case OLED_PAGE_BT_INFO:
                oled_display_page_env_bt_sniffer_info(lines);
                break;
            case OLED_PAGE_LORAWAN:
                oled_display_page_lora(lines);
                break;
            case OLED_PAGE_DIAGNOSIS:
                oled_display_page_diagnosis(lines);
                break;
            default:
                break;
            }
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
}

void oled_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(OLED_TASK_LOOP_DELAY_MS));
        oled_display_refresh();
    }
}
