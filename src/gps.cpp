#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <esp_task_wdt.h>
#include "gps.h"
#include "hardware_facts.h"

static const char LOG_TAG[] = __FILE__;

static unsigned long num_decoded_bytes = 0;
static HardwareSerial gps_serial(1);
static TinyGPSPlus gps;
// $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx (http://aprs.gids.nl/nmea/#zda).
static TinyGPSCustom gps_time_field(gps, "GPZDA", 1);
static TinyGPSCustom gps_day_field(gps, "GPZDA", 2);
static TinyGPSCustom gps_month_field(gps, "GPZDA", 3);
static TinyGPSCustom gps_year_field(gps, "GPZDA", 4);

// https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
// GPQ - Poll message - Supported on u-blox 6 from firmware version 6.00 up to version 7.03. Polls a standard NMEA message.
static const String poll_date_time_request = "$EIGPQ,ZDA*39\r\n";

void gps_setup()
{
    // Be aware of the reversed polarity noted on seller's pinout diagram.
    gps_serial.begin(9600, SERIAL_8N1, GPS_SERIAL_TX, GPS_SERIAL_RX);
    ESP_LOGI(LOG_TAG, "successfully initialised GPS");
}

void gps_read_decode()
{
    while (gps_serial.available())
    {
        int b = gps_serial.read();
        gps.encode(b);
        ++num_decoded_bytes;
    }
    // Explicitly ask the GPS chip for date & time output, which will be read when this function is invoked again.
    gps_serial.print(poll_date_time_request);
}

struct gps_data gps_get_data()
{
    struct gps_data ret;
    memset(&ret, 0, sizeof(ret));
    if (gps.charsProcessed() < 20)
    {
        // GPS is not ready.
        return ret;
    }
    ret.satellites = (int)gps.satellites.value();
    ret.hdop = gps.hdop.hdop();
    ret.valid_pos = gps.location.isValid();
    ret.pos_age_sec = gps.location.age() / 1000;
    if (ret.pos_age_sec > 999)
    {
        ret.pos_age_sec = 999;
    }
    // gps.time.isValid() appears to always return true even when there is no GPS reception.
    ret.valid_time = gps.date.isValid() || ret.valid_pos;
    if (gps_year_field.isValid())
    {
        ret.utc_year = atoi(gps_year_field.value());
        ret.utc_month = atoi(gps_month_field.value());
        ret.utc_day = atoi(gps_day_field.value());
    }
    if (ret.valid_time)
    {
        ret.utc_hour = gps.time.hour();
        ret.utc_minute = gps.time.minute();
        ret.utc_second = gps.time.second();
    }
    if (ret.valid_pos && ret.hdop < 50 && ret.pos_age_sec < 120)
    {
        // Apparently useless & stale position readings could be considered valid too.
        ret.latitude = gps.location.lat();
        ret.longitude = gps.location.lng();
        ret.altitude_metre = gps.altitude.meters();
        ret.heading_deg = gps.course.deg();
        ret.speed_kmh = gps.speed.kmph();
    }
    ESP_LOGI(LOG_TAG, "valid time? %d, valid position? %d, hdop %f", ret.valid_time, ret.valid_pos, ret.hdop);
    return ret;
}

void gps_task_loop(void *_)
{
    while (true)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_LOOP_DELAY_MS));
        gps_read_decode();
    }
}

unsigned long gps_get_chars_processed()
{
    return gps.charsProcessed();
}
