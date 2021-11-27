#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <esp_task_wdt.h>
#include "gps.h"
#include "hardware_facts.h"

static const char LOG_TAG[] = __FILE__;

static TinyGPSPlus gps;
static unsigned long num_decoded_bytes = 0;

void gps_setup()
{
    // Be aware of the reversed polarity noted on seller's pinout diagram.
    Serial1.begin(9600, SERIAL_8N1, GPS_SERIAL_TX, GPS_SERIAL_RX);
    ESP_LOGI(LOG_TAG, "successfully initialised GPS");
}

void gps_read_decode()
{
    while (Serial1.available())
    {
        gps.encode(Serial1.read());
        ++num_decoded_bytes;
    }
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
    // gps.time.isValid() appears to always return true even when there is no GPS reception.
    ret.valid_time = gps.date.isValid() || gps.location.isValid();
    if (ret.valid_time)
    {
        ret.utc_year = gps.date.year();
        ret.utc_month = gps.date.month();
        ret.utc_day = gps.date.day();
        ret.utc_hour = gps.time.hour();
        ret.utc_minute = gps.time.minute();
        ret.utc_second = gps.time.second();
    }
    ret.valid_pos = gps.location.isValid();
    if (ret.valid_pos)
    {
        ret.latitude = gps.location.lat();
        ret.longitude = gps.location.lng();
        ret.altitude_metre = gps.altitude.meters();
        ret.pos_age_sec = gps.location.age() / 1000;
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
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_LOOP_DELAY_MS));
        gps_read_decode();
        esp_task_wdt_reset();
    }
}

unsigned long gps_get_chars_processed()
{
    return gps.charsProcessed();
}
