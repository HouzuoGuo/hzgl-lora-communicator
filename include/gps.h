#pragma once

// GPS_TASK_LOOP_DELAY_MS is the sleep interval of the GPS receiver task loop.
#define GPS_TASK_LOOP_DELAY_MS 1000

// gps_data describes the coordinates and clock time read from GPS.
struct gps_data
{
    double latitude, longitude, altitude_metre, hdop, speed_kmh, heading_deg;
    int satellites, pos_age_sec;
    bool valid_pos;

    int unix_time, utc_year, utc_month, utc_day, utc_hour, utc_minute, utc_second;
    bool valid_time;
};

// gps_get_data returns the latest coordinates and clock time read from GPS.
struct gps_data gps_get_data();

void gps_setup();
void gps_on();
void gps_off();
void gps_read_decode();
void gps_task_loop(void *_);
unsigned long gps_get_chars_processed();