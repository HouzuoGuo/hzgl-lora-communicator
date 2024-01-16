#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <esp_task_wdt.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "gps.h"
#include "oled.h"
#include "hardware_facts.h"
#include "power_management.h"

static const char LOG_TAG[] = __FILE__;

static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
static bool is_powered_on = false, is_initialised = false;
static unsigned long num_decoded_bytes = 0;
static HardwareSerial gps_serial(1);
// gps interprets NMEA output.
static TinyGPSPlus gps;
// ublox configures the GPS chip.
static SFE_UBLOX_GNSS ublox;

// $--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx (http://aprs.gids.nl/nmea/#zda).
static TinyGPSCustom gps_time_field(gps, "GPZDA", 1);
static TinyGPSCustom gps_day_field(gps, "GPZDA", 2);
static TinyGPSCustom gps_month_field(gps, "GPZDA", 3);
static TinyGPSCustom gps_year_field(gps, "GPZDA", 4);

void gps_on()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (is_powered_on)
    {
        xSemaphoreGive(mutex);
        return;
    }
    if (!is_initialised)
    {
        gps_serial.begin(9600, SERIAL_8N1, GPS_SERIAL_TX, GPS_SERIAL_RX);
        for (int i = 0; i < 30; i++)
        {
            if (ublox.begin(gps_serial))
            {
                for (int j = 0; j < 3; j++)
                {
                    bool success = true;
                    // Change serial output content type to NMEA.
                    bool nmeaOut = ublox.setUART1Output(COM_TYPE_NMEA);
                    success = success & nmeaOut;
                    ESP_LOGI(LOG_TAG, "demand NMEA output: %d", nmeaOut);
                    // Disable unused NMEA sentences.
                    ESP_LOGI(LOG_TAG, "disable GLL: %d, disable GSA: %d, disable GSV: %d, disable VTG: %d",
                             ublox.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1),
                             ublox.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1),
                             ublox.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1),
                             ublox.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1));
                    // GGA - position fix, ZDA - date and time, RMC - position + course + speed.
                    ESP_LOGI(LOG_TAG, "enable GGA: %d, enable ZDA: %d, enable RMC: %d",
                             ublox.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1),
                             ublox.enableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1),
                             ublox.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1));
                    bool save = ublox.saveConfiguration(5000);
                    success = success & save;
                    ESP_LOGI(LOG_TAG, "save config: %d", save);
                    if (success)
                    {
                        break;
                    }
                }
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        is_initialised = true;
    }
    // According to the ublox library, sending an info query wakes the GPS up.
    ESP_LOGI(LOG_TAG, "turning on GPS - time %d:%d lat %d long %d", ublox.getMinute(), ublox.getSecond(), ublox.getLatitude(), ublox.getLongitude());
    is_powered_on = true;
    xSemaphoreGive(mutex);
}

bool gps_get_state()
{
    return is_powered_on;
}

void gps_off()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (!is_powered_on)
    {
        xSemaphoreGive(mutex);
        return;
    }
    // The off duration does not really matter as gps_on wakes the GPS up when needed.
    ESP_LOGI(LOG_TAG, "turning off GPS - cmd result: %d", ublox.powerOff(3600 * 1000));
    ublox.end();
    is_powered_on = false;
    xSemaphoreGive(mutex);
}

void gps_read_decode()
{
    while (gps_serial.available())
    {
        int b = gps_serial.read();
        gps.encode(b);
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
    ret.valid_pos = gps.location.isValid();
    ret.pos_age_sec = gps.location.age() / 1000;
    if (ret.pos_age_sec > 600)
    {
        ret.pos_age_sec = 600;
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
    if (ret.valid_pos && ret.hdop < 50 && ret.pos_age_sec < 600)
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
        if ((power_get_todo() & POWER_TODO_TURN_ON_GPS) || (oled_get_state() && oled_get_page_number() == OLED_PAGE_GPS_INFO))
        {
            gps_on();
            gps_read_decode();
        }
        else
        {
            gps_off();
        }
        vTaskDelay(pdMS_TO_TICKS(GPS_TASK_LOOP_DELAY_MS));
    }
}

unsigned long gps_get_chars_processed()
{
    return gps.charsProcessed();
}
