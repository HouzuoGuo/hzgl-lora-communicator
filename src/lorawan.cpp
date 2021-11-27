#include <Arduino.h>
#include <SPI.h>
#include <esp_task_wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include "data_packet.h"
#include "env_sensor.h"
#include "gp_button.h"
#include "gps.h"
#include "hardware_facts.h"
#include "i2c.h"
#include "lorawan.h"
#include "lorawan_creds.h"
#include "oled.h"
#include "power_management.h"

static const char LOG_TAG[] = __FILE__;

const lmic_pinmap lmic_pins = {
    .nss = SPI_NSS_GPIO,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST_GPIO,
    .dio = {LORA_DIO0_GPIO, LORA_DIO1_GPIO, LORA_DIO2_GPIO},
};

static lorawan_message_buf_t next_tx_message, last_rx_message;
static unsigned long last_transmision_timestamp = 0, tx_counter = 0;

// os_getArtEui is referenced by "engineUpdate" symbol defined by the "MCCI LoRaWAN LMIC" library.
void os_getArtEui(u1_t *buf) {}
// os_getDevEui is referenced by "engineUpdate" symbol defined by the "MCCI LoRaWAN LMIC" library.
void os_getDevEui(u1_t *buf) {}
// os_getDevKey is referenced by "engineUpdate" symbol defined by the "MCCI LoRaWAN LMIC" library.
void os_getDevKey(u1_t *buf) {}

void lorawan_handle_message(uint8_t message)
{
  switch (message)
  {
  case EV_JOINING:
    ESP_LOGI(LOG_TAG, "joining network");
    break;
  case EV_JOINED:
    ESP_LOGI(LOG_TAG, "joined network");
    break;
  case EV_JOIN_FAILED:
    ESP_LOGI(LOG_TAG, "failed to join network");
    break;
  case EV_REJOIN_FAILED:
    ESP_LOGI(LOG_TAG, "failed to rejoin network");
    break;
  case EV_RESET:
    ESP_LOGI(LOG_TAG, "reset network connection");
    break;
  case EV_LINK_DEAD:
    // This is only applicable when adaptive-data-rate (see LMIC_setAdrMode) is enabled.
    ESP_LOGI(LOG_TAG, "network link is dead");
    break;
  case LORAWAN_EV_ACK:
    ESP_LOGI(LOG_TAG, "my transmitted message was acknowledged");
    break;
  case LORAWAN_EV_IDLING_BEFORE_TXRX:
    ESP_LOGI(LOG_TAG, "idling before upcoming TX/RX");
    break;
  case LORAWAN_EV_QUEUED_FOR_TX:
    ESP_LOGI(LOG_TAG, "prepared a message for transmission");
    break;
  case EV_TXCOMPLETE:
    ESP_LOGI(LOG_TAG, "transmitted a message");
    break;
  case EV_RXCOMPLETE:
    ESP_LOGI(LOG_TAG, "received a message");
    break;
  case LORAWAN_EV_RESPONSE:
    // I've managed to receive a downlink message 41 bytes long maximum.
    ESP_LOGI(LOG_TAG, "received a downlink message %d bytes long", LMIC.dataLen);
    size_t data_len = LMIC.dataLen;
    if (data_len > 0)
    {
      if (data_len > LORAWAN_MAX_MESSAGE_LEN)
      {
        data_len = LORAWAN_MAX_MESSAGE_LEN;
      }
      for (uint8_t i = 0; i < data_len; i++)
      {
        last_rx_message.buf[i] = LMIC.frame[LMIC.dataBeg + i];
      }
      last_rx_message.len = data_len;
      last_rx_message.buf[last_rx_message.len] = 0;
      last_rx_message.timestamp_millis = millis();
    }
    break;
  }
}

// onEvent is referenced by MCCI LMIC library.
void onEvent(ev_t event)
{
  switch (event)
  {
  case EV_TXCOMPLETE:
    next_tx_message.timestamp_millis = millis();
    ESP_LOGI(LOG_TAG, "transmission completed");
    if (LMIC.txrxFlags & TXRX_ACK)
    {
      ESP_LOGI(LOG_TAG, "received an acknowledgement of my transmitted message");
      lorawan_handle_message(LORAWAN_EV_ACK);
    }
    if (LMIC.dataLen > 0)
    {
      ESP_LOGI(LOG_TAG, "received a downlink message");
      lorawan_handle_message(LORAWAN_EV_RESPONSE);
    }
    break;
  case EV_TXSTART:
    ESP_LOGI(LOG_TAG, "transmitting now");
    break;
  default:
    ESP_LOGI(LOG_TAG, "ignored unrecognised event %d", event);
    break;
  }
  lorawan_handle_message(event);
}

void lorawan_setup()
{
  SPI.begin(SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO, SPI_NSS_GPIO);
  memset(&last_rx_message, 0, sizeof(last_rx_message));
  memset(&next_tx_message, 0, sizeof(next_tx_message));

  // Initialise the library's internal states.
  os_init();
  LMIC_reset();

  // Prepare network keys for the library to use. They are defined in #include "lorawan_creds.h":
  // static const u1_t PROGMEM NWKSKEY[16] = {0x00, 0x00, ...};
  // static const u1_t PROGMEM APPSKEY[16] = {0x00, 0x00, ...};
  // static const u4_t DEVADDR = 0x00000000;
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);

  // The Things Stack Community Edition could potentially use all of these channels.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
  // Though I am unsure if The Things Stack Community Edition uses FSK.
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);
  // This is in the frequency plan - "Europe 863-870 MHz (SF9 for RX2 - recommended)".
  LMIC.dn2Dr = DR_SF9;

  // Do not ask gateways for a downlink message to check the connectivity.
  LMIC_setLinkCheckMode(0);
  // Set transmission power to the maximum allowed on TTGO T-Beam.
  LMIC_setDrTxpow(DR_SF8, 20);
  // Do not lower transmission power automatically. According to The Things Network this feature is tricky to use.
  LMIC_setAdrMode(0);
  // Set the "max clock error to compensate for" to 10%.
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  // The transmitter is activated by personalisation (i.e. static keys), so it has already "joined" the network.
  lorawan_handle_message(EV_JOINED);
  ESP_LOGI(LOG_TAG, "successfully initialised LoRaWAN");
}

void lorawan_set_next_transmission(uint8_t *buf, size_t len, int port)
{
  if (len > LORAWAN_MAX_MESSAGE_LEN)
  {
    len = LORAWAN_MAX_MESSAGE_LEN;
  }
  memcpy(next_tx_message.buf, buf, len);
  next_tx_message.len = len;
  next_tx_message.port = port;
}

lorawan_message_buf_t lorawan_get_last_reception()
{
  return last_rx_message;
}

lorawan_message_buf_t lorawan_get_transmission()
{
  return next_tx_message;
}

void lorawan_prepare_uplink_transmission()
{
  if (tx_counter++ % 2 == 0)
  {
    // Transmit the text message/command in each even round.
    String morse_message = gp_button_get_morse_message_buf();
    uint8_t buf[LORAWAN_MAX_MESSAGE_LEN] = {0};
    for (int i = 0; i < morse_message.length(); ++i)
    {
      buf[i] = (uint8_t)morse_message.charAt(i);
    }

    // Determine the type of the message according to which OLED page the input came from.
    int port = LORAWAN_PORT_MESSAGE;
    if (oled_get_last_morse_input_page_num() == OLED_PAGE_TX_COMMAND)
    {
      port = LORAWAN_PORT_COMMAND;
    }
    // Set the transmission buffer only if user is has finished typing a message.
    if (oled_get_page_number() != OLED_PAGE_TX_COMMAND && oled_get_page_number() != OLED_PAGE_TX_MESSAGE)
    {
      lorawan_set_next_transmission(buf, morse_message.length(), port);
      ESP_LOGI(LOG_TAG, "going to transmit message/command \"%s\"", morse_message.c_str());
    }
  }
  else
  {
    // Transmit sensor readings in each odd round.
    DataPacket pkt(LORAWAN_MAX_MESSAGE_LEN);
    // Byte 0, 1 - number of seconds since the reception of last downlink message (0 - 65535).
    lorawan_message_buf_t last_reception = lorawan_get_last_reception();
    unsigned long last_rx = (millis() - last_reception.timestamp_millis) / 1000;
    if (last_rx > 65536 || last_rx == 0)
    {
      last_rx = 65536;
    }
    pkt.writeInteger(last_rx, 2);
    // Byte 2, 3, 4, 5 - uptime in seconds.
    pkt.writeInteger(power_get_uptime_sec(), 4);
    // Byte 6, 7 - heap usage in KB.
    pkt.writeInteger((ESP.getHeapSize() - ESP.getFreeHeap()) / 1024, 2);
    // Byte 8, 9 - battery voltage in millivolts.
    pkt.writeInteger(power_get_battery_millivolt(), 2);
    // Byte 10 - is battery charging (0 - false, 1 - true).
    pkt.writeInteger(power_is_batt_charging() ? 1 : 0, 1);
    // Byte 11, 12, 13, 14 - ambient temperature in celcius.
    struct env_data env = env_sensor_get_data();
    pkt.write32BitDouble(env.temp_celcius);
    // Byte 15 - ambient humidity in percentage.
    pkt.writeInteger((int)env.humidity_pct, 1);
    // Byte 16, 17, 18, 19 - ambient pressure in hpa.
    pkt.write32BitDouble(env.pressure_hpa);
    // Byte 20, 21, 22, 23 - pressure altitude in meters.
    pkt.write32BitDouble(env.altitude_metre);
    // Byte 24, 25, 26, 27 - GPS latitude.
    struct gps_data gps = gps_get_data();
    pkt.write32BitDouble(gps.latitude);
    // Byte 28, 29, 30, 31 - GPS longitude.
    pkt.write32BitDouble(gps.longitude);
    // Byte 32, 33 - GPS speed in km/h.
    pkt.writeInteger((int)gps.speed_kmh, 2);
    // Byte 34, 35 - GPS heading in degrees.
    pkt.writeInteger((int)gps.heading_deg, 2);
    // Byte 36, 37, 38, 39 - GPS altitude in metres.
    pkt.write32BitDouble(gps.altitude_metre);
    // Byte 40, 41 - the age of last GPS fix in seconds (0 - 65536).
    if (gps.pos_age_sec > 65536)
    {
      gps.pos_age_sec = 65536;
    }
    pkt.writeInteger(gps.pos_age_sec, 2);
    // Byte 42 - HDOP in integer (0 - 256).
    int hdop = (int)gps.hdop;
    if (hdop > 256)
    {
      hdop = 256;
    }
    pkt.writeInteger(hdop, 1);
    // Byte 43 - number of GPS satellites in view.
    pkt.writeInteger(gps.satellites, 1);
    lorawan_set_next_transmission(pkt.content, pkt.cursor, LORAWAN_PORT_STATUS);
    ESP_LOGI(LOG_TAG, "going to transmit status and sensor readings in %d bytes", pkt.cursor);
  }
}

void lorawan_transceive()
{
  // Give the LoRaWAN library a chance to do its work.
  os_runloop_once();
  // Rate-limit transmission to observe duty cycle.
  if (last_transmision_timestamp == 0 || millis() - last_transmision_timestamp > LORAWAN_TRANSMISSION_INTERVAL_MS)
  {
    lorawan_prepare_uplink_transmission();
    last_transmision_timestamp = millis();
    LMIC_setTxData2(next_tx_message.port, next_tx_message.buf, next_tx_message.len, false);
    if (LMIC.opmode & OP_TXRXPEND)
    {
      // My gut feeling says this state exists simply for avoiding to exceed duty cycle limit.
      lorawan_handle_message(LORAWAN_EV_IDLING_BEFORE_TXRX);
      return;
    }
    lorawan_handle_message(LORAWAN_EV_QUEUED_FOR_TX);
  }
}

void lorawan_task_loop(void *_)
{
  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(LORAWAN_TASK_LOOP_DELAY_MS));
    lorawan_transceive();
    esp_task_wdt_reset();
  }
}
