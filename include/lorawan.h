#pragma once

#include <lmic.h>

// LORAWAN_EV_QUEUED_FOR_TX is a magic event number unrelated to LoRaWAN standard, it is exclusively used by this program's LoRaWAN event loop.
#define LORAWAN_EV_QUEUED_FOR_TX 100
// LORAWAN_EV_IDLING_BEFORE_TXRX is a magic event number unrelated to LoRaWAN standard, it is exclusively used by this program's LoRaWAN event loop.
// Acording to LMIC library, when LMIC.opmode contains the OP_TXRXPEND bit, there is a "TX/RX transaction pending".
#define LORAWAN_EV_IDLING_BEFORE_TXRX 101
// LORAWAN_EV_ACK is a magic event number unrelated to LoRaWAN standard, it is exclusively used by this program's LoRaWAN event loop.
#define LORAWAN_EV_ACK 102
// LORAWAN_EV_RESPONSE is a magic event number unrelated to LoRaWAN standard, it is exclusively used by this program's LoRaWAN event loop.
#define LORAWAN_EV_RESPONSE 103
// LORAWAN_TASK_LOOP_DELAY_MS is the maximum sleep interval of the LoRaWAN transceiver task loop.
// The actual interval will be determined at runtime depending on the deadline of LMIC internal tasks.
#define LORAWAN_TASK_LOOP_DELAY_MS 256

// LORAWAN_PORT_COMMAND is the numeric port number used for transmitting uplink toolbox command messages.
#define LORAWAN_PORT_COMMAND 112
// LORAWAN_PORT_MESSAGE is the numeric port number used for transmitting uplink text messages.
#define LORAWAN_PORT_MESSAGE 129
// LORAWAN_PORT_STATUS_SENSOR is the numeric port number used for transmitting system status and sensor readings.
#define LORAWAN_PORT_STATUS_SENSOR 119
// LORAWAN_PORT_STATUS_SENSOR is the numeric port number used for transmitting GPS location and wifi foxhunt info.
#define LORAWAN_PORT_GPS_WIFI 120
// LORAWAN_TX_INTERVAL_MS is the interval to wait in between two routine uplink transmissions.
#define LORAWAN_TX_INTERVAL_MS 20000

#define LORAWAN_WARM_UP_MS (5 * 1000)

// LORAWAN_MAX_MESSAGE_LEN is the length never exceeded by a message received from or transmitted to The Things Network.
static const size_t LORAWAN_MAX_MESSAGE_LEN = 256;

// lorawan_message_buf_t represents a message received from or to be transmitted on The Things Network.
typedef struct
{
    uint8_t buf[LORAWAN_MAX_MESSAGE_LEN + 1];
    size_t len;
    unsigned long timestamp_millis;
    int port;
} lorawan_message_buf_t;

// lorawan_setup initialises LoRaWAN library and prepares it for transmission/receiving operations.
void lorawan_setup();
// lorawan_task_loop transmits the last message set repeatedly at regular interval and receives downlink messages.
// The function blocks caller indefinitely.
void lorawan_task_loop(void *);
// lorawan_set_next_transmission sets the message to be transmitted repeatedly at regular intervals.
void lorawan_set_next_transmission(uint8_t *buf, size_t len, int port);
// lorawan_get_last_reception returns the last received downlink message.
lorawan_message_buf_t lorawan_get_last_reception();
void lorawan_prepare_uplink_transmission();
// lorawan_get_transmission returns the message previously set for transmission.
// If the transmission has already occurred, the returned value will contain the transmission's timestamp.
lorawan_message_buf_t lorawan_get_transmission();

void lorawan_reset();
size_t lorawan_get_total_rx_bytes();
size_t lorawan_get_total_tx_bytes();
unsigned long lorawan_get_tx_counter();
void lorawan_transceive();
void lorawan_debug_to_log();
void lorawan_reset_tx_stats();
bool lorawan_is_warming_up();