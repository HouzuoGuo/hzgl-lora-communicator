#pragma once

#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include <stdint.h>

#define WIFI_TASK_LOOP_DELAY_MS 200
#define WIFI_MAX_CHANNEL_NUM 13
#define WIFI_RSSI_FLOOR -120

typedef struct
{
    unsigned frame_ctrl : 16;
    unsigned duration_id : 16;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    unsigned sequence_ctrl : 16;
    uint8_t addr4[6];
} wifi_ieee80211_mac_hdr_t;

typedef struct
{
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0];
} wifi_ieee80211_packet_t;

void wifi_on();
void wifi_off();
bool wifi_get_state();
void wifi_task_loop(void *_);
void wifi_next_channel();
size_t wifi_get_total_num_pkts();
size_t wifi_get_total_pkt_data_len();
size_t wifi_get_channel_num();
unsigned long wifi_get_round_num();
void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);
int wifi_get_last_loudest_sender_rssi();
uint8_t *wifi_get_last_loudest_sender_mac();
size_t wifi_get_last_loudest_sender_channel();