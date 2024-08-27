/*ResPECT - Transactional networking stack development.
ESP32 - Firmware for transmitting a UDP packet bypassing the esp_netif()
library. Depends on only FreeRTOS and the standard ESP32 libraries 
*/

// Standard libraries
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

// WiFi driver library
#include "esp_private/wifi.h"
#include "esp_wifi.h"

// Event library
#include "esp_event.h"

// ESP error definitions
#include "esp_err.h"
#include "esp_log.h"

// Defining connection parameters
// #define WIFI_SSID "Espressif"
// #define WIFI_PASS ""
#define WIFI_SSID "ResPECT"
#define WIFI_PASS "twoflower"

#define SCAN_AUTH_MODE_THRESHOLD ESP_WIFI_AUTH_WPA2_PSK // Not used since it is default
#define MAX_RETRY 5

// Register definitions
#define WIFI_TX_CONFIG 0x60033d04
#define WIFI_MAC_CTRL 0x60033ca0
#define WIFI_TX_PLCP0 0x60033d08
#define WIFI_TX_PLCP1 0x600342f8
#define WIFI_TX_PLCP1_2 0x600342fc
#define WIFI_TX_PLCP2 0x60034314
#define WIFI_TX_DURATION 0x60034318

#define SEND_RAW

// typedef uint8_t macaddr_t[6];

// // Struct for 80211 packet format
// typedef struct mac80211_frame {
//     struct mac80211_frame_control {
//         unsigned    protocol_version    : 2;
//         unsigned    type            : 2;
//         unsigned    sub_type        : 4;
//         unsigned    to_ds           : 1;
//         unsigned    from_ds         : 1;
//         unsigned    _flags:6;
//     } __attribute__((packed)) frame_control;
//     uint16_t  duration_id;
//     macaddr_t receiver_address;
//     macaddr_t transmitter_address;
//     macaddr_t address_3;
//     struct mac80211_sequence_control {
//         unsigned    fragment_number     : 4;
//         unsigned    sequence_number     : 12;
//     } __attribute__((packed));
//     uint8_t data_and_fcs[2316];
// }  __attribute__((packed)) mac80211_frame;

#ifdef SEND_RAW
typedef struct dma_list_item {
	uint16_t size : 12;
	uint16_t length : 12;
	uint8_t _unknown : 6;
	uint8_t has_data : 1;
	uint8_t owner : 1; // What does this mean?
	void* packet;
	struct dma_list_item* next;
} __attribute__((packed,aligned(4))) dma_list_item_t;

extern bool pp_post(uint32_t requestnum, uint32_t argument);
#endif

esp_err_t ret;

uint8_t s_retry_num = 0;

// // // Hand crafted UDP packet
#ifndef SEND_RAW
const uint8_t udp_packet[] = {
    // MAC layer
    0xc8, 0x15, 0x4e, 0xd4, 0x65, 0x1b,
    0x84, 0xf7, 0x03, 0x60, 0x81, 0x5c, // Source MAC address
    0x08, 0x00, // protocol type - IPV4
    
    // IPv4 header
    0x45, //version :4 (obv) with IHL = 5
    0x00, // DSCP and ECN
    0x00, 0x21, // Total length (33 bytes), IPv4 Header + UDP Header + "Hello"
    0x00, 0x00, // Identification and fragmentation data
    0x00, 0x00, // Flags and fragment offset
    0x05, // TTL
    0x11, // Protocol type UDP 
    0x33, 0x55, // Header checksum (update after calculating length of total packet)
    0xc0, 0xa8, 0x00, 0x7A, // Source IP address - Arbitrary IP address for the ESP-32
    0xc0, 0xa8, 0x00, 0xb7, // Destination IP address - Laptop's IP address (assigned through DHCP by the router)

    // UDP header - 8 bytes
    0x1f, 0x45, // Source port - Both source and destination ports are random since it is UDP
    0xf0, 0xf0, // Destination port
    0x00, 0x0d, // Length
    0x00, 0x00,  // Checksum - calculate using the pseudo ipv4 header
    'h', 'e', 'l', 'l', 'o' // The message :)

};
#else
// MAC 802.11 frame captured from the blobs for the above UDP datagram
uint8_t packet[] = {
    0x88, 0x41, 0x2c, 0x00, 0x40, 0x9b, 0xcd, 0x25, 0x2a, 0xf8, 0x84, 0xf7,
    0x03, 0x60, 0x81, 0x5c, 0xc8, 0x15, 0x4e, 0xd4, 0x65, 0x1b, 0x00, 0x00, 
    0x00, 0x00, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0xbe, 0x00, 
    0x78, 0x8b, 0xa8, 0x7c, 0x41, 0x7b, 0xa3, 0xe1, 0x0b, 0xb4, 0xa0, 0xf5, 
    0x59, 0x08, 0x6b, 0x71, 0x44, 0x8e, 0x17, 0xd0, 0x58, 0x70, 0x60, 0x0a,
    0x7c, 0x7e, 0xed, 0xf8, 0xc2, 0x8b, 0x8d, 0x32, 0x1d, 0x8b, 0x77, 0xb3, 
    0x61, 0xcd, 0xc7, 0x1d, 0x75, 0x13, 0x13, 0x08, 0xcc, 0x83, 0xfb, 0xd4, 
    0x7f, 0x73, 0xde
};

dma_list_item_t tx_item = {
    .size = 83,
    .length = 95,
    ._unknown = 32,
    .has_data = 1,
    .owner = 1,
    .packet = &packet[0],
    .next = NULL
};
#endif

// Event handlers for connecting to the wifi. If event notifies that station is started, perform connection
// esp_wifi_connect() is part of the blobs
void event_handler(void *arg, esp_event_base_t event_base,
                    int32_t event_id, void* event_data)
{
    // Check the event base for a WiFi event. If device disconnects, retry until MAX_RETRY is exceeded
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
        ESP_ERROR_CHECK(esp_wifi_connect());
        printf("event_handler(): Connected...\n");
    }else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
        if (s_retry_num < MAX_RETRY){
            printf("Retrying...\n");
            ESP_ERROR_CHECK(esp_wifi_connect());
            s_retry_num++;
        }
    }
}


void app_main(){

    // Get default configuration for the wifi drivers. See esp_wifi.h for the default config. The config also 
    // describes the esp32c3 specific OS adapter functions. 
    // NVS storage of wifi parameters was disabled in the SDKConfig menu.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;

    // initialize WiFi. This may have some hardware settings. Part of the .a files
    ret = esp_wifi_init(&cfg);
    if(ret != ESP_OK){
        printf("Wifi could not be initialized (0x%x)\n", ret);
    }

    // Create the event loop to monitor the wifi events such as connecting and station
    // Declare the handlers for connecting to event.
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

    // // Create WiFi configuration with defined SSID and PASSWORD
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    // Set to station mode. Set the WiFi configuration. Provide wifi interface handle (WIFI_IF_STA)
    // Start the esp WiFi connection. All are called directly from the blobs
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    #ifdef SEND_RAW
    ESP_LOGW("main", "Killing proprietary wifi task (ppTask)");
	pp_post(0xf, 0);


    // // ic_mac_init decompilation
    uint32_t mac_val = REG_READ(WIFI_MAC_CTRL);
    REG_WRITE(WIFI_MAC_CTRL, mac_val &  0xff00efff);

    #endif 

    while(1){
        ESP_LOGI("Main:", "Trying to send");

#ifdef SEND_RAW
        // Write 0xa to the last byte (why? - to enable tx?)
        uint32_t cfg_val = REG_READ(WIFI_TX_CONFIG);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_CONFIG, (unsigned int)REG_READ(WIFI_TX_CONFIG));
        REG_WRITE(WIFI_TX_CONFIG, cfg_val | 0xa);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_CONFIG, (unsigned int)REG_READ(WIFI_TX_CONFIG));

        // Write the address of DMA struct to PLCP0 and set the 6th byte to 6 (why?) and 7th byte to 1 (why?)
        REG_WRITE(WIFI_TX_PLCP0, ((uint32_t)&tx_item & 0xfffff) | 0x600000);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_PLCP0, (unsigned int)REG_READ(WIFI_TX_PLCP0));

        // Copied from Qemu output. Something to do with length of the packet
        uint32_t rate = WIFI_PHY_RATE_54M;
        uint32_t is_n_enabled = (rate >= 16);
        REG_WRITE(WIFI_TX_PLCP1, 0x10000000 | (0x5f & 0xfff) | ((rate & 0x1f) << 12) | ((is_n_enabled & 0b1) << 25));
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_PLCP1, (unsigned int)REG_READ(WIFI_TX_PLCP1));

        REG_WRITE(WIFI_TX_PLCP1_2, 0x00);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_PLCP1_2, (unsigned int)REG_READ(WIFI_TX_PLCP1_2));

        // Write 0x00000020 (why?)
        REG_WRITE(WIFI_TX_PLCP2, 0x00000020);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_PLCP2, (unsigned int)REG_READ(WIFI_TX_PLCP2));
        
        // Duration to 0 (Why?)
        REG_WRITE(WIFI_TX_DURATION, 0x00);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_DURATION, (unsigned int)REG_READ(WIFI_TX_DURATION));

        // Setting some more unknown configurations
        cfg_val = REG_READ(WIFI_TX_CONFIG);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_CONFIG, (unsigned int)REG_READ(WIFI_TX_CONFIG));
        cfg_val = cfg_val | 0x02000000;
        cfg_val = cfg_val | 0x00003000;
        REG_WRITE(WIFI_TX_CONFIG, cfg_val);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_CONFIG, (unsigned int)REG_READ(WIFI_TX_CONFIG));

        // Finally enable tx
        printf("Enabling TX...\n");
        cfg_val = REG_READ(WIFI_TX_PLCP0);
        REG_WRITE(WIFI_TX_PLCP0, cfg_val | 0xc0000000);
        ESP_LOGI("WIFI_REG", "Value in %08x: %08x", (unsigned int)WIFI_TX_PLCP0, (unsigned int)REG_READ(WIFI_TX_PLCP0));
#else
        ret = esp_wifi_internal_tx(WIFI_IF_STA, &udp_packet[0], sizeof(udp_packet));
        ESP_LOGI("Main:", "Sent with result %s", esp_err_to_name(ret));
#endif        

        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
    }

}