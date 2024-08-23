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

#define QEMU_AP
// #undef QEMU_AP

// Defining connection parameters
#ifdef QEMU_AP
#define WIFI_SSID "Espressif"
#define WIFI_PASS ""
#else
#define WIFI_SSID "ResPECT"
#define WIFI_PASS "twoflower"
#endif

#define SCAN_AUTH_MODE_THRESHOLD ESP_WIFI_AUTH_WPA2_PSK // Not used since it is default
#define MAX_RETRY 5

typedef uint8_t macaddr_t[6];

// Struct for 80211 packet format
typedef struct mac80211_frame {
    struct mac80211_frame_control {
        unsigned    protocol_version    : 2;
        unsigned    type            : 2;
        unsigned    sub_type        : 4;
        unsigned    to_ds           : 1;
        unsigned    from_ds         : 1;
        unsigned    _flags:6;
    } __attribute__((packed)) frame_control;
    uint16_t  duration_id;
    macaddr_t receiver_address;
    macaddr_t transmitter_address;
    macaddr_t address_3;
    struct mac80211_sequence_control {
        unsigned    fragment_number     : 4;
        unsigned    sequence_number     : 12;
    } __attribute__((packed));
    uint8_t data_and_fcs[2316];
}  __attribute__((packed)) mac80211_frame;

typedef struct dma_list_item {
	uint16_t size : 12;
	uint16_t length : 12;
	uint8_t _unknown : 6;
	uint8_t has_data : 1;
	uint8_t owner : 1; // What does this mean?
	void* packet;
	struct dma_list_item* next;
} __attribute__((packed)) dma_list_item;

extern bool pp_post(uint32_t requestnum, uint32_t argument);

esp_err_t ret;

uint8_t s_retry_num = 0;

// Hand crafted UDP packet
const uint8_t packet[] = {
    // MAC layer
    0x74, 0xdf, 0xbf, 0xa4, 0xf7, 0x97, // Destination MAC address
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

    ESP_LOGW("main", "Killing proprietary wifi task (ppTask)");
	pp_post(0xf, 0);

}