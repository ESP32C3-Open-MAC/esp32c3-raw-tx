/*ResPECT - Transactional networking stack development.
ESP32 - POC sketch to transmit a packet using register access. 
Depends on the esp_wifi library for initialization.
FreeRTOS is still needed
*/

// Standard libraries
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

// WiFi driver library
#include "esp_private/wifi.h"
#include "esp_wifi.h" // Needed for init tasks

// Event library
#include "esp_event.h"

// ESP error definitions
#include "esp_err.h"
#include "esp_log.h"

#include "sdkconfig.h"

#define MAX_RETRY 5

// Register definitions
#define WIFI_TX_CONFIG 0x60033d04
#define WIFI_MAC_CTRL 0x60033ca0
#define WIFI_TX_PLCP0 0x60033d08
#define WIFI_TX_PLCP1 0x600342f8
#define WIFI_TX_PLCP1_2 0x600342fc
#define WIFI_TX_PLCP2 0x60034314
#define WIFI_TX_DURATION 0x60034318

typedef struct dma_list_item {
	uint16_t size : 12;
	uint16_t length : 12;
	uint8_t _unknown : 6;
	uint8_t has_data : 1;
	uint8_t owner : 1; // What does this mean?
	void* packet;
	struct dma_list_item* next;
} __attribute__((packed,aligned(4))) dma_list_item_t;

esp_err_t ret;

uint8_t s_retry_num = 0;

// Hand crafted 80211 packet. Currently only visible in airmon-ng
uint8_t packet[] = {
    0x08, 0x01, // frame control
    0x00, 0x00, // duration/ID

    // Replace with required MAC addresses
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Receiver address
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Transmitter address
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Destination address
    
    0x00, 0x00, // sequence control
    0xaa, 0xaa, // SNAP
    0x03, 0x00, 0x00, 0x00, // other LLC headers

    0x08, 0x00,
    // IPv4 header
    0x45, //version :4 (obv) with IHL = 5
    0x00, // DSCP and ECN
    0x00, 0x21, // Total length (33 bytes), IPv4 Header + UDP Header + "Hello"
    0x00, 0x00, // Identification and fragmentation data
    0x00, 0x00, // Flags and fragment offset
    0x05, // TTL
    0x11, // Protocol type UDP 
    0x33, 0x4a, // Header checksum
    0xc0, 0xa8, 0x00, 0x7A, // Source IP address - Arbitrary IP address for the ESP-32
    0xc0, 0xa8, 0x00, 0xb8, // Destination IP address - Laptop's IP address (assigned through DHCP by the router)

    // UDP header - 8 bytes
    0x1f, 0x45, // Source port - Both source and destination ports are random since it is UDP
    0xf0, 0xf0, // Destination port
    0x00, 0x0d, // Length
    0x00, 0x00,  // Checksum - calculate using the pseudo ipv4 header
    'h', 'e', 'l', 'l', 'o', // The message :)

    // FCS
    0x00, 0x00, 0x00, 0x00

};

// Declare the DMA list item
dma_list_item_t tx_item = {
    .size = 83,
    .length = 95,
    ._unknown = 32,
    .has_data = 1,
    .owner = 1,
    .packet = &packet[0],
    .next = NULL
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

void respect_mac_init(){
    // ic_mac_init decompilation
    uint32_t mac_val = REG_READ(WIFI_MAC_CTRL);
    REG_WRITE(WIFI_MAC_CTRL, mac_val &  0xff00efff);

}

// Still needs the proprietary task to be running
void respect_raw_tx(dma_list_item_t* tx_item){
    // Write 0xa to the last byte (why?)
    uint32_t cfg_val = REG_READ(WIFI_TX_CONFIG);
    REG_WRITE(WIFI_TX_CONFIG, cfg_val | 0xa);

    // Write the address of DMA struct to PLCP0 and set the 6th byte to 6 (why?) and 7th byte to 1 (why?)
    REG_WRITE(WIFI_TX_PLCP0, (((uint32_t)tx_item & 0xfffff) | 0x600000) | 0x01000000);

    // Using esp32-open-mac's implementation for reference
    uint32_t rate = WIFI_PHY_RATE_54M;
    uint32_t is_n_enabled = (rate >= 16);

    REG_WRITE(WIFI_TX_PLCP1, 0x10000000 | (tx_item->length & 0xfff) | ((rate & 0x1f) << 12) | ((is_n_enabled & 0b1) << 25));
    REG_WRITE(WIFI_TX_PLCP1_2, 0x0010000);

    // Write 0x00000020 (why?)
    REG_WRITE(WIFI_TX_PLCP2, 0x00000020);
    
    // Duration to 0 (Why?)
    REG_WRITE(WIFI_TX_DURATION, 0x00);

    // // Configure EDCA
    cfg_val = REG_READ(WIFI_TX_CONFIG);
    cfg_val = cfg_val | 0x02000000;
    
    cfg_val = cfg_val | 0x00003000;
    REG_WRITE(WIFI_TX_CONFIG, cfg_val);

    // Finally enable tx
    cfg_val = REG_READ(WIFI_TX_PLCP0);
    REG_WRITE(WIFI_TX_PLCP0, cfg_val | 0xc0000000);
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
        ESP_LOGE("Main", "Wifi could not be initialized %s", esp_err_to_name(ret));
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
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASS,
        },
    };

    // Set to station mode. Set the WiFi configuration. Provide wifi interface handle (WIFI_IF_STA)
    // Start the esp WiFi connection. All are called directly from the blobs
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGW("Main", "Sending Raw packets");

    // Killing proprietary wifi task seems to prevent any tx
    // ESP_LOGW("main", "Killing proprietary wifi task (ppTask)");
	// pp_post(0xf, 0);

    respect_mac_init();

    while(1){
        ESP_LOGI("Main", "Trying to send");

        respect_raw_tx(&tx_item);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

}