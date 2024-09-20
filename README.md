# Raw WiFi Tx
ESP-IDF project to send data packets using an "open source" tx function. The project follows from the developments in the [ESP32 Open MAC](https://esp32-open-mac.be/) project. 

In `respect_raw_tx()` A hand crafted IEEE 802.11 packet is transmitted by register manipulation. A DMA struct is required to provide metadata about the packet to the registers.

```
typedef struct dma_list_item {
	uint16_t size : 12;
	uint16_t length : 12;
	uint8_t _unknown : 6;
	uint8_t has_data : 1;
	uint8_t owner : 1; 
	void* packet;
	struct dma_list_item* next;
} __attribute__((packed,aligned(4))) dma_list_item_t;
```

Registers that have been identified are listed below

|                  |            |
| ---------------- | ---------- |
| WIFI_TX_CONFIG   | 0x60033d04 |
| WIFI_MAC_CTRL    | 0x60033ca0 |
| WIFI_TX_PLCP0    | 0x60033d08 |
| WIFI_TX_PLCP1    | 0x600342f8 |
| WIFI_TX_PLCP1_2  | 0x600342fc |
| WIFI_TX_PLCP2    | 0x60034314 |
| WIFI_TX_DURATION | 0x60034318 |

This currently relies on the proprietary wifi thread running in the background for handling interrupts.

## Testing:
 - Connected ESP32 and laptop to a WiFi network without internet access.
 - Used Wireshark to identify the repeated UDP packets. Currently only identifiable in monitor mode (TODO).

## TODO:
 - [x] Setting up custom interrupts
 - [x] Adding queues for event management
 - [ ] Handling of collisions
 - [ ] Adding custom auth messages. Port from ESP32-Open-MAC
 - [ ] RX needs to be handled. Identify registers first
 - [ ] Open sourcing the init function