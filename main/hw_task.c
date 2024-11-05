#include "hw_task.h"

#include "esp_attr.h"
#include "riscv/interrupt.h"

#include "esp_wifi.h"

#include "esp_log.h"
#include "rom/ets_sys.h"

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

void wDev_ProcessFiq(void);

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


// Process Txqcomplete
static void processTxComplete() {
	uint32_t txq_state_complete = REG_READ(WIFI_TX_STATUS);
	if (txq_state_complete == 0) {
		return;
	}
	uint32_t slot = 31 - __builtin_clz(txq_state_complete);
	uint32_t clear_mask = 1 << slot;
    uint32_t wifi_tx_clr = REG_READ(WIFI_TX_CLR);
	REG_WRITE(WIFI_TX_CLR,  wifi_tx_clr |= clear_mask);
}

// esp32-open-mac interrupt handler
void IRAM_ATTR wifi_interrupt_handler(void){
    uint32_t cause = REG_READ(WIFI_INT_STATUS_GET);
    ets_printf("In ISR: Cause %lx\n", cause);
    wDev_ProcessFiq();    
    return;
}

void respect_setup_interrupt(){
    // This setup works perfectly "sometimes"

    ESP_LOGI("setup_intr", "Clearing existing interrupts");
    // ic_set_interrupt_handler() in ghidra
    // Mask out power interrupt and the MAC interrupt sources (temporarily)
    // From decompilation of intr_matrix_set. Both are mapped to CPU interrupt number 1
    // Disable the wifi CPU interrupt and renable after setting the handler
    REG_WRITE(INTR_SRC_MAC, 0);
    REG_WRITE(INTR_SRC_PWR, 0);

    // Disable all interrupt sources. A bit hacky but works
    uint32_t value = REG_READ(INTR_ENABLE_REG);
    REG_WRITE(INTR_ENABLE_REG, 0);
    intr_handler_set(WIFI_INTR_NUMBER, (intr_handler_t)wifi_interrupt_handler, 0);
    REG_WRITE(INTR_ENABLE_REG, value);

    // Unmask the interrupt source again. So called "routing"
    REG_WRITE(INTR_SRC_MAC, WIFI_INTR_NUMBER);
    REG_WRITE(INTR_SRC_PWR, WIFI_INTR_NUMBER);

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
    ESP_LOGI("raw_tx", "Enabled TX");
}

void respect_hardware_task(void* pvParameters){

    // Yielding for main task to init send task
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Setup our own interrupts
    respect_setup_interrupt();
    respect_mac_init();

    ESP_LOGI("hw_task", "going into loop mode");

    // Do nothing
    // @todo : Process tx/rx queues similar to esp32_open_mac or simply try porting
    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void respect_send_task(void* pvParameters){
    while(1){
        ESP_LOGI("send_task", "Sending packet");
        respect_raw_tx(&tx_item);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}