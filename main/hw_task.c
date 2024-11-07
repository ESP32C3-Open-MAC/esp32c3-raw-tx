#include "hw_task.h"

#include "esp_attr.h"
#include "riscv/interrupt.h"

#include "esp_wifi.h"

#include "esp_log.h"
#include "rom/ets_sys.h"
// #include "freertos/queue.h"

#define BUFFER_SIZE 10

// Hand crafted 80211 packet. Currently only visible in airmon-ng
uint8_t packet[] = {
    0x08, 0x01, // frame control
    0x00, 0x00, // duration/ID

    // Replace with required MAC addresses
    0x40, 0x9b, 0xcd, 0x25, 0x2a, 0xf8, // Receiver address
    0x84, 0xf7, 0x03, 0x60, 0x81, 0x5c, // Transmitter address
    0xc8, 0x15, 0x4e, 0xd4, 0x65, 0x1b, // Destination address

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

uint8_t rx_pkt[1600];

dma_list_item_t rx_item = {
    .size = 1600,
    .has_data = 0,
    .owner = 1,
    ._unknown = 32,
    .length = 1600,
    .packet = &rx_pkt[0],
    .next = NULL
};

void print_registers();

static QueueHandle_t tx_event_q_hdl;
static StaticQueue_t tx_event_q;
static uint8_t tx_q_buffer[BUFFER_SIZE * sizeof(hardware_queue_entry_t)];
static StaticSemaphore_t tx_event_q_sem_buf;
static SemaphoreHandle_t tx_event_q_sem_hdl;

static inline void enable_tx(){
    // Finally enable tx
    uint32_t cfg_val = REG_READ(WIFI_TX_PLCP0);
    REG_WRITE(WIFI_TX_PLCP0, cfg_val | 0xc0000000);
}

static inline void set_base_rx_addr(dma_list_item_t * addr){
    REG_WRITE(WIFI_BASE_RX_DSCR, (uint32_t)addr);
}

static void respect_setup_rx(){
    set_base_rx_addr(&rx_item);
}

// Process Txqcomplete
static void processTxComplete() {
    // Set longer timeout

	uint32_t txq_state_complete = REG_READ(WIFI_TX_STATUS);
	if (txq_state_complete == 0) {
		return;
	}
	uint32_t slot = 31 - __builtin_clz(txq_state_complete);
	uint32_t clear_mask = 1 << slot;
    uint32_t wifi_tx_clr = REG_READ(WIFI_TX_CLR);
	REG_WRITE(WIFI_TX_CLR,  wifi_tx_clr |= clear_mask);
    REG_WRITE(WIFI_TX_STATUS, txq_state_complete &= ~clear_mask);
}

// Process Timeouts. Ideally shouldn't happen but why does it occur on crowded channel?
static void processTxTimeouts(){
    uint32_t wifi_tx_err = REG_READ(WIFI_TX_GET_ERR);
    wifi_tx_err = (wifi_tx_err >> 16) && 0xff;
    if(wifi_tx_err == 0){
        return;
    }
    uint32_t slot = 31 - __builtin_clz(wifi_tx_err);
    uint32_t clear_mask = (1 << slot);
    
    // See espressif timeout handler
    // Request CCA and wait for the length of 8 OFDM symbols (?)
    uint32_t cca_value = (REG_READ(WIFI_MAC_CCA_REG) & 0x3fffffff) | (3 << 30);
    REG_WRITE(WIFI_MAC_CCA_REG, cca_value);
    ets_delay_us(16);

    // Set the txq to be invalid
    uint32_t plcp0_value = REG_READ(WIFI_TX_PLCP0);
    // Check for validity
    uint32_t is_valid = (plcp0_value >> 30) & 0x01;

    // Reset CCA to zero (why)
    cca_value = REG_READ(WIFI_MAC_CCA_REG) & 0x3fffffff;
    REG_WRITE(WIFI_MAC_CCA_REG, cca_value);

    // Disable tx
    if(is_valid){
        REG_WRITE(WIFI_TX_PLCP0, plcp0_value & 0x3fffffff);
        // enable_tx();
    }

    uint32_t wifi_tx_err_clr = REG_READ(WIFI_TX_CLR_ERR);
    REG_WRITE(WIFI_TX_CLR_ERR, (wifi_tx_err_clr & 0xff00ffff) | (clear_mask << 16));
}

// only needed to handle the power interrupt
static void unblock_txq(void){
    uint32_t value = REG_READ(WIFI_TX_CONFIG);
    REG_WRITE(WIFI_TX_CONFIG, value & 0xfff1ffff);
}

// esp32-open-mac interrupt handler
void IRAM_ATTR wifi_interrupt_handler(void){
    uint32_t mac_cause = REG_READ(WIFI_INT_STATUS_GET);
    // uint32_t pwr_cause = REG_READ(PWR_INT_STATUS_GET);

    // This probably means the power management peripheral triggered
    // the interrupt as this interrupt is shared between the MAC and PWR peripherals
    // if((mac_cause | pwr_cause) == 0){
    if(mac_cause == 0){
        // ets_printf("No cause ISR\n");
        return;
    }

    REG_WRITE(WIFI_INT_STATUS_CLR, mac_cause);
    // REG_WRITE(PWR_INT_STATUS_CLR, pwr_cause);

    volatile bool tmp = pdFALSE;
    if(xSemaphoreTakeFromISR(tx_event_q_sem_hdl, &tmp) == pdTRUE){
        hardware_queue_entry_t rx_queue_entry;    
        rx_queue_entry.type = RX_ENTRY;
        // if(mac_cause == 0){
            // rx_queue_entry.content.rx.interrupt_received = pwr_cause;
        // }else{
        rx_queue_entry.content.rx.interrupt_received = mac_cause;
        // }
        xQueueSendFromISR(tx_event_q_hdl, &rx_queue_entry, pdFALSE);
    }
    ets_printf("Returning from ISR with mac cause %lx\n", mac_cause);
    
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
    // Mask this interrupt for now. Results in a constant interval interrupt at what seems to be the TBTT
    // REG_WRITE(INTR_SRC_PWR, WIFI_INTR_NUMBER);

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
    REG_WRITE(WIFI_TX_PLCP1_2, 0x00100000);

    // Write 0x00000020 (why?)
    cfg_val = REG_READ(WIFI_TX_PLCP2);
    REG_WRITE(WIFI_TX_PLCP2, (cfg_val & (0xffffc03f | 0x00000020))); // Using the proprietary code
    
    // Duration to 0 (Why?)
    REG_WRITE(WIFI_TX_DURATION, 0x00);

    // // Configure EDCA
    cfg_val = REG_READ(WIFI_TX_CONFIG);
    cfg_val = cfg_val | 0x02003000;
    // cfg_val = cfg_val & (0xffffefff & 0x00003000);
    REG_WRITE(WIFI_TX_CONFIG, cfg_val);

    enable_tx();
}

void respect_send_packet(uint8_t *pkt, uint32_t len){
    // Send the already crafted DMA struct
    // @todo: Implement proper DMA struct creation (MAC layer)
    respect_raw_tx(&tx_item);
}

uint32_t respect_rx_packet(){
    if(rx_item.has_data == 1){
        printf("Dumping data from DMA struct");
        DUMP_MEMORY(rx_item.packet, rx_item.length);
        rx_item.has_data = 0; // Clear it for reuse
        return rx_item.length;
    }else{
        return 0;
    }
}

void print_registers(){
    PRINT_REGISTER(WIFI_MAC_BASE);
    PRINT_REGISTER(WIFI_TX_CONFIG);
    PRINT_REGISTER(WIFI_MAC_CTRL);
    PRINT_REGISTER(WIFI_TX_PLCP0);
    PRINT_REGISTER(WIFI_TX_PLCP1);
    PRINT_REGISTER(WIFI_TX_PLCP1_2);
    PRINT_REGISTER(WIFI_TX_HTSIG);
    PRINT_REGISTER(WIFI_TX_PLCP2);
    PRINT_REGISTER(WIFI_TX_DURATION);
    PRINT_REGISTER(WIFI_MAC_CCA_REG);
}

void respect_hardware_task(void* pvParameters){

    // create event queue for sharing of both TX and RX tasks together
    tx_event_q_hdl = xQueueCreateStatic(BUFFER_SIZE, sizeof(hardware_queue_entry_t), tx_q_buffer, &tx_event_q);
    tx_event_q_sem_hdl = xQueueCreateCountingSemaphoreStatic(BUFFER_SIZE, BUFFER_SIZE, &tx_event_q_sem_buf);

    ESP_LOGW("hw_task", "Killing proprietary wifi task (ppTask)");
	pp_post(0xf, 0); 

    // Setup our own interrupts
    respect_setup_interrupt();
    respect_setup_rx();
    respect_mac_init();

    // @todo : Process tx/rx queues similar to esp32_open_mac or simply try porting
    hardware_queue_entry_t queue_entry;
    uint32_t cause;
    
    while(1){
        if(xQueueReceive(tx_event_q_hdl, &queue_entry, 10)){
            if(queue_entry.type == TX_ENTRY){
                ESP_LOGI("hw_task", "Sending queued packet");
                respect_send_packet(queue_entry.content.tx.packet, queue_entry.content.tx.len);

                // Free up the queue resources
                xSemaphoreGive(tx_event_q_sem_hdl);
            }else if(queue_entry.type == RX_ENTRY){
                cause = queue_entry.content.rx.interrupt_received;

                if(cause & 0x80){
                    // Packet received
                    ESP_LOGI("hw_task", "TX complete");
                    processTxComplete();
                }
                
                if(cause & 0x800){
                    ESP_LOGE("hw_task", "Watchdog panic");
                }
                
                if (cause & 0x600000) {
					// TODO this is bad, we should reboot
					ESP_LOGE("hw_task", "something bad, we should reboot");
				}
				
                if (cause & 0x1000024) {
					ESP_LOGW("hw_task", "received message");					
                    uint32_t len = respect_rx_packet();
				}
                
                if (cause & 0x80000) {
					ESP_LOGE("hw_task", "lmacProcessAllTxTimeout");
                    processTxTimeouts();
				}

				if (cause & 0x100) {
					ESP_LOGE("hw_task", "lmacProcessCollisions");
				}
                
                // // For now does not conflict with mac stuff
                // if (cause & 0x10)
                // {
                //     ESP_LOGE("hw_task", "unblock_txq");
                //     unblock_txq();
                // }

                xSemaphoreGive(tx_event_q_sem_hdl);
            }
        }
        // Single core processor so suspend task briefly for IDLE task
        vTaskDelay(50 / portTICK_PERIOD_MS); 
    }
}

// For now, "MAC layer and above" is just this function queuing a single for tx
void respect_send_task(void* pvParameters){
    while(1){
        ESP_LOGI("send_task", "Sending packet");

        if(tx_event_q_hdl != NULL){
            // Put the packet into the queue
            if(xSemaphoreTake(tx_event_q_sem_hdl, 1) == pdTRUE){
                hardware_queue_entry_t send_queue_entry;
                send_queue_entry.type = TX_ENTRY;
                send_queue_entry.content.tx.len = sizeof(packet);
                send_queue_entry.content.tx.packet = &packet[0];
                xQueueSendToFront(tx_event_q_hdl, &send_queue_entry, 0);
            }else{
                ESP_LOGW("send_task", "Queue is full!");
            }
        }else{
            ESP_LOGW("send_task", "Queue not yet allocated");
        }

        // Every second, send one packet
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}