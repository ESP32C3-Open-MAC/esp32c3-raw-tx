#ifndef __HW_TASK_H__
#define __HW_TASK_H__

#include <stdint.h>
#include <stdbool.h>

// macro to dump mempry
#define DUMP_MEMORY(addr, len) {\
    for(int ii = 0; ii < len; ii++){\
        printf("%02x ", *(uint8_t *)(addr + ii));\
    }\
    printf("\n");\
}

#define PRINT_REGISTER(_r){\
    uint32_t _v = REG_READ(_r);\
    printf("Reg: %x, value: %lx\n", _r, _v);\
}


// Register definitions
// MAC registers
#define WIFI_MAC_BASE 0x60033000
#define WIFI_TX_CONFIG 0x60033d04
#define WIFI_MAC_CTRL 0x60033ca0
#define WIFI_TX_PLCP0 0x60033d08
#define WIFI_TX_PLCP1 0x600342f8
#define WIFI_TX_PLCP1_2 0x600342fc
#define WIFI_TX_HTSIG 0x60034310
#define WIFI_TX_PLCP2 0x60034314
#define WIFI_TX_DURATION 0x60034318
#define WIFI_TX_STATUS 0x60033cb0
#define WIFI_TX_CLR 0x60033cac
#define WIFI_TX_GET_ERR 0x60033ca8
#define WIFI_TX_CLR_ERR 0x60033ca4
#define WIFI_INT_STATUS_GET 0x60033c3c
#define WIFI_INT_STATUS_CLR 0x60033c40
#define PWR_INT_STATUS_GET 0x60035118
#define PWR_INT_STATUS_CLR 0x6003511c
#define WIFI_MAC_CCA_REG 0x60033c50


// Intererupt registers
#define INTR_SRC_MAC 0x600c2000
#define INTR_SRC_PWR 0x600c2008
#define INTR_ENABLE_REG 0x600c2104 // Writing a 1 to corresponding bit enables and writing 0 disables
#define INTR_STATUS_REG 0x600c00F8

#define WIFI_INTR_NUMBER 1
#define SYSTICK_INTR_NUMBER 7 // Tick 
#define TIMER_ALARM_NUMBER 3
#define TASK_WDT_NUMBER 9

typedef struct dma_list_item {
	uint16_t size : 12;
	uint16_t length : 12;
	uint8_t _unknown : 6;
	uint8_t has_data : 1;
	uint8_t owner : 1; // What does this mean?
	void* packet;
	struct dma_list_item* next;
} __attribute__((packed,aligned(4))) dma_list_item_t;

// Data types for queueing
typedef enum {
	RX_ENTRY,
	TX_ENTRY
} hardware_queue_entry_type_t;

typedef struct
{
	uint32_t interrupt_received;
} rx_queue_entry_t;

typedef struct
{
	uint8_t* packet;
	uint32_t len;
} tx_queue_entry_t;

typedef struct {
	hardware_queue_entry_type_t type;
	union {
		rx_queue_entry_t rx;
		tx_queue_entry_t tx;
	} content;
} hardware_queue_entry_t;


// RTOS functions
extern bool pp_post(uint32_t requestnum, uint32_t argument);

void respect_setup_interrupt(void);

void respect_mac_init(void);

void respect_raw_tx(dma_list_item_t* tx_item);

void respect_hardware_task(void* pvParameters);

void respect_send_task(void* pvParameters);

#endif