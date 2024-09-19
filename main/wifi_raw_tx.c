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
#include "net_setup.h"

// Hardware task
#include "hw_task.h"

#include "esp_log.h"

#define STACK_SIZE 4096

StackType_t hwStack[STACK_SIZE];
StackType_t sendStack[STACK_SIZE];

StaticTask_t hwTaskBuffer;
StaticTask_t sendTaskBuffer;

TaskHandle_t hwTaskHandle = NULL, sendTaskHandle = NULL;

// BaseType_t hw_task_success, send_task_success;

void app_main(){

    net_setup();

    // Add some delay to make sure the association and authentication are completed
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Ideally, hardware task monitors the queue for events on the wifi peripherals
    // Priority 5
    hwTaskHandle = xTaskCreateStaticPinnedToCore(&respect_hardware_task, "hw_task", STACK_SIZE, NULL, 23, hwStack, &hwTaskBuffer, 0);

    if(hwTaskHandle == NULL){
        ESP_LOGE("main", "Failed to create H/W task. Reboot!");
    }else{
        ESP_LOGI("main", "Created h/w task");
    }
    // Send task queues the packets
    sendTaskHandle = xTaskCreateStaticPinnedToCore(&respect_send_task, "send_task", STACK_SIZE, NULL, 23, sendStack, &sendTaskBuffer, 0);

    if(sendTaskHandle == NULL){
        ESP_LOGE("main", "Failed to create send task. Reboot!");
    }else{
        ESP_LOGI("main", "Created send task");
    }
}