#include "net_setup.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "sdkconfig.h"

wifi_config_t wifi_config = {
    .sta = {
            .ssid = CONFIG_RESPECT_WIFI_SSID,
            .password = CONFIG_RESPECT_WIFI_PASS,
        },
};

uint8_t s_retry_num = 0;

esp_err_t respect_wifi_init(){
    esp_err_t result;

    // Disabling nvs in configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;
    result = esp_wifi_init(&cfg);
    return result;

}

esp_err_t respect_wifi_deinit(){
    // Disable wifi
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());
    return esp_wifi_deinit();
}

esp_err_t respect_start_wifi_sta(wifi_config_t* cfg){
    esp_err_t result;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, cfg));
    result = esp_wifi_start();
    return result;
}

void wifi_event_handlers(void *arg, esp_event_base_t event_base,
                    int32_t event_id, void* event_data)
{
    // Check the event base for a WiFi event. If device disconnects, retry until MAX_RETRY is exceeded
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI("net_setup", "WIFI_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
        ESP_LOGI("net_setup", "successfully connected to wifi");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
        ESP_LOGI("net_setup", "WIFI_EVENT_STA_DISCONNECTED");
        if (s_retry_num < CONFIG_RESPECT_MAX_RETRY) {
            ESP_LOGI("net_setup", "retrying connect");
            ESP_ERROR_CHECK(esp_wifi_connect());
            s_retry_num++;
            ESP_LOGI("net_setup", "retrying connect finished");
        }
    } else if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_CONNECTED) {
            ESP_LOGI("net_setup", "WIFI_EVENT_STA_CONNECTED");
            /* TODO: why is this not defined? should be 41 */
            /* } else if (event_id == WIFI_EVENT_HOME_CHANNEL_CHANGE) { */
            /*     ESP_LOGI("net_setup", "WIFI_EVENT_HOME_CHANNEL_CHANGE"); */
        } else {
            ESP_LOGW("net_setup", "unexpected WIFI_EVENT id %d", (int) event_id);
        }
    } else {
        ESP_LOGW("net_setup", "unexpected event base %p id %d", (void *) event_base, (int) event_id);
    }
}

void net_setup() {
    // Init and connect to wifi
    ESP_ERROR_CHECK(respect_wifi_init());

    // create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_event_handler_instance_t instance_any_id;
    
    // register the event hanndler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handlers,
                                                        NULL,
                                                        &instance_any_id));
    
    ESP_ERROR_CHECK(respect_start_wifi_sta(&wifi_config));
    printf("Done with the connection...\n");
    
}
