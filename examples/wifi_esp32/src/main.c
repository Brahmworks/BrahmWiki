#include "esp_log.h"
#include "esp_wifi.h"
#include "wifi.h"   // your Wi-Fi init and status checking module
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Configurable retry interval
#define RETRY_INTERVAL_MS 2000  // 2 seconds

static const char *TAG = "wifi_app";

void wifi_retry_task(void* pvParameters) {
    while (1) {
        if (!wifi_is_connected()) {
            ESP_LOGI(TAG, "WiFi not connected, trying to reconnect...");
            esp_wifi_connect();
        }
        vTaskDelay(pdMS_TO_TICKS(RETRY_INTERVAL_MS));
    }
}

void app_main(void) {
    wifi_init("aaaaaaaa", "bbbbbbbb");

    // Create the retry task pinned to core 0
    xTaskCreatePinnedToCore(wifi_retry_task, "WiFiRetryTask", 4096, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "WiFi initialized with retry interval: %d ms", RETRY_INTERVAL_MS);
}
