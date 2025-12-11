#include "rtos_blink_esp32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "uart_standard.h"
#include "iot_protocol.h"
#include "esp_log.h"

static const char *TAG = "rtos_blink";

#define BLINK_GPIO 2 // Using a hardcoded value as this is now a library

// Subscription callbacks for various topics
static void torso_cmd_callback(const char *topic, const uint8_t *payload, uint32_t payload_len)
{
    ESP_LOGI(TAG, "Torso command received: %.*s", (int)payload_len, (const char *)payload);
}

static void neck_cmd_callback(const char *topic, const uint8_t *payload, uint32_t payload_len)
{
    ESP_LOGI(TAG, "Neck command received: %.*s", (int)payload_len, (const char *)payload);
}

static void head_cmd_callback(const char *topic, const uint8_t *payload, uint32_t payload_len)
{
    ESP_LOGI(TAG, "Head command received: %.*s", (int)payload_len, (const char *)payload);
}

static void batt_callback(const char *topic, const uint8_t *payload, uint32_t payload_len)
{
    ESP_LOGI(TAG, "Battery info received: %.*s", (int)payload_len, (const char *)payload);
}

void blink_task(void *pvParameter)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BLINK_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Initialize UART for logging
    uart_init(UART_0_PROG, 115200);
    ESP_LOGI(TAG, "blink_task started");

    // Initialize Ethernet and Pub/Sub protocol
    if (iot_protocol_init(UART_0_PROG) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IoT protocol");
        return;
    }

    // Register subscriptions
    iot_protocol_subscribe("torso_cmd", torso_cmd_callback);
    iot_protocol_subscribe("neck_cmd", neck_cmd_callback);
    iot_protocol_subscribe("head_cmd", head_cmd_callback);
    iot_protocol_subscribe("batt", batt_callback);

    // Perform handshake with Windows host
    if (iot_protocol_handshake() != ESP_OK) {
        ESP_LOGW(TAG, "Handshake failed, continuing anyway");
    }

    // Main event loop
    int toggle = 0;
    while(1) {
        // Process incoming pub/sub messages
        iot_protocol_process_incoming();

        // Blink LED
        gpio_set_level(BLINK_GPIO, toggle);
        toggle = !toggle;

        // Publish some data periodically
        if (toggle == 0) {
            char led_payload[] = "{\"state\":\"listen\"}";
            iot_protocol_publish("led", led_payload);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void init_blink_task(void)
{
    BaseType_t res = xTaskCreate(&blink_task, "blink_task", 8192, NULL, 5, NULL);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create blink_task: %d", res);
    } else {
        ESP_LOGI(TAG, "blink_task created");
    }
}
