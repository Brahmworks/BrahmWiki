# ESP32 WiFi Library

This library provides a simple interface to connect an ESP32 to a WiFi network.

## Functions

### `wifi_init`

Initializes the WiFi connection.

```c
void wifi_init(const char* ssid, const char* password);
```

**Parameters:**

*   `ssid`: The SSID of the WiFi network.
*   `password`: The password for the WiFi network.

### `wifi_is_connected`

Checks if the ESP32 is currently connected to the WiFi network.

```c
bool wifi_is_connected(void);
```

**Returns:**

*   `true` if connected, `false` otherwise.

## Example Usage

Here's a complete example of how to use the library to connect to a WiFi network:

```c
#include "wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "wifi_example";

void app_main(void) {
    // Initialize WiFi
    wifi_init("your_ssid", "your_password");

    // Wait for connection
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
    }

    ESP_LOGI(TAG, "WiFi connected!");

    // Your application code here...
}
