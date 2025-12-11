#include "rtos_blink_esp32.h"
#include "esp_log.h"

static const char *TAG = "app_main";

void app_main()
{
    ESP_LOGI(TAG, "Calling init_blink_task()");
    init_blink_task();
    ESP_LOGI(TAG, "Returned from init_blink_task()");
}
