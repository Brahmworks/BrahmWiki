#include "rtos_blink_esp32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "uart_standard.h"
#include "iot_protocol.h"

#define BLINK_GPIO 2 // Using a hardcoded value as this is now a library

void blink_task(void *pvParameter)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BLINK_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    iot_protocol_init(UART_0_PROG);
    iot_protocol_handshake();

    while(1) {
        iot_protocol_process_incoming();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void init_blink_task(void)
{
    uart_init(UART_0_PROG, 115200);
    xTaskCreate(&blink_task, "blink_task", 4096, NULL, 5, NULL);
}
