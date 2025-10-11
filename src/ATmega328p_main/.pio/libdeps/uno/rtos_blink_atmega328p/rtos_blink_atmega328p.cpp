#include "rtos_blink_atmega328p.h"
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

extern "C" {
  #include "uart_standard.h"
}

#define BLINK_PIN LED_BUILTIN

void blinkTask(void *pvParameters) {
  pinMode(BLINK_PIN, OUTPUT);

  for (;;) {
    digitalWrite(BLINK_PIN, HIGH);
    uart_write(UART_0_PROG, "LED ON\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(BLINK_PIN, LOW);
    uart_write(UART_0_PROG, "LED OFF\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void init_blink_task(void) {
  uart_init(UART_0_PROG, 9600);

  xTaskCreate(
    blinkTask,
    "Blink",
    128,
    NULL,
    1,
    NULL
  );
}
