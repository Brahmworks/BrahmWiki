# Real-Time Operating System (RTOS)

This project uses [FreeRTOS](https://www.freertos.org/) to manage tasks and scheduling on the microcontrollers. FreeRTOS is a market-leading real-time operating system for microcontrollers and small microprocessors.

## Integration

FreeRTOS is integrated into the projects via the PlatformIO library manager.

-   **ESP32:** The ESP-IDF framework comes with its own version of FreeRTOS.
-   **AVR (ATmega328P & ATmega2560):** The `feilipu/FreeRTOS` library is used, which is a popular port of FreeRTOS for the Arduino framework.

## Application Notes

-   **Task Management:** Each major function, such as blinking an LED or reading a sensor, should be implemented in its own task.
-   **Stack Size:** Ensure that each task is allocated a sufficient stack size. Stack overflows are a common source of bugs in RTOS applications. The `configMINIMAL_STACK_SIZE` constant can be used as a starting point, but may need to be increased depending on the complexity of the task.
-   **Delays:** Use `vTaskDelay()` or `vTaskDelayUntil()` for delays instead of `delay()`. This allows the RTOS to switch to other tasks while the current task is waiting, improving efficiency.
