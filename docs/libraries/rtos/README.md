# Real-Time Operating System (RTOS)

This project uses [FreeRTOS](https://www.freertos.org/) to manage tasks and scheduling on the microcontrollers. FreeRTOS is a market-leading real-time operating system for microcontrollers and small microprocessors.

## Integration

FreeRTOS is integrated into the projects via the PlatformIO library manager.

-   **ESP32:** The ESP-IDF framework comes with its own version of FreeRTOS.
-   **AVR (ATmega328P & ATmega2560):** The `feilipu/FreeRTOS` library is used, which is a popular port of FreeRTOS for the Arduino framework.

## Key API Functions

### `xTaskCreate`

This is the fundamental function used to create a new task.

```c
BaseType_t xTaskCreate(
    TaskFunction_t pvTaskCode,
    const char * const pcName,
    configSTACK_DEPTH_TYPE usStackDepth,
    void * const pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t * const pvCreatedTask
);
```

-   **`pvTaskCode`**: A pointer to the function that implements the task.
-   **`pcName`**: A descriptive name for the task. This is mainly used for debugging.
-   **`usStackDepth`**: The size of the task stack in words. `configMINIMAL_STACK_SIZE` is a good starting point.
-   **`pvParameters`**: A value that is passed as the parameter to the created task.
-   **`uxPriority`**: The priority at which the task should run. A higher number indicates a higher priority.
-   **`pvCreatedTask`**: Can be used to pass out a handle to the created task.

---

### `vTaskDelay`

This function delays a task for a given number of ticks.

```c
void vTaskDelay(const TickType_t xTicksToDelay);
```

-   **`xTicksToDelay`**: The number of system ticks to delay the task for. The constant `portTICK_PERIOD_MS` can be used to convert milliseconds to ticks (e.g., `1000 / portTICK_PERIOD_MS` for a 1-second delay).

## Application Notes

-   **Task Management:** Each major function, such as blinking an LED or reading a sensor, should be implemented in its own task.
-   **Stack Size:** Ensure that each task is allocated a sufficient stack size. Stack overflows are a common source of bugs in RTOS applications. The `configMINIMAL_STACK_SIZE` constant can be used as a starting point, but may need to be increased depending on the complexity of the task.
-   **Delays:** Use `vTaskDelay()` or `vTaskDelayUntil()` for delays instead of `delay()`. This allows the RTOS to switch to other tasks while the current task is waiting, improving efficiency.
