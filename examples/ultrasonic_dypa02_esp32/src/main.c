/*
 * ESP-IDF Standalone Example for DYP-A02 UART Ultrasonic Library
 *
 * This example demonstrates:
 * 1. Initializing the sensor with a handle.
 * 2. Creating a separate task for sensor reading.
 * 3. Using the robust `dyp_a02_get_stable_distance_mm` function.
 * 4. Correctly handling errors and printing descriptive messages.
 * 5. Using the `dyp_a02_is_object_detected` helper function.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h" // For UART_NUM_2

// Include the library
#include "ultrasonic_dypa02.h"

static const char *TAG = "ULTRASONIC_EXAMPLE";

// --- Configuration ---
#define SENSOR_RX_PIN 27 // ESP32's RX pin (connected to Sensor's TX)
#define SENSOR_TX_PIN 26 // ESP32's TX pin (connected to Sensor's RX)

// --- Robust Reading Parameters ---
// How many times to try and read the sensor
#define US_SAMPLES_TO_TRY 20
// How many good hardware reads we need for a valid average
#define US_MIN_SAMPLES_REQ 5
// Timeout for a single packet read
#define US_PER_READ_MS 200
// Total time to wait before giving up
#define US_OVERALL_MS 3000
// 8.0 cm threshold for cup detection
#define OBJECT_THRESHOLD_MM 80

// --- Global sensor handle ---
// We initialize it here, which is safe.
static dyp_sensor_t ultrasonic_sensor = DYP_SENSOR_INITIALIZER;


/**
 * @brief This task runs in a loop, dedicated to reading the sensor.
 */
static void sensor_read_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor read task started.");
    
    // Give the sensor a moment to boot after power-on
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        // --- 1. Call the robust read function ---
        long dist_mm = dyp_a02_get_stable_distance_mm(
            &ultrasonic_sensor,
            US_SAMPLES_TO_TRY,
            US_MIN_SAMPLES_REQ,
            US_PER_READ_MS,
            US_OVERALL_MS
        );

        // --- 2. Check the return value ---
        if (dist_mm != DYP_A02_READ_ERROR) {
            // --- SUCCESS ---
            float dist_cm = (float)dist_mm / 10.0f;
            ESP_LOGI(TAG, "Average distance: %ld mm (%.2f cm)", dist_mm, dist_cm);

            // --- 3. Use the helper function to check for an object ---
            if (dyp_a02_is_object_detected(dist_mm, OBJECT_THRESHOLD_MM)) {
                ESP_LOGW(TAG, "  --> Object DETECTED!");
            } else {
                ESP_LOGI(TAG, "  --> No object detected.");
            }

        } else {
            // --- ERROR ---
            // The read failed. Get the specific error from the handle.
            dyp_err_t err = dyp_a02_get_last_error(&ultrasonic_sensor);
            
            // Print the human-readable error message.
            ESP_LOGE(TAG, "Failed to read sensor: %s", dyp_a02_strerror(err));
        }

        // Wait 1 second before the next read
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Ultrasonic Sensor Example...");
    ESP_LOGI(TAG, "Initializing sensor on RX=%d, TX=%d", SENSOR_RX_PIN, SENSOR_TX_PIN);

    // Note: The handle `ultrasonic_sensor` was already initialized globally.
    // If it were a local variable, you would do:
    // ultrasonic_sensor = DYP_SENSOR_INITIALIZER;

    // Initialize the driver
    dyp_err_t err = dyp_a02_init_esp_hw(
        &ultrasonic_sensor,
        DYP_A02_DEFAULT_UART_PORT, // Use UART_NUM_2
        SENSOR_RX_PIN,
        SENSOR_TX_PIN
    );

    if (err != DYP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor driver: %s", dyp_a02_strerror(err));
        ESP_LOGE(TAG, "Check wiring, pins, and voltage divider. Halting.");
        return;
    }

    ESP_LOGI(TAG, "Sensor initialized successfully.");
    
    // Create the read task
    xTaskCreate(sensor_read_task, "sensor_read_task", 4096, NULL, 5, NULL);
}