#include <stdio.h>
#include <math.h> // For isnan()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"

// Include the library header
#include "max6675.h"


static const char* TAG = "MAX6675_Test";

// --- Configuration ---
// Set this to 1 for Hardware SPI, 0 for Software SPI
#define USE_HW_SPI 1

// --- Hardware SPI Pins ---
#define SPI_HOST_ID   HSPI_HOST
#define PIN_NUM_MISO  19
#define PIN_NUM_MOSI  23
#define PIN_NUM_CLK   18
#define HW_CS_PIN     5  // GPIO 5 for Hardware SPI CS

// --- Software SPI Pins ---
#define SW_CS_PIN     5  // GPIO 5 for Software SPI CS
#define SW_MISO_PIN   19 // GPIO 19 for Software MISO
#define SW_CLK_PIN    18 // GPIO 18 for Software CLK


/**
 * @brief Main sensor reading task
 */
void sensor_read_task(void *pvParameters) {
    
    // Give sensors time for first conversion
    vTaskDelay(pdMS_TO_TICKS(500)); 

    while(1) {
        ESP_LOGI(TAG, "--- Reading All Values ---");

        // 1. Read all the different temperature formats
        float temp_c_float = max6675_read_celsius_float(2);   // °C with 2 decimals
        float temp_f_float = max6675_read_fahrenheit_float(2); // °F with 2 decimals
        long  temp_c_int   = max6675_read_celsius_int(2);     // °C scaled by 100 (e.g., 2525)
        long  temp_f_int   = max6675_read_fahrenheit_int(2);   // °F scaled by 100 (e.g., 7745)

        // 2. Read the raw 16-bit data
        uint16_t raw_data;
        int raw_read_result = max6675_read_raw(&raw_data);

        // 3. Check for errors
        if (temp_c_int == MAX6675_ERROR || raw_read_result != 0) {
            
            ESP_LOGW(TAG, "ERROR: Read failed.");

            // 4. Use is_open() to find out why
            if (max6675_is_open()) {
                ESP_LOGW(TAG, "Cause: Thermocouple is open (disconnected).");
            } else {
                ESP_LOGW(TAG, "Cause: SPI communication error.");
            }

        } else {
            // 5. Print all the successful readings
            ESP_LOGI(TAG, "  Celsius (float):    %.2f C", temp_c_float);
            ESP_LOGI(TAG, "  Fahrenheit (float): %.2f F", temp_f_float);
            ESP_LOGI(TAG, "  Celsius (int x100): %ld", temp_c_int);
            ESP_LOGI(TAG, "  Fahrenheit (int x100):%ld", temp_f_int);
            ESP_LOGI(TAG, "  Raw 16-bit Value:   0x%04X", raw_data); // 0xXXXX format
        }

        // Note: You can call max6675_deinit() here if you want to
        // stop using the sensor and free the SPI device.
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void app_main(void) {
    ESP_LOGI(TAG, "Initializing MAX6675 driver...");

#if USE_HW_SPI
    // --- Hardware SPI Initialization ---
    ESP_LOGI(TAG, "Using Hardware SPI on HSPI_HOST");
    
    // 1. Initialize SPI bus (HSPI_HOST)
    spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1
    };
    spi_bus_initialize(SPI_HOST_ID, &buscfg, SPI_DMA_CH_AUTO);

    // 2. Initialize the MAX6675 device
    max6675_init_hw(HW_CS_PIN, 1000000); // CS=5, clock 1MHz

#else
    // --- Software SPI Initialization ---
    ESP_LOGI(TAG, "Using Software (bit-banged) SPI");
    max6675_init_sw(SW_CS_PIN, SW_CLK_PIN, SW_MISO_PIN);

#endif

    // 3. Start the reading task
    xTaskCreate(sensor_read_task, "sensor_read_task", 4096, NULL, 5, NULL);
}