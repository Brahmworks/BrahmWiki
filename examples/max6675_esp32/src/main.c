// main.c - ESP-IDF example for MAX6675
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "max6675.h"
#include <math.h>

static const char *TAG = "app_main";

/* Example pins used for bus init. Change to match your board:
   SCLK = 18, MISO = 19, MOSI = 23 (MOSI not used by max6675 but required by spi_bus_init)
   CS = 5
*/
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_SCLK 18
#define PIN_NUM_CS   5

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "MAX6675 ESP-IDF example");

    /* Initialize SPI bus explicitly (preferred). You can also rely on library
       default bus init in max6675_init_hw(), but explicit is safer. */
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", ret);
    } else {
        ESP_LOGI(TAG, "SPI bus init OK (or already initialized)");
    }

    /* Initialize MAX6675 device (hardware SPI) */
    int rc = max6675_init_hw(PIN_NUM_CS, 500000); // CS pin and clock_hz
    if (rc != 0) {
        ESP_LOGE(TAG, "max6675_init_hw failed %d", rc);
    } else {
        ESP_LOGI(TAG, "max6675 init ok");
    }

    while (1) {
        float c = max6675_read_celsius_float(2);
        if (isnan(c)) {
            ESP_LOGW(TAG, "Temperature read failed or thermocouple open");
        } else {
            float f = max6675_read_fahrenheit_float(2);
            long c_int = max6675_read_celsius_int(0);
            long c_scaled = max6675_read_celsius_int(2);
            ESP_LOGI(TAG, "Temp: %.2f °C, %.2f °F | int: %ld | scaled x100: %ld",
                     c, f, c_int, c_scaled);
        }

        if (max6675_is_open()) {
            ESP_LOGW(TAG, "Thermocouple open/fault!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /* If you want to deinit at some point:
       max6675_deinit();
       spi_bus_free(HSPI_HOST);
    */
}