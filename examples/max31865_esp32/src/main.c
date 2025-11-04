// ESP-IDF example for MAX31865 (main.c)
// Pins used:
//  SCLK = 18, MISO = 19, MOSI = 23, CS = 5

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "max31865.h"
#include "math.h"

static const char *TAG = "app_main";

#define PIN_NUM_SCLK 18
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CS   21

void app_main(void)
{
    ESP_LOGI(TAG, "MAX31865 ESP-IDF example");

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %d", ret);
    } else {
        ESP_LOGI(TAG, "spi bus init ok");
    }

    int rc = max31865_init_hw(PIN_NUM_CS, 1000000);
    if (rc != 0) {
        ESP_LOGE(TAG, "max31865_init_hw failed %d", rc);
    }

    /* Configure RTD and mode */
    max31865_set_rtd_nominal(100.0f, 430.0f); // PT100 with 430R ref
    max31865_set_wire_mode(MAX31865_WIRES_3);
    max31865_set_filter_50hz(true);

    while (1) {
        max31865_enable_bias(true);
        esp_rom_delay_us(10000); /* 10 ms bias settle */
        max31865_start_one_shot();
        vTaskDelay(pdMS_TO_TICKS(200)); /* wait for conversion (200ms safe) */
        max31865_enable_bias(false);

        float c = max31865_read_temperature_c_float(2);
        if (!isnan(c)) {
            float f = max31865_read_temperature_f_float(2);
            long ci = max31865_read_temperature_c_int(2);
            ESP_LOGI(TAG, "Temp: %.2f C, %.2f F, int x100: %ld", c, f, ci);
        } else {
            ESP_LOGW(TAG, "Read failed or fault");
            uint8_t fault = max31865_last_fault();
            ESP_LOGW(TAG, "Fault reg: 0x%02X", fault);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
