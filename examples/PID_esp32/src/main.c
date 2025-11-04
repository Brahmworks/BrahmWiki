#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "max31865.h"
#include "pid_heater.h"

#define CS_MAX31865 21
#define SSR_GPIO 15
#define CONTROL_PERIOD_MS 3000U

static const char *TAG = "HEATER_CASCADE_PID";

static inline void ssr_init(void) { gpio_reset_pin(SSR_GPIO); gpio_set_direction(SSR_GPIO, GPIO_MODE_OUTPUT); gpio_set_level(SSR_GPIO, 0); }
static inline void ssr_set(bool on) { gpio_set_level(SSR_GPIO, on ? 1 : 0); }

void app_main(void) {
    // init sensor
    max31865_init_hw(CS_MAX31865, 1000000U);
    max31865_set_continuous_mode(true);

    ssr_init();

    // create and configure pid
    pid_heater_handle_t pid = pid_heater_create();
    pid_heater_tune_t tune = { .Kp = 50.33f, .Ki = 0.162f, .Kd = 0.0f, .mode = PID_CTRL_PI };
    pid_heater_init_manual(pid, tune, PID_CTRL_PI, PID_OUTMODE_TIME_WINDOW, 0, 100, CONTROL_PERIOD_MS, CONTROL_PERIOD_MS);
    pid_heater_set_setpoint(pid, 85.0f);

    const uint64_t start_us = esp_timer_get_time();

    while (1) {
        // get measurement (safe wrapper)
        float temp = max31865_read_temperature_safe();

        // compute output (int 0..100)
        int out = pid_heater_update(pid, temp, CONTROL_PERIOD_MS);

        // time-proportional: on_ms computed from out percent
        uint32_t on_ms = (uint32_t)roundf(((float)out / 100.0f) * (float)CONTROL_PERIOD_MS);

        // simple TP control
        if (on_ms > 0) {
            ssr_set(true);
            vTaskDelay(pdMS_TO_TICKS(on_ms));
        }
        ssr_set(false);
        vTaskDelay(pdMS_TO_TICKS(20)); // small guard before next read

        // log
        float t_s = (esp_timer_get_time() - start_us) / 1e6f;
        ESP_LOGI(TAG, "t=%.1fs, PT100=%.2f, Duty=%d%%", t_s, temp, out);

        // wait remainder of window (we used on_ms + 20ms already)
        uint32_t consumed = (on_ms + 20);
        if (consumed < CONTROL_PERIOD_MS) vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS - consumed));
    }
}
