#include <ultrasonic_dypa02.h>
#include <string.h>
#include <stdio.h>

static int g_rx_pin = -1;
static int g_tx_pin = -1;
static int g_baud = DYP_UART_BAUD_DEFAULT;
static bool g_initialized = false;
static bool g_last_timeout = false;

/* Internal parser buffer: rolling buffer to detect frames */
#define DYP_FRAME_LEN 4
static uint8_t g_frame_buf[64];
static size_t  g_frame_idx = 0;

/* Helper to validate frame and compute distance in cm:
   Frame layout: 0xFF, XX, YY, SUM
   raw = (XX << 8) | YY
   distance_cm = raw / 10.0f
*/
static bool validate_and_parse_frame(const uint8_t *f, float *out_cm) {
    if (!f) return false;
    if (f[0] != 0xFF) return false;
    uint16_t raw = ((uint16_t)f[1] << 8) | (uint16_t)f[2];
    uint8_t sum = f[3];
    uint8_t check = (uint8_t)((0xFF + f[1] + f[2]) & 0xFF);
    if (check != sum) return false;
    if (out_cm) *out_cm = ((float)raw) / 10.0f;
    return true;
}

/* Platform-specific implementations expose a consistent platform_* API.
   The bottom of the file provides the public dyp_uart_* wrappers that call
   into the selected platform implementation. */

/* ------------------------ ESP-IDF implementation ------------------------ */
#if defined(ESP_PLATFORM)

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "dyp_uart_esp";
static uart_port_t g_uart_num = UART_NUM_1;
static int g_uart_buf_size = 1024;

static int platform_init(int rx_pin, int tx_pin, int baud) {
    if (g_initialized) {
        /* re-init: deinit first */
        uart_driver_delete(g_uart_num);
        g_initialized = false;
    }
    g_rx_pin = rx_pin;
    g_tx_pin = tx_pin;
    g_baud = (baud == 0) ? DYP_UART_BAUD_DEFAULT : baud;

    uart_config_t uart_config = {
        .baud_rate = g_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    if (uart_param_config(g_uart_num, &uart_config) != ESP_OK) return DYP_UART_ERROR;
    if (uart_set_pin(g_uart_num, g_tx_pin, g_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return DYP_UART_ERROR;
    if (uart_driver_install(g_uart_num, g_uart_buf_size, 0, 0, NULL, 0) != ESP_OK) return DYP_UART_ERROR;

    g_frame_idx = 0;
    g_initialized = true;
    g_last_timeout = false;
    ESP_LOGI(TAG, "initialized rx=%d tx=%d @%d", g_rx_pin, g_tx_pin, g_baud);
    return DYP_UART_OK;
}

static int platform_deinit(void) {
    if (!g_initialized) return DYP_UART_OK;
    uart_driver_delete(g_uart_num);
    g_initialized = false;
    g_frame_idx = 0;
    return DYP_UART_OK;
}

static int platform_set_baud(int baud) {
    if (baud <= 0) return DYP_UART_ERROR;
    if (!g_initialized) { g_baud = baud; return DYP_UART_OK; }
    if (uart_set_baudrate(g_uart_num, baud) != ESP_OK) return DYP_UART_ERROR;
    g_baud = baud;
    return DYP_UART_OK;
}

static int platform_read_distance_cm_float(float *out_cm, unsigned timeout_ms) {
    if (!g_initialized) return -2;
    if (!out_cm) return -2;
    g_last_timeout = false;
    uint32_t timeout_us = (timeout_ms == 0) ? 100u * 1000u : timeout_ms * 1000u;
    int64_t start = esp_timer_get_time();
    uint8_t byte;
    while ((esp_timer_get_time() - start) <= (int64_t)timeout_us) {
        int len = uart_read_bytes(g_uart_num, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            /* push to rolling buffer */
            if (g_frame_idx < sizeof(g_frame_buf)) {
                g_frame_buf[g_frame_idx++] = byte;
            } else {
                /* shift by one */
                memmove(g_frame_buf, g_frame_buf + 1, sizeof(g_frame_buf) - 1);
                g_frame_buf[sizeof(g_frame_buf) - 1] = byte;
            }

            /* check last 4 bytes for a frame */
            if (g_frame_idx >= DYP_FRAME_LEN) {
                size_t base = g_frame_idx - DYP_FRAME_LEN;
                if (validate_and_parse_frame(&g_frame_buf[base], out_cm)) {
                    /* keep tail for next parse */
                    if (g_frame_idx > DYP_FRAME_LEN) {
                        memmove(g_frame_buf, &g_frame_buf[g_frame_idx - DYP_FRAME_LEN], DYP_FRAME_LEN);
                        g_frame_idx = DYP_FRAME_LEN;
                    } else {
                        g_frame_idx = 0;
                    }
                    return 0;
                }
            }
        }
    }
    g_last_timeout = true;
    return -1;
}

static bool platform_last_timeout(void) { return g_last_timeout; }

#elif defined(ARDUINO)
/* ------------------------ Arduino implementation (SoftwareSerial) ------------------------ */

#include <Arduino.h>
#include <SoftwareSerial.h>

static SoftwareSerial *g_sw = NULL;

static int platform_init(int rx_pin, int tx_pin, int baud) {
    if (g_initialized) {
        if (g_sw) {
            g_sw->end();
            g_sw = NULL; /* avoid delete to prevent delete-non-virtual-dtor warning */
        }
        g_initialized = false;
    }
    g_rx_pin = rx_pin;
    g_tx_pin = tx_pin;
    g_baud = (baud == 0) ? DYP_UART_BAUD_DEFAULT : baud;

    g_sw = new SoftwareSerial(g_rx_pin, g_tx_pin);
    if (!g_sw) return DYP_UART_ERROR;
    g_sw->begin(g_baud);

    g_frame_idx = 0;
    g_initialized = true;
    g_last_timeout = false;
    return DYP_UART_OK;
}

static int platform_deinit(void) {
    if (g_sw) {
        g_sw->end();
        /* do not delete g_sw to avoid the -Wdelete-non-virtual-dtor warning in some toolchains.
           This intentionally leaves the small allocation (one-time) — acceptable on microcontrollers
           for typical usage. If you want to avoid any leak, we can use a static SoftwareSerial object
           constructed via placement-new. */
        g_sw = NULL;
    }
    g_initialized = false;
    g_frame_idx = 0;
    return DYP_UART_OK;
}

static int platform_set_baud(int baud) {
    if (baud <= 0) return DYP_UART_ERROR;
    g_baud = baud;
    if (g_sw && g_initialized) {
        g_sw->end();
        g_sw->begin(g_baud);
    }
    return DYP_UART_OK;
}

static int platform_read_distance_cm_float(float *out_cm, unsigned timeout_ms) {
    if (!g_initialized || !g_sw) return -2;
    if (!out_cm) return -2;
    g_last_timeout = false;

    // Clear internal rolling buffer before reading
    g_frame_idx = 0;
    memset(g_frame_buf, 0, sizeof(g_frame_buf));
    
    // Drain SoftwareSerial RX buffer
    unsigned long flush_start = millis();
    while (g_sw->available() > 0 && (millis() - flush_start) < 50) {
        g_sw->read();
    }
    
    unsigned long start = millis();
    unsigned long timeout = (timeout_ms == 0) ? 100u : timeout_ms;

    while ((millis() - start) <= timeout) {
        while (g_sw->available() > 0) {
            int b = g_sw->read();
            if (b < 0) continue;
            /* push into rolling buffer */
            if (g_frame_idx < sizeof(g_frame_buf)) {
                g_frame_buf[g_frame_idx++] = (uint8_t)b;
            } else {
                memmove(g_frame_buf, g_frame_buf + 1, sizeof(g_frame_buf) - 1);
                g_frame_buf[sizeof(g_frame_buf) - 1] = (uint8_t)b;
            }

            if (g_frame_idx >= DYP_FRAME_LEN) {
                size_t base = g_frame_idx - DYP_FRAME_LEN;
                if (validate_and_parse_frame(&g_frame_buf[base], out_cm)) {
                    if (g_frame_idx > DYP_FRAME_LEN) {
                        memmove(g_frame_buf, &g_frame_buf[g_frame_idx - DYP_FRAME_LEN], DYP_FRAME_LEN);
                        g_frame_idx = DYP_FRAME_LEN;
                    } else {
                        g_frame_idx = 0;
                    }
                    return 0;
                }
            }
        }
        delay(2);
    }

    g_last_timeout = true;
    return -1;
}

static bool platform_last_timeout(void) { return g_last_timeout; }

#else
#error "Unsupported platform: define ESP_PLATFORM or ARDUINO"
#endif /* platform branches */


/* ------------------------ Public wrappers (platform-agnostic) ------------------------ */

/* Flush buffer function - MUST come after platform implementations */
int dyp_uart_flush_buffer(void) {
    if (!g_initialized) return DYP_UART_ERROR;
    
    // Clear internal rolling buffer
    g_frame_idx = 0;
    memset(g_frame_buf, 0, sizeof(g_frame_buf));
    
#if defined(ESP_PLATFORM)
    // ESP-IDF: flush UART hardware buffer
    uart_flush(g_uart_num);
    return DYP_UART_OK;
    
#elif defined(ARDUINO)
    // Arduino SoftwareSerial: drain any available bytes
    if (g_sw) {
        unsigned long start = millis();
        // Read and discard for max 50ms
        while (g_sw->available() > 0 && (millis() - start) < 50) {
            g_sw->read();
        }
    }
    return DYP_UART_OK;
    
#else
    return DYP_UART_ERROR;
#endif
}

/* Initialize the sensor */
int dyp_uart_init(int rx_pin, int tx_pin, int baud) {
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    return platform_init(rx_pin, tx_pin, baud);
#else
    (void)rx_pin; (void)tx_pin; (void)baud;
    return DYP_UART_ERROR;
#endif
}

/* Deinitialize */
int dyp_uart_deinit(void) {
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    return platform_deinit();
#else
    return DYP_UART_OK;
#endif
}

/* Set baudrate */
int dyp_uart_set_baud(int baud) {
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    return platform_set_baud(baud);
#else
    (void)baud;
    return DYP_UART_ERROR;
#endif
}

/* ------------------------ Distance reading functions (all units) ------------------------ */

/* Centimeters (float) */
int dyp_uart_read_distance_cm_float(float *out_cm, unsigned timeout_ms) {
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    return platform_read_distance_cm_float(out_cm, timeout_ms);
#else
    (void)out_cm; (void)timeout_ms;
    return -2;
#endif
}

/* Inches (float) */
int dyp_uart_read_distance_inch_float(float *out_inch, unsigned timeout_ms) {
    float cm = 0.0f;
    int r = dyp_uart_read_distance_cm_float(&cm, timeout_ms);
    if (r != 0) return r;
    if (out_inch) *out_inch = cm * DYP_CM_TO_INCH;
    return 0;
}

/* Feet (float) */
int dyp_uart_read_distance_feet_float(float *out_feet, unsigned timeout_ms) {
    float cm = 0.0f;
    int r = dyp_uart_read_distance_cm_float(&cm, timeout_ms);
    if (r != 0) return r;
    if (out_feet) *out_feet = cm * DYP_CM_TO_FEET;
    return 0;
}

/* Meters (float) */
int dyp_uart_read_distance_meter_float(float *out_meter, unsigned timeout_ms) {
    float cm = 0.0f;
    int r = dyp_uart_read_distance_cm_float(&cm, timeout_ms);
    if (r != 0) return r;
    if (out_meter) *out_meter = cm * DYP_CM_TO_METER;
    return 0;
}

int dyp_uart_read_distance_mm_float(float *out_mm, unsigned timeout_ms) {
    float cm = 0.0f;
    int r = dyp_uart_read_distance_cm_float(&cm, timeout_ms);
    if (r != 0) return r;
    if (out_mm) *out_mm = cm * DYP_CM_TO_MM;
    return 0;
}

/* ------------------------ Integer variants ------------------------ */

/* Centimeters (integer) */
long dyp_uart_read_distance_cm_int(unsigned timeout_ms) {
    float cm = 0.0f;
    int r = dyp_uart_read_distance_cm_float(&cm, timeout_ms);
    if (r != 0) return DYP_UART_ERROR;
    return (long)(cm + 0.5f);
}

/* Inches (integer) */
long dyp_uart_read_distance_inch_int(unsigned timeout_ms) {
    float inch = 0.0f;
    int r = dyp_uart_read_distance_inch_float(&inch, timeout_ms);
    if (r != 0) return DYP_UART_ERROR;
    return (long)(inch + 0.5f);
}

/* Feet (integer) */
long dyp_uart_read_distance_feet_int(unsigned timeout_ms) {
    float feet = 0.0f;
    int r = dyp_uart_read_distance_feet_float(&feet, timeout_ms);
    if (r != 0) return DYP_UART_ERROR;
    return (long)(feet + 0.5f);
}

/* Meters (integer) */
long dyp_uart_read_distance_meter_int(unsigned timeout_ms) {
    float meter = 0.0f;
    int r = dyp_uart_read_distance_meter_float(&meter, timeout_ms);
    if (r != 0) return DYP_UART_ERROR;
    return (long)(meter + 0.5f);
}

long dyp_uart_read_distance_mm_int(unsigned timeout_ms) {
    float mm = 0.0f;
    int r = dyp_uart_read_distance_mm_float(&mm, timeout_ms);
    if (r != 0) return DYP_UART_ERROR;
    return (long)(mm + 0.5f);
}

/* last timeout query */
bool dyp_uart_last_timeout(void) {
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    return platform_last_timeout();
#else
    return true;
#endif
}

/* ------------------------ Object detection functions (all units) ------------------------ */

/* Centimeters */
int dyp_uart_object_detected_cm(float threshold_cm, unsigned timeout_ms) {
    float cm = 0.0f;
    int r = dyp_uart_read_distance_cm_float(&cm, timeout_ms);
    if (r != 0) return -1;
    return (cm > 0.0f && cm <= threshold_cm) ? 1 : 0;
}

/* Inches */
int dyp_uart_object_detected_inch(float threshold_inch, unsigned timeout_ms) {
    float inch = 0.0f;
    int r = dyp_uart_read_distance_inch_float(&inch, timeout_ms);
    if (r != 0) return -1;
    return (inch > 0.0f && inch <= threshold_inch) ? 1 : 0;
}

/* Feet */
int dyp_uart_object_detected_feet(float threshold_feet, unsigned timeout_ms) {
    float feet = 0.0f;
    int r = dyp_uart_read_distance_feet_float(&feet, timeout_ms);
    if (r != 0) return -1;
    return (feet > 0.0f && feet <= threshold_feet) ? 1 : 0;
}

/* Meters */
int dyp_uart_object_detected_meter(float threshold_meter, unsigned timeout_ms) {
    float meter = 0.0f;
    int r = dyp_uart_read_distance_meter_float(&meter, timeout_ms);
    if (r != 0) return -1;
    return (meter > 0.0f && meter <= threshold_meter) ? 1 : 0;
}

int dyp_uart_object_detected_mm(float threshold_mm, unsigned timeout_ms) {
    float mm = 0.0f;
    int r = dyp_uart_read_distance_mm_float(&mm, timeout_ms);
    if (r != 0) return -1;
    return (mm > 0.0f && mm <= threshold_mm) ? 1 : 0;
}