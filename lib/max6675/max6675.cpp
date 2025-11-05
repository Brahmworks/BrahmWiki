/*
 * max6675.cpp
 *
 * Single implementation file that compiles on:
 * - Arduino (ARDUINO defined)
 * - ESP-IDF (ESP_PLATFORM defined)
 *
 * - ESP-IDF: This version is THREAD-SAFE. It uses a recursive mutex
 * and renamed static variables (s_max6675_*) to prevent conflicts.
 *
 * See max6675.h for API.
 */

#include "max6675.h"
#include <math.h>
#include <string.h>

#if defined(ESP_PLATFORM)
  /* ESP-IDF */
  #include "driver/spi_master.h"
  #include "driver/gpio.h"
  #include "esp_err.h"
  #include "esp_rom_sys.h" /* for esp_rom_delay_us */
  // --- THREAD-SAFETY ADDITIONS ---
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
  // --- END THREAD-SAFETY ---
#elif defined(ARDUINO)
  /* Arduino */
  #include <Arduino.h>
  #include <SPI.h>
#else
  #error "Either ESP_PLATFORM (ESP-IDF) or ARDUINO must be defined"
#endif

/* ------------------ Thread-Safety Mutex (ESP-IDF Only) ------------------ */

#if defined(ESP_PLATFORM)
    static SemaphoreHandle_t s_max6675_lib_mutex = NULL;

    // Helper function to create the mutex on first use
    static void max6675_init_mutex(void) {
        if (s_max6675_lib_mutex == NULL) {
            s_max6675_lib_mutex = xSemaphoreCreateRecursiveMutex();
        }
    }

    #define MAX6675_MUTEX_TAKE() do { \
        max6675_init_mutex(); /* Create if it doesn't exist */ \
        if (s_max6675_lib_mutex) { \
            xSemaphoreTakeRecursive(s_max6675_lib_mutex, portMAX_DELAY); \
        } \
    } while(0)
    
    #define MAX6675_MUTEX_GIVE() do { \
        if (s_max6675_lib_mutex) { \
            xSemaphoreGiveRecursive(s_max6675_lib_mutex); \
        } \
    } while(0)
#else
    // On Arduino, these macros do nothing.
    #define MAX6675_MUTEX_TAKE()
    #define MAX6675_MUTEX_GIVE()
#endif


/* Fault bit (D2) indicates an open thermocouple */
#define MAX6675_FAULT_BIT 0x04

/* ---------- Internal state ---------- */

static enum {
    MAX6675_MODE_NONE = 0,
    MAX6675_MODE_HW,
    MAX6675_MODE_SW
} s_mode = MAX6675_MODE_NONE;

static int s_cs = -1;
static int s_sck = -1;  /* used for SW mode */
static int s_miso = -1; /* used for SW mode */
static uint32_t s_clock_hz = 400000; /* default */

#if defined(ESP_PLATFORM)
// --- RENAMED STATIC VARIABLES ---
static spi_device_handle_t s_max6675_spi_dev = NULL;
static bool s_max6675_bus_initialized = false;
#elif defined(ARDUINO)
static SPISettings s_spi_settings(400000, MSBFIRST, SPI_MODE0);
#endif

/* ---------- Helpers ---------- */

static unsigned clamp_decimals(unsigned d) {
    if (d > 2) return 2;
    return d;
}

static float raw_to_celsius_or_nan(uint16_t raw) {
    /* Check D2 fault bit for open thermocouple */
    if (raw & MAX6675_FAULT_BIT) {
        return NAN;
    }
    uint16_t temp12 = raw >> 3;
    return ((float)temp12) * 0.25f;
}

/* Read raw 16-bit from hardware or SW. Internal function. */
static int read16_internal(uint16_t *out_raw) {
    if (!out_raw) return -1;

    if (s_mode == MAX6675_MODE_NONE) return -2;

#if defined(ESP_PLATFORM)

    if (s_mode == MAX6675_MODE_HW) {
        if (!s_max6675_spi_dev) return -3; // <-- Use renamed variable
        esp_err_t ret;
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        
        /* Use stack-allocated buffers per user request (no heap, no USE_RXDATA) */
        uint8_t tx_buf[2] = {0x00, 0x00};
        uint8_t rx_buf[2];

        t.length = 16; /* 16 bits */
        t.tx_buffer = tx_buf;
        t.rx_buffer = rx_buf;

        ret = spi_device_polling_transmit(s_max6675_spi_dev, &t); // <-- Use renamed variable
        if (ret != ESP_OK) {
            return -4;
        }
        
        /* Combine received bytes (MSB first) */
        uint16_t rx = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];
        *out_raw = rx;
        return 0;

    } else { /* SW bitbanged on ESP */
        if (s_cs < 0 || s_sck < 0 || s_miso < 0) return -5;
        gpio_set_level((gpio_num_t)s_cs, 0);
        esp_rom_delay_us(1);
        uint16_t val = 0;
        for (int i = 15; i >= 0; --i) {
            gpio_set_level((gpio_num_t)s_sck, 0);
            esp_rom_delay_us(1);
            int bit = gpio_get_level((gpio_num_t)s_miso);
            val |= ((uint16_t)(bit & 1) << i);
            gpio_set_level((gpio_num_t)s_sck, 1);
            esp_rom_delay_us(1);
        }
        gpio_set_level((gpio_num_t)s_cs, 1);
        *out_raw = val;
        return 0;
    }

#elif defined(ARDUINO)
    // ... (Arduino code is unchanged) ...
    if (s_mode == MAX6675_MODE_HW) {
        if (s_cs < 0) return -6;
        digitalWrite(s_cs, LOW);
        SPI.beginTransaction(s_spi_settings);
        delayMicroseconds(1);
        uint16_t val = SPI.transfer16(0x0000); /* MSB-first */
        SPI.endTransaction();
        digitalWrite(s_cs, HIGH);
        *out_raw = val;
        return 0;
    } else { /* SW: bit-banged */
        if (s_cs < 0 || s_sck < 0 || s_miso < 0) return -7;
        digitalWrite(s_cs, LOW);
        delayMicroseconds(1);
        uint16_t val = 0;
        for (int i = 15; i >= 0; --i) {
            digitalWrite(s_sck, LOW);
            delayMicroseconds(1);
            int bit = digitalRead(s_miso);
            val |= ((uint16_t)(bit & 1) << i);
            digitalWrite(s_sck, HIGH);
            delayMicroseconds(1);
        }
        digitalWrite(s_cs, HIGH);
        *out_raw = val;
        return 0;
    }

#endif

    return -99;
}

/* ---------- Public API ---------- */

int max6675_init_hw(int cs_pin, uint32_t clock_hz) {
    MAX6675_MUTEX_TAKE();
    
    max6675_deinit();
    s_cs = cs_pin;
    s_clock_hz = clock_hz ? clock_hz : 400000;

#if defined(ESP_PLATFORM)
    esp_err_t ret;

    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    /* default/common pins (will be used only if bus isn't initialized already) */
    buscfg.miso_io_num = 19;
    buscfg.mosi_io_num = 23;
    buscfg.sclk_io_num = 18;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 16;

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_OK) {
        s_max6675_bus_initialized = true; // <-- Use renamed variable
    } else if (ret == ESP_ERR_INVALID_STATE) {
        /* bus already initialized externally; treat as OK */
        s_max6675_bus_initialized = true; // <-- Use renamed variable
    } else {
        /* continue and attempt to add device - spi_bus_add_device may still fail */
    }


    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = s_clock_hz;
    devcfg.mode = 0;
    devcfg.spics_io_num = s_cs;
    devcfg.queue_size = 1;
    /* Use full-duplex device (no HALFDUPLEX). We only read MISO and send zeros on MOSI. */
    devcfg.flags = 0;


    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &s_max6675_spi_dev); // <-- Use renamed variable
    if (ret != ESP_OK) {
        s_max6675_spi_dev = NULL; // <-- Use renamed variable
        MAX6675_MUTEX_GIVE();
        return -10;
    }
    s_mode = MAX6675_MODE_HW;
    MAX6675_MUTEX_GIVE();
    return 0;

#elif defined(ARDUINO)
    // ... (Arduino code is unchanged) ...
    pinMode(s_cs, OUTPUT);
    digitalWrite(s_cs, HIGH);
    s_spi_settings = SPISettings(s_clock_hz, MSBFIRST, SPI_MODE0);
    SPI.begin();
    s_mode = MAX6675_MODE_HW;
    MAX6675_MUTEX_GIVE();
    return 0;

#endif
}

int max6675_init_sw(int cs_pin, int sck_pin, int miso_pin) {
    MAX6675_MUTEX_TAKE();
    
    max6675_deinit();
    s_cs = cs_pin;
    s_sck = sck_pin;
    s_miso = miso_pin;

#if defined(ESP_PLATFORM)
    // ... (ESP32 code is unchanged) ...
    /* Configure GPIOs for SW SPI */
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << s_cs) | (1ULL << s_sck);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    /* MISO as input */
    gpio_config_t in_conf;
    memset(&in_conf, 0, sizeof(in_conf));
    in_conf.intr_type = GPIO_INTR_DISABLE;
    in_conf.mode = GPIO_MODE_INPUT;
    in_conf.pin_bit_mask = (1ULL << s_miso);
    in_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    in_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&in_conf);

    gpio_set_level((gpio_num_t)s_cs, 1);
    gpio_set_level((gpio_num_t)s_sck, 0);

    s_mode = MAX6675_MODE_SW;
    MAX6675_MUTEX_GIVE();
    return 0;

#elif defined(ARDUINO)
    // ... (Arduino code is unchanged) ...
    pinMode(s_cs, OUTPUT);
    pinMode(s_sck, OUTPUT);
    pinMode(s_miso, INPUT);
    digitalWrite(s_cs, HIGH);
    digitalWrite(s_sck, LOW);

    s_mode = MAX6675_MODE_SW;
    MAX6675_MUTEX_GIVE();
    return 0;

#endif
}

void max6675_deinit(void) {
    MAX6675_MUTEX_TAKE();
    
#if defined(ESP_PLATFORM)
    if (s_max6675_spi_dev) { // <-- Use renamed variable
        spi_bus_remove_device(s_max6675_spi_dev); // <-- Use renamed variable
        s_max6675_spi_dev = NULL; // <-- Use renamed variable
    }
    /* We do not call spi_bus_free here to avoid interfering with other components */
    s_max6675_bus_initialized = false; // <-- Use renamed variable
    if (s_cs >= 0) {
        if (s_mode == MAX6675_MODE_SW) {
             gpio_reset_pin((gpio_num_t)s_cs);
        }
    }
#elif defined(ARDUINO)
    // ... (Arduino code is unchanged) ...
    if (s_cs >= 0) {
        digitalWrite(s_cs, LOW);
    }
#endif

    s_mode = MAX6675_MODE_NONE;
    s_cs = s_sck = s_miso = -1;
    
    MAX6675_MUTEX_GIVE();
}

int max6675_read_raw(uint16_t *out_raw) {
    MAX6675_MUTEX_TAKE();
    int ret = read16_internal(out_raw);
    MAX6675_MUTEX_GIVE();
    return ret;
}

float max6675_read_celsius_float(unsigned decimals) {
    MAX6675_MUTEX_TAKE();
    uint16_t raw;
    int ret = read16_internal(&raw);
    if (ret != 0) {
        MAX6675_MUTEX_GIVE();
        return NAN;
    }
    float c = raw_to_celsius_or_nan(raw);
    if (isnan(c)) {
        MAX6675_MUTEX_GIVE();
        return NAN;
    }
    unsigned d = clamp_decimals(decimals);
    float val;
    if (d == 0) val = roundf(c);
    else if (d == 1) val = roundf(c * 10.0f) / 10.0f;
    else val = roundf(c * 100.0f) / 100.0f;
    
    MAX6675_MUTEX_GIVE();
    return val;
}

float max6675_read_fahrenheit_float(unsigned decimals) {
    MAX6675_MUTEX_TAKE();
    float c = max6675_read_celsius_float(3);
    if (isnan(c)) {
        MAX6675_MUTEX_GIVE();
        return NAN;
    }
    float f = c * 9.0f / 5.0f + 32.0f;
    unsigned d = clamp_decimals(decimals);
    float val;
    if (d == 0) val = roundf(f);
    else if (d == 1) val = roundf(f * 10.0f) / 10.0f;
    else val = roundf(f * 100.0f) / 100.0f;
    
    MAX6675_MUTEX_GIVE();
    return val;
}

long max6675_read_celsius_int(unsigned decimals) {
    MAX6675_MUTEX_TAKE();
    uint16_t raw;
    int ret = read16_internal(&raw);
    if (ret != 0) {
        MAX6675_MUTEX_GIVE();
        return MAX6675_ERROR;
    }
    if (raw & MAX6675_FAULT_BIT) {
        MAX6675_MUTEX_GIVE();
        return MAX6675_ERROR; /* thermocouple open */
    }
    float c = raw_to_celsius_or_nan(raw);
    if (isnan(c)) {
        MAX6675_MUTEX_GIVE();
        return MAX6675_ERROR;
    }
    unsigned d = clamp_decimals(decimals);
    long val;
    if (d == 0) val = (long) (roundf(c));
    else if (d == 1) val = (long) (roundf(c * 10.0f));
    else val = (long) (roundf(c * 100.0f));
    
    MAX6675_MUTEX_GIVE();
    return val;
}

long max6675_read_fahrenheit_int(unsigned decimals) {
    MAX6675_MUTEX_TAKE();
    long c_int = max6675_read_celsius_int(2); /* scaled x100 */
    if (c_int == MAX6675_ERROR) {
        MAX6675_MUTEX_GIVE();
        return MAX6675_ERROR;
    }
    unsigned d = clamp_decimals(decimals);
    float c = ((float)c_int) / 100.0f;
    float f = c * 9.0f / 5.0f + 32.0f;
    long val;
    if (d == 0) val = (long) roundf(f);
    else if (d == 1) val = (long) roundf(f * 10.0f);
    else val = (long) roundf(f * 100.0f);
    
    MAX6675_MUTEX_GIVE();
    return val;
}

bool max6675_is_open(void) {
    MAX6675_MUTEX_TAKE();
    uint16_t raw;
    int ret = read16_internal(&raw);
    if (ret != 0) {
        MAX6675_MUTEX_GIVE();
        return true;
    }
    bool is_open = (raw & MAX6675_FAULT_BIT) != 0;
    MAX6675_MUTEX_GIVE();
    return is_open;
}