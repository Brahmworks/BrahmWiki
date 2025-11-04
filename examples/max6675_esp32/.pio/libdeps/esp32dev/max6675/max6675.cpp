/*
 * max6675.cpp
 *
 * Single implementation file that compiles on:
 *  - Arduino (ARDUINO defined)
 *  - ESP-IDF (ESP_PLATFORM defined)
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
#elif defined(ARDUINO)
  /* Arduino */
  #include <Arduino.h>
  #include <SPI.h>
#else
  #error "Either ESP_PLATFORM (ESP-IDF) or ARDUINO must be defined"
#endif

/* ---------- Internal state ---------- */

static enum {
    MAX6675_MODE_NONE = 0,
    MAX6675_MODE_HW,
    MAX6675_MODE_SW
} s_mode = MAX6675_MODE_NONE;

static int s_cs = -1;
static int s_sck = -1;   /* used for SW mode */
static int s_miso = -1;  /* used for SW mode */
static uint32_t s_clock_hz = 400000; /* default */

#if defined(ESP_PLATFORM)
static spi_device_handle_t s_spi_dev = NULL;
static bool s_esp_bus_initialized = false;
#elif defined(ARDUINO)
static SPISettings s_spi_settings(400000, MSBFIRST, SPI_MODE0);
#endif

/* ---------- Helpers ---------- */

static unsigned clamp_decimals(unsigned d) {
    if (d > 2) return 2;
    return d;
}

static float raw_to_celsius_or_nan(uint16_t raw) {
    if (raw & 0x4) {
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
        if (!s_spi_dev) return -3;
        esp_err_t ret;
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = 16; /* bits */
        t.rxlength = 16;
        t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
        /* tx_data defaults to zeros; perform a polling transmit */
        ret = spi_device_polling_transmit(s_spi_dev, &t);
        if (ret != ESP_OK) {
            return -4;
        }
        /* rx data available in t.rx_data (first two bytes) */
        uint8_t *rbytes = t.rx_data;
        uint16_t rx = (uint16_t)rbytes[0] << 8 | (uint16_t)rbytes[1];
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
        s_esp_bus_initialized = true;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        /* bus already initialized externally; treat as OK */
        s_esp_bus_initialized = true;
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


    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        s_spi_dev = NULL;
        return -10;
    }
    s_mode = MAX6675_MODE_HW;
    return 0;

#elif defined(ARDUINO)

    pinMode(s_cs, OUTPUT);
    digitalWrite(s_cs, HIGH);
    s_spi_settings = SPISettings(s_clock_hz, MSBFIRST, SPI_MODE0);
    SPI.begin();
    s_mode = MAX6675_MODE_HW;
    return 0;

#endif
}

int max6675_init_sw(int cs_pin, int sck_pin, int miso_pin) {
    max6675_deinit();
    s_cs = cs_pin;
    s_sck = sck_pin;
    s_miso = miso_pin;

#if defined(ESP_PLATFORM)
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
    return 0;

#elif defined(ARDUINO)

    pinMode(s_cs, OUTPUT);
    pinMode(s_sck, OUTPUT);
    pinMode(s_miso, INPUT);
    digitalWrite(s_cs, HIGH);
    digitalWrite(s_sck, LOW);

    s_mode = MAX6675_MODE_SW;
    return 0;

#endif
}

void max6675_deinit(void) {
#if defined(ESP_PLATFORM)
    if (s_spi_dev) {
        spi_bus_remove_device(s_spi_dev);
        s_spi_dev = NULL;
    }
    /* We do not call spi_bus_free here to avoid interfering with other components */
    s_esp_bus_initialized = false;
    if (s_cs >= 0) {
        gpio_set_level((gpio_num_t)s_cs, 0);
    }
#elif defined(ARDUINO)
    if (s_cs >= 0) {
        digitalWrite(s_cs, LOW);
    }
#endif

    s_mode = MAX6675_MODE_NONE;
    s_cs = s_sck = s_miso = -1;
}

int max6675_read_raw(uint16_t *out_raw) {
    return read16_internal(out_raw);
}

float max6675_read_celsius_float(unsigned decimals) {
    uint16_t raw;
    int ret = read16_internal(&raw);
    if (ret != 0) return NAN;
    float c = raw_to_celsius_or_nan(raw);
    if (isnan(c)) return NAN;
    unsigned d = clamp_decimals(decimals);
    if (d == 0) return roundf(c);
    if (d == 1) return roundf(c * 10.0f) / 10.0f;
    return roundf(c * 100.0f) / 100.0f;
}

float max6675_read_fahrenheit_float(unsigned decimals) {
    float c = max6675_read_celsius_float(3);
    if (isnan(c)) return NAN;
    float f = c * 9.0f / 5.0f + 32.0f;
    unsigned d = clamp_decimals(decimals);
    if (d == 0) return roundf(f);
    if (d == 1) return roundf(f * 10.0f) / 10.0f;
    return roundf(f * 100.0f) / 100.0f;
}

long max6675_read_celsius_int(unsigned decimals) {
    uint16_t raw;
    int ret = read16_internal(&raw);
    if (ret != 0) return MAX6675_ERROR;
    if (raw & 0x4) return MAX6675_ERROR; /* thermocouple open */
    float c = raw_to_celsius_or_nan(raw);
    if (isnan(c)) return MAX6675_ERROR;
    unsigned d = clamp_decimals(decimals);
    if (d == 0) {
        return (long) (roundf(c));
    } else if (d == 1) {
        return (long) (roundf(c * 10.0f));
    } else {
        return (long) (roundf(c * 100.0f));
    }
}

long max6675_read_fahrenheit_int(unsigned decimals) {
    long c_int = max6675_read_celsius_int(2); /* scaled x100 */
    if (c_int == MAX6675_ERROR) return MAX6675_ERROR;
    unsigned d = clamp_decimals(decimals);
    float c = ((float)c_int) / 100.0f;
    float f = c * 9.0f / 5.0f + 32.0f;
    if (d == 0) return (long) roundf(f);
    if (d == 1) return (long) roundf(f * 10.0f);
    return (long) roundf(f * 100.0f);
}

bool max6675_is_open(void) {
    uint16_t raw;
    int ret = read16_internal(&raw);
    if (ret != 0) return true;
    return (raw & 0x4) != 0;
}
