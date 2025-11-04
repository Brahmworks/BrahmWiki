/*
 * max31865.cpp
 *
 * Cross-platform MAX31865 library (Arduino UNO/MEGA and ESP-IDF ESP32)
 * - Single-device global state (simpler). To extend to multiple devices,
 * store the state struct per device and change APIs to accept a handle/pointer.
 *
 * - Use C-style functions only. Header uses extern "C" so main.c can link.
 *
 * CAUTION (copy/paste to README):
 * - ESP-IDF: avoid SPI_DEVICE_HALFDUPLEX + DMA + MOSI+MISO transactions.
 * This library uses full-duplex device flags and robust transfer paths.
 * - If your toolchain returns zeros, set MAX31865_ESP_SAFE_SPI to 1 below.
 */

/* ------------------ Configuration toggles ------------------ */

/* If set to 1, force the safe DMA-capable heap-transfer path on ESP-IDF.
   If 0 (default), the library uses the small-transfer fast path for <=3 bytes
   and the heap path for larger transfers. Change if you see spurious zero reads. */
#ifndef MAX31865_ESP_SAFE_SPI
#define MAX31865_ESP_SAFE_SPI 0
#endif

/* ------------------ Includes & platform branching ------------------ */

#include "max31865.h"
#include <math.h>
#include <string.h> /* memset */

#if defined(ESP_PLATFORM)
  #include "driver/spi_master.h"
  #include "driver/gpio.h"
  #include "esp_err.h"
  #include "esp_rom_sys.h" /* esp_rom_delay_us */
  #include "esp_heap_caps.h"
  #include "esp_log.h"
#elif defined(ARDUINO)
  #include <Arduino.h>
  #include <SPI.h>
#else
  #error "Either ESP_PLATFORM or ARDUINO must be defined"
#endif

/* ------------------ MAX31865 register definitions ------------------ */
/* Per datasheet: read addresses are 0x0X, write uses 0x8X (OR 0x80) */
#define MAX31865_REG_CONFIG     0x00
#define MAX31865_REG_RTD_MSB    0x01
#define MAX31865_REG_RTD_LSB    0x02
#define MAX31865_REG_HFT_MSB    0x03
#define MAX31865_REG_HFT_LSB    0x04
#define MAX31865_REG_LFT_MSB    0x05
#define MAX31865_REG_LFT_LSB    0x06
#define MAX31865_REG_FAULT_STAT 0x07

/* Config register bit masks */
#define MAX31865_CONFIG_BIAS      (1 << 7)
#define MAX31865_CONFIG_AUTO      (1 << 6)
#define MAX31865_CONFIG_ONE_SHOT  (1 << 5)
#define MAX31865_CONFIG_3WIRE     (1 << 4)
#define MAX31865_CONFIG_FAULT1    (1 << 3)
#define MAX31865_CONFIG_FAULT0    (1 << 2)
#define MAX31865_CONFIG_FAULT_CLR (1 << 1)
#define MAX31865_CONFIG_FILTER50  (1 << 0) /* 1=50Hz, 0=60Hz */

/* Fault bits (in fault status register) - use as-is in docs */
#define MAX31865_FAULT_HIGHTHRESH  (1 << 7)
#define MAX31865_FAULT_LOWTHRESH   (1 << 6)
#define MAX31865_FAULT_REFINLOW    (1 << 5)
#define MAX31865_FAULT_REFINHIGH   (1 << 4)
#define MAX31865_FAULT_RTDINLOW    (1 << 3)
#define MAX31865_FAULT_OVUV        (1 << 2)

/* ------------------ Callendar-Van Dusen coefficients for platinum (ITS-90) ----
 * R(T) = R0 * (1 + A*T + B*T^2) for T >= 0
 * For T < 0: R(T) = R0 * (1 + A*T + B*T^2 + C*(T-100)*T^3)
 *
 * Coefficients (commonly used):
 * A = 3.9083e-3
 * B = -5.775e-7
 * C = -4.183e-12
 *
 * For T >= 0 you can solve the quadratic analytically:
 * R/R0 = 1 + A*T + B*T^2
 * => B*T^2 + A*T + (1 - R/R0) = 0
 * Use the positive root:
 * T = (-A + sqrt(A^2 - 4*B*(1 - R/R0))) / (2*B)
 *
 * For T < 0 use Newton-Raphson on f(T) = Rcalc(T) - Rmeas
 * where Rcalc uses the full Callendar-Van Dusen equation including C term.
 * Start with -50C guess or quadratic result and iterate.
 * ------------------------------------------------------------------------- */
static const double CV_A = 3.9083e-3;
static const double CV_B = -5.775e-7;
static const double CV_C = -4.183e-12;

/* ------------------ Internal state (single-device) ------------------ */

static enum {
    MAX31865_MODE_NONE = 0,
    MAX31865_MODE_HW,
    MAX31865_MODE_SW
} s_mode = MAX31865_MODE_NONE;

/* pins */
static int s_cs = -1;
static int s_sck = -1;
static int s_mosi = -1;
static int s_miso = -1;
static uint32_t s_clock_hz = 1000000; /* default 1MHz safe */

/* rtd and ref values & wire config */
static float s_rtd_nominal = 100.0f;  /* PT100 default */
static float s_ref_resistor = 430.0f; /* example breakout uses 430 ohm for PT100 R0=100 */
static int s_wire_count = MAX31865_WIRES_2;

/* last fault cached */
static uint8_t s_last_fault = 0;

/* debug verbosity */
static int s_debug = 0;

/* --- Added from new snippet --- */
static float s_last_good_temp_c = NAN;
static const float SPIKE_DELTA_THRESHOLD_C = 10.0f; // tuneable
static const uint32_t SPIKE_RETRY_MS = 50;
/* --- End of added variables --- */


#if defined(ESP_PLATFORM)
static spi_device_handle_t s_spi_dev = NULL;
static bool s_esp_bus_initialized = false;
static const char *TAG = "max31865";
#endif

/* ------------------ Utility: clamp decimals ------------------ */
static unsigned clamp_decimals(unsigned d) {
    if (d > 2) return 2;
    return d;
}

/* --- Added helper for cross-platform delay --- */
/* Platform-specific delay (milliseconds) */
static void platform_delay_ms(uint32_t ms) {
#if defined(ESP_PLATFORM)
    esp_rom_delay_us(ms * 1000UL);
#elif defined(ARDUINO)
    delay(ms);
#endif
}
/* --- End of added helper --- */


/* ------------------ Low-level SPI helpers (platform-specific) ------------- */

/* Write register (single byte) */
static int spi_write_register(uint8_t regaddr, uint8_t value) {
    uint8_t txbuf[2];
    txbuf[0] = (uint8_t)(regaddr | 0x80); /* write address per datasheet (0x8X) */
    txbuf[1] = value;

#if defined(ESP_PLATFORM)
    if (s_mode != MAX31865_MODE_HW) return -1;
    if (!s_spi_dev) return -2;

    /* Small-transfer fast path: <=2 bytes -> use rx/txdata if available */
#if MAX31865_ESP_SAFE_SPI == 0
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 2 * 8;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = txbuf[0];
    t.tx_data[1] = txbuf[1];
    esp_err_t ret = spi_device_polling_transmit(s_spi_dev, &t);
    if (ret != ESP_OK) return -3;
    return 0;
#else
    /* Safe heap path */
    uint8_t *tx = (uint8_t*)heap_caps_malloc(2, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!tx) return -4;
    tx[0] = txbuf[0];
    tx[1] = txbuf[1];
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 2 * 8;
    t.tx_buffer = tx;
    esp_err_t ret = spi_device_transmit(s_spi_dev, &t);
    heap_caps_free(tx);
    if (ret != ESP_OK) return -5;
    return 0;
#endif

#elif defined(ARDUINO)

    if (s_mode != MAX31865_MODE_HW) return -1;
    digitalWrite(s_cs, LOW);
    SPI.beginTransaction(SPISettings(s_clock_hz, MSBFIRST, SPI_MODE1));
    SPI.transfer(txbuf[0]);
    SPI.transfer(txbuf[1]);
    SPI.endTransaction();
    digitalWrite(s_cs, HIGH);
    return 0;

#endif
}

/* Read multiple registers: read len bytes starting at regaddr into out (len >= 1)
 * Implementation: send regaddr & 0x7F (read) then read len bytes.
 */
static int spi_read_registers(uint8_t regaddr, uint8_t *out, size_t len) {
    if (!out || len == 0) return -1;

#if defined(ESP_PLATFORM)
    if (s_mode != MAX31865_MODE_HW) return -2;
    if (!s_spi_dev) return -3;

    /* If len+1 <= 3 use small-transfer polling path (address + ≤2 data bytes)
       using SPI_TRANS_USE_RXDATA / TXDATA. Else use heap buffers for DMA safety. */
    size_t total = 1 + len;

#if MAX31865_ESP_SAFE_SPI == 0
    if (total <= 3) {
        /* Use internal rx_data */
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = total * 8;
        t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
        t.tx_data[0] = (uint8_t)(regaddr & 0x7F); /* read address */
        /* remaining tx_data bytes default to 0 (dummy) */
        esp_err_t ret = spi_device_polling_transmit(s_spi_dev, &t);
        if (ret != ESP_OK) return -4;
        /* rx_data[0] is the shifted response to first tx (usually dummy), rx_data[1..] contain read bytes */
        /* For small transfers, the rx positions are: rx_data[1]..rx_data[total-1] */
        for (size_t i = 0; i < len; ++i) {
            out[i] = t.rx_data[i + 1];
        }
        return 0;
    }
#endif

    /* Safe heap path for larger transfers or when forced */
    {
        uint8_t *tx = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        uint8_t *rx = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (!tx || !rx) {
            if (tx) heap_caps_free(tx);
            if (rx) heap_caps_free(rx);
            return -5;
        }
        tx[0] = (uint8_t)(regaddr & 0x7F);
        memset(tx + 1, 0, len);
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        t.length = total * 8;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        esp_err_t ret = spi_device_transmit(s_spi_dev, &t);
        if (ret != ESP_OK) {
            heap_caps_free(tx);
            heap_caps_free(rx);
            return -6;
        }
        memcpy(out, rx + 1, len);
        heap_caps_free(tx);
        heap_caps_free(rx);
        return 0;
    }

#elif defined(ARDUINO)

    if (s_mode != MAX31865_MODE_HW) return -2;
    digitalWrite(s_cs, LOW);
    SPI.beginTransaction(SPISettings(s_clock_hz, MSBFIRST, SPI_MODE1));
    SPI.transfer((uint8_t)(regaddr & 0x7F));
    for (size_t i = 0; i < len; ++i) {
        out[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    digitalWrite(s_cs, HIGH);
    return 0;

#endif
}

/* ------------------ Software (bit-banged) SPI helpers ------------------ */
/* Uses manual CS toggling and clocking MSB-first. s_mosi is master->slave, s_miso slave->master.
 * NOTE: timing uses microsecond delays; adjust if needed.
 */
static void sw_spi_cs_low(void) {
#if defined(ESP_PLATFORM)
    gpio_set_level((gpio_num_t)s_cs, 0);
    esp_rom_delay_us(1);
#elif defined(ARDUINO)
    digitalWrite(s_cs, LOW);
    delayMicroseconds(1);
#endif
}
static void sw_spi_cs_high(void) {
#if defined(ESP_PLATFORM)
    gpio_set_level((gpio_num_t)s_cs, 1);
    esp_rom_delay_us(1);
#elif defined(ARDUINO)
    digitalWrite(s_cs, HIGH);
    delayMicroseconds(1);
#endif
}

static void sw_spi_clock_pulse_low(void) {
#if defined(ESP_PLATFORM)
    gpio_set_level((gpio_num_t)s_sck, 0);
    esp_rom_delay_us(1);
#elif defined(ARDUINO)
    digitalWrite(s_sck, LOW);
    delayMicroseconds(1);
#endif
}
static void sw_spi_clock_pulse_high(void) {
#if defined(ESP_PLATFORM)
    gpio_set_level((gpio_num_t)s_sck, 1);
    esp_rom_delay_us(1);
#elif defined(ARDUINO)
    digitalWrite(s_sck, HIGH);
    delayMicroseconds(1);
#endif
}

/* SW write/read arrays */
static int sw_spi_write_then_read(uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen) {
    if (s_mode != MAX31865_MODE_SW) return -1;
    if (s_cs < 0 || s_sck < 0 || s_mosi < 0 || s_miso < 0) return -2;
    sw_spi_cs_low();

    /* send tx bytes MSB-first */
    for (size_t i = 0; i < txlen; ++i) {
        uint8_t b = tx[i];
        for (int bit = 7; bit >= 0; --bit) {
            int outb = (b >> bit) & 1;
#if defined(ESP_PLATFORM)
            gpio_set_level((gpio_num_t)s_mosi, outb);
#else
            digitalWrite(s_mosi, outb);
#endif
            /* clock rising edge -> sample on MISO for mode1: CPOL=0 CPHA=1, data sampled on second edge */
            sw_spi_clock_pulse_low();
            sw_spi_clock_pulse_high();
        }
    }

    /* read dummy bytes if requested (send zeros while reading) */
    for (size_t i = 0; i < rxlen; ++i) {
        uint8_t b = 0;
        for (int bit = 7; bit >= 0; --bit) {
            /* MOSI zeros */
#if defined(ESP_PLATFORM)
            gpio_set_level((gpio_num_t)s_mosi, 0);
#else
            digitalWrite(s_mosi, LOW);
#endif
            sw_spi_clock_pulse_low();
            /* sample MISO on rising edge (mode1) */
#if defined(ESP_PLATFORM)
            int inb = gpio_get_level((gpio_num_t)s_miso);
#else
            int inb = digitalRead(s_miso);
#endif
            b |= (uint8_t)(inb & 1) << bit;
            sw_spi_clock_pulse_high();
        }
        rx[i] = b;
    }

    sw_spi_cs_high();
    return 0;
}

/* Convenience wrappers for SW register r/w */
static int sw_write_register(uint8_t regaddr, uint8_t value) {
    uint8_t tx[2];
    tx[0] = (uint8_t)(regaddr | 0x80);
    tx[1] = value;
    return sw_spi_write_then_read(tx, 2, NULL, 0);
}

static int sw_read_registers(uint8_t regaddr, uint8_t *out, size_t len) {
    uint8_t tx = (uint8_t)(regaddr & 0x7F);
    return sw_spi_write_then_read(&tx, 1, out, len);
}

/* ------------------ Register utilities ------------------ */

static int write_config(uint8_t cfg) {
    if (s_mode == MAX31865_MODE_SW) return sw_write_register(MAX31865_REG_CONFIG, cfg);
    return spi_write_register(MAX31865_REG_CONFIG, cfg);
}

static int read_config(uint8_t *out) {
    if (!out) return -1;
    if (s_mode == MAX31865_MODE_SW) {
        return sw_read_registers(MAX31865_REG_CONFIG, out, 1);
    }
    return spi_read_registers(MAX31865_REG_CONFIG, out, 1);
}

/* ------------------ Public API implementation ------------------ */

int max31865_init_hw(int cs_pin, uint32_t clock_hz) {
    /* Deinit any previous */
    max31865_deinit();

    s_cs = cs_pin;
    if (clock_hz) s_clock_hz = clock_hz;

#if defined(ESP_PLATFORM)
    /* Try to init HSPI host bus with common defaults; if bus already exists
       spi_bus_initialize returns ESP_ERR_INVALID_STATE which we treat as OK. */
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = 19;
    buscfg.mosi_io_num = 23;
    buscfg.sclk_io_num = 18;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 16;

    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_OK) {
        s_esp_bus_initialized = true;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        s_esp_bus_initialized = true;
    } else {
        /* Not fatal; continue and try to add device (may fail) */
        s_esp_bus_initialized = false;
    }

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = s_clock_hz;
    devcfg.mode = 1; /* MAX31865 uses SPI mode 1 (CPOL=0, CPHA=1) */
    devcfg.spics_io_num = s_cs;
    devcfg.queue_size = 1;
    devcfg.flags = 0; /* full-duplex recommended (see caution) */

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        s_spi_dev = NULL;
        return -10;
    }

    s_mode = MAX31865_MODE_HW;
    return 0;

#elif defined(ARDUINO)

    /* Configure pin and SPI */
    pinMode(s_cs, OUTPUT);
    digitalWrite(s_cs, HIGH);
    SPI.begin();
    /* config: MSBFIRST, SPI_MODE1 */
    s_clock_hz = s_clock_hz ? s_clock_hz : 1000000;
    s_mode = MAX31865_MODE_HW;
    return 0;

#endif
}

int max31865_init_sw(int cs_pin, int sck_pin, int mosi_pin, int miso_pin) {
    max31865_deinit();
    s_cs = cs_pin;
    s_sck = sck_pin;
    s_mosi = mosi_pin;
    s_miso = miso_pin;

#if defined(ESP_PLATFORM)
    /* configure GPIOs */
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << s_cs) | (1ULL << s_sck) | (1ULL << s_mosi);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

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
    gpio_set_level((gpio_num_t)s_mosi, 0);

    s_mode = MAX31865_MODE_SW;
    return 0;

#elif defined(ARDUINO)

    pinMode(s_cs, OUTPUT);
    pinMode(s_sck, OUTPUT);
    pinMode(s_mosi, OUTPUT);
    pinMode(s_miso, INPUT);
    digitalWrite(s_cs, HIGH);
    digitalWrite(s_sck, LOW);
    digitalWrite(s_mosi, LOW);

    s_mode = MAX31865_MODE_SW;
    return 0;

#endif
}

void max31865_deinit(void) {
#if defined(ESP_PLATFORM)
    if (s_spi_dev) {
        spi_bus_remove_device(s_spi_dev);
        s_spi_dev = NULL;
    }
    /* Do not call spi_bus_free to avoid interfering with other components */
    s_esp_bus_initialized = false;
#elif defined(ARDUINO)
    /* Leave SPI as shared. */
#endif

    s_mode = MAX31865_MODE_NONE;
    s_cs = s_sck = s_mosi = s_miso = -1;
}

/* Set RTD nominal and ref resistor */
void max31865_set_rtd_nominal(float rtd_nominal_ohms, float ref_resistor_ohms) {
    if (rtd_nominal_ohms > 0.0f) s_rtd_nominal = rtd_nominal_ohms;
    if (ref_resistor_ohms > 0.0f) s_ref_resistor = ref_resistor_ohms;
}

/* Set wire mode: 2/3/4 */
int max31865_set_wire_mode(int wires) {
    if (!(wires == MAX31865_WIRES_2 || wires == MAX31865_WIRES_3 || wires == MAX31865_WIRES_4)) {
        return -1;
    }
    uint8_t cfg;
    if (read_config(&cfg) != 0) {
        return -2;
    }
    if (wires == MAX31865_WIRES_3) cfg |= MAX31865_CONFIG_3WIRE;
    else cfg &= ~MAX31865_CONFIG_3WIRE;
    if (write_config(cfg) != 0) return -3;
    s_wire_count = wires;
    return 0;
}

/* Set 50Hz filter (true) else 60Hz */
int max31865_set_filter_50hz(bool enable) {
    uint8_t cfg;
    if (read_config(&cfg) != 0) return -1;
    if (enable) cfg |= MAX31865_CONFIG_FILTER50;
    else cfg &= ~MAX31865_CONFIG_FILTER50;
    if (write_config(cfg) != 0) return -2;
    return 0;
}

/* enable_bias: set/clear VBIAS */
int max31865_enable_bias(bool enable) {
    uint8_t cfg;
    if (read_config(&cfg) != 0) return -1;
    if (enable) cfg |= MAX31865_CONFIG_BIAS;
    else cfg &= ~MAX31865_CONFIG_BIAS;
    if (write_config(cfg) != 0) return -2;
    return 0;
}

/* Start one-shot conversion: set ONE_SHOT bit (auto clears) */
int max31865_start_one_shot(void) {
    uint8_t cfg;
    if (read_config(&cfg) != 0) return -1;
    cfg |= MAX31865_CONFIG_ONE_SHOT;
    if (write_config(cfg) != 0) return -2;
    return 0;
}

/* Clear faults: set fault clear bit (writing 1 will clear) */
int max31865_clear_faults(void) {
    uint8_t cfg;
    if (read_config(&cfg) != 0) return -1;
    cfg |= MAX31865_CONFIG_FAULT_CLR;
    if (write_config(cfg) != 0) return -2;
    return 0;
}

/* Low-level read raw (15-bit) */
int max31865_read_raw(uint16_t *out_raw) {
    if (!out_raw) return -1;

    uint8_t buf[2];
    int rc;
    if (s_mode == MAX31865_MODE_SW) {
        rc = sw_read_registers(MAX31865_REG_RTD_MSB, buf, 2);
    } else {
        rc = spi_read_registers(MAX31865_REG_RTD_MSB, buf, 2);
    }
    if (rc != 0) return -2;

    /* Combine MSB and LSB (MSB first), then shift right by 1 (LSB bit0 is fault) */
    uint16_t msb = buf[0];
    uint16_t lsb = buf[1];
    uint16_t raw = (uint16_t)((msb << 8) | lsb) >> 1;
    *out_raw = raw;
    return 0;
}

/* Debug helper: read two bytes (msb, lsb) directly */
int max31865_read_raw_bytes(uint8_t out[2]) {
    if (!out) return -1;
    if (s_mode == MAX31865_MODE_SW) return sw_read_registers(MAX31865_REG_RTD_MSB, out, 2);
    return spi_read_registers(MAX31865_REG_RTD_MSB, out, 2);
}

/* Read fault status register */
int max31865_read_fault_status(uint8_t *out_fault) {
    if (!out_fault) return -1;
    uint8_t buf;
    int rc;
    if (s_mode == MAX31865_MODE_SW) rc = sw_read_registers(MAX31865_REG_FAULT_STAT, &buf, 1);
    else rc = spi_read_registers(MAX31865_REG_FAULT_STAT, &buf, 1);
    if (rc != 0) return -2;
    s_last_fault = buf;
    *out_fault = buf;
    return 0;
}

uint8_t max31865_last_fault(void) {
    return s_last_fault;
}

bool max31865_has_fault(void) {
    return s_last_fault != 0;
}

/* Convert raw to resistance (float), per datasheet:
 * raw (0..32767) represents ratio: raw/32768 = RTD_resistance / R_ref
 * => R_rtd = (raw / 32768.0) * R_ref
 */
float max31865_read_resistance_float(void) {
    uint16_t raw;
    if (max31865_read_raw(&raw) != 0) return NAN;
    /* also read fault register: the LSB bit0 included in the RTD register indicates fault when set?
       We will also check the fault status register separately. */
    float r = ((float)raw) * (s_ref_resistor / 32768.0f);
    return r;
}

long max31865_read_resistance_int(unsigned decimals) {
    uint16_t raw;
    if (max31865_read_raw(&raw) != 0) return MAX31865_ERROR;
    float r = ((float)raw) * (s_ref_resistor / 32768.0f);
    if (!isfinite(r)) return MAX31865_ERROR;
    unsigned d = clamp_decimals(decimals);
    if (d == 0) return (long)roundf(r);
    if (d == 1) return (long)roundf(r * 10.0f);
    return (long)roundf(r * 100.0f);
}

/* Solve Callendar–Van Dusen:
 * Input: measured resistance (ohms)
 * Output: temperature in °C (float)
 *
 * Strategy:
 * - Compute ratio = R / R0
 * - For T >= 0 => use quadratic analytic solution:
 * B*T^2 + A*T + (1 - R/R0) = 0
 * choose positive root: T = (-A + sqrt(A^2 - 4*B*(1 - R/R0))) / (2*B)
 *
 * - For T < 0 => use Newton-Raphson on f(T) = Rcalc(T) - Rmeas
 * where Rcalc(T) = R0*(1 + A*T + B*T^2 + C*(T-100)*T^3)
 * NR update: Tnew = Told - f(T)/f'(T)
 * iterate up to N times until convergence.
 */
static float resistance_to_temperature_c(float r) {
    if (!isfinite(r) || r <= 0.0f) return NAN;
    double R0 = (double)s_rtd_nominal;
    double ratio = (double)r / R0;
    /* Quadratic coefficients for T >= 0 */
    double a = CV_B;
    double b = CV_A;
    double c = 1.0 - ratio;
    /* discriminant */
    double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
        double tpos = (-b + sqrt(disc)) / (2.0 * a);
        double tneg = (-b - sqrt(disc)) / (2.0 * a);
        /* choose the physically plausible root; for typical RTD  -200..850 C,
           tpos is usually >= 0 when ratio >=1; pick tpos if >=0 else use Newton-Raphson */
        if (tpos >= -1e-6) {
            return (float)tpos;
        }
    }

    /* Negative or no analytic solution -> Newton-Raphson */
    /* Initial guess: use quadratic root (if available) else -50.0 */
    double T = -50.0;
    if (disc >= 0.0) {
        double tguess = (-b - sqrt(disc)) / (2.0 * a);
        if (tguess < 0.0) T = tguess;
    }

    /* Newton-Raphson */
    for (int iter = 0; iter < 30; ++iter) {
        double T2 = T * T;
        double T3 = T2 * T;
        double Rcalc = R0 * (1.0 + CV_A * T + CV_B * T2 + CV_C * (T - 100.0) * T3);
        double f = Rcalc - (double)r;
        if (fabs(f) < 1e-6) break;
        /* derivative f'(T) = R0 * (A + 2B*T + C*(4*T^3 - 300*T^2 + 30000*T - 10000?) ) - complicated.
           We'll compute derivative numerically for stability or compute exact derivative:
           R'(T) = R0*(A + 2*B*T + C*(4*T^3 - 300*T^2 + 30000*T - 10000)) ??? -> to avoid mistakes do numeric derivative */
        double h = 1e-4;
        double Tph = T + h;
        double Tph2 = Tph * Tph;
        double Tph3 = Tph2 * Tph;
        double Rph = R0 * (1.0 + CV_A * Tph + CV_B * Tph2 + CV_C * (Tph - 100.0) * Tph3);
        double dfdT = (Rph - Rcalc) / h;
        if (dfdT == 0.0) break;
        double delta = f / dfdT;
        T -= delta;
        if (fabs(delta) < 1e-6) break;
    }

    return (float)T;
}

/* Public temperature read (C float) */
float max31865_read_temperature_c_float(unsigned decimals) {
    unsigned d = clamp_decimals(decimals);
    float r = max31865_read_resistance_float();
    if (!isfinite(r)) return NAN;
    /* read fault status as well */
    uint8_t fault;
    if (max31865_read_fault_status(&fault) != 0) {
        /* ignore perhaps, rely on RTD validity */
        s_last_fault = 0;
    } else {
        s_last_fault = fault;
        if (fault != 0) {
            return NAN;
        }
    }
    float c = resistance_to_temperature_c(r);
    if (!isfinite(c)) return NAN;
    if (d == 0) return roundf(c);
    if (d == 1) return roundf(c * 10.0f) / 10.0f;
    return roundf(c * 100.0f) / 100.0f;
}

long max31865_read_temperature_c_int(unsigned decimals) {
    unsigned d = clamp_decimals(decimals);
    float r = max31865_read_resistance_float();
    if (!isfinite(r)) return MAX31865_ERROR;
    uint8_t fault;
    if (max31865_read_fault_status(&fault) != 0) {
        s_last_fault = 0;
    } else {
        s_last_fault = fault;
        if (fault != 0) return MAX31865_ERROR;
    }

    float c = resistance_to_temperature_c(r);
    if (!isfinite(c)) return MAX31865_ERROR;

    if (d == 0) return (long)roundf(c);
    if (d == 1) return (long)roundf(c * 10.0f);
    return (long)roundf(c * 100.0f);
}

/* Fahrenheit conversions */
float max31865_read_temperature_f_float(unsigned decimals) {
    float c = max31865_read_temperature_c_float(3); /* get high precision */
    if (!isfinite(c)) return NAN;
    float f = c * 9.0f / 5.0f + 32.0f;
    unsigned d = clamp_decimals(decimals);
    if (d == 0) return roundf(f);
    if (d == 1) return roundf(f * 10.0f) / 10.0f;
    return roundf(f * 100.0f) / 100.0f;
}

long max31865_read_temperature_f_int(unsigned decimals) {
    long c100 = max31865_read_temperature_c_int(2); /* scaled x100 */
    if (c100 == MAX31865_ERROR) return MAX31865_ERROR;
    float c = ((float)c100) / 100.0f;
    float f = c * 9.0f / 5.0f + 32.0f;
    unsigned d = clamp_decimals(decimals);
    if (d == 0) return (long)roundf(f);
    if (d == 1) return (long)roundf(f * 10.0f);
    return (long)roundf(f * 100.0f);
}

/* Debug setter */
void max31865_set_debug(int verbosity) {
    s_debug = verbosity;
}


/* --- Added functions from new snippet (adapted to this library) --- */

/* enable/disable continuous conversions and leave bias on when enabled */
int max31865_set_continuous_mode(bool enable) {
    uint8_t cfg;
    if (read_config(&cfg) != 0) return -1;
    
    if (enable) {
        cfg |= MAX31865_CONFIG_BIAS;   /* VBIAS on */
        cfg |= MAX31865_CONFIG_AUTO;   /* Auto-conversion mode */
        cfg &= ~MAX31865_CONFIG_ONE_SHOT; /* Ensure one-shot is off */
    } else {
        cfg &= ~MAX31865_CONFIG_AUTO;   /* Auto-conversion off */
        cfg &= ~MAX31865_CONFIG_BIAS;  /* VBIAS off */
    }
    
    if (write_config(cfg) != 0) return -2;
    return 0;
}


/* read RTD and convert (existing conversion logic is used).
   This wrapper implements spike-detection and retry. Returns last-good if transient. */
float max31865_read_temperature_safe(void) {
    /* Read temperature using existing function (handles faults) */
    /* Use 3 decimals for internal precision */
    float t = max31865_read_temperature_c_float(3); 

    if (!isfinite(t)) {
        /* Read failed (e.g., fault). Retry once. */
        platform_delay_ms(SPIKE_RETRY_MS);
        t = max31865_read_temperature_c_float(3);

        if (!isfinite(t)) {
            /* Still bad? Return last good, or NAN if no last good */
            return isnan(s_last_good_temp_c) ? NAN : s_last_good_temp_c;
        }
    }

    /* spike detection */
    if (isfinite(s_last_good_temp_c) && fabsf(t - s_last_good_temp_c) > SPIKE_DELTA_THRESHOLD_C) {
        /* Suspicious spike. Retry once. */
        platform_delay_ms(SPIKE_RETRY_MS);
        float tc = max31865_read_temperature_c_float(3);

        if (isfinite(tc) && fabsf(tc - s_last_good_temp_c) <= SPIKE_DELTA_THRESHOLD_C) {
            /* Retry value is good and close to last good. Use it. */
            t = tc;
        } else {
            /* Retry value is also bad or still a spike. Reject. */
            return s_last_good_temp_c;
        }
    }

    /* accept reading */
    s_last_good_temp_c = t;
    return t;
}

float max31865_get_last_good(void) {
    return s_last_good_temp_c;
}

/* --- End of added functions --- */


/* ------------------ Wiring notes & conversion delays (documentation) -------
 * Wiring:
 * - MAX31865 breakout pins: VIN (3.3V), GND, SCK, SDI (MOSI), SDO (MISO), CS
 * - For ESP32 use 3.3V only.
 *
 * Recommended R_ref:
 * - Adafruit breakout uses 430Ω for PT100 (R0=100Ω). For PT1000 boards it's often 4300Ω.
 * - If you use PT100 (R0=100) and R_ref=430 use max31865_set_rtd_nominal(100.0, 430.0).
 *
 * 2/3/4-wire differences:
 * - 2-wire: simplest, but wire resistance adds error.
 * - 3-wire: offers lead-compensation using third wire; requires correct board jumper/config.
 * - 4-wire: best accuracy; differential measurement removes lead resistance.
 *
 * Conversion delays:
 * - Enable bias (VBIAS) then wait ~10 ms before a one-shot conversion.
 * - One-shot conversion time depends on filter setting:
 * - filter 50Hz -> typical conversion ~200 ms
 * - filter 60Hz -> typical conversion ~100 ms
 * - For safe operation use 200 ms; you can reduce to ~100 ms for 60 Hz or if you use continuous conversion.
 * ------------------------------------------------------------------------- */

/* ------------------ End of implementation ------------------ */