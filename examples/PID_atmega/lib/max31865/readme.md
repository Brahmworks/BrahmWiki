
---

# README — MAX31865 (RTD — PT100/PT1000)

## Overview

This cross-platform C-style library implements a MAX31865 driver compatible with **Arduino (UNO/MEGA)** and **ESP-IDF (ESP32)**.

* **Files:** `max31865.h` and `max31865.cpp`
* **Drop-in:** just include and call — no C++ classes, no dynamic memory
* **Sensors supported:** PT100 and PT1000
* **Wiring modes:** 2-wire / 3-wire / 4-wire RTDs

### Features

* Hardware SPI and Software (bit-banged) SPI
* PT100 and PT1000 via configurable `rtd_nominal` and `ref_resistor`
* 2/3/4-wire RTD support
* Bias control, one-shot conversions, and fault management
* Read resistance and temperature (float or integer scaled)
* Implements the full **Callendar–Van Dusen** equation with Newton–Raphson refinement
* **Continuous (auto) conversion mode** — no blocking delays
* **Safe read API** — spike filtering, automatic retries, returns last-good value on transient faults

---

## Integration (Where to Put Files)

* **PlatformIO / Arduino:**
  Place `max31865.h` + `max31865.cpp` under `lib/max31865/` or directly in `src/`.
* **ESP-IDF:**
  Add them under `components/max31865/` or any folder in your CMake project and link normally.

---

## Important ESP-IDF Precautions

### 1. SPI Configuration

* ESP-IDF’s SPI driver defaults to **half-duplex + DMA**, which may fail with MAX31865:

  ```
  SPI half duplex mode does not support using DMA with both MOSI and MISO phases
  ```
* **Fix:** Use full-duplex mode.
  Inside the driver, ensure `spi_device_interface_config_t devcfg.flags = 0;`
* The MAX31865 expects **SPI mode 1** (`CPOL=0`, `CPHA=1`).

### 2. Timing and Task Context

* Do **not** call blocking read functions (with bias toggling) from ISR or watchdog-sensitive tasks.
  Use `max31865_set_continuous_mode(true)` once at startup and read periodically using `max31865_read_temperature_safe()`.

### 3. Electrical Noise (SSR / Heater Loads)

If the RTD circuit shares power or ground with high-current heater or SSR switching circuits, you may see **false zero readings** or **fault 0x04** (RTD open).
Mitigate as follows:

| Problem                                       | Fix                                                                        |
| --------------------------------------------- | -------------------------------------------------------------------------- |
| Sudden drop to 0.0°C or NAN when SSR switches | Add 20–50 ms read guard after SSR turns **off**                            |
| MAX31865 fault 0x04 after spike               | Call `max31865_clear_faults()` and re-enable continuous mode               |
| ESP32 brownout reset when heater turns on     | Use separate 3.3 V regulator or add ≥ 470 µF bulk capacitor                |
| EMI on long RTD wires                         | Use twisted-pair shielded cable; connect shield to ground at MCU side only |
| RTD floating at startup                       | Always call `max31865_set_continuous_mode(true)` before first read         |

### 4. Power Supply & Grounding

* The MAX31865 draws current spikes when bias is enabled — make sure 3.3 V rail is stable.
* Add 100 nF + 10 µF decoupling capacitors near the breakout board.
* Keep RTD cable and heater/mains wiring physically separated.

---

## Key API Summary

All functions are `extern "C"` compatible.

```c
// Initialization
int   max31865_init_hw(int cs_pin, uint32_t clock_hz);
int   max31865_init_sw(int cs_pin, int sck_pin, int mosi_pin, int miso_pin);
void  max31865_deinit(void);

// Configuration
void  max31865_set_rtd_nominal(float rtd_nominal_ohms, float ref_resistor_ohms);
int   max31865_set_wire_mode(int wires);     // 2,3,4
int   max31865_set_filter_50hz(bool enable);

// Manual Control (One-Shot)
int   max31865_enable_bias(bool enable);
int   max31865_start_one_shot(void);
int   max31865_clear_faults(void);

// Data Reading
float max31865_read_resistance_float(void);
long  max31865_read_resistance_int(unsigned decimals);
float max31865_read_temperature_c_float(unsigned decimals);
float max31865_read_temperature_safe(void);
long  max31865_read_temperature_c_int(unsigned decimals);

// Faults & Diagnostics
int   max31865_read_fault_status(uint8_t *out_fault);
bool  max31865_has_fault(void);

// Continuous Mode
int   max31865_set_continuous_mode(bool enable);
float max31865_get_last_good(void);

// Error sentinel
#define MAX31865_ERROR ((long)0x80000000L)
```

---

## Recommended Usage (ESP-IDF / Arduino)

The **continuous + safe-read** combination gives the most stable, non-blocking behavior.

```c
#include "max31865.h"

#define CS_PIN 21

void app_main(void) {
    // initialize SPI elsewhere (ESP-IDF) or rely on Arduino SPI
    max31865_init_hw(CS_PIN, 1000000);
    max31865_set_rtd_nominal(100.0f, 430.0f);  // PT100, 430 Ω ref
    max31865_set_wire_mode(3);                 // 3-wire RTD
    max31865_set_filter_50hz(true);

    // Enable continuous conversion mode
    max31865_set_continuous_mode(true);

    while (1) {
        float t = max31865_read_temperature_safe();
        if (!isnan(t)) {
            ESP_LOGI("MAX31865", "PT100 = %.2f C", t);
        } else {
            ESP_LOGW("MAX31865", "Waiting for valid reading...");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## How the Safe Read Works

`max31865_read_temperature_safe()` internally:

1. Reads the temperature using the normal function.
2. If the value is `NAN` or jumps > 10 °C from the last known-good reading, retries once.
3. If still invalid, returns the last good value instead of `NAN`.

This eliminates spikes caused by EMI or transient bus errors, while staying non-blocking when continuous mode is active.

---

## Manual One-Shot Mode (for low-power designs)

Use when you want to turn bias on only during conversions.
This gives lower self-heating but requires explicit delays.

### Recommended Timing

| Step | Action                                 | Typical Delay         |
| ---- | -------------------------------------- | --------------------- |
| 1    | `max31865_enable_bias(true)`           | 10 ms (bridge settle) |
| 2    | `max31865_start_one_shot()`            | —                     |
| 3    | wait for conversion                    | 100–200 ms            |
| 4    | `max31865_enable_bias(false)`          | —                     |
| 5    | `max31865_read_temperature_c_float(2)` | —                     |

---

## Common Fault Codes

| Fault Code | Meaning              | Action                                   |
| ---------- | -------------------- | ---------------------------------------- |
| 0x01       | RTD High Threshold   | Check wiring, sensor                     |
| 0x02       | RTD Low Threshold    | Check wiring, sensor                     |
| 0x04       | **RTD Open Circuit** | Clear faults + re-enable continuous mode |
| 0x08       | RefIn-/RefIn+ fault  | Check reference resistor connection      |
| 0x10       | RTDIN-/RTDIN+ short  | Check RTD wires                          |
| 0x20       | Over/Under Voltage   | Check power rail stability               |

Example fault recovery:

```c
uint8_t fault = 0;
if (max31865_read_fault_status(&fault) == 0 && fault != 0) {
    ESP_LOGW("MAX31865", "Fault 0x%02x detected -> clear & retry", fault);
    max31865_clear_faults();
    max31865_set_continuous_mode(true);
}
```

---

## Practical Design Notes

* **Keep SPI lines short** (< 15 cm) and twisted if possible.
  If longer, use shielded cable or lower the SPI clock (≤ 1 MHz).
* **Use separate ground return** for RTD / MAX31865 board when driving large loads (SSR, heater).
  Star-ground layout helps prevent false 0 °C readings.
* **Add RC snubber or MOV** across inductive heater loads to reduce EMI spikes coupling into SPI bus.
* **Always read temperature ≥ 10 ms after SSR turns ON or 50 ms after OFF** if heater power shares the same supply rail.

---

## Example Recovery Snippet (ESP-IDF)

If fault `0x04` appears repeatedly:

```c
if (max31865_read_fault_status(&fault) == 0 && fault == 0x04) {
    max31865_clear_faults();
    max31865_set_continuous_mode(true);
    vTaskDelay(pdMS_TO_TICKS(80));
}
```

---