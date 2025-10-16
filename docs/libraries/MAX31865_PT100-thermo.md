# README — MAX31865 (RTD — PT100/PT1000) 

## Overview

This cross-platform C-style library implements a MAX31865 driver compatible with Arduino (UNO/MEGA) and ESP-IDF (ESP32).

- **Files**: `max31865.h` and `max31865.cpp` (drop these into your project; no renaming required).

### Features
- Hardware SPI and Software (bit-banged) SPI
- Support for PT100 and PT1000 via configurable `rtd_nominal` and `ref_resistor`
- 2/3/4-wire RTD support
- Enable bias, one-shot conversions, and fault management
- Readings for resistance and temperature (float + scaled int)
- Callendar–Van Dusen and Newton–Raphson analytic solutions

---

## Integration (Where to put files)

- **PlatformIO / Arduino**: place `max31865.h` + `max31865.cpp` in `lib/max31865/src/` or `src/`.
- **ESP-IDF**: place them in `components/max31865/`.

---

##  Important ESP-IDF Caution

Problems have been observed on ESP-IDF:
- SPI half-duplex + DMA can cause transactions to return zeros or runtime errors:
  > `SPI half duplex mode does not support using DMA with both MOSI and MISO phases`
- **Solution**: Use full-duplex (`devcfg.flags = 0`) and robust SPI transaction strategies.
- If you still see zeros, set the compile-time toggle `MAX31865_ESP_SAFE_SPI` to `1` in `max31865.cpp`.

---

## Key API (summary)

All functions are `extern "C"` compatible.

```c
// Initialization
int   max31865_init_hw(int cs_pin, uint32_t clock_hz);
int   max31865_init_sw(int cs_pin, int sck_pin, int mosi_pin, int miso_pin);
void  max31865_deinit(void);

// Configuration
void  max31865_set_rtd_nominal(float rtd_nominal_ohms, float ref_resistor_ohms);
int   max31865_set_wire_mode(int wires); /* 2,3,4 */
int   max31865_set_filter_50hz(bool enable);

// Control
int   max31865_enable_bias(bool enable);
int   max31865_start_one_shot(void);
int   max31865_clear_faults(void);

// Data Reading
float max31865_read_resistance_float(void);
long  max31865_read_resistance_int(unsigned decimals);
float max31865_read_temperature_c_float(unsigned decimals);
long  max31865_read_temperature_c_int(unsigned decimals);
float max31865_read_temperature_f_float(unsigned decimals);
long  max31865_read_temperature_f_int(unsigned decimals);

// Faults & Diagnostics
int   max31865_read_fault_status(uint8_t *out_fault);
bool  max31865_has_fault(void);

// Error Sentinel
#define MAX31865_ERROR ((long)0x80000000L)
````

-----

## Quick Start — Arduino UNO Example

**Wiring (Adafruit/typical breakout):**

  * **VIN** -\> 3.3V (preferred)
  * **GND** -\> GND
  * **SCK** -\> D13
  * **MOSI** -\> D11
  * **MISO** -\> D12
  * **CS** -\> D10

**Example:**

```cpp
#include "max31865.h"

#define CS_PIN 10

void setup() {
  Serial.begin(115200);
  max31865_init_hw(CS_PIN, 1000000);
  max31865_set_rtd_nominal(100.0f, 430.0f); // PT100 + 430R ref
  max31865_set_wire_mode(3); // 3-wire mode
  max31865_set_filter_50hz(false); // Use 60Hz filter
}

void loop() {
  max31865_enable_bias(true);
  delay(10); // Bias settle time
  max31865_start_one_shot();
  delay(200); // Conversion time
  max31865_enable_bias(false);

  float c = max31865_read_temperature_c_float(2);
  if (!isnan(c)) {
    Serial.print("Temp: ");
    Serial.print(c, 2);
    Serial.println(" C");
  } else {
    Serial.println("Read failed or fault occurred.");
  }
  delay(1000);
}
```

-----

## Quick Start — ESP-IDF (ESP32) Example

Example `app_main()` snippet:

```c
// Initialize SPI bus
spi_bus_config_t buscfg = {
  .miso_io_num = 19,
  .mosi_io_num = 23,
  .sclk_io_num = 18,
};
spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

// Configure and initialize MAX31865
max31865_init_hw(5, 1000000); // CS=5, 1MHz
max31865_set_rtd_nominal(100.0f, 430.0f);
max31865_set_wire_mode(3);
max31865_set_filter_50hz(true);

// Perform a single reading
max31865_enable_bias(true);
esp_rom_delay_us(10000);
max31865_start_one_shot();
vTaskDelay(pdMS_TO_TICKS(200));
max31865_enable_bias(false);

float c = max31865_read_temperature_c_float(2);
ESP_LOGI("MAIN", "Temperature: %.2f C", c);
```

-----

## Timing Guidance (Bias & Conversion)

A safe sequence for a one-shot reading is:

1.  `max31865_enable_bias(true);`
2.  Wait \~10 ms for the bridge to settle.
3.  `max31865_start_one_shot();`
4.  Wait for conversion (e.g., 100-200 ms).
5.  `max31865_enable_bias(false);` (to save power)
6.  `max31865_read_temperature_*()`

-----

## Faults & Diagnostics

  - Read the fault register with `max31865_read_fault_status(&fault_byte);`
  - If raw ADC bytes are `0x00 0x00`, check CS wiring, SPI mode (should be **mode 1**), or VCC.
  - Use `max31865_read_raw_bytes()` to inspect the exact bytes returned by the chip for debugging.

<!-- end list -->

```
```