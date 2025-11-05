
-----

# README — MAX31865 (RTD — PT100/PT1000)

## Overview

This cross-platform C-style library implements a MAX31865 driver compatible with Arduino (UNO/MEGA) and ESP-IDF (ESP32).

  * **Files**: `max31865.h` and `max31865.cpp`

-----

## Features

  * Hardware SPI and Software (bit-banged) SPI
  * Support for PT100 and PT1000 (configurable `rtd_nominal` and `ref_resistor`)
  * 2/3/4-wire RTD support
  * Readings for resistance and temperature (float + scaled int)
  * Callendar–Van Dusen and Newton–Raphson analytic solutions
  * **New:** Continuous (auto) conversion mode
  * **New:** `read_temperature_safe()` for spike-filtering and auto-retry
  *  Full support for Fahrenheit and integer-based readings
  * **New (ESP-IDF):** Fully **Thread-Safe** using an internal mutex.
  * **New (ESP-IDF):** Can be used on the same SPI bus with the `max6675` library without conflicts.

-----

## Integration (Where to put files)

  * **PlatformIO / Arduino**: place `max31865.h` + `max31865.cpp` in `lib/max31865/src/` or `src/`.
  * **ESP-IDF**: place them in `components/max31865/`.

-----

## ESP-IDF Production Readiness

This library is now **ESP-IDF multitasking**.

1.  **Thread-Safe:** All public functions are protected by a mutex. You can safely call `max31865_read_temperature_safe()` from different tasks without crashing.
2.  **Multi-Device Compatible:** The library's internal (static) variables have been renamed (e.g., `s_max31865_spi_dev`). This **prevents conflicts** with other sensor libraries (like `max6675`) that are also on the SPI bus.

### SPI/DMA Caution

The original caution about SPI/DMA still applies:

  * Problems have been observed on ESP-IDF with SPI half-duplex + DMA.
  * **Solution**: This library uses full-duplex (`devcfg.flags = 0`) by default.
  * If you still see zeros, set the compile-time toggle `MAX31865_ESP_SAFE_SPI` to `1` in `max31865.cpp`.

-----

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

// Manual Control (One-Shot)
int   max31865_enable_bias(bool enable);
int   max31865_start_one_shot(void);

// Data Reading (Full API)
float max31865_read_resistance_float(void);
long  max31865_read_resistance_int(unsigned decimals);
float max31865_read_temperature_c_float(unsigned decimals);
long  max31865_read_temperature_c_int(unsigned decimals);
float max31865_read_temperature_f_float(unsigned decimals);
long  max31865_read_temperature_f_int(unsigned decimals);

// Faults & Diagnostics
int   max31865_read_fault_status(uint8_t *out_fault);
bool  max31865_has_fault(void);
int   max31865_read_raw_bytes(uint8_t out[2]); // Debug helper
int   max31865_clear_faults(void);

// New: Continuous Mode & Safe Read API
int   max31865_set_continuous_mode(bool enable);
float max31865_read_temperature_safe(void);
float max31865_get_last_good(void);

// Error Sentinel
#define MAX31865_ERROR ((long)0x80000000L)
```

-----

## Quick Start — Arduino UNO (Recommended Example)

This example uses **Continuous Mode** and **Safe Read**, which is the simplest and most robust way to use the library on Arduino.

**Wiring:**

  * **VIN** -\> 3.3V
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
  while (!Serial && millis() < 2000) { /* wait */ }

  Serial.println("Initializing MAX31865...");
  
  max31865_init_hw(CS_PIN, 1000000);
  max31865_set_rtd_nominal(100.0f, 430.0f); // PT100 + 430R ref
  max31865_set_wire_mode(3); // 3-wire mode

  // 1. Turn on continuous conversion mode
  // This keeps VBIAS on and the chip auto-converting.
  max31865_set_continuous_mode(true); 
}

void loop() {
  // 2. Read using the safe, filtered function.
  // This function handles spikes and transient errors.
  float c = max31865_read_temperature_safe();

  if (!isnan(c)) {
    Serial.print("Stable Temp: ");
    Serial.print(c, 2);
    Serial.println(" C");
  } else {
    Serial.println("Waiting for first valid reading...");
    // Check for a hardware fault
    if (max31865_has_fault()) {
        uint8_t fault;
        max31865_read_fault_status(&fault);
        Serial.print("FAULT DETECTED: 0x");
        Serial.println(fault, HEX);
        max31865_clear_faults();
    }
  }

  delay(1000); // Poll for a new value every second
}
```

-----

## Quick Start — ESP-IDF (ESP32) Example

This example shows how to initialize the sensor in `app_main`. For a full multi-tasking example (e.g., with a PID controller), see the `pid_heater` library's README.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "max31865.h"

static const char* TAG = "sensor_test";

#define CS_PIN 21

void app_main(void) {
    // 1. Initialize SPI bus (HSPI_HOST)
    spi_bus_config_t buscfg = {
      .miso_io_num = 19,
      .mosi_io_num = 23,
      .sclk_io_num = 18,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1
    };
    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    // 2. Configure and initialize MAX31865
    max31865_init_hw(CS_PIN, 1000000); // CS=21, 1MHz
    max31865_set_rtd_nominal(100.0f, 430.0f);
    max31865_set_wire_mode(3);
    max31865_set_continuous_mode(true);

    // Give sensor time for first conversion
    vTaskDelay(pdMS_TO_TICKS(500));

    // 3. Read in a loop
    while(1) {
        float c = max31865_read_temperature_safe();
        if (!isnan(c)) {
            ESP_LOGI(TAG, "Temperature: %.2f C", c);
        } else {
            ESP_LOGW(TAG, "Read failed or fault occurred.");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

-----

## Timing Guidance (Manual One-Shot)

**Note:** This logic is not needed if you use the recommended **Continuous Mode** (`max31865_set_continuous_mode(true)`).

A safe sequence for a manual one-shot reading is:

1.  `max31865_enable_bias(true);`
2.  Wait \~10 ms for the bridge to settle.
3.  `max31865_start_one_shot();`
4.  Wait for conversion (e.g., 100-200 ms).
5.  `max31865_enable_bias(false);` (to save power)
6.  `max31865_read_temperature_*()`

-----

## Faults & Diagnostics

If your reads fail, use `max31865_has_fault()` to check if the chip reported a hardware error.

1.  Call `max31865_read_fault_status(&fault_byte);` to get the 8-bit fault code.
2.  Use `max31865_clear_faults();` to clear the fault and try again.

**Common Faults (from `max31865.h`):**

  * `MAX31865_FAULT_RTDINLOW` (Bit 3): **Most common.** RTD is shorted or disconnected. Check your wires.
  * `MAX31865_FAULT_OVUV` (Bit 2): Over/under voltage. Check your power supply.
  * `MAX31865_FAULT_REFINLOW` / `_REFINHIGH` (Bits 5, 4): Reference resistor is out of range.