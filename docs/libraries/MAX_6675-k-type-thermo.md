# README — MAX6675 (K-type Thermocouple) 

## Overview

This small, cross-platform C-style library provides an easy API to read a K-type thermocouple using the MAX6675 IC on Arduino (UNO/MEGA) and ESP-IDF (ESP32).

-   **Files**: `max6675.h` and `max6675.cpp` (drop these into your project; no renaming required).
-   **Key highlights**:
    -   Hardware (SPI) and Software (bit-banged) SPI supported.
    -   Same API for both platforms.
    -   Fault detection (thermocouple open) — floats return `NAN`, integer reads return `MAX6675_ERROR`.
    -   Decimal precision handling (0..2).

---

## Integration (Where to put files)

-   **PlatformIO / Arduino**: place `max6675.h` + `max6675.cpp` inside `lib/max6675/src/` or `src/`.
-   **ESP-IDF**: place them inside `components/max6675/` or compile them into your component.

Make sure your project uses the correct framework:
-   Arduino projects use `framework = arduino`
-   ESP-IDF projects use `framework = espidf`

---

## Key API (summary)

All functions are `extern "C"` compatible (callable from C or C++).

```c
int   max6675_init_hw(int cs_pin, uint32_t clock_hz);
int   max6675_init_sw(int cs_pin, int sck_pin, int miso_pin);
void  max6675_deinit(void);

int   max6675_read_raw(uint16_t *out_raw);

float max6675_read_celsius_float(unsigned decimals);
float max6675_read_fahrenheit_float(unsigned decimals);

long  max6675_read_celsius_int(unsigned decimals);
long  max6675_read_fahrenheit_int(unsigned decimals);

bool  max6675_is_open(void);

#define MAX6675_ERROR  ((long)0x80000000L)
````

**Notes:**

  * `decimals` is clamped to 0..2.
  * Float functions return `NAN` on error/fault.
  * Int functions return `MAX6675_ERROR` on fault/error.
  * `max6675_is_open()` returns `true` if thermocouple is open or a communications error occurs.

-----

## Quick Start — Arduino (UNO) Example

**Wiring (hardware SPI):**

  * **MAX6675 VCC** -\> 5V (or 3.3V for some modules)
  * **MAX6675 GND** -\> GND
  * **MAX6675 CS** -\> D10 (or any digital pin)
  * **MAX6675 SCK** -\> D13 (hardware SCK)
  * **MAX6675 SO** -\> D12 (MISO)

**Example usage:**

```cpp
#include "max6675.h"

#define CS_PIN 10

void setup() {
  Serial.begin(115200);
  max6675_init_hw(CS_PIN, 400000); // CS, clock_hz
}

void loop() {
  float c = max6675_read_celsius_float(2); // 2 decimal places
  if (isnan(c)) {
    Serial.println("Read error or thermocouple open");
  } else {
    Serial.print("Temp: ");
    Serial.print(c, 2);
    Serial.println(" C");
  }
  delay(1000);
}
```

> **Software SPI**: call `max6675_init_sw(cs, sck, miso)` instead and wire accordingly.

-----

## Quick Start — ESP-IDF (ESP32) Example

Common default pins used in examples: SCLK=18, MISO=19, MOSI=23, CS=5.
Do this in `app_main()` (or equivalent):

```c
// Initialize SPI bus
spi_bus_config_t buscfg = { .miso_io_num = 19, .mosi_io_num = 23, .sclk_io_num = 18, ... };
spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

// Initialize the device
max6675_init_hw(5, 400000); // CS=5, clock 400kHz
```

Then call the same API as Arduino. Use `ESP_LOGI` for logging.

-----

## Error handling & fault detection

If the ADC read or communication fails, or if the MAX6675 D2 fault bit indicates an open thermocouple:

  - Float functions return `NAN`.
  - Int functions return `MAX6675_ERROR`.
  - `max6675_is_open()` returns `true`.

Always check for `isnan()` or equality with `MAX6675_ERROR` after calls.

-----

## Troubleshooting checklist

If all reads return `0` or `0.00`:

  - Confirm the **CS pin** is wired correctly and toggling.
  - Confirm SPI mode/clock or swap to **SW SPI** to isolate driver vs. wiring issues.

If you get `NAN` / `MAX6675_ERROR`:

  - The thermocouple may be open. Check the thermocouple connector and wiring.

<!-- end list -->