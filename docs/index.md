# ESP32 & ATmega Library Collection

Complete embedded systems library collection with examples for ESP32, ATmega328P, and ATmega2560 microcontrollers.

## 🎯 Quick Start Guide

1. **Choose your microcontroller** from the supported platforms
2. **Browse available libraries** in the Library Reference
3. **Check examples** for your specific MCU
4. **Fork this repository** to start your project
5. **Modify `main.c`** according to your requirements

## 🔧 Supported Microcontrollers

### ESP32

**Dual-core 32-bit MCU with Wi-Fi & Bluetooth**

| Specification | Value |
|---------------|-------|
| Operating Voltage | 3.3V |
| GPIO Pins | 34 |
| Analog Pins | 18 |
| Key Features | Wi-Fi, Bluetooth, 240MHz, 520KB RAM, 4MB Flash |

### ATmega328P

**8-bit AVR microcontroller**

| Specification | Value |
|---------------|-------|
| Operating Voltage | 5V |
| GPIO Pins | 23 |
| Analog Pins | 8 |
| Key Features | UART, SPI, I2C, 16MHz, 2KB SRAM, 32KB Flash |

### ATmega2560

**8-bit AVR microcontroller with more I/O**

| Specification | Value |
|---------------|-------|
| Operating Voltage | 5V |
| GPIO Pins | 86 |
| Analog Pins | 16 |
| Key Features | 4x UART, 2x SPI, I2C, 16MHz, 8KB SRAM, 256KB Flash |

## 📚 Available Library Categories

### Actuators

- **[Dc Motor](libraries/actuators/dc_motor/index.md)**
- **[Servo Control](libraries/actuators/servo_control/index.md)**
- **[Stepper Motor](libraries/actuators/stepper_motor/index.md)**

### Communication

- **[I2C Common](libraries/communication/i2c_common/index.md)**
- **[Uart Common](libraries/communication/uart_common/index.md)**
- **[Wifi Esp32](libraries/communication/wifi_esp32/index.md)**

### Sensors

- **[Humidity Dht22](libraries/sensors/humidity_dht22/index.md)**
- **[Pressure Bmp280](libraries/sensors/pressure_bmp280/index.md)**
- **[Sensor Template](libraries/sensors/sensor_template/index.md)** - Brief description of the sensor functionality and its applications in embedded systems.
- **[Temperature Ds18B20](libraries/sensors/temperature_ds18b20/index.md)** - Brief description of the sensor functionality and its applications in embedded systems.

## 🚀 Main Code Template

The main.c file serves as your project's entry point. Modify it according to your specific requirements:

```c
/*
 * Embedded Systems Project Template
 * Supports: ESP32, ATmega328P, ATmega2560
 *
 * Modify this file according to your project requirements
 */

#ifdef ESP32
#include <Arduino.h>
// ESP32 specific includes
#elif defined(__AVR_ATmega328P__)
#include <avr/io.h>
#include <util/delay.h>
// ATmega328P specific includes
#elif defined(__AVR_ATmega2560__)
#include <avr/io.h>
#include <util/delay.h>
// ATmega2560 specific includes
#endif

// Include your library headers here
// #include "lib/sensors/temperature_ds18b20/src/ds18b20.h"

void setup()
{
// Initialization code
#ifdef ESP32
    Serial.begin(115200);
#else
    // AVR initialization
#endif

    // Initialize your libraries here
}

void loop()
{
    // Main program logic

#ifdef ESP32
    delay(1000);
#else
    _delay_ms(1000);
#endif
}

#ifndef ESP32
int main()
{
    setup();
    while (1)
    {
        loop();
    }
    return 0;
}
#endif
```

