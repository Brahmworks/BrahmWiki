# ESP32 Setup Guide

**Dual-core 32-bit MCU with Wi-Fi & Bluetooth**

## Overview

| Specification | Details |
|---------------|----------|
| Architecture | 32-bit Xtensa |
| Operating Voltage | 3.3V |
| GPIO Pins | 34 |
| Analog Inputs | 18 |

## Development Environment Setup

### Arduino IDE Setup

1. Install Arduino IDE 2.0 or later
2. Add ESP32 board manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Install ESP32 board package via Board Manager
4. Select your ESP32 board from Tools > Board menu

### PlatformIO Setup

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
```

