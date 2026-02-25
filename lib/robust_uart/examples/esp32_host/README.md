# ESP32 RobustUART Host Example

This example demonstrates using the ESP32 as a Host (master) device that sends commands to an ATmega client.

## Wiring

| ESP32 | ATmega328P |
|-------|------------|
| TX (GPIO17) | RX (Pin 2) |
| RX (GPIO16) | TX (Pin 3) |
| GND | GND |

## Building

```bash
cd examples/esp32_host
pio run
```

## Running

```bash
pio device monitor
```

The host will send ping commands every 5 seconds and display the results.
