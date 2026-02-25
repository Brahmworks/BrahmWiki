# ATmega328P RobustUART Client Example

This example demonstrates using the ATmega328P as a Client (slave) device that responds to commands from an ESP32 host.

## Wiring

| ATmega328P | ESP32 |
|------------|-------|
| RX (Pin 2) | TX (GPIO17) |
| TX (Pin 3) | RX (GPIO16) |
| GND | GND |

## Building

```bash
cd examples/atmega_client
pio run
```

## Running

```bash
pio device monitor
```

The client will wait for commands from the host and respond accordingly.
