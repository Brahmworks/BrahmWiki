# ATmega328P SoftwareSerial RobustUART Client Example

This example demonstrates using SoftwareSerial for RobustUART communication on ATmega328P. Useful when hardware UART is already in use for debugging.

## Wiring

| ATmega328P | Description |
|------------|-------------|
| Pin 10 (RX) | Connect to Host TX |
| Pin 11 (TX) | Connect to Host RX |
| GND | Connect to Host GND |

## Building

```bash
cd examples/software_serial
pio run
```

## Running

```bash
pio device monitor
```

## Notes

- SoftwareSerial is slower than hardware UART but allows flexible pin selection
- Maximum baud rate for SoftwareSerial is typically 115200
- Only one SoftwareSerial can listen at a time
