# Standard UART Communication Library

This library provides a standardized interface for UART communication across different microcontrollers, currently supporting ESP32 and AVR platforms.

## Features

-   **Cross-platform:** Write UART code once and use it on both ESP32 and AVR microcontrollers.
-   **Standardized Functions:** Provides a consistent API for initializing, writing to, and reading from UART ports.
-   **Multiple UART Ports:** Supports multiple UART ports on devices that have them (e.g., ATmega2560).

## API

### `void uart_init(uint8_t uart_num, uint32_t baud_rate)`

Initializes the specified UART port with the given baud rate.

-   `uart_num`: The UART port to initialize (e.g., `UART_0_PROG`, `UART_1_COMM`).
-   `baud_rate`: The baud rate for the communication (e.g., 9600, 115200).

### `void uart_write(uint8_t uart_num, const char *str)`

Writes a null-terminated string to the specified UART port.

-   `uart_num`: The UART port to write to.
-   `str`: The string to send.

### `int uart_read(uint8_t uart_num, uint8_t *buffer, uint32_t len)`

Reads a specified number of bytes from the UART port into a buffer.

-   `uart_num`: The UART port to read from.
-   `buffer`: A pointer to the buffer where the data will be stored.
-   `len`: The maximum number of bytes to read.
-   **Returns:** The number of bytes read, or -1 on error.

## Example Usage

```c
#include "uart_standard.h"

void setup() {
  // Initialize UART 0 for programming/debugging at 115200 baud
  uart_init(UART_0_PROG, 115200);
}

void loop() {
  uart_write(UART_0_PROG, "Hello, world!\n");
  delay(1000);
}
