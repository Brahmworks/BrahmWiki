# UART Libraries

BrahmWiki provides two UART libraries for different use cases:

## 1. RobustUART Library (Recommended for packetized communication)

For robust, packetized UART communication with error handling, retries, and acknowledgment, use the **RobustUART** library.

### Features

- **Packet Protocol**: `SOH[LEN][CMD][PAYLOAD][CHKSUM]EOT` with ACK/NAK responses
- **Cross-Platform**: ESP32 (ESP-IDF/Arduino), ATmega328P, ATmega2560
- **Error Handling**: XOR checksum, timeout, automatic retries
- **Heartbeat**: Automatic connection monitoring
- **Zero Dependencies**: <2KB flash footprint

### Quick Start

**Host (ESP32):**
```cpp
#include <RobustUART.h>
#include <RobustUART_host.h>

RobustHost<HardwareSerial> host(&Serial2);
host.begin(115200, 16, 17);

// Send ping
if (host.ping(500)) {
    Serial.println("Connected!");
}
```

**Client (ATmega):**
```cpp
#include <RobustUART.h>
#include <RobustUART_client.h>

RobustClient<HardwareSerial> client(&Serial);
client.begin(115200);

void loop() {
    client.process();
}
```

For detailed documentation, see [RobustUART README](../../lib/robust_uart/README.md).

---

## 2. Standard UART Library

For simple, raw UART communication without packetization, use the **uart_standard** library. The goal is to write UART code once and have it work on multiple platforms without modification.

## Features

-   **Cross-platform:** Write UART code once and use it on both ESP32 and AVR microcontrollers.
-   **Standardized Functions:** Provides a consistent API for initializing, writing to, and reading from UART ports.
-   **Multiple UART Ports:** Supports multiple UART ports on devices that have them (e.g., ATmega2560).

## API Reference

### `uart_init`

```c
void uart_init(uint8_t uart_num, uint32_t baud_rate);
```

Initializes a specific UART port for communication. This function must be called before any other UART operations on that port.

-   **`uart_num`**: The UART port to initialize. Use the provided definitions:
    -   `UART_0_PROG`: The primary UART port, typically used for programming and serial monitoring (e.g., `Serial` on Arduino).
    -   `UART_1_COMM`: The second UART port (e.g., `Serial1` on an Arduino Mega).
    -   `UART_2_DEBUG`: The third UART port (e.g., `Serial2` on an Arduino Mega).
    -   `UART_3_COMM2`: The fourth UART port (e.g., `Serial3` on an Arduino Mega).
-   **`baud_rate`**: The communication speed in bits per second (e.g., 9600, 115200).

---

### `uart_write`

```c
void uart_write(uint8_t uart_num, const char *str);
```

Transmits a null-terminated string of characters over the specified UART port.

-   **`uart_num`**: The UART port to write to.
-   **`str`**: The character string to send.

---

### `uart_read`

```c
int uart_read(uint8_t uart_num, uint8_t *buffer, uint32_t len);
```

Reads a specified number of bytes from the UART port's receive buffer.

-   **`uart_num`**: The UART port to read from.
-   **`buffer`**: A pointer to the byte array where the incoming data will be stored.
-   **`len`**: The maximum number of bytes to read into the buffer.
-   **Returns**: The number of bytes actually read, or -1 if an error occurred. *Note: The AVR implementation of this function is currently a placeholder and will always return -1.*

## Example Usage (Arduino / ATmega328P)

This example demonstrates how to initialize the UART port and send a message every second. This code will work on an Arduino Uno, Nano, or any other ATmega328P-based board.

```cpp
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

// The extern "C" is important because the main file is C++ (Arduino)
// while our library is plain C. This prevents linker errors.
extern "C" {
  #include "uart_standard.h"
}

// Define the task that will handle the UART communication
void uartTask(void *pvParameters) {
  // Initialize the primary UART port at 9600 baud.
  // This is equivalent to Serial.begin(9600) in standard Arduino.
  uart_init(UART_0_PROG, 9600);

  for (;;) {
    // Send a string over the UART port.
    uart_write(UART_0_PROG, "Hello from the UART task!\n");

    // Wait for 1 second before sending the next message.
    // vTaskDelay is used instead of delay() in an RTOS environment.
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Create the UART task.
  xTaskCreate(
    uartTask,           // Function that implements the task.
    "UART Task",        // Text name for the task.
    128,                // Stack size in words.
    NULL,               // Parameter passed into the task.
    1,                  // Priority of the task.
    NULL                // Task handle.
  );
}

void loop() {
  // The loop is empty because the RTOS scheduler is now in control.
}
```

### Expected Output

If you open the Serial Monitor in the Arduino IDE or PlatformIO at 9600 baud, you will see the following message printed every second:

```
Hello from the UART task!
Hello from the UART task!
Hello from the UART task!
...
