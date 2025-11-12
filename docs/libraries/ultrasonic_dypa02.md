# DYP-A02 UART Ultrasonic Sensor Library

[![PlatformIO](https://img.shields.io/badge/PlatformIO-FF6B35?style=flat&logo=platformio&logoColor=white)](https://platformio.org/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-3.3.5-blue)](https://docs.espressif.com/projects/esp-idf/en/latest/)
[![Arduino](https://img.shields.io/badge/Arduino-1.8.19-green)](https://www.arduino.cc/)

A cross-platform C library designed to interface with the DYP-A02 (and compatible) UART-based ultrasonic distance sensors. It supports both multi-threaded environments, such as ESP-IDF, and simpler platforms like Arduino.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Usage](#usage)
- [Configuration](#configuration)
- [Examples](#examples)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)
- [Best Practices](#best-practices)

## Features

- **Cross-Platform**: Works on ESP-IDF (ESP32) and Arduino platforms with a single, unified C API.
- **Structured API**: Uses an object-oriented C approach. Pass a `dyp_sensor_t` handle (struct) to all functions, allowing multiple sensors on one device.
- **Thread-Safe (ESP-IDF)**: Each sensor handle contains its own mutex, making all library calls 100% thread-safe. Safely read from different tasks.
- **Robust Error Handling**: Functions return clear, enumerated `dyp_err_t` error codes.
- **Human-Readable Errors**: Built-in helper `dyp_a02_strerror()` converts error codes into descriptive strings for logging (e.g., "Packet checksum mismatch").
- **Production-Ready Read Function**: Includes `dyp_a02_get_stable_distance_mm()`, designed to get reliable readings even with occasional hardware errors.
- **Application-Specific Logic**: Correctly handles "no object" (e.g., pointing at a wall) as a successful long-distance reading, not a filter error.
- **Voltage Compatibility**: Works with both 3.3V and 5V systems without additional circuitry.

## Installation

1. Clone or download this repository.
2. Copy the `lib/ultrasonic_dypa02/` directory to your project's library folder.
3. Include the header in your code: `#include "ultrasonic_dypa02.h"`

For PlatformIO projects, add the library path to your `platformio.ini`:

```ini
lib_extra_dirs = lib/
```

## Hardware Setup

The DYP-A02 sensor is compatible with both 3.3V and 5V systems. Connect the sensor directly to your microcontroller without voltage dividers or level shifters.

### Wiring Instructions

- **Sensor Power**: Connect the sensor's VCC pin to your system's voltage (3.3V or 5V).
- **Sensor TX -> MCU RX**: Connect directly.
- **MCU TX -> Sensor RX**: Connect directly.
- **GND**: Connect to common ground.

### Wiring Table

| Sensor Pin | ESP32/Arduino Pin | Notes |
|------------|-------------------|-------|
| VCC | 3.3V or 5V | Compatible with both voltages. |
| GND | GND | Connect to common ground. |
| TX | MCU RX Pin (e.g., GPIO 27 on ESP32, Pin 10 on Arduino) | Direct connection. |
| RX | MCU TX Pin (e.g., GPIO 26 on ESP32, Pin 11 on Arduino) | Direct connection. |

## Usage

### Basic Setup

#### Step 1: Include the Library

```c
#include "ultrasonic_dypa02.h"
```

#### Step 2: Create a Sensor Handle

Create one `dyp_sensor_t` struct for each sensor:

```c
// Create a sensor handle and initialize it to zero
static dyp_sensor_t my_sensor = DYP_SENSOR_INITIALIZER;
```

#### Step 3: Initialize the Sensor

For ESP-IDF (ESP32):

```c
// Define your pins
#define MY_SENSOR_RX_PIN 27 // ESP32's RX
#define MY_SENSOR_TX_PIN 26 // ESP32's TX

// Initialize the sensor handle
dyp_err_t err = dyp_a02_init_esp_hw(
    &my_sensor,                 // Pointer to your handle
    DYP_A02_DEFAULT_UART_PORT,  // Use UART_NUM_2
    MY_SENSOR_RX_PIN,
    MY_SENSOR_TX_PIN
);

if (err != DYP_OK) {
    printf("Failed to init sensor: %s\n", dyp_a02_strerror(err));
    // Halt or return
}
```

For Arduino (using SoftwareSerial):

```cpp
#include <SoftwareSerial.h>

// Define your pins
#define SENSOR_RX_PIN 10 // Arduino RX
#define SENSOR_TX_PIN 11 // Arduino TX

SoftwareSerial mySerial(SENSOR_RX_PIN, SENSOR_TX_PIN);

// Initialize the sensor handle
dyp_err_t err = dyp_a02_init(&my_sensor, &mySerial);

if (err != DYP_OK) {
    Serial.print("Failed to init sensor: ");
    Serial.println(dyp_a02_strerror(err));
    // Halt or return
}
```

For Arduino (using Hardware Serial):

```cpp
// Initialize the sensor handle with Serial
dyp_err_t err = dyp_a02_init_stream(&my_sensor, &Serial);

if (err != DYP_OK) {
    Serial.print("Failed to init sensor: ");
    Serial.println(dyp_a02_strerror(err));
    // Halt or return
}
```

### Reading the Sensor

#### Recommended Method: Stable Distance Reading

The best function for production use is `dyp_a02_get_stable_distance_mm()`. It tries to get multiple hardware-valid readings and averages them for reliability.

```c
// Configuration parameters
const int SAMPLES_TO_TRY = 20;     // Try 20 times to get a packet
const int MIN_SAMPLES_REQ = 5;     // Need at least 5 good ones
const int PER_READ_TIMEOUT = 200;  // 200ms for each packet
const int OVERALL_TIMEOUT = 3000;  // Give up after 3 seconds

// Read the sensor
long distance_mm = dyp_a02_get_stable_distance_mm(
    &my_sensor,
    SAMPLES_TO_TRY,
    MIN_SAMPLES_REQ,
    PER_READ_TIMEOUT,
    OVERALL_TIMEOUT
);
```

#### Alternative Method: Single Packet Reading

For applications requiring faster readings or handling raw data:

```c
dyp_packet_t packet;
bool success = dyp_a02_read_packet(&my_sensor, &packet, 1000); // 1 second timeout

if (success) {
    long distance_mm = packet.distance_mm;
    // Use distance_mm
} else {
    // Handle error
}
```

### Handling Results and Errors

Always check the return value from read functions:

```c
if (distance_mm != DYP_A02_READ_ERROR) {
    // SUCCESS! We have a valid distance.
    printf("Distance: %ld mm (%.2f cm)\n", distance_mm, distance_mm / 10.0f);
    
    // Check for an object (e.g., a cup)
    bool is_cup_present = dyp_a02_is_object_detected(distance_mm, 80); // 80mm threshold
    if (is_cup_present) {
        printf("Cup is PRESENT\n");
    } else {
        printf("No cup detected.\n");
    }
} else {
    // ERROR! The read failed.
    // Get the specific hardware error from the handle
    dyp_err_t last_err = dyp_a02_get_last_error(&my_sensor);
    
    // Log the human-readable error string
    printf("Sensor read failed: %s\n", dyp_a02_strerror(last_err));
}
```

### Multiple Sensors

You can use multiple sensors by creating separate handles:

```c
static dyp_sensor_t sensor1 = DYP_SENSOR_INITIALIZER;
static dyp_sensor_t sensor2 = DYP_SENSOR_INITIALIZER;

// Initialize each sensor with different pins/ports
// Then read from each independently
```

## Examples

See the `examples/` directory for complete working examples:

- [ESP32 Example](examples/ultrasonic_dypa02_esp32/)
- [ATmega328p Example](examples/ultrasonic_dypa02_atmega328p/)
- [ATmega2560 Example](examples/ultrasonic_dypa02_atmega2560/)

## Configuration

### Tuning Parameters

The `dyp_a02_get_stable_distance_mm()` function accepts several parameters to tune performance:

- **`samples_to_try`**: Maximum number of read attempts (default: 20). Higher values increase reliability but take longer.
- **`min_samples_req`**: Minimum good readings needed (default: 5). Ensures statistical validity.
- **`per_read_timeout_ms`**: Timeout per packet (default: 200ms). Should match sensor's measurement cycle.
- **`overall_timeout_ms`**: Total timeout (default: 3000ms). Prevents indefinite waiting.

### Example Configurations

**Fast Mode** (for quick readings, less reliable):
```c
long distance = dyp_a02_get_stable_distance_mm(&sensor, 5, 1, 100, 500);
```

**High Precision Mode** (for accurate measurements):
```c
long distance = dyp_a02_get_stable_distance_mm(&sensor, 50, 10, 300, 10000);
```

## API Reference

### Data Types

- **`dyp_sensor_t`**: Sensor handle struct. Initialize with `DYP_SENSOR_INITIALIZER`.
- **`dyp_packet_t`**: Raw packet data struct containing distance and other sensor info.
- **`dyp_err_t`**: Error code enumeration.

### Initialization Functions

#### `dyp_err_t dyp_a02_init_esp_hw(dyp_sensor_t *sensor, uart_port_t uart_port, int rx_pin, int tx_pin)`

Initializes the sensor for ESP-IDF hardware UART.

**Parameters:**
- `sensor`: Pointer to sensor handle
- `uart_port`: UART port (e.g., `UART_NUM_2`)
- `rx_pin`: MCU RX pin connected to sensor TX
- `tx_pin`: MCU TX pin connected to sensor RX

**Returns:** `DYP_OK` on success, error code otherwise.

**Example:**
```c
dyp_err_t err = dyp_a02_init_esp_hw(&my_sensor, UART_NUM_2, 27, 26);
```

#### `dyp_err_t dyp_a02_init(dyp_sensor_t *sensor, SoftwareSerial *serial)`

Initializes the sensor for Arduino using SoftwareSerial.

**Parameters:**
- `sensor`: Pointer to sensor handle
- `serial`: Pointer to SoftwareSerial object

**Returns:** `DYP_OK` on success, error code otherwise.

**Example:**
```cpp
SoftwareSerial mySerial(10, 11);
dyp_err_t err = dyp_a02_init(&my_sensor, &mySerial);
```

#### `dyp_err_t dyp_a02_init_stream(dyp_sensor_t *sensor, Stream *serial)`

Initializes the sensor for Arduino using hardware Serial.

**Parameters:**
- `sensor`: Pointer to sensor handle
- `serial`: Pointer to Stream object (e.g., `&Serial`)

**Returns:** `DYP_OK` on success, error code otherwise.

**Example:**
```cpp
dyp_err_t err = dyp_a02_init_stream(&my_sensor, &Serial);
```

#### `void dyp_a02_deinit(dyp_sensor_t *sensor)`

Safely de-initializes the sensor driver and frees resources.

**Parameters:**
- `sensor`: Pointer to sensor handle

### Reading Functions

#### `long dyp_a02_get_stable_distance_mm(dyp_sensor_t *sensor, int samples_to_try, int min_samples_req, int per_read_timeout_ms, int overall_timeout_ms)`

Production-ready function that attempts multiple readings and returns an averaged distance.

**Parameters:**
- `sensor`: Pointer to sensor handle
- `samples_to_try`: Max attempts to read packets
- `min_samples_req`: Min valid readings required
- `per_read_timeout_ms`: Timeout per packet read
- `overall_timeout_ms`: Total timeout

**Returns:** Distance in mm, or `DYP_A02_READ_ERROR` on failure.

**Example:**
```c
long dist = dyp_a02_get_stable_distance_mm(&sensor, 20, 5, 200, 3000);
if (dist != DYP_A02_READ_ERROR) {
    printf("Distance: %ld mm\n", dist);
}
```

#### `bool dyp_a02_read_packet(dyp_sensor_t *sensor, dyp_packet_t *packet, int timeout_ms)`

Low-level function to read a single raw packet from the sensor.

**Parameters:**
- `sensor`: Pointer to sensor handle
- `packet`: Pointer to packet struct to fill
- `timeout_ms`: Read timeout

**Returns:** `true` on success, `false` on failure.

**Example:**
```c
dyp_packet_t pkt;
if (dyp_a02_read_packet(&sensor, &pkt, 1000)) {
    long dist = pkt.distance_mm;
}
```

#### `long dyp_a02_read_average_filtered_int(dyp_sensor_t *sensor, int samples, int timeout_ms)`

**DEPRECATED**: Do not use. Use `dyp_a02_get_stable_distance_mm()` instead.

### Utility Functions

#### `bool dyp_a02_is_object_detected(long distance_mm, long threshold_mm)`

Checks if an object is detected based on distance threshold.

**Parameters:**
- `distance_mm`: Measured distance
- `threshold_mm`: Detection threshold

**Returns:** `true` if object detected (distance < threshold), `false` otherwise.

**Example:**
```c
if (dyp_a02_is_object_detected(distance, 100)) {
    printf("Object detected!\n");
}
```

#### `dyp_err_t dyp_a02_get_last_error(dyp_sensor_t *sensor)`

Gets the last error code from the sensor handle.

**Parameters:**
- `sensor`: Pointer to sensor handle

**Returns:** Last error code.

#### `const char *dyp_a02_strerror(dyp_err_t err)`

Converts error code to human-readable string.

**Parameters:**
- `err`: Error code

**Returns:** Error description string.

**Example:**
```c
dyp_err_t err = dyp_a02_get_last_error(&sensor);
printf("Error: %s\n", dyp_a02_strerror(err));
```

## Troubleshooting

All read functions return `DYP_A02_READ_ERROR` on failure. Call `dyp_a02_get_last_error(&my_sensor)` to get the specific error.

### Common Issues

1. **No readings or timeout errors**: Check wiring connections. Ensure sensor is powered and GND is connected properly.
2. **Inconsistent readings**: Sensor may be affected by acoustic noise or multiple echoes. Try adjusting position or adding acoustic damping.
3. **Voltage issues**: Although the sensor works with both 3.3V and 5V, ensure stable power supply.

| Error Code | String | Description & Troubleshooting |
|------------|--------|-------------------------------|
| `DYP_OK` | "Success" | All good. |
| `DYP_ERR_TIMEOUT` | "Read timeout (no bytes)" | Check wiring. Ensure Sensor TX is connected to MCU RX. |
| `DYP_ERR_NOT_ENOUGH_SAMPLES` | "Not enough good samples" | Sensor not returning valid data. Check power supply and GND connections. |
| `DYP_ERR_TIMEOUT_OVERALL` | "Filtered average timed out" | Increase timeout parameters or check sensor health. |
| `DYP_ERR_CHECKSUM` | "Packet checksum mismatch" | Data corruption. Check for electrical noise or wiring issues. |
| `DYP_ERR_INVALID_READING` | "Invalid reading (e.g., 0mm)" | Sensor returned invalid data. Library will retry automatically. |
| `DYP_ERR_UART_*` | Various | UART driver errors. Check pins not used elsewhere and UART configuration. |
| `DYP_ERR_MUTEX_FAIL` | "Mutex operation failed" | FreeRTOS error. Should not happen normally. |
| `DYP_ERR_NOT_INITIALIZED` | "Sensor not initialized" | Call initialization function before reading. |

## Best Practices

- **Power Supply**: Use a stable power source. Ultrasonic sensors can be sensitive to voltage fluctuations.
- **Mounting**: Mount the sensor securely to avoid vibrations that could affect readings.
- **Environment**: Avoid acoustic noise sources. Soft surfaces can cause multiple echoes.
- **Timing**: Don't read too frequently. Allow time for sound waves to dissipate (typically 50-100ms between readings).
- **Error Handling**: Always check return values and implement proper error recovery.
- **Multiple Sensors**: If using multiple sensors, space them apart to avoid cross-interference.
- **Calibration**: Verify readings against known distances periodically.
