# BrahmWiki Embedded Systems Projects

This repository contains a collection of embedded systems projects and libraries, focusing on RTOS-based development for ESP32, ATmega328P, and ATmega2560 microcontrollers.

For detailed documentation, please visit the [BrahmWiki Documentation Site](https://brahmworks.github.io/BrahmWiki/).

## Current Projects

-   **RTOS Blink Examples:** Simple "Blink LED" projects using a Real-Time Operating System (RTOS) for all three platforms, demonstrating the use of the custom serial library.

## IoT Communication Protocol

This project includes a standardized JSON-based communication protocol for IoT devices. The protocol is designed to be lightweight and efficient, making it suitable for resource-constrained devices.

### Message Types

The protocol defines the following message types:

-   `handshake`: Establishes the initial connection and authentication between the device and the server.
-   `health_check`: Provides a periodic status update from the device to the server.
-   `command`: Allows the server to send commands to the device.
-   `error`: Reports errors that occur during communication.

### JSON Architecture

The following is a brief overview of the JSON architecture for each message type. For a more detailed explanation, please refer to the [IoT Communication Protocol Documentation](docs/iot_communication_protocol.md).

#### Handshake


**Request:**
```json
{
  "messageType": "handshake",
  "version": "1.0",
  "request": {
    "deviceId": "device_12345",
    "deviceType": "sensor",
    "firmwareVersion": "2.1.4"
  }
}
```

**Response:**
```json
{
  "status": "accepted",
  "statusCode": 200,
  "sessionId": "sess_67890",
  "serverTime": "2025-11-01T16:40:00.500Z",
  "heartbeatInterval": 30,
  "commandEndpoint": "mqtt://broker.example.com:8883/device/12345/commands",
  "healthEndpoint": "https://api.example.com/device/12345/health",
  "configuration": {
    "reportingInterval": 60
  }
}
```

#### Health Check

**Request:**
```json
{
  "messageType": "health_check",
  "version": "1.0",
  "deviceId": "device_12345"
}
```

**Response:**
```json
{
  "messageType": "health_check",
  "version": "1.0",
  "deviceId": "device_12345",
  "status": "pass",
  "statusCode": 200,
  "checks": {
    "sensor:sensor_1": [
      {
        "componentId": "sensor_1",
        "componentType": "sensor",
        "status": "pass",
        "observedValue": 85.5,
        "observedUnit": "celsius"
      }
    ],
    "sensor:sensor_2": [
      {
        "componentId": "sensor_2",
        "componentType": "sensor",
        "status": "pass",
        "observedValue": 25.0,
        "observedUnit": "celsius"
      }
    ],
    "sensor:sensor_3": [
      {
        "componentId": "sensor_3",
        "componentType": "sensor",
        "status": "pass",
        "observedValue": 15.2,
        "observedUnit": "cm"
      }
    ],
    "actuator:actuator_1": [
      {
        "componentId": "actuator_1",
        "componentType": "actuator",
        "status": "pass",
        "observedValue": "off"
      }
    ],
    "actuator:actuator_2": [
      {
        "componentId": "actuator_2",
        "componentType": "actuator",
        "status": "pass",
        "observedValue": "closed"
      }
    ],
    "actuator:actuator_3": [
      {
        "componentId": "actuator_3",
        "componentType": "actuator",
        "status": "pass",
        "observedValue": "off"
      }
    ]
  }
}
```

#### Command

**Request:**
```json
{
  "messageType": "command",
  "commandType": "control",
  "version": "1.0",
  "commandId": "cmd_789123",
  "deviceId": "device_12345",
  "command": {
    "action": "set_state",
    "parameters": {
      "component": "actuator_1",
      "state": "on"
    }
  }
}
```

**Response:**
```json
{
  "statusCode": 200,
  "status": "success"
}
```

### Constants

To ensure consistency and reduce storage overhead, the protocol uses a set of predefined constants for message types, device types, and sensor/actuator IDs. These constants are defined in the `src/esp32_main/components/iot_protocol/iot_protocol_defs.h` file.

### Status and Error Codes

The following table provides a list of status and error codes used in the protocol:

| Code | Description |
| --- | --- |
| **2xx: Success** | |
| 200 | OK |
| 201 | Created |
| 202 | Accepted |
| 204 | No Content |
| **4xx: Client Error** | |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 408 | Request Timeout |
| 429 | Too Many Requests |
| **5xx: Server Error** | |
| 500 | Internal Server Error |
| 502 | Bad Gateway |
| 503 | Service Unavailable |
| 504 | Gateway Timeout |
| **6xx: Device-Specific Error** | |
| 600 | Device Offline |
| 601 | Low Battery |
| 602 | Sensor Error |
| 603 | Memory Full |
| 604 | Invalid State |

### Usage Example

The `iot_protocol` library is designed to be transport-agnostic. The following example demonstrates how to use it with the `uart_standard` library, but it can be easily adapted for other communication channels like Wi-Fi or Bluetooth.

A complete, runnable example can be found in the `examples/iot_protocol_example` directory.

#### 1. Device Configuration

First, create a separate header file (e.g., `device_settings.h`) to define all your project-specific constants. This keeps your configuration clean and separate from the application logic.

```c
// examples/iot_protocol_example/src/device_settings.h

#ifndef DEVICE_SETTINGS_H
#define DEVICE_SETTINGS_H

// --- Project-Specific Constants ---
#define DEVICE_ID "device_12345"
#define DEVICE_TYPE "ESP32-System"
#define FIRMWARE_VERSION "3.0.0"

// --- Sensor and Actuator IDs ---
#define SENSOR_ID_HEATER_TEMP "heater_temp"
#define SENSOR_ID_WATER_TEMP "water_temp"
#define SENSOR_ID_ULTRASONIC "ultrasonic"

#define ACTUATOR_ID_HEATER "heater"
#define ACTUATOR_ID_SOLENOID "solenoid"
#define ACTUATOR_ID_PUMP "pump"

// --- User-Defined Sensor Thresholds ---
#define HEATER_TEMP_WARN_THRESHOLD 80.0
#define HEATER_TEMP_FAIL_THRESHOLD 100.0

#define WATER_TEMP_WARN_THRESHOLD 40.0
#define WATER_TEMP_FAIL_THRESHOLD 60.0

#define ULTRASONIC_WARN_THRESHOLD 5.0
#define ULTRASONIC_FAIL_THRESHOLD 20.0

#endif // DEVICE_SETTINGS_H
```

#### 2. Application Logic

In your main application file, include the necessary headers and create the configuration and data structures that will be used by the library.

```c
// examples/iot_protocol_example/src/main.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_standard.h"
#include "iot_protocol.h"
#include "device_settings.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// --- Device Configuration ---
static const sensor_config_t SENSOR_CONFIGS[] = {
    {SENSOR_ID_HEATER_TEMP, {HEATER_TEMP_WARN_THRESHOLD, HEATER_TEMP_FAIL_THRESHOLD}},
    {SENSOR_ID_WATER_TEMP, {WATER_TEMP_WARN_THRESHOLD, WATER_TEMP_FAIL_THRESHOLD}},
    {SENSOR_ID_ULTRASONIC, {ULTRASONIC_WARN_THRESHOLD, ULTRASONIC_FAIL_THRESHOLD}}
};

static const actuator_config_t ACTUATOR_CONFIGS[] = {
    {ACTUATOR_ID_HEATER},
    {ACTUATOR_ID_SOLENOID},
    {ACTUATOR_ID_PUMP}
};

static const device_config_t DEVICE_CONFIG = {
    .device_id = DEVICE_ID,
    .device_type = DEVICE_TYPE,
    .firmware_version = FIRMWARE_VERSION,
    .sensors = (sensor_config_t*)SENSOR_CONFIGS,
    .num_sensors = sizeof(SENSOR_CONFIGS) / sizeof(sensor_config_t),
    .actuators = (actuator_config_t*)ACTUATOR_CONFIGS,
    .num_actuators = sizeof(ACTUATOR_CONFIGS) / sizeof(actuator_config_t)
};

// --- Simulation Functions ---
// In a real application, these would read from actual hardware
double read_sensor(const char* id) {
    if (strcmp(id, SENSOR_ID_HEATER_TEMP) == 0) {
        int r = rand() % 100;
        if (r > 95) return 105.0;
        if (r > 85) return 85.0;
        return 75.0;
    }
    if (strcmp(id, SENSOR_ID_WATER_TEMP) == 0) return 25.0;
    if (strcmp(id, SENSOR_ID_ULTRASONIC) == 0) return 15.2;
    return 0.0;
}

const char* read_actuator(const char* id) {
    if (strcmp(id, ACTUATOR_ID_HEATER) == 0) return "off";
    if (strcmp(id, ACTUATOR_ID_SOLENOID) == 0) return "closed";
    if (strcmp(id, ACTUATOR_ID_PUMP) == 0) return "off";
    return "unknown";
}

void app_main()
{
    uart_init(UART_0_PROG, 115200);

    char* handshake_request = iot_protocol_get_handshake_request(&DEVICE_CONFIG);
    if (handshake_request != NULL) {
        uart_write(UART_0_PROG, "Sending Handshake:\n");
        uart_write(UART_0_PROG, handshake_request);
        uart_write(UART_0_PROG, "\n\n");
        free(handshake_request);
    }

    while(1) {
        uint8_t buffer[512];
        int len = uart_read(UART_0_PROG, buffer, sizeof(buffer) - 1);
        if (len > 0) {
            buffer[len] = '\0';
            uart_write(UART_0_PROG, "Received Data:\n");
            uart_write(UART_0_PROG, (const char*)buffer);
            uart_write(UART_0_PROG, "\n");

            sensor_data_t sensors[DEVICE_CONFIG.num_sensors];
            for (int i = 0; i < DEVICE_CONFIG.num_sensors; i++) {
                sensors[i] = (sensor_data_t){DEVICE_CONFIG.sensors[i].id, read_sensor(DEVICE_CONFIG.sensors[i].id)};
            }

            actuator_data_t actuators[DEVICE_CONFIG.num_actuators];
            for (int i = 0; i < DEVICE_CONFIG.num_actuators; i++) {
                actuators[i] = (actuator_data_t){DEVICE_CONFIG.actuators[i].id, read_actuator(DEVICE_CONFIG.actuators[i].id)};
            }

            device_data_t device_data = {
                .sensors = sensors,
                .num_sensors = DEVICE_CONFIG.num_sensors,
                .actuators = actuators,
                .num_actuators = DEVICE_CONFIG.num_actuators
            };

            char* response = iot_protocol_process_incoming((const char*)buffer, &DEVICE_CONFIG, &device_data);
            if (response != NULL) {
                uart_write(UART_0_PROG, "Sending Response:\n");
                uart_write(UART_0_PROG, response);
                uart_write(UART_0_PROG, "\n\n");
                free(response);
            }
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
```
