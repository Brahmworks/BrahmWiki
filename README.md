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

#### Health Check

```json
{
  "messageType": "health_check",
  "version": "1.0",
  "deviceId": "device_12345",
  "status": "pass",
  "statusCode": 200
}
```

#### Command

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

### Constants

To ensure consistency and reduce storage overhead, the protocol uses a set of predefined constants for message types, device types, and sensor/actuator IDs. These constants are defined in the `src/esp32_main/components/iot_protocol/iot_protocol_defs.h` file.
