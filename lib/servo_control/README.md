# Servo Control Library

This library provides an interface to control daisy-chained servos (specifically using the SCServo SDK) via JSON commands or direct function calls. It is designed to be transport-agnostic, allowing integration with UART, SPI, or network-based communication layers.

## Dependencies

-   **SCServo**: The Feetech SCServo SDK for Arduino/ESP32.
-   **cJSON**: For parsing JSON commands.
-   **Arduino Framework**: Uses `HardwareSerial` and `Arduino.h`.

## Usage

### Initialization

```cpp
#include "servo_control.h"

ServoControl servos;
HardwareSerial MySerial(1);

void setup() {
    // Initialize with Serial object, RX pin, and TX pin
    servos.begin(&MySerial, 22, 21);
}
```

### Controlling Servos

You can control servos directly:

```cpp
// ID, Angle (0-360), Speed, Acceleration
servos.moveServo(SERVO_ID_NECK, 180, 1000, 50);
```

Or using JSON commands:

```cpp
const char* command = "{\"angle\":180,\"speed\":600,\"accel\":50}";
servos.processCommand(command, SERVO_ID_TORSO);
```

### JSON Command Structure

The library expects a JSON object with the following fields:

-   `angle`: Target angle in degrees (0-360). Default: 0.
-   `speed`: Movement speed. Default: 1000.
-   `accel`: Acceleration. Default: 20.

Example: `{"angle": 90, "speed": 500, "accel": 10}`
