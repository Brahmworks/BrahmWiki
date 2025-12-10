# Machani ROS2 Motor Control System

This repository hosts the ESP32 firmware and libraries for the **Machani ROS2 Motor Control System**, a 3-axis robot control platform (Torso, Neck, Head). It communicates with a Jetson Nano (running ROS 2 Foxy) over WiFi using a JSON-based HTTP protocol.

## System Architecture

The Jetson Nano acts as the command controller, translating ROS 2 topic messages into HTTP requests sent to the ESP32. The ESP32 parses these requests and drives the motors, controls LEDs, handles touch inputs, and monitors battery status.

```
             ROS2 Foxy on Jetson
                 +---------------------------+
                 | motor_controller.py       |
                 | - Subscribes to ROS topics|
                 | - Sends HTTP -> ESP       |
                 +------------+--------------+
                              |
                        WiFi HTTP (JSON)
                              |
                   +----------v-----------+
                   |      ESP32          |
                   | WebServer Endpoints |
                   | /torso_move         |
                   | /neck_move          |
                   | /head_move          |
                   | /led                |
                   | /reb_jet            |
                   +----------+----------+
                              |
          +-------------------+--------------------+
          |                   |                    |
 +--------v--------+  +-------v-------+   +--------v--------+
 |  Servo Drivers  |  |  Touch/LEDs   |   | Battery Monitor |
 +-----------------+  +---------------+   +-----------------+
```

## Communication Protocol

The system uses a JSON-based protocol over HTTP. The ESP32 hosts a web server with specific endpoints for control and monitoring.

### Endpoints & Topics

| Function | ROS Topic | ESP Endpoint | Direction | Payload Example | Description |
|----------|-----------|--------------|-----------|-----------------|-------------|
| **Torso** | `/torso_cmd` | `/torso_move` | Jetson -> ESP | `{"angle":180,"speed":600,"accel":50}` | Angle: -135 to +135<br>Speed: up to 999<br>Accel: up to 100 |
| **Neck** | `/neck_cmd` | `/neck_move` | Jetson -> ESP | `{"angle":180,"speed":600,"accel":50}` | Angle: -30 to +30<br>Speed: up to 999<br>Accel: up to 100 |
| **Head** | `/head_cmd` | `/head_move` | Jetson -> ESP | `{"angle":180,"speed":600,"accel":50}` | Angle: -15 to +15<br>Speed: up to 999<br>Accel: up to 100 |
| **LED** | `/led` | `/led` | Jetson -> ESP | `{"state":"listen"}` | States: listen, mute, loading, offline, connecting, low battery |
| **Reboot** | `/reb_jet` | `/reb_jet` | Jetson -> ESP | `{"reb_jet":true}` | Signal to reboot the Jetson system |
| **Touch** | `/touch` | N/A (Sub) | ESP -> Jetson | `{"state":"tap"}` | States: tap, double, up, down, long |
| **Battery** | `/batt` | N/A (Sub) | ESP -> Jetson | `{"batt":80, "ps":true, "hb_out":1}` | batt: 0-100%<br>ps: power source connected<br>hb_out: counter 1-255 |

### Usage Examples

**ROS 2 Publish Command (Jetson Side):**
```bash
ros2 topic pub /torso_cmd std_msgs/msg/String "data: '{\"angle\":180,\"speed\":600,\"accel\":50}'"
```

**Direct HTTP POST (Testing):**
```bash
curl -X POST http://<ESP_IP>:5000/torso_move -d "cmd={\"angle\":150,\"speed\":500,\"accel\":40}"
```

## Hardware & Features

The ESP32 firmware manages the following hardware components:

*   **Motors:** 3-Axis control via Servo Drivers.
*   **Touch Interface:** Capacitive touch input detection (Tap, Swipe, Long press).
*   **Feedback:** WS2812B LED status indication.
*   **Power:** Battery monitoring via I2C.

## ROS Package Structure (Jetson)

For reference, the companion ROS 2 package on the Jetson has the following structure:

```
ros2_ws/
├── src/
│   └── motor_control_pkg/
│       ├── motor_controller.py  # Subscribes to ROS topics, sends HTTP to ESP
│       ├── esp_monitor.py       # Pings ESP, publishes status
│       └── ...
```

## Base Libraries & Documentation

This repository also includes the core libraries used to build the Machani system, supporting ESP32, ATmega328P, and ATmega2560.

*   **[IoT Communication Protocol](docs/iot_communication_protocol.md):** Detailed JSON architecture for device handshakes and health checks.
*   **Sensors:** Libraries for [MAX6675](docs/libraries/MAX_6675-k-type-thermo.md), [MAX31865](docs/libraries/MAX31865_PT100-thermo.md), and [Ultrasonic DYPA02](docs/libraries/ultrasonic_dypa02.md).
*   **Connectivity:** [WiFi](docs/libraries/wifi.md) and [UART](docs/libraries/uart_standard.md) standard libraries.
*   **Microcontrollers:** Documentation for [ESP32](docs/microcontrollers/esp32.md), [ATmega328P](docs/microcontrollers/atmega328.md), and [ATmega2560](docs/microcontrollers/atmega2560.md).

For detailed documentation on all libraries, please visit the [BrahmWiki Documentation Site](https://brahmworks.github.io/BrahmWiki/).
