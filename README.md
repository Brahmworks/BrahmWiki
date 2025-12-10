# Machani ROS2 Motor Control System

This repository hosts the ESP32 firmware and libraries for the **Machani ROS2 Motor Control System**, a 3-axis robot control platform (Torso, Neck, Head). It communicates with a Jetson Nano (running ROS 2 Foxy) over **Ethernet (LAN)** using **MicroROS**.

## System Architecture

The Jetson Nano acts as the command controller. The ESP32 runs a MicroROS node that subscribes to command topics and publishes sensor data.

```
             ROS2 Foxy on Jetson
                 +---------------------------+
                 | MicroROS Agent            |
                 | - Serial/UDP Bridge       |
                 +------------+--------------+
                              |
                      Ethernet (LAN)
                       DDS/XRCE-DDS
                              |
                   +----------v-----------+
                   |      ESP32          |
                   | MicroROS Node       |
                   | /torso_cmd (Sub)    |
                   | /neck_cmd  (Sub)    |
                   | /head_cmd  (Sub)    |
                   | /led       (Sub)    |
                   | /touch     (Pub)    |
                   | /batt      (Pub)    |
                   +----------+----------+
                              |
          +-------------------+--------------------+
          |                   |                    |
 +--------v--------+  +-------v-------+   +--------v--------+
 |  Servo Drivers  |  |  Touch/LEDs   |   | Battery Monitor |
 +-----------------+  +---------------+   +-----------------+
```

## Communication Protocol (MicroROS)

The system uses standard ROS 2 topics. The payload for commands is a JSON string to maintain flexibility.

### Topics

| Topic | Type | Direction | Payload Example | Description |
|-------|------|-----------|-----------------|-------------|
| `/torso_cmd` | `std_msgs/String` | Jetson -> ESP | `{"angle":180,"speed":600,"accel":50}` | Control Torso Servo |
| `/neck_cmd` | `std_msgs/String` | Jetson -> ESP | `{"angle":0,"speed":100,"accel":20}` | Control Neck Servo |
| `/head_cmd` | `std_msgs/String` | Jetson -> ESP | `{"angle":10,"speed":200,"accel":30}` | Control Head Servo |
| `/led` | `std_msgs/String` | Jetson -> ESP | `{"state":"listen"}` | Control Status LED |
| `/reb_jet` | `std_msgs/String` | Jetson -> ESP | `{"reb_jet":true}` | Reboot Jetson Signal |
| `/touch` | `std_msgs/String` | ESP -> Jetson | `{"state":"tap"}` | Touch Event Detected |
| `/batt` | `std_msgs/String` | ESP -> Jetson | `{"batt":80, "ps":true, "hb_out":1}` | Battery Status |

## Hardware & Features

The ESP32 firmware manages the following hardware components:

*   **Motors:** 3-Axis control via Servo Drivers (UART).
*   **Touch Interface:** Capacitive touch input detection (GPIO 33).
*   **Feedback:** WS2812B LED status indication (GPIO 27).
*   **Power:** Battery monitoring via I2C (GPIO 21 SDA, 22 SCL).
*   **Connectivity:** Wired Ethernet (LAN) via ENC28J60 (SPI).

## Setup & Usage

For detailed instructions on setting up the hardware, flashing the firmware, and running the system, please refer to the **[User Manual](USER_MANUAL.md)**.

## Base Libraries & Documentation

*   **Connectivity:** [UART](docs/libraries/uart_standard.md) standard library.
*   **Microcontrollers:** Documentation for [ESP32](docs/microcontrollers/esp32.md).

For detailed documentation on all libraries, please visit the [BrahmWiki Documentation Site](https://brahmworks.github.io/BrahmWiki/).
