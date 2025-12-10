# User Manual: Machani ROS2 Motor Control System

This manual provides instructions for setting up, configuring, and running the Machani ROS2 Motor Control System, which integrates a Jetson Nano (ROS 2) with an ESP32 (MicroROS) over Ethernet.

## 1. Hardware Setup

### 1.1 Components
*   **Controller:** Jetson Nano (Running Ubuntu 20.04 & ROS 2 Foxy)
*   **Driver:** ESP32 Development Board
*   **Ethernet:** ENC28J60 Module
*   **Motors:** 3x Serial Bus Servos (Torso, Neck, Head)
*   **Interface:** Capacitive Touch Sensor, WS2812B LED Ring
*   **Power:** Battery Pack with I2C Monitoring

### 1.2 Wiring Diagram

**ESP32 Pinout:**

| Component | Pin Name | GPIO | Notes |
|-----------|----------|------|-------|
| **Ethernet** (ENC28J60) | MOSI | 23 | SPI MOSI |
| | MISO | 19 | SPI MISO |
| | SCK | 18 | SPI Clock |
| | CS | 5 | Chip Select |
| **Motors** (UART) | TX | 4 | To Servo RX |
| | RX | 2 | To Servo TX |
| **LED** (WS2812B) | DIN | 27 | Data In |
| **Touch** | SIG | 33 | Capacitive Input |
| **Battery** (I2C) | SDA | 21 | I2C Data |
| | SCL | 22 | I2C Clock |

**Note:** Ensure common ground between ESP32, Servos, and Battery.

## 2. Software Installation

### 2.1 Jetson Nano (ROS 2)
1.  Install ROS 2 Foxy:
    ```bash
    sudo apt update
    sudo apt install ros-foxy-desktop
    ```
2.  Install MicroROS Agent:
    ```bash
    mkdir -p microros_ws/src
    cd microros_ws
    git clone -b foxy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    colcon build
    source install/local_setup.bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash
    ```
3.  Run the Agent (UDP/Ethernet):
    ```bash
    # Replace 8888 with your configured port
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    ```

### 2.2 ESP32 Firmware
1.  **Prerequisites:** Install ESP-IDF v4.4+.
2.  **Configuration:**
    *   Open `src/esp32_main/src/main.c`.
    *   Update `AGENT_IP` to match your Jetson Nano's IP address.
    *   Update `AGENT_PORT` to match the Agent's port (default 8888).
3.  **Build & Flash:**
    ```bash
    cd src/esp32_main
    idf.py set-target esp32
    idf.py menuconfig # Ensure MicroROS component is enabled and configured for UDP
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor
    ```

## 3. Operation

### 3.1 Startup Sequence
1.  Connect the Jetson Nano and ESP32 to the same Ethernet network (Switch/Router) or directly via cable.
2.  Power on the Jetson and start the MicroROS Agent (see 2.1).
3.  Power on the ESP32.
4.  The ESP32 logs will show "Ethernet Link Up", then "Got IP", and finally "MicroROS Node Started".

### 3.2 Controlling Motors
Publish JSON commands to the respective topics:

```bash
# Move Head
ros2 topic pub --once /head_cmd std_msgs/msg/String "data: '{\"angle\":10, \"speed\":200, \"accel\":20}'"

# Move Torso
ros2 topic pub --once /torso_cmd std_msgs/msg/String "data: '{\"angle\":90, \"speed\":500}'"
```

### 3.3 Monitoring Sensors
Subscribe to sensor topics:

```bash
# Monitor Touch Events
ros2 topic echo /touch

# Monitor Battery Status
ros2 topic echo /batt
```

### 3.4 LED Status
Change the LED indicator:
```bash
ros2 topic pub --once /led std_msgs/msg/String "data: '{\"state\":\"listen\"}'"
ros2 topic pub --once /led std_msgs/msg/String "data: '{\"state\":\"loading\"}'"
```

## 4. Troubleshooting

*   **Agent Connection Failed:**
    *   Check Ethernet cables.
    *   Verify `AGENT_IP` in `main.c` matches the Jetson's IP.
    *   Ensure the Jetson firewall (ufw) allows UDP traffic on port 8888.
*   **Motors Not Moving:**
    *   Check UART wiring (TX to RX).
    *   Verify Servo ID (1=Torso, 2=Neck, 3=Head).
    *   Check external power supply for servos (ESP32 cannot power them).
*   **Touch Not Detected:**
    *   Adjust `TOUCH_THRESH` in `main.c` if sensitivity is too low/high.
