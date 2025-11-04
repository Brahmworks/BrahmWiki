# 🧭 DYP-A02YY Ultrasonic Sensor – UART Library

A simple and reliable Arduino/ESP32 library for communicating with **DYP-A02YY ultrasonic distance sensors** over UART.

---

## 📘 Overview

The **DYP-A02YY ultrasonic sensor** continuously outputs distance data via UART (serial communication).
This library reads that data, validates frames with checksum verification, and converts measurements into **centimeters, millimeters, meters, feet, or inches** — whichever unit you prefer.

It also includes helper functions for **threshold-based object detection**.

---

## 📡 Supported Sensors

This library is designed for **DYP-A02YY** and should work with similar UART-based DYP sensors:
- ✅ DYP-A02YYGDW
- ✅ DYP-A02YY
- ✅ DYP-A02 (UART version)

⚠️ **Not compatible** with I2C or PWM versions of DYP sensors.

---

## ⚙️ Features

* 🎯 Easy initialization using UART pins
* 📏 Multiple units: **mm, cm, m, inch, feet**
* 🚨 Threshold-based object detection (`object_detected_*`)
* ⏱️ Configurable timeout detection
* 🧹 Buffer flush function to clear corrupted data
* 🖥️ Works on both **Arduino** and **ESP-IDF (ESP32)** platforms
* ✅ Frame validation with checksum

---

## 📦 Installation

### Arduino IDE
1. Download this library as a ZIP file
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library**
3. Select the downloaded ZIP file


### Manual Installation
1. Copy `ultrasonic_dypa02.h` and `ultrasonic_dypa02.cpp` into your project's `lib/` or `src/` directory
2. Include the header in your code:
   ```cpp
   #include <ultrasonic_dypa02.h>
   ```

---

## 🪝 Hardware Connections

| DYP Sensor Pin | Description     | Connect To MCU Pin     |
| -------------- | --------------- | ---------------------- |
| **VCC**        | Power           | 5V (or 3.3V for ESP32) |
| **GND**        | Ground          | GND                    |
| **TX**         | Sensor → MCU    | RX pin (you define)    |
| **RX**         | MCU → Sensor    | TX pin (you define)    |

⚠️ **Important:** Sensor TX connects to MCU RX, and vice versa (crossover connection).

---

## 🖥️ Platform Notes

### Arduino (Uno, Nano, Mega, etc.)
- Uses `SoftwareSerial` library
- Works with any digital pins
- Recommended: Use pins that support pin change interrupts for better stability

### ESP32
- Uses **hardware UART** (UART1 by default)
- Recommended pins: **RX=16, TX=17** (or any available GPIO)
- Much more stable and faster than SoftwareSerial
- Avoid pins used by flash/boot (GPIO 6-11)

---

## 🧠 API Reference

### 🟢 Initialization

```cpp
int dyp_uart_init(int rx_pin, int tx_pin, int baud);
```

**Purpose:** Initializes the sensor with specified RX/TX pins and baud rate.

**Parameters:**
* `rx_pin` → MCU pin connected to sensor TX
* `tx_pin` → MCU pin connected to sensor RX
* `baud` → UART baud rate (use `9600` for DYP sensors, or `0` for default)

**Returns:** `0` on success, negative on failure.

**Example:**
```cpp
if (dyp_uart_init(A4, A5, 9600) != 0) {
    Serial.println("Sensor init failed!");
}
```

---

### 🔴 Deinitialization

```cpp
int dyp_uart_deinit(void);
```

**Purpose:** Safely releases UART resources. Call this if you need to stop using the sensor or reinitialize it.

---

### ⚡ Read Distance (Float Output)

All read functions wait until a valid data frame is received or the timeout expires.

| Function | Unit | Example |
|----------|------|---------|
| `dyp_uart_read_distance_mm_float(float *out, unsigned timeout_ms)` | millimeters | `dyp_uart_read_distance_mm_float(&dist, 100);` |
| `dyp_uart_read_distance_cm_float(float *out, unsigned timeout_ms)` | centimeters | `dyp_uart_read_distance_cm_float(&dist, 100);` |
| `dyp_uart_read_distance_meter_float(float *out, unsigned timeout_ms)` | meters | `dyp_uart_read_distance_meter_float(&dist, 100);` |
| `dyp_uart_read_distance_feet_float(float *out, unsigned timeout_ms)` | feet | `dyp_uart_read_distance_feet_float(&dist, 100);` |
| `dyp_uart_read_distance_inch_float(float *out, unsigned timeout_ms)` | inches | `dyp_uart_read_distance_inch_float(&dist, 100);` |

**Returns:**
* `0` = Success (valid reading stored in output variable)
* `-1` = Timeout (no valid frame received)
* `-2` = Not initialized or invalid arguments

---

### 🔢 Read Distance (Integer Output)

Same as float versions, but returns rounded integer directly.

| Function | Returns |
|----------|---------|
| `dyp_uart_read_distance_mm_int(unsigned timeout_ms)` | `long` (rounded mm) |
| `dyp_uart_read_distance_cm_int(unsigned timeout_ms)` | `long` (rounded cm) |
| `dyp_uart_read_distance_meter_int(unsigned timeout_ms)` | `long` (rounded m) |
| `dyp_uart_read_distance_feet_int(unsigned timeout_ms)` | `long` (rounded ft) |
| `dyp_uart_read_distance_inch_int(unsigned timeout_ms)` | `long` (rounded in) |

**Example:**
```cpp
long dist_cm = dyp_uart_read_distance_cm_int(100);
if (dist_cm >= 0) {
    Serial.print("Distance: ");
    Serial.println(dist_cm);
}
```

---

### 🚨 Object Detection (Threshold-based)

Check if an object is present within a certain range.

| Function | Threshold Unit | Example |
|----------|----------------|---------|
| `dyp_uart_object_detected_mm(float threshold, unsigned timeout_ms)` | mm | `dyp_uart_object_detected_mm(100.0f, 100);` |
| `dyp_uart_object_detected_cm(float threshold, unsigned timeout_ms)` | cm | `dyp_uart_object_detected_cm(10.0f, 100);` |
| `dyp_uart_object_detected_meter(float threshold, unsigned timeout_ms)` | m | `dyp_uart_object_detected_meter(0.1f, 100);` |
| `dyp_uart_object_detected_feet(float threshold, unsigned timeout_ms)` | ft | `dyp_uart_object_detected_feet(0.3f, 100);` |
| `dyp_uart_object_detected_inch(float threshold, unsigned timeout_ms)` | inch | `dyp_uart_object_detected_inch(4.0f, 100);` |

**Returns:**
* `1` = Object detected (distance ≤ threshold)
* `0` = No object detected (distance > threshold)
* `-1` = Error or timeout

**Example:**
```cpp
int detected = dyp_uart_object_detected_cm(10.0f, 100);
if (detected == 1) {
    Serial.println("Object within 10 cm!");
}
```

---

### 🧹 Flush Buffer

```cpp
int dyp_uart_flush_buffer(void);
```

**Purpose:** Clears any pending or corrupted bytes in the internal UART buffer.

**When to use:** If you suspect noisy readings or after long idle periods.

---

### 🕒 Timeout Check

```cpp
bool dyp_uart_last_timeout(void);
```

**Returns:** `true` if the last distance read timed out (no valid frame received).

---

### ⚙️ Change Baud Rate (Optional)

```cpp
int dyp_uart_set_baud(int baud);
```

**Purpose:** Reconfigures the UART baud rate dynamically without reinitializing.

---

## 💡 Example Code

### Basic Distance Reading

```cpp
#include <Arduino.h>
#include <ultrasonic_dypa02.h>

#define RX_PIN A4  // Sensor TX → MCU RX
#define TX_PIN A5  // Sensor RX ← MCU TX

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }
  
  if (dyp_uart_init(RX_PIN, TX_PIN, 9600) != 0) {
    Serial.println("ERROR: Sensor init failed!");
    while (1) delay(1000);
  }
  
  Serial.println("Sensor initialized!");
}

void loop() {
  float distance_cm = 0.0f;
  
  if (dyp_uart_read_distance_cm_float(&distance_cm, 100) == 0) {
    Serial.print("Distance: ");
    Serial.print(distance_cm, 2);
    Serial.println(" cm");
  } else {
    Serial.println("Timeout - no valid reading");
  }
  
  delay(500);
}
```

### Object Detection Example

```cpp
#include <Arduino.h>
#include <ultrasonic_dypa02.h>

#define RX_PIN A4
#define TX_PIN A5
#define THRESHOLD_CM 8.0f

void setup() {
  Serial.begin(9600);
  dyp_uart_init(RX_PIN, TX_PIN, 9600);
}

void loop() {
  int detected = dyp_uart_object_detected_cm(THRESHOLD_CM, 100);
  
  if (detected == 1) {
    Serial.println("🟢 Object DETECTED within 8 cm!");
  } else if (detected == 0) {
    Serial.println("⚪ No object detected");
  } else {
    Serial.println("❌ Error reading sensor");
  }
  
  delay(500);
}
```

### Multi-Unit Display Example

```cpp
#include <Arduino.h>
#include <ultrasonic_dypa02.h>

#define RX_PIN A4
#define TX_PIN A5

void setup() {
  Serial.begin(9600);
  dyp_uart_init(RX_PIN, TX_PIN, 9600);
}

void loop() {
  float cm = 0.0f;
  
  if (dyp_uart_read_distance_cm_float(&cm, 100) == 0) {
    // Convert to other units
    float mm = cm * 10.0f;
    float m = cm / 100.0f;
    float ft = cm * 0.0328084f;
    
    Serial.print("Distance: ");
    Serial.print(mm, 1); Serial.print(" mm | ");
    Serial.print(cm, 2); Serial.print(" cm | ");
    Serial.print(m, 3); Serial.print(" m | ");
    Serial.print(ft, 3); Serial.println(" ft");
  }
  
  delay(500);
}
```

---

## 📊 Return Value Reference

### Read Functions
| Value | Meaning |
|-------|---------|
| `0` | Success - valid reading obtained |
| `-1` | Timeout - no valid frame received within timeout period |
| `-2` | Error - sensor not initialized or invalid parameters |

### Object Detection Functions
| Value | Meaning |
|-------|---------|
| `1` | Object detected (distance ≤ threshold) |
| `0` | No object detected (distance > threshold) |
| `-1` | Error or timeout |

---

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| **Always getting timeout** | • Check wiring: Sensor TX → MCU RX, Sensor RX → MCU TX<br>• Verify 9600 baud rate<br>• Ensure sensor has stable 5V power supply<br>• Try calling `dyp_uart_flush_buffer()` |
| **Unstable/jumping readings** | • Call `dyp_uart_flush_buffer()` before reading<br>• Increase timeout to 200ms or more<br>• Check for electrical noise sources nearby<br>• Ensure sensor faces a flat surface directly |
| **Compiles but no output** | • Verify `Serial.begin(9600)` is called in `setup()`<br>• Check if `dyp_uart_init()` returns 0<br>• Test with a simple LED blink to verify MCU is running |
| **ESP32 not working** | • Use hardware UART capable pins (RX=16, TX=17 recommended)<br>• Avoid GPIO 6-11 (used by flash)<br>• Check pin definitions in code |
| **Sensor returns 0 or max value** | • Object may be too close (< 3-4cm) or too far (> 450cm)<br>• Ensure sensor has clear line of sight<br>• Surface may be too absorbent (fabric, foam) |

---

## 🧭 Important Notes

* **Default baud rate:** 9600 (standard for DYP sensors)
* **Measurement range:** Typically 20 cm to 450 cm (sensor-dependent)
* **Best accuracy:** When sensor faces a flat, hard surface directly
* **Avoid:** Strong ultrasonic sources, electrical noise, or reflective surfaces nearby
* **Power requirement:** Stable 5V supply (3.3V may work but not guaranteed)
* **Always initialize** the sensor with `dyp_uart_init()` before calling any read functions

---

## 🧩 Sample Output

```
----------------------------------------
Distance: 93.5 mm | 9.35 cm | 0.093 m | 0.306 ft
⚪ No object detected (beyond 8 cm)

----------------------------------------
Distance: 55.2 mm | 5.52 cm | 0.055 m | 0.181 ft
🟢 Object DETECTED within 8 cm!

----------------------------------------
Timeout - no valid reading
```

---

## ✅ Quick Reference

| Action | Function |
|--------|----------|
| Initialize sensor | `dyp_uart_init(RX_PIN, TX_PIN, 9600)` |
| Read distance (cm) | `dyp_uart_read_distance_cm_float(&cm, 100)` |
| Read distance (mm) | `dyp_uart_read_distance_mm_float(&mm, 100)` |
| Detect object (10cm) | `dyp_uart_object_detected_cm(10.0f, 100)` |
| Clear buffer | `dyp_uart_flush_buffer()` |
| Check timeout | `dyp_uart_last_timeout()` |

---


**Made with ❤️ for the maker community**