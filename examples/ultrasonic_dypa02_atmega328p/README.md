
# 🧭 DYP-A02YY Ultrasonic Sensor – UART Library

This library provides a simple and reliable interface to communicate with the **DYP-A02YY ultrasonic distance sensor** over UART.
It supports **Arduino** and **ESP-IDF (ESP32)** platforms.

---

## 📘 Overview

The **DYP-A02YY ultrasonic sensor** outputs distance data continuously via UART (serial communication).
This library reads that data, validates the frame, and converts it into **centimeters, millimeters, meters, feet, or inches** — whichever unit you prefer.

It also includes helper functions for **object detection** based on a distance threshold.

---

## ⚙️ Features

* Easy initialization using UART pins
* Supports multiple units: **mm, cm, m, inch, feet**
* Threshold-based object detection (`object_detected_*`)
* Timeout detection if no valid data is received
* Flush function to clear the internal buffer
* Works on both **Arduino** and **ESP-IDF (ESP32)** platforms

---

## 📦 Installation

1. Copy the files `ultrasonic_dypa02.h` and `ultrasonic_dypa02.cpp` into your project’s `lib/` or `src/` directory.
2. Include the header in your sketch or main file:

   ```cpp
   #include <ultrasonic_dypa02.h>
   ```

---

## 🪝 Pin Connections

| DYP Sensor Pin | Description     | Connect To MCU Pin     |
| -------------- | --------------- | ---------------------- |
| **VCC**        | Power (5V)      | 5V (or 3.3V for ESP32) |
| **GND**        | Ground          | GND                    |
| **TX**         | Sensor → MCU RX | RX pin you define      |
| **RX**         | MCU TX → Sensor | TX pin you define      |

---

## 🧠 Library Usage

Here’s a breakdown of all the **user-accessible functions** you’ll use.

---

### 🟢 1. Initialization

```cpp
int dyp_uart_init(int rx_pin, int tx_pin, int baud);
```

**Purpose:**
Initializes the sensor with the specified RX/TX pins and baud rate.

**Parameters:**

* `rx_pin` → MCU pin connected to sensor TX
* `tx_pin` → MCU pin connected to sensor RX
* `baud` → UART baud rate (use `9600` for DYP sensors, or `0` for default)

**Returns:**
`0` on success, negative on failure.

🧩 *Example:*

```cpp
dyp_uart_init(A4, A5, 9600);
```

---

### 🔴 2. Deinitialization

```cpp
int dyp_uart_deinit(void);
```

**Purpose:**
Safely releases UART resources if you want to stop using the sensor or reinitialize it.

---

### ⚡ 3. Read Distance (Float Output)

All read functions wait until a valid data frame is received or the timeout expires.

| Function                                                                        | Unit        | Example                                           |
| ------------------------------------------------------------------------------- | ----------- | ------------------------------------------------- |
| `int dyp_uart_read_distance_mm_float(float *out_mm, unsigned timeout_ms)`       | millimeters | `dyp_uart_read_distance_mm_float(&dist, 100);`    |
| `int dyp_uart_read_distance_cm_float(float *out_cm, unsigned timeout_ms)`       | centimeters | `dyp_uart_read_distance_cm_float(&dist, 100);`    |
| `int dyp_uart_read_distance_meter_float(float *out_meter, unsigned timeout_ms)` | meters      | `dyp_uart_read_distance_meter_float(&dist, 100);` |
| `int dyp_uart_read_distance_feet_float(float *out_feet, unsigned timeout_ms)`   | feet        | `dyp_uart_read_distance_feet_float(&dist, 100);`  |
| `int dyp_uart_read_distance_inch_float(float *out_inch, unsigned timeout_ms)`   | inches      | `dyp_uart_read_distance_inch_float(&dist, 100);`  |

**Returns:**

* `0` on success (valid reading stored in variable)
* `-1` on timeout
* `-2` if not initialized or invalid arguments

---

### 🔢 4. Read Distance (Integer Output)

Same as float, but returns an integer directly (rounded).

| Function                                                     | Unit |
| ------------------------------------------------------------ | ---- |
| `long dyp_uart_read_distance_mm_int(unsigned timeout_ms)`    |      |
| `long dyp_uart_read_distance_cm_int(unsigned timeout_ms)`    |      |
| `long dyp_uart_read_distance_meter_int(unsigned timeout_ms)` |      |
| `long dyp_uart_read_distance_feet_int(unsigned timeout_ms)`  |      |
| `long dyp_uart_read_distance_inch_int(unsigned timeout_ms)`  |      |

**Example:**

```cpp
long dist_cm = dyp_uart_read_distance_cm_int(100);
```

---

### 🚨 5. Object Detection (Threshold-based)

Check if an object is present within a certain range.
Returns:

* `1` → Object detected (within threshold)
* `0` → No object detected (beyond threshold)
* `-1` → Error or timeout

| Function                                                                         | Threshold Unit | Example                                      |
| -------------------------------------------------------------------------------- | -------------- | -------------------------------------------- |
| `int dyp_uart_object_detected_mm(float threshold_mm, unsigned timeout_ms)`       | mm             | `dyp_uart_object_detected_mm(80.0f, 100);`   |
| `int dyp_uart_object_detected_cm(float threshold_cm, unsigned timeout_ms)`       | cm             | `dyp_uart_object_detected_cm(10.0f, 100);`   |
| `int dyp_uart_object_detected_meter(float threshold_meter, unsigned timeout_ms)` | m              | `dyp_uart_object_detected_meter(0.1f, 100);` |
| `int dyp_uart_object_detected_feet(float threshold_feet, unsigned timeout_ms)`   | ft             | `dyp_uart_object_detected_feet(0.3f, 100);`  |
| `int dyp_uart_object_detected_inch(float threshold_inch, unsigned timeout_ms)`   | inch           | `dyp_uart_object_detected_inch(4.0f, 100);`  |

---

### 🧹 6. Flush Buffer

```cpp
int dyp_uart_flush_buffer(void);
```

Clears any pending or corrupted bytes in the internal UART buffer.
Call this if you suspect noisy readings or after long idle times.

---

### 🕒 7. Timeout Check

```cpp
bool dyp_uart_last_timeout(void);
```

Returns `true` if the last distance read timed out (no valid frame received).

---

### ⚙️ 8. Change Baud Rate (Optional)

```cpp
int dyp_uart_set_baud(int baud);
```

Reconfigures the UART baud rate dynamically without reinitializing.

---

## 💡 Example Explained

**Example File:**

```cpp
#include <Arduino.h>
#include <ultrasonic_dypa02.h>

#define RX_PIN A4
#define TX_PIN A5
const float THRESHOLD_CM = 8.0f;

void setup() {
  Serial.begin(9600);
  dyp_uart_init(RX_PIN, TX_PIN, 9600);
}

void loop() {
  float distance_cm = 0.0f;
  int res_cm = dyp_uart_read_distance_cm_float(&distance_cm, 100);

  if (res_cm == 0) {
    Serial.print("Distance: ");
    Serial.print(distance_cm, 2);
    Serial.println(" cm");

    bool detected = (distance_cm > 0.0f && distance_cm <= THRESHOLD_CM);
    if (detected)
      Serial.println("🟢 Object DETECTED within 8 cm!");
    else
      Serial.println("⚪ No object detected (beyond 8 cm)");
  } else {
    Serial.println("TIMEOUT/ERROR - No valid frame");
  }

  delay(500);
}
```

### 🔍 Step-by-Step Breakdown

1. **Initialize UART**
   The sensor starts communicating over the defined RX/TX pins at 9600 baud.

2. **Read Distance**
   `dyp_uart_read_distance_cm_float()` waits up to 100 ms for a valid frame and stores the distance in centimeters.

3. **Threshold Check**
   If the measured distance ≤ 8 cm, it reports an object as **detected**.

4. **Print Results**
   Displays the distance in various units and indicates detection status.

5. **Loop Continuously**
   The loop repeats every 500 ms, continuously monitoring the distance.

---

## 🧭 Notes

* Default UART baud rate: **9600**
* Works best when the sensor faces a flat surface directly.
* Avoid placing the sensor near strong ultrasonic or electrical noise sources.
* If readings are unstable, call `dyp_uart_flush_buffer()` before reading again.
* Always initialize the sensor **before** calling any read functions.

---

## 🧩 Example Outputs

```
----------------------------------------
Distance: 93.5 mm  |  9.35 cm  |  0.093 m  |  0.306 ft
⚪ No object detected (beyond 8 cm)

----------------------------------------
Distance: 55.2 mm  |  5.52 cm  |  0.055 m  |  0.181 ft
🟢 Object DETECTED within 8 cm!
```

---

## ✅ Quick Summary

| Action                      | Function                                    |
| --------------------------- | ------------------------------------------- | 
| Initialize sensor           | `dyp_uart_init(A4, A5, 9600)`               |  
| Read distance (cm)          | `dyp_uart_read_distance_cm_float(&cm, 100)` |  
| Detect object (within 8 cm) | `dyp_uart_object_detected_cm(8.0f, 100)`    |  

---


