#  DYP-A02YYGDW UART Ultrasonic Sensor Library

**Cross-Platform Embedded Library for Arduino (UNO/MEGA) and ESP-IDF (ESP32)**

---

##  Overview

This library provides a simple and unified API for reading distance measurements from the **DYP-A02YYGDW ultrasonic distance sensor (UART version)**.

The DYP-A02YYGDW continuously sends distance data over a serial interface (`9600 bps`) in the form of 4-byte frames:

```

FF XX YY SUM

```

Where:
* `0xFF` → frame header
* `XX YY` → 16-bit distance in millimeters
* `SUM = (0xFF + XX + YY) & 0xFF` (Checksum)

The library automatically parses these frames and provides the distance in **centimeters** with simple function calls.

It works on:
*  **Arduino UNO/MEGA/ESP32** (via `SoftwareSerial`)
*  **ESP32** (ESP-IDF) (via UART driver)

---

##  Integration

###  Folder Structure (for PlatformIO or Manual Use)

For **PlatformIO**, place the files like this:

```

project/
├── platformio.ini
├── src/
│   └── main.cpp
└── lib/
└── ultrasonic\_dypa02/
├── ultrasonic\_dypa02.h
└── ultrasonic\_dypa02.cpp

```
PlatformIO will automatically include everything under `lib/`.

For **ESP-IDF**, place the files in the `components` folder:

```

components/ultrasonic\_dypa02/
├── ultrasonic\_dypa02.cpp
├── ultrasonic\_dypa02.h
└── CMakeLists.txt

````

And then add the component to your main `CMakeLists.txt`:
```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ultrasonic_dypa02)
````

-----

##  Key API Functions

| Function | Description |
| :--- | :--- |
| `int dyp_uart_init(int rx_pin, int tx_pin, int baud)` | Initialize UART communication. Use `0` for default `9600` baud. |
| `int dyp_uart_deinit(void)` | Stop UART communication. |
| `int dyp_uart_read_distance_cm_float(float *out_cm, unsigned timeout_ms)` | Blocking read — waits for a valid frame (returns `0` on success). |
| `long dyp_uart_read_distance_cm_int(unsigned timeout_ms)` | Returns integer cm or `DYP_UART_ERROR` on failure. |
| `int dyp_uart_object_detected(float threshold_cm, unsigned timeout_ms)` | Returns `1` if object ≤ threshold, `0` otherwise. |
| `bool dyp_uart_last_timeout(void)` | True if last read timed out (no data). |
| `int dyp_uart_set_baud(int baud)` | Change the UART baudrate. Default = `9600`. |

-----

## 🪜 How to Use

###  Arduino Example (UNO/MEGA)

This example uses `SoftwareSerial` (handled internally by the library) on an Arduino Uno or Mega. **Note the updated header file include.**

```cpp
#include <Arduino.h>
#include "ultrasonic_dypa02.h"

#define RX_PIN A4   // connect to sensor TX
#define TX_PIN A5   // connect to sensor RX

const float THRESHOLD_CM = 8.0f;

void setup() {
  Serial.begin(9600);
  Serial.println("DYP-A02YYGDW UART example");

  if (dyp_uart_init(RX_PIN, TX_PIN, 9600) != 0) {
    Serial.println("Failed to init DYP UART");
    while (1);
  }
}

void loop() {
  float distance_cm = 0.0f;
  int r = dyp_uart_read_distance_cm_float(&distance_cm, 200); // 200 ms timeout

  if (r == 0) {
    bool detected = distance_cm <= THRESHOLD_CM && distance_cm > 0;
    Serial.print("Distance: ");
    Serial.print(distance_cm, 1);
    Serial.print(" cm | Object: ");
    Serial.println(detected ? "YES" : "NO");
  } else {
    Serial.println("No valid frame / timeout");
  }

  delay(100);
}
```

| DYP-A02YYGDW Pin | Arduino Pin | Direction |
| :--- | :--- | :--- |
| **VCC** | 5V | Power |
| **GND** | GND | Ground |
| **TX** | A4 | Sensor → MCU |
| **RX** | A5 | MCU → Sensor |

>  **Ensure your sensor is the UART version** (sends serial frames). The PWM version requires a different (trigger/echo) library.

-----

###  ESP32 Example (ESP-IDF)

This example uses the built-in ESP-IDF UART driver. **Note the updated header file include.**

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ultrasonic_dypa02.h"

#define DYP_RX_GPIO 19  // connect to sensor TX
#define DYP_TX_GPIO 18  // connect to sensor RX

static const char *TAG = "DYP_UART_EX";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting DYP UART demo...");

    if (dyp_uart_init(DYP_RX_GPIO, DYP_TX_GPIO, 9600) != 0) {
        ESP_LOGE(TAG, "Initialization failed");
        return;
    }

    while (1) {
        float cm = 0.0f;
        if (dyp_uart_read_distance_cm_float(&cm, 200) == 0) {
            ESP_LOGI(TAG, "Distance: %.1f cm", cm);
        } else {
            ESP_LOGW(TAG, "Timeout / no frame");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
```

| DYP-A02YYGDW Pin | ESP32 Pin | Direction |
| :--- | :--- | :--- |
| **VCC** | 5V | Power |
| **GND** | GND | Ground |
| **TX** | GPIO19 | Sensor → MCU |
| **RX** | GPIO18 | MCU → Sensor |

>  **Use a level shifter or voltage divider for sensor TX → ESP32 RX**
> (Sensor TX outputs \~5 V; ESP32 is 3.3 V logic).

-----

##  Application Notes

### 1\. Frame Rate & Timeout

The sensor sends new frames every **\~100 ms**.
Set `timeout_ms ≥ 100 ms` (e.g., 150–200 ms) for reliable reading.

### 2\. Speed / Performance

The library uses **blocking reads with timeouts**. For real-time tasks, call `dyp_uart_read_distance_cm_float()` from a dedicated task (ESP-IDF) or loop (Arduino).

### 3\. Object Detection

Use the built-in helper for simple object presence detection:

```c
int object = dyp_uart_object_detected(8.0f, 200);
if (object > 0) {
  Serial.println("Object detected!");
}
```

### 4\. Power & Voltage

Typical supply: **5 V**, current ≈ **15–20 mA**. If using ESP32, you *must* add a level shifter on the sensor TX line to prevent damage to the ESP32's 3.3 V RX pin.

### 5\. Data Validity

  * Maximum range: **\~450 cm**
  * Typical noise: **±1 cm**

-----