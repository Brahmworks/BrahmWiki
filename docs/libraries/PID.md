
-----

# README — PID Heater Controller Library

## Overview

This C-style PID controller library is designed for **heater and thermal system control** on **ESP32** (ESP-IDF) or **Arduino** platforms.

It provides a **robust, floating-point PID core** with built-in anti-windup, fixed-window SSR control support, and a simple opaque handle API.

  * Designed for use with sensors such as **MAX31865 (PT100)** or **MAX6675 (K-type)**.
  * Compatible with **time-proportional SSR control** (common in heater applications).

-----

## Features

  * Floating-point PID core (`P`, `PI`, or `PID` modes)
  * Configurable output range (e.g. 0–100 % or 0–255 for PWM)
  * Time-proportional SSR window support
  * Anti-windup integral limiting
  * Simple opaque handle API (`pid_heater_create`, `pid_heater_update`, etc.)
  * Fully non-blocking (no `delay()` calls inside)
  * Cross-platform C API (`extern "C"` safe)
  * Ideal for ESP32 or embedded systems with multitasking

-----

## File Structure

| File | Description |
| :--- | :--- |
| `pid_heater.h` | Header with opaque API and configuration structures |
| `pid_heater.cpp` | Implementation with float-precision PID logic and anti-windup |
| `examples/` | Example implementations for ESP-IDF and Arduino |

Place these files under:

  * **ESP-IDF:** `components/pid_heater/`
  * **PlatformIO / Arduino:** `lib/pid_heater/`

-----

## API Overview

### Creation & Initialization

```c
pid_heater_handle_t pid = pid_heater_create();

pid_heater_tune_t tune = {
    .Kp = 0.5033f,
    .Ki = 0.00162f,
    .Kd = 0.0f,
    .mode = PID_CTRL_PI
};

// initialize with manual parameters
pid_heater_init_manual(
    pid, tune,
    PID_CTRL_PI, PID_OUTMODE_TIME_WINDOW,
    0, 100,       // output range: 0–100%
    3000, 3000    // update period and SSR window in ms
);

// set the desired target temperature
pid_heater_set_setpoint(pid, 85.0f);
```

### Update (Manual)

This function runs the PID math and returns the new output. You must manage timing and SSR control yourself.

```c
float measured = 74.2f; // sensor reading (°C)
int output = pid_heater_update(pid, measured, 3000); // dt=3000 ms
```

### Update (Managed)

This new function runs all logic *for* you. It manages window timing, runs the PID math, and controls the SSR pin via a callback. **This is recommended for Arduino.**

```c
// Define your SSR control function
void my_ssr_control(bool on) {
    digitalWrite(SSR_PIN, on);
}

// Call this in your loop()
unsigned long now = millis();
float temp = read_sensor();
bool new_window = pid_heater_manage_window(pid, now, temp, my_ssr_control);
```

### Utility

```c
pid_heater_reset(pid);                     // reset integrator
int pwm = pid_heater_frac_to_output(pid, 0.45f); // 45% -> mapped to range
int last_out = pid_heater_get_last_output(pid);
```

-----

## Usage Example 1: ESP-IDF (Blocking Task)

This pattern is simple and clear for an RTOS like FreeRTOS (ESP-IDF). It uses `vTaskDelay` to manage the SSR on/off time, which blocks *this specific task* but allows the rest of the system (like Wi-Fi) to run.

```c
#include "max31865.h"
#include "pid_heater.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define SSR_GPIO 15
#define CONTROL_PERIOD_MS 3000

static void ssr_set(bool on) { gpio_set_level(SSR_GPIO, on ? 1 : 0); }

void app_main(void) {
    max31865_init_hw(21, 1000000);
    max31865_set_continuous_mode(true);

    pid_heater_handle_t pid = pid_heater_create();
    pid_heater_tune_t tune = { .Kp = 0.5033f, .Ki = 0.00162f, .Kd = 0.0f, .mode = PID_CTRL_PI };
    pid_heater_init_manual(pid, tune, PID_CTRL_PI, PID_OUTMODE_TIME_WINDOW, 0, 100, CONTROL_PERIOD_MS, CONTROL_PERIOD_MS);
    pid_heater_set_setpoint(pid, 85.0f);

    while (1) {
        float pt100 = max31865_read_temperature_safe();
        int duty = pid_heater_update(pid, pt100, CONTROL_PERIOD_MS); // returns 0–100 %

        uint32_t on_ms = (uint32_t)((duty / 100.0f) * CONTROL_PERIOD_MS);
        if (on_ms > 0) {
            ssr_set(true);
            vTaskDelay(pdMS_TO_TICKS(on_ms));
        }
        ssr_set(false);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS - on_ms));

        ESP_LOGI("PID", "PT100=%.2f°C, Duty=%d%%", pt100, duty);
    }
}
```

-----

## Usage Example 2: Arduino (Non-Blocking Loop)

This pattern is non-blocking and **recommended for Arduino**. It uses the `pid_heater_manage_window()` function to handle all timing and SSR control. Your `loop()` function is only responsible for reading the sensor and calling the manager.

```cpp
// src/main.cpp - Arduino UNO example using pid_heater_manage_window()
#include <Arduino.h>
#include <math.h> // For isnan()

// Include your custom libraries
extern "C" {
  #include "max31865.h"
  #include "pid_heater.h"
}

// --- Pin Definitions ---
#define CS_MAX31865        10
#define SSR_PIN            8

// --- PID & Timing Configuration ---
#define CONTROL_PERIOD_MS  3000U
#define READ_AFTER_ON_MS   10U
#define OFF_GUARD_MS       50U

// --- Global State ---
static bool read_done_this_window = false;
static float last_pt100 = NAN;
static pid_heater_handle_t pid = nullptr;
static uint32_t main_window_start_ms = 0; // Tracks window start time for logging

/**
 * @brief Sets the SSR state (HIGH for ON, LOW for OFF).
 * This is the callback function we will pass to the PID library.
 */
static void ssr_set(bool on) {
  digitalWrite(SSR_PIN, on ? HIGH : LOW);
}

/**
 * @brief Main setup function, runs once at power-on.
 */
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { /* wait for serial */ }

  Serial.println(F("PID Heater Control Initializing..."));

  // 1. Initialize MAX31865 sensor
  if (max31865_init_hw(CS_MAX31865, 1000000U) != 0) {
    Serial.println(F("MAX31865 init failed! Check wiring."));
    while(1); // Halt
  }
  max31865_set_rtd_nominal(100.0f, 430.0f);
  max31865_set_wire_mode(3);
  max31865_set_filter_50hz(false);
  max31865_set_continuous_mode(true); // Sensor auto-converts

  // 2. Initialize SSR pin
  pinMode(SSR_PIN, OUTPUT);
  ssr_set(false); // Start OFF

  // 3. Initialize PID controller
  pid = pid_heater_create();
  pid_heater_tune_t tune = { .Kp = 0.5033f, .Ki = 0.00162f, .Kd = 0.0f, .mode = PID_CTRL_PI };
  pid_heater_init_manual(pid, tune, PID_CTRL_PI, PID_OUTMODE_TIME_WINDOW, 
                         0, 100, CONTROL_PERIOD_MS, CONTROL_PERIOD_MS);
  pid_heater_set_setpoint(pid, 85.0f); // Set target temperature
  Serial.println(F("PID initialized. Target: 85.0 C"));

  // Get one initial temperature reading
  float t = max31865_read_temperature_safe();
  if (isfinite(t)) {
    last_pt100 = t;
  }
  
  // Initialize our local window tracker
  main_window_start_ms = millis();
}

/**
 * @brief Main loop function, runs repeatedly.
 */
void loop() {
  unsigned long now = millis();

  // --- 1. Sensor Reading Policy ---
  // Read the sensor once per window
  if (!read_done_this_window) {
      unsigned long elapsed = now - main_window_start_ms;
      
      // A simple policy: Read 50ms into the window
      if (elapsed >= 50) { 
          float t = max31865_read_temperature_safe();
          if (isfinite(t)) {
              last_pt100 = t;
          }
          read_done_this_window = true;
      }
  }

  // --- 2. PID & SSR Management ---
  // This one function does all the work:
  // - Checks if the window is over
  // - Runs the PID calculation
  // - Controls the SSR pin via the 'ssr_set' callback
  bool new_window = pid_heater_manage_window(pid, now, last_pt100, ssr_set);

  // --- 3. Logging ---
  if (new_window) {
    // The function returned 'true', so the window just finished.
    // We can print our log message.
    float ptprint = isnan(last_pt100) ? 0.0f : last_pt100;
    int last_out = pid_heater_get_last_output(pid);
    float t_s = (now / 1000.0f);
    
    Serial.print("t=");
    Serial.print(t_s, 1);
    Serial.print("s, PT100=");
    Serial.print(ptprint, 2);
    Serial.print(" C, Duty=");
    Serial.print(last_out);
    Serial.println("%");

    // Update our local tracker when a new window begins
    main_window_start_ms = now;

    // Reset our sensor read flag for the new window
    read_done_this_window = false;
  }

  delay(1); // Main loop delay
}
```

-----

## Design Notes

### PID Modes

| Mode | Description |
| :--- | :--- |
| `PID_CTRL_P` | Proportional-only control |
| `PID_CTRL_PI` | Proportional + Integral (best for thermal systems) |
| `PID_CTRL_PID` | Full PID (adds derivative term) |

### Output Modes

| Mode | Description |
| :--- | :--- |
| `PID_OUTMODE_PWM` | Fast PWM style (continuous analog output) |
| `PID_OUTMODE_TIME_WINDOW` | Time-proportional control (SSR) — output = ON duration within window |

### Anti-Windup

Integral (`I`) term is automatically clamped to ensure
`P + I + D` always stays within `[out_min .. out_max]`.

### Integral Time Constant Tuning

For systems with slow thermal inertia (water baths, heaters):

  * Use **larger Ki** for tighter steady-state control.
  * Use **smaller Kp** to reduce overshoot.
  * Usually Kd ≈ 0 for heaters (since derivative amplifies sensor noise).

Example starting values for a slow heater:

```
Kp = 0.5
Ki = 0.0016
Kd = 0.0
```

-----

## Practical Guidelines (Heater Control)

| Problem | Solution |
| :--- | :--- |
| Output toggles 0 / 100 % only | Increase Ki slightly or lower Kp |
| Overshoot on startup | Add small deadband or lower Kp |
| Oscillation near setpoint | Reduce Ki or use longer control period |
| SSR clicking or flicker | Increase control window (e.g., 5 s) |
| Slow temperature rise | Check sensor calibration / heater power |

-----

## Integration With MAX31865

This library is designed to pair seamlessly with the `max31865` driver:

```c
float pt100 = max31865_read_temperature_safe();
int duty = pid_heater_update(pid, pt100, CONTROL_PERIOD_MS);
```

For best results:

  * Use **3-s control window** (`CONTROL_PERIOD_MS = 3000`)
  * Enable **continuous conversion** on the MAX31865
  * Wait at least **10 ms after SSR turns ON** or **50 ms after OFF** before sampling

-----

## Example Log Output

```
I (350) HEATER_CASCADE_PID: t=10.1s, PT100=34.25, KType=123.50, Duty=38%
I (6350) HEATER_CASCADE_PID: t=13.1s, PT100=35.50, KType=125.25, Duty=52%
I (9350) HEATER_CASCADE_PID: t=16.1s, PT100=36.80, KType=126.00, Duty=64%
```

-----

## Advanced: Integrator Reset Logic

For systems that may “latch up” (e.g. faulted sensors):

  * Call `pid_heater_reset()` whenever you clear faults.
  * This prevents residual integral windup from causing false overshoot on recovery.

Example:

```c
if (sensor_fault_detected()) {
    max31865_clear_faults();
    pid_heater_reset(pid);
}
```

-----

## Typical Parameter Ranges

| Parameter | Typical Range | Notes |
| :--- | :--- | :--- |
| `Kp` | 0.1 – 2.0 | Heater responsiveness |
| `Ki` | 0.0005 – 0.01 | Integral accumulation speed |
| `Kd` | 0 – 0.1 | Rarely used for heaters |
| Control Period | 2 – 5 s | SSR-friendly |
| Output Range | 0 – 100 | % duty |

-----

## Safety Integration

  * Always disable SSR if sensor read fails or MAX31865 reports fault.
  * Implement “hard latch” (`g_hard_latched`) to prevent auto-restart after error.
  * Optional: log temperature to SD card or UART for tuning analysis.

-----