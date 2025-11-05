
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
  * Cross-platform C API (`extern "C"` safe)
  * Non-blocking `pid_heater_manage_window()` helper that moves all timing logic *inside* the library, simplifying your Arduino `loop()`.

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

### Update (Manual Method - for RTOS)

This function just runs the PID math and returns the new output. You must manage all timing and SSR control yourself. This is ideal for RTOS tasks.

```c
float measured = 74.2f; // sensor reading (°C)
int output = pid_heater_update(pid, measured, 3000); // dt=3000 ms
```

### Update (Managed Method - for Arduino)

This function runs all logic *for* you. It manages window timing, runs the PID math, and controls the SSR pin via a callback. Call this in your `loop()`.

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
int last_out = pid_heater_get_last_output(pid); // Get last 0-100 output
```

-----

## Usage Example 1: ESP-IDF (Two-Task RTOS Method)

This is the recommended production-ready design for an RTOS.

  * **`pid_calc_task`** runs *exactly* every 3000ms to do the slow PID math.
  * **`ssr_control_task`** runs *very fast* (every 20ms) to handle the real-time SSR pin control and sensor reading (with spike delay).


```c
/* main.c - ESP-IDF Two-Task Example */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "max31865.h"
#include "max6675.h"
#include "pid_heater.h"

// --- Hardware Pin Configuration ---
#define CS_MAX31865         21  // GPIO for PT100 (MAX31865) Chip Select
#define CS_MAX6675          22  // GPIO for K-Type (MAX6675) Chip Select
#define SSR_GPIO            15  // GPIO to control the Solid State Relay

// --- PID Tuning Configuration ---
#define PID_KP              50.33f  // Proportional gain
#define PID_KI              0.162f  // Integral gain
#define PID_KD              0.0f    // Derivative gain
#define PID_SETPOINT_C      85.0f   // Target temperature in Celsius

// --- Timing Configuration ---
#define CONTROL_PERIOD_MS   3000U   // PID loop runs exactly every 3000ms
#define SSR_LOOP_PERIOD_MS  20      // How often the fast SSR loop runs (20ms)
#define SENSOR_READ_DELAY_MS 20     // Wait 20ms after SSR state change to read sensor

static const char *TAG = "HEATER_PID";

// --- Shared Variables ---
static volatile uint32_t g_ssr_on_ms = 0;   // Calculated by PID, read by SSR task
static volatile float g_latest_pt100_temp = NAN; // Read by SSR task, used by PID task
static volatile float g_latest_ktype_temp = NAN; // Read by SSR task, for logging
static pid_heater_handle_t g_pid;
static uint64_t g_start_us = 0;

// --- SSR Hardware Functions ---
static inline void ssr_init(void) { 
    gpio_reset_pin(SSR_GPIO); 
    gpio_set_direction(SSR_GPIO, GPIO_MODE_OUTPUT); 
    gpio_set_level(SSR_GPIO, 0); 
}
static inline void ssr_set(bool on) { 
    // NOTE: If your SSR is "Active LOW", change this to:
    // gpio_set_level(SSR_GPIO, on ? 0 : 1);
    gpio_set_level(SSR_GPIO, on ? 1 : 0); 
}


/**
 * @brief TASK B: SSR Actuator & Sensor Read Task (Fast, Non-blocking)
 */
void ssr_control_task(void *pvParameters) {
    uint32_t window_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    bool read_done_this_window = false;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t elapsed = now_ms - window_start_ms;

        if (elapsed >= CONTROL_PERIOD_MS) {
            window_start_ms = now_ms;
            elapsed = 0;
            read_done_this_window = false; 
        }

        uint32_t on_ms = g_ssr_on_ms;

        // --- 1. SSR Pin Control ---
        if (elapsed < on_ms) ssr_set(true);
        else ssr_set(false);

        // --- 2. Sensor Read Logic (with spike delay) ---
        if (!read_done_this_window && (elapsed >= SENSOR_READ_DELAY_MS)) {
            float temp_pt100 = max31865_read_temperature_safe();
            if (isfinite(temp_pt100)) {
                g_latest_pt100_temp = temp_pt100;
            }
            float temp_ktype = max6675_read_celsius_float(2);
            if (isfinite(temp_ktype)) {
                g_latest_ktype_temp = temp_ktype;
            }
            read_done_this_window = true;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SSR_LOOP_PERIOD_MS));
    }
}

/**
 * @brief TASK A: PID Brain Task (Slow, Periodic)
 */
void pid_calc_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while (1) {
        // 1. Wait for the next 3000ms cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 2. Get latest temperature from other task
        float temp_to_use = g_latest_pt100_temp;
        if (!isfinite(temp_to_use)) {
            temp_to_use = 0.0f; 
        }

        // 3. Compute PID output
        int out = pid_heater_update(g_pid, temp_to_use, CONTROL_PERIOD_MS);

        // 4. Calculate ON time for the *next* window
        uint32_t on_ms = (uint32_t)roundf(((float)out / 100.0f) * (float)CONTROL_PERIOD_MS);

        // 5. Share the result with the SSR task
        g_ssr_on_ms = on_ms;

        // 6. Log
        float t_s = (esp_timer_get_time() - g_start_us) / 1e6f;
        float temp_ktype_to_log = g_latest_ktype_temp; 
        ESP_LOGI(TAG, "t=%.1fs, PT100=%.2f, KType=%.2f, Duty=%d%% (New ON_ms: %d)", 
                 t_s, 
                 temp_to_use, 
                 isnan(temp_ktype_to_log) ? 0.0f : temp_ktype_to_log,
                 out, 
                 on_ms);
    }
}


void app_main(void) {
    // init PT100 sensor
    max31865_init_hw(CS_MAX31865, 1000000U);
    max31865_set_continuous_mode(true);
    
    // init K-Type sensor
    max6675_init_hw(CS_MAX6675, 1000000U); 

    // Give sensors time for first conversion
    vTaskDelay(pdMS_TO_TICKS(500)); 
    g_latest_pt100_temp = max31865_read_temperature_safe(); 
    g_latest_ktype_temp = max6675_read_celsius_float(2); 

    ssr_init();

    // create and configure pid (store in global)
    g_pid = pid_heater_create();
    pid_heater_tune_t tune = { 
        .Kp = PID_KP, 
        .Ki = PID_KI, 
        .Kd = PID_KD, 
        .mode = PID_CTRL_PI 
    };
    pid_heater_init_manual(g_pid, tune, PID_CTRL_PI, PID_OUTMODE_TIME_WINDOW, 
                           0, 100, // Output 0-100%
                           CONTROL_PERIOD_MS, 
                           CONTROL_PERIOD_MS);
                           
    pid_heater_set_setpoint(g_pid, PID_SETPOINT_C);
    
    g_start_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Initialization complete. Starting PID tasks. Setpoint: %.1f C", PID_SETPOINT_C);

    // Start the two tasks
    xTaskCreate(pid_calc_task, "pid_calc", 4096, NULL, 5, NULL);
    xTaskCreate(ssr_control_task, "ssr_ctrl", 2048, NULL, 10, NULL); // SSR task at higher priority
}
```

-----

## Usage Example 2: Arduino (Non-Blocking Loop)

This is the recommended pattern for Arduino. It uses the `pid_heater_manage_window()` helper to keep the main `loop()` clean and non-blocking. The timing logic is handled *inside* the library.

```cpp
// src/main.cpp - Arduino UNO example
#include <Arduino.h>
#include <math.h>

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
#define SENSOR_READ_DELAY_MS 50U // Read 50ms into the window

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
  // NOTE: If your SSR is "Active LOW", change this to:
  // digitalWrite(SSR_PIN, on ? LOW : HIGH);
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
  // Read the sensor once per window, 50ms after the window starts
  if (!read_done_this_window) {
      unsigned long elapsed = now - main_window_start_ms;
      if (elapsed >= SENSOR_READ_DELAY_MS) { 
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

Integral (`I`) term is automatically clamped to ensure `P + I + D` always stays within `[out_min .. out_max]`.

### Integral Time Constant Tuning

For systems with slow thermal inertia (water baths, heaters):

  * Use **larger Ki** for tighter steady-state control.
  * Use **smaller Kp** to reduce overshoot.
  * Usually Kd ≈ 0 for heaters (since derivative amplifies sensor noise).

**Note on Gains:** The gain values (`Kp`, `Ki`) depend on your `CONTROL_PERIOD_MS`. The gains in the ESP-IDF example (`Kp=50.33`) are much larger than the Arduino example (`Kp=0.5033`) because they were tuned differently. **Always tune PID gains for your specific hardware and loop timing.**

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

  * Use a **2-5 second control window** (`CONTROL_PERIOD_MS = 2000` to `5000`)
  * Enable **continuous conversion** on the MAX31865
  * Wait at least **20-50 ms after SSR state changes** before sampling the temperature to avoid electrical noise.

-----

## Example Log Output (ESP-IDF)

```
I (78781) HEATER_PID: t=78.0s, PT100=23.91, KType=24.50, Duty=16% (New ON_ms: 480)
I (81781) HEATER_PID: t=81.0s, PT100=23.57, KType=24.25, Duty=63% (New ON_ms: 1890)
I (84781) HEATER_PID: t=84.0s, PT100=23.91, KType=24.50, Duty=76% (New ON_ms: 2280)
```

-----

## Advanced: Integrator Reset Logic

For systems that may “latch up” (e.g. faulted sensors):

  * Call `pid_heater_reset()` whenever you clear faults.
  * This prevents residual integral windup from causing false overshoot on recovery.

Example:

```c
if (max31865_has_fault()) {
    max31865_clear_faults();
    pid_heater_reset(pid);
}
```

-----

## Safety Integration

  * Always disable SSR if sensor read fails or MAX31865 reports fault.
  * Implement “hard latch” to prevent auto-restart after error.
  * Optional: log temperature to SD card or UART for tuning analysis.