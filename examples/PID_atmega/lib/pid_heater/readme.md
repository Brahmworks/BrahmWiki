
---

# README — PID Heater Controller Library

## Overview

This C-style PID controller library is designed for **heater and thermal system control** on **ESP32** (ESP-IDF) or **Arduino** platforms.

It provides a **robust, floating-point PID core** with built-in anti-windup, fixed-window SSR control support, and a simple opaque handle API.

 Designed for use with sensors such as **MAX31865 (PT100)** or **MAX6675 (K-type)**.
 Compatible with **time-proportional SSR control** (common in heater applications).

---

## Features

* Floating-point PID core (`P`, `PI`, or `PID` modes)
* Configurable output range (e.g. 0–100 % or 0–255 for PWM)
* Time-proportional SSR window support
* Anti-windup integral limiting
* Simple opaque handle API (`pid_heater_create`, `pid_heater_update`, etc.)
* Fully non-blocking (no `delay()` calls inside)
* Cross-platform C API (`extern "C"` safe)
* Ideal for ESP32 or embedded systems with multitasking

---

## File Structure

| File              | Description                                                   |
| ----------------- | ------------------------------------------------------------- |
| `pid_heater.h`    | Header with opaque API and configuration structures           |
| `pid_heater.cpp`  | Implementation with float-precision PID logic and anti-windup |
| `examples/main.c` | Minimal working example with MAX31865 and MAX6675 sensors     |

Place these files under:

* **ESP-IDF:** `components/pid_heater/`
* **PlatformIO / Arduino:** `lib/pid_heater/`

---

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
    0, 100,         // output range: 0–100%
    3000, 3000      // update period and SSR window in ms
);

// set the desired target temperature
pid_heater_set_setpoint(pid, 85.0f);
```

### Update Loop

```c
float measured = 74.2f; // sensor reading (°C)
int output = pid_heater_update(pid, measured, 3000); // dt=3000 ms

// output = 0–100 (%)
apply_ssr_time_proportional(output);
```

### Utility

```c
pid_heater_reset(pid);                   // reset integrator
int pwm = pid_heater_frac_to_output(pid, 0.45f); // 45% → mapped to range
```

---

## Typical Usage — ESP-IDF Example

Example for a **3-second control loop** driving an SSR with **MAX31865 (PT100)**:

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

---

## Design Notes

### PID Modes

| Mode           | Description                                        |
| -------------- | -------------------------------------------------- |
| `PID_CTRL_P`   | Proportional-only control                          |
| `PID_CTRL_PI`  | Proportional + Integral (best for thermal systems) |
| `PID_CTRL_PID` | Full PID (adds derivative term)                    |

### Output Modes

| Mode                      | Description                                                          |
| ------------------------- | -------------------------------------------------------------------- |
| `PID_OUTMODE_PWM`         | Fast PWM style (continuous analog output)                            |
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

---

## Practical Guidelines (Heater Control)

| Problem                       | Solution                                |
| ----------------------------- | --------------------------------------- |
| Output toggles 0 / 100 % only | Increase Ki slightly or lower Kp        |
| Overshoot on startup          | Add small deadband or lower Kp          |
| Oscillation near setpoint     | Reduce Ki or use longer control period  |
| SSR clicking or flicker       | Increase control window (e.g., 5 s)     |
| Slow temperature rise         | Check sensor calibration / heater power |

---

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

---

## Example Log Output

```
I (350) HEATER_CASCADE_PID: t=10.1s, PT100=34.25, KType=123.50, Duty=38%
I (6350) HEATER_CASCADE_PID: t=13.1s, PT100=35.50, KType=125.25, Duty=52%
I (9350) HEATER_CASCADE_PID: t=16.1s, PT100=36.80, KType=126.00, Duty=64%
```

---

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

---

## Typical Parameter Ranges

| Parameter      | Typical Range | Notes                       |
| -------------- | ------------- | --------------------------- |
| `Kp`           | 0.1 – 2.0     | Heater responsiveness       |
| `Ki`           | 0.0005 – 0.01 | Integral accumulation speed |
| `Kd`           | 0 – 0.1       | Rarely used for heaters     |
| Control Period | 2 – 5 s       | SSR-friendly                |
| Output Range   | 0 – 100       | % duty                      |

---

## Safety Integration

* Always disable SSR if sensor read fails or MAX31865 reports fault.
* Implement “hard latch” (`g_hard_latched`) to prevent auto-restart after error.
* Optional: log temperature to SD card or UART for tuning analysis.

---