#ifndef PID_HEATER_H
#define PID_HEATER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*
 * pid_heater - cross-platform PID helper for heaters
 * - Manages PID state
 * - Supports time-proportional (SSR) or PWM output modes
 * - Agnostic to sensor hardware (receives measurement)
 * - Uses an opaque handle for state management
 */

/* Control mode selection */
typedef enum {
    PID_CTRL_P = 0,
    PID_CTRL_PI,
    PID_CTRL_PID
} pid_ctrl_mode_t;

/* Output mode selection */
typedef enum {
    PID_OUTMODE_PWM = 0,    /* fast PWM-style control (output is proportional value) */
    PID_OUTMODE_TIME_WINDOW /* SSR time-proportional window (output is 0 or out_max) */
} pid_out_mode_t;

/* Tuning parameters */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    pid_ctrl_mode_t mode;
} pid_heater_tune_t;

/* Opaque handle */
struct pid_heater_s;
typedef struct pid_heater_s* pid_heater_handle_t;

/**
 * @brief Create a new PID heater instance.
 * @return Handle or NULL on failure
 */
pid_heater_handle_t pid_heater_create(void);

/**
 * @brief Manually initialize the controller with pre-tuned gains.
 *
 * @param h Handle from pid_heater_create()
 * @param tune Struct with your Kp, Ki, Kd values
 * @param mode PID_CTRL_P, PID_CTRL_PI, or PID_CTRL_PID
 * @param out_mode PID_OUTMODE_PWM or PID_OUTMODE_TIME_WINDOW
 * @param out_min 0 (for OFF)
 * @param out_max 100 (for ON)
 * @param update_period_ms How often the PID calculation runs (e.g., 3000)
 * @param window_period_ms The SSR cycle time (e.g., 3000)
 */
void pid_heater_init_manual(pid_heater_handle_t h,
                            pid_heater_tune_t tune,
                            pid_ctrl_mode_t mode,
                            pid_out_mode_t out_mode,
                            int out_min, int out_max,
                            uint32_t update_period_ms,
                            uint32_t window_period_ms);

/**
 * @brief Set the target temperature for the controller.
 * @param h Handle
 * @param setpoint_c Target temperature in Celsius
 */
void pid_heater_set_setpoint(pid_heater_handle_t h, float setpoint_c);

/**
 * @brief Reset integrator (i_term) and other internal states.
 * @param h Handle
 */
void pid_heater_reset(pid_heater_handle_t h);

/**
 * @brief Runtime update function. Call this in your main loop.
 *
 * @param h Handle
 * @param measured_c The current temperature reading from your sensor.
 * @param dt_ms The time elapsed (in milliseconds) since the last call to this function.
 * @return int The computed output value (e.g., 0-100), ready to be sent to your actuator (PWM or SSR).
 */
int pid_heater_update(pid_heater_handle_t h, float measured_c, uint32_t dt_ms);

/**
 * @brief Utility to convert a 0.0-1.0 fraction to the configured integer output range.
 * @param h Handle
 * @param fraction Value from 0.0 to 1.0
 * @return int The integer output (e.g., 0-100)
 */
int pid_heater_frac_to_output(pid_heater_handle_t h, float fraction);

/**
 * @brief Get the last calculated internal PID output fraction (0.0 - 1.0).
 * @param h Handle
 * @return float The last computed duty cycle fraction.
 */
float pid_heater_get_last_output_fraction(pid_heater_handle_t h);

/**
 * @brief Get the last integer output computed by the controller.
 * @param h Handle
 * @return Last output (int, in configured out_min..out_max), or 0 if invalid handle.
 */
int pid_heater_get_last_output(pid_heater_handle_t h);

/**
 * @brief Compute PID update and return on-time (milliseconds) for a given window length.
 *
 * @param h Handle
 * @param measured_c measured temperature (°C)
 * @param window_ms time-proportional window length in ms (e.g. 3000)
 * @param dt_ms dt passed to pid_heater_update (ms)
 * @return on-time in ms (0..window_ms)
 */
uint32_t pid_heater_compute_on_ms(pid_heater_handle_t h, float measured_c, uint32_t window_ms, uint32_t dt_ms);


/* --- NEW NON-BLOCKING MANAGER --- */

/**
 * @brief Non-blocking PID window manager (for Arduino-style loops).
 *
 * Call this function rapidly in your main loop(). It automatically handles:
 * 1. PID window timing (using window_period_ms).
 * 2. Calling pid_heater_compute_on_ms() at the start of a new window.
 * 3. Controlling the SSR state via the callback during the window.
 *
 * @param h Handle
 * @param now_ms The current time from millis()
 * @param current_temp The latest temperature reading from your sensor.
 * @param ssr_set_callback A function pointer to your ssr_set(bool on) function.
 * @return true if a new window just started (and PID was updated), false otherwise.
 */
bool pid_heater_manage_window(pid_heater_handle_t h,
                              uint32_t now_ms,
                              float current_temp,
                              void (*ssr_set_callback)(bool));

#ifdef __cplusplus
}
#endif
#endif // PID_HEATER_H