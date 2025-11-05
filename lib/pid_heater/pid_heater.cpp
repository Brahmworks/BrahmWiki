// pid_heater.cpp
// Implementation of pid_heater.h opaque PID heater controller
// Produces integer outputs in configured [out_min..out_max] range
// Uses floating-point PID internally and anti-windup

#include "pid_heater.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

// --- Add platform-specific includes for math ---
#if defined(ESP_PLATFORM)
  #include "esp_idf_version.h"
#elif defined(ARDUINO)
  #include <Arduino.h> // For lrint
#endif
// --- End add ---

struct pid_heater_s {
    pid_heater_tune_t tune;
    pid_ctrl_mode_t ctrl_mode;
    pid_out_mode_t out_mode;
    int out_min;
    int out_max;
    uint32_t update_period_ms;
    uint32_t window_period_ms;
    float setpoint_c;

    // internal state
    double i_term;         // integrator (double for stability)
    double last_error;
    int last_output;       // most recent integer output
    bool inited;

    // --- Added for non-blocking manager ---
    uint32_t window_start_ms;
    uint32_t current_on_ms;
    float last_measured_c; // Store last good temp
};

// Create new instance (heap-allocated)
pid_heater_handle_t pid_heater_create(void) {
    pid_heater_handle_t h = (pid_heater_handle_t)calloc(1, sizeof(struct pid_heater_s));
    if (h) {
        h->i_term = 0.0;
        h->last_error = 0.0;
        h->last_output = 0;
        h->setpoint_c = 0.0f;
        h->inited = false;
        // default tune
        h->tune.Kp = 1.0f; h->tune.Ki = 0.0f; h->tune.Kd = 0.0f; h->tune.mode = PID_CTRL_PI;
        
        // Init non-blocking fields
        h->window_start_ms = 0;
        h->current_on_ms = 0;
        h->last_measured_c = NAN;
    }
    return h;
}

void pid_heater_reset(pid_heater_handle_t h) {
    if (!h) return;
    h->i_term = 0.0;
    h->last_error = 0.0;
    h->last_output = h->out_min;
    
    // Reset non-blocking fields
    h->window_start_ms = 0;
    h->current_on_ms = 0;
    h->last_measured_c = NAN;
}

void pid_heater_init_manual(pid_heater_handle_t h,
                            pid_heater_tune_t tune,
                            pid_ctrl_mode_t mode,
                            pid_out_mode_t out_mode,
                            int out_min, int out_max,
                            uint32_t update_period_ms,
                            uint32_t window_period_ms)
{
    if (!h) return;
    h->tune = tune;
    h->ctrl_mode = mode;
    h->out_mode = out_mode;
    h->out_min = out_min;
    h->out_max = out_max;
    if (h->out_min > h->out_max) { int tmp = h->out_min; h->out_min = h->out_max; h->out_max = tmp; }
    h->update_period_ms = update_period_ms;
    h->window_period_ms = window_period_ms;
    h->i_term = 0.0;
    h->last_error = 0.0;
    h->last_output = h->out_min;
    h->inited = true;
}

void pid_heater_set_setpoint(pid_heater_handle_t h, float setpoint_c) {
    if (!h) return;
    h->setpoint_c = setpoint_c;
}

static inline int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline double clamp_double(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * pid_heater_update
 * - measured_c: measured temperature in Celsius
 * - dt_ms: elapsed milliseconds since last update (use configured update_period_ms if fixed)
 * Returns integer output in [out_min..out_max]
 */
int pid_heater_update(pid_heater_handle_t h, float measured_c, uint32_t dt_ms) {
    if (!h) return 0;
    if (!h->inited) {
        // default behavior: return min output until initialized
        return h->out_min;
    }

    double dt = (dt_ms > 0) ? ((double)dt_ms / 1000.0) : ((double)h->update_period_ms / 1000.0);
    // Protect huge dt (avoid exploding integral)
    if (dt > 60.0) dt = 60.0;

    double error = (double)h->setpoint_c - (double)measured_c;
    double P = h->tune.Kp * error;
    double D = 0.0;
    double I = h->i_term;

    if (h->ctrl_mode == PID_CTRL_P) {
        // P only
    } else if (h->ctrl_mode == PID_CTRL_PI || h->ctrl_mode == PID_CTRL_PID) {
        // integrate
        I += (double)h->tune.Ki * error * dt;
    }
    if (h->ctrl_mode == PID_CTRL_PID) {
        D = (dt > 0.0) ? (h->tune.Kd * (error - h->last_error) / dt) : 0.0;
    }

    // Preliminary unclamped output (floating)
    double out_unc = P + I + D;

    // Anti-windup: clamp integrator so output won't exceed bounds
    // Map desired out range to floating scale: out_min..out_max
    double out_min = (double)h->out_min;
    double out_max = (double)h->out_max;

    // If out_unc outside limits, adjust integrator to bring it inside:
    if (I != h->i_term) {
        // (already updated above) - keep as is for now
    }

    // If output would be out of range, limit integrator to keep output within range
    if (out_unc > out_max) {
        // set I so that P + I + D == out_max => I = out_max - P - D
        I = out_max - P - D;
    } else if (out_unc < out_min) {
        I = out_min - P - D;
    }

    // Now recompute unclamped output
    out_unc = P + I + D;

    // Clamp final output
    double out_clamped = clamp_double(out_unc, out_min, out_max);

    // Commit integrator back to state
    h->i_term = I;
    h->last_error = error;

    // Convert to integer output
    int out_int = (int)lrint(out_clamped); // round to nearest
    out_int = clamp_int(out_int, h->out_min, h->out_max);
    h->last_output = out_int;
    return out_int;
}

/**
 * pid_heater_frac_to_output - convert fraction (0.0..1.0) to output int in configured range
 */
int pid_heater_frac_to_output(pid_heater_handle_t h, float fraction) {
    if (!h) return 0;
    if (!isfinite(fraction)) fraction = 0.0f;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    double r = (double)h->out_min + (double)(h->out_max - h->out_min) * (double)fraction;
    return clamp_int((int)lrint(r), h->out_min, h->out_max);
}

// --- Add extern C wrapper for C++ file ---
extern "C" {

float pid_heater_get_last_output_fraction(pid_heater_handle_t h) {
    if (!h || !h->inited) return 0.0f;
    if (h->out_max == h->out_min) return 0.0f;
    
    double frac = (double)(h->last_output - h->out_min) / (double)(h->out_max - h->out_min);
    return (float)clamp_double(frac, 0.0, 1.0);
}

int pid_heater_get_last_output(pid_heater_handle_t h) {
    if (!h) return 0;
    return h->last_output;
}

uint32_t pid_heater_compute_on_ms(pid_heater_handle_t h, float measured_c, uint32_t window_ms, uint32_t dt_ms) {
    if (!h) return 0;
    // call existing update (this updates integrator, last_error and last_output)
    int out = pid_heater_update(h, measured_c, dt_ms);
    
    // ensure in range
    if (out < h->out_min) out = h->out_min;
    if (out > h->out_max) out = h->out_max;
    
    // compute fraction w.r.t configured range (out_min..out_max)
    double fraction = 0.0;
    if (h->out_max > h->out_min) {
        fraction = (double)(out - h->out_min) / (double)(h->out_max - h->out_min);
        if (fraction < 0.0) fraction = 0.0;
        if (fraction > 1.0) fraction = 1.0;
    }
    
    uint32_t on_ms = (uint32_t)lrint(fraction * (double)window_ms);
    
    // Save last_output (already done by pid_heater_update) but ensure consistency:
    h->last_output = out;
    return on_ms;
}


/* --- NEW NON-BLOCKING MANAGER --- */

bool pid_heater_manage_window(pid_heater_handle_t h,
                              uint32_t now_ms,
                              float current_temp,
                              void (*ssr_set_callback)(bool))
{
    if (!h || !ssr_set_callback) return false;

    // Store the last good temperature
    if (isfinite(current_temp)) {
        h->last_measured_c = current_temp;
    }
    float temp_to_use = isfinite(h->last_measured_c) ? h->last_measured_c : 0.0f;

    uint32_t elapsed = now_ms - h->window_start_ms;
    bool new_window = false;

    // 1. Check if window is complete
    if (elapsed >= h->window_period_ms) {
        // Start a new window
        new_window = true;

        // Calculate the on_ms for this new window
        h->current_on_ms = pid_heater_compute_on_ms(h, 
                                                  temp_to_use, 
                                                  h->window_period_ms, 
                                                  h->update_period_ms);
        
        // Reset window timing
        h->window_start_ms = now_ms;
        elapsed = 0;

        // Ensure SSR is OFF at the start
        ssr_set_callback(false);
    }

    // 2. Drive SSR based on state for the current window
    if (h->current_on_ms > 0 && elapsed < h->current_on_ms) {
        ssr_set_callback(true);
    } else {
        ssr_set_callback(false);
    }

    return new_window;
}


} // extern "C"