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
#define CS_MAX6675          25  // GPIO for K-Type (MAX6675) Chip Select
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
// These are shared between the two tasks
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
    // --- NOTE ---
    // If your SSR is "Active LOW", change this to:
    // gpio_set_level(SSR_GPIO, on ? 0 : 1);
    gpio_set_level(SSR_GPIO, on ? 1 : 0); 
}


/**
 * @brief TASK B: SSR Actuator & Sensor Read Task (Fast, Non-blocking)
 *
 * This task runs every 20ms. It has two jobs:
 * 1. Control the SSR pin based on the g_ssr_on_ms value.
 * 2. Read the temperature *after* the 20ms spike delay and save it for the PID task.
 */
void ssr_control_task(void *pvParameters) {
    uint32_t window_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    bool read_done_this_window = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t elapsed = now_ms - window_start_ms;

        // Check if we need to start a new window
        if (elapsed >= CONTROL_PERIOD_MS) {
            window_start_ms = now_ms;
            elapsed = 0;
            read_done_this_window = false; // Reset read flag for new window
        }

        // Read the shared value (calculated by the other task)
        uint32_t on_ms = g_ssr_on_ms;

        // --- 1. SSR Pin Control ---
        if (elapsed < on_ms) {
            ssr_set(true);
        } else {
            ssr_set(false);
        }

        // --- 2. Sensor Read Logic ---
        if (!read_done_this_window && (elapsed >= SENSOR_READ_DELAY_MS)) {
            
            // --- Read PT100 (for control) ---
            float temp_pt100 = max31865_read_temperature_safe();
            
            if (isfinite(temp_pt100)) {
                g_latest_pt100_temp = temp_pt100; // Share the good reading
            } 
            else { 
                // --- THIS IS THE FIX ---
                // Propagate the NAN failure to the PID task
                g_latest_pt100_temp = NAN;
                // --- END FIX ---
                
                // Now, log the specific hardware fault
                // The library has now cached the fault for us.
                uint8_t fault_code = max31865_last_fault();
                
                if (fault_code != 0) {
                    ESP_LOGW(TAG, "MAX31865 FAULT DETECTED: 0x%02X", fault_code);
                    max31865_clear_faults(); // Auto-clear the fault
                } else {
                    ESP_LOGW(TAG, "MAX31865 read failed (NAN). Check wiring or power.");
                }
            }

            
            // --- Read K-Type (for logging) ---
            float temp_ktype = max6675_read_celsius_float(2);
            if (isfinite(temp_ktype)) {
                g_latest_ktype_temp = temp_ktype; // Share the good reading
            }

            read_done_this_window = true; // Mark as read
        }

        // Run this check every 20ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SSR_LOOP_PERIOD_MS));
    }
}

/**
 * @brief TASK A: PID Brain Task (Slow, Periodic)
 *
 * This task wakes up *exactly* every 3000ms. Its only job is to:
 * 1. Read the latest temperature (that the other task got for us).
 * 2. Calculate the new PID output.
 * 3. Share the result (g_ssr_on_ms) for the SSR task to use.
 */
void pid_calc_task(void *pvParameters) {
    // This makes the task run exactly on the 3000ms mark
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while (1) {
        // 1. Wait for the next 3000ms cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // --- We are now at the start of a new 3000ms window ---

        // 2. Get the latest temperature (that the SSR task read for us)
       float temp_to_use = g_latest_pt100_temp;
        
        // --- THIS IS THE FIX ---
        // Check for the NAN that the SSR task just passed to us
        if (!isfinite(temp_to_use)) {
            temp_to_use = 0.0f; // Set to 0.0f for logging
            
            // This prevents integrator windup from the 55.09 value
            pid_heater_reset(g_pid);
            ESP_LOGW(TAG, "Sensor fault detected. Resetting PID integrator.");
        }
        // --- END FIX ---

        // 3. Compute output (fast)
        int out = pid_heater_update(g_pid, temp_to_use, CONTROL_PERIOD_MS);

        // 4. Calculate the ON time for the *next* window
        uint32_t on_ms = (uint32_t)roundf(((float)out / 100.0f) * (float)CONTROL_PERIOD_MS);

        // 5. Share the result with the SSR task
        // This is the "hand-off"
        g_ssr_on_ms = on_ms;

        // 6. Log
        float t_s = (esp_timer_get_time() - g_start_us) / 1e6f;
        float temp_ktype_to_log = g_latest_ktype_temp; // Read the shared K-type value

        ESP_LOGI(TAG, "t=%.1fs, PT100=%.2f, KType=%.2f, Duty=%d%% (New ON_ms: %d)", 
                 t_s, 
                 temp_to_use, 
                 isnan(temp_ktype_to_log) ? 0.0f : temp_ktype_to_log, // Handle NAN for logging
                 out, 
                 on_ms);

        // 7. Go back to sleep.
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
    g_latest_pt100_temp = max31865_read_temperature_safe(); // Get one initial reading
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