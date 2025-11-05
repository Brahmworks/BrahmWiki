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
#define READ_AFTER_ON_MS   10U
#define OFF_GUARD_MS       50U

// --- Global State ---
static bool read_done_this_window = false;
static float last_pt100 = NAN;
static pid_heater_handle_t pid = nullptr;

// <<< FIX 1: Add a local variable to track the window start time
static uint32_t main_window_start_ms = 0;

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
  if (!read_done_this_window) {
    
      // <<< FIX 2: Use our local 'main_window_start_ms' variable
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

    // <<< FIX 3: Update our local tracker when a new window begins
    main_window_start_ms = now;

    // Reset our sensor read flag for the new window
    read_done_this_window = false;
  }

  delay(1); // Main loop delay
}