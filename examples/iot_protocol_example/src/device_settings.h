#ifndef DEVICE_SETTINGS_H
#define DEVICE_SETTINGS_H

// --- Project-Specific Constants ---

#define DEVICE_ID "device_12345"
#define DEVICE_TYPE "ESP32-System"
#define FIRMWARE_VERSION "3.0.0"

// --- Sensor and Actuator IDs ---
#define SENSOR_ID_HEATER_TEMP "heater_temp"
#define SENSOR_ID_WATER_TEMP "water_temp"
#define SENSOR_ID_ULTRASONIC "ultrasonic"

#define ACTUATOR_ID_HEATER "heater"
#define ACTUATOR_ID_SOLENOID "solenoid"
#define ACTUATOR_ID_PUMP "pump"

// --- User-Defined Sensor Thresholds ---
#define HEATER_TEMP_WARN_THRESHOLD 80.0
#define HEATER_TEMP_FAIL_THRESHOLD 100.0

#define WATER_TEMP_WARN_THRESHOLD 40.0
#define WATER_TEMP_FAIL_THRESHOLD 60.0

#define ULTRASONIC_WARN_THRESHOLD 5.0
#define ULTRASONIC_FAIL_THRESHOLD 20.0

#endif // DEVICE_SETTINGS_H
