#ifndef ULTRASONIC_DYPA02_H
#define ULTRASONIC_DYPA02_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    UNIT_MM,
    UNIT_CM,
    UNIT_INCH,
    UNIT_FEET
} distance_unit_t;

/**
 * @brief Initializes the ultrasonic sensor.
 *
 * @param trigger_pin The pin connected to the sensor's trigger input.
 * @param echo_pin The pin connected to the sensor's echo output.
 */
void ultrasonic_init(uint8_t trigger_pin, uint8_t echo_pin);

/**
 * @brief Measures the distance and returns it in millimeters.
 *
 * @return The distance in millimeters.
 */
float measureDistanceInMillimeters();

/**
 * @brief Measures the distance and returns it in centimeters.
 *
 * @return The distance in centimeters.
 */
float measureDistanceInCentimeters();

/**
 * @brief Measures the distance and returns it in inches.
 *
 * @return The distance in inches.
 */
float measureDistanceInInches();

/**
 * @brief Measures the distance and returns it in feet.
 *
 * @return The distance in feet.
 */
float measureDistanceInFeet();

/**
 * @brief Detects if an obstacle is within a specified distance.
 *
 * @param unit The unit of distance (MM, CM, INCH, FEET).
 * @param distance The distance threshold.
 * @return true if an obstacle is detected within the distance, false otherwise.
 */
bool isObstacleDetected(distance_unit_t unit, float distance);

#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_DYPA02_H
