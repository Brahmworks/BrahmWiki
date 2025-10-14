#include "ultrasonic_dypa02.h"

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_timer.h>

static gpio_num_t trigger_pin_global;
static gpio_num_t echo_pin_global;

void ultrasonic_init(uint8_t trigger_pin, uint8_t echo_pin) {
    trigger_pin_global = (gpio_num_t)trigger_pin;
    echo_pin_global = (gpio_num_t)echo_pin;
    gpio_set_direction(trigger_pin_global, GPIO_MODE_OUTPUT);
    gpio_set_direction(echo_pin_global, GPIO_MODE_INPUT);
}

float measureDistanceInMillimeters() {
    gpio_set_level(trigger_pin_global, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(trigger_pin_global, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(trigger_pin_global, 0);

    while (gpio_get_level(echo_pin_global) == 0) {}
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin_global) == 1) {}
    int64_t end_time = esp_timer_get_time();

    long duration = end_time - start_time;
    return (duration * 0.343) / 2;
}

#else // ARDUINO

#include <Arduino.h>

static uint8_t trigger_pin_global;
static uint8_t echo_pin_global;

void ultrasonic_init(uint8_t trigger_pin, uint8_t echo_pin) {
    trigger_pin_global = trigger_pin;
    echo_pin_global = echo_pin;
    pinMode(trigger_pin_global, OUTPUT);
    pinMode(echo_pin_global, INPUT);
}

float measureDistanceInMillimeters() {
    digitalWrite(trigger_pin_global, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin_global, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin_global, LOW);

    long duration = pulseIn(echo_pin_global, HIGH);
    return (duration * 0.343) / 2;
}

#endif // ESP_PLATFORM

// Common functions
float measureDistanceInCentimeters() {
    return measureDistanceInMillimeters() / 10.0;
}

float measureDistanceInInches() {
    return measureDistanceInCentimeters() * 0.393701;
}

float measureDistanceInFeet() {
    return measureDistanceInInches() / 12.0;
}

bool isObstacleDetected(distance_unit_t unit, float distance) {
    float measured_distance;
    switch (unit) {
        case UNIT_MM:
            measured_distance = measureDistanceInMillimeters();
            break;
        case UNIT_CM:
            measured_distance = measureDistanceInCentimeters();
            break;
        case UNIT_INCH:
            measured_distance = measureDistanceInInches();
            break;
        case UNIT_FEET:
            measured_distance = measureDistanceInFeet();
            break;
        default:
            return false;
    }
    return measured_distance < distance;
}
