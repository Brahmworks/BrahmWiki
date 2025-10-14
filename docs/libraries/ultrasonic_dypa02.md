# Ultrasonic Sensor Library (DYP-A02YYGDW-V2.0)

This library provides a simple interface for the DYP-A02YYGDW-V2.0 ultrasonic sensor.

## API

### `void ultrasonic_init(uint8_t trigger_pin, uint8_t echo_pin)`

Initializes the sensor with the specified trigger and echo pins.

### `float measureDistanceInMillimeters()`

Returns the measured distance in millimeters.

### `float measureDistanceInCentimeters()`

Returns the measured distance in centimeters.

### `float measureDistanceInInches()`

Returns the measured distance in inches.

### `float measureDistanceInFeet()`

Returns the measured distance in feet.

### `bool isObstacleDetected(distance_unit_t unit, float distance)`

Checks if an obstacle is detected within a certain distance and returns `true` or `false`.

`distance_unit_t` can be one of the following:
- `UNIT_MM`
- `UNIT_CM`
- `UNIT_INCH`
- `UNIT_FEET`

## Example Usage

### Arduino (atmega328p, atmega2560)

```cpp
#include <Arduino.h>
#include "ultrasonic_dypa02.h"

#define TRIGGER_PIN 9
#define ECHO_PIN 10

void setup() {
  Serial.begin(9600);
  ultrasonic_init(TRIGGER_PIN, ECHO_PIN);
}

void loop() {
  float distance_cm = measureDistanceInCentimeters();
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  if (isObstacleDetected(UNIT_CM, 20)) {
    Serial.println("Obstacle detected within 20 cm!");
  }

  delay(1000);
}
```

### ESP-IDF (esp32)

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic_dypa02.h"

#define TRIGGER_PIN 25
#define ECHO_PIN 26

void app_main(void)
{
    ultrasonic_init(TRIGGER_PIN, ECHO_PIN);

    while (1) {
        float distance_cm = measureDistanceInCentimeters();
        printf("Distance: %.2f cm\n", distance_cm);

        if (isObstacleDetected(UNIT_CM, 20)) {
            printf("Obstacle detected within 20 cm!\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
