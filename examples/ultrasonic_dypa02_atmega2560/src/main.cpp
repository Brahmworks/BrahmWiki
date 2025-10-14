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
