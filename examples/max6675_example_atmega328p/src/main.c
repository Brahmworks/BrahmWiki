#include <Arduino.h>
#include "max6675.h"

// Define pins for Arduino Uno (standard SPI pins)
#define MAX_CS_PIN   10
#define MAX_SCK_PIN  13
#define MAX_SO_PIN   12

void setup() {
  Serial.begin(9600);
  Serial.println("MAX6675 Arduino Uno Example");
  max6675_init(MAX_SCK_PIN, MAX_CS_PIN, MAX_SO_PIN);
  delay(500); // Allow sensor to stabilize
}

void loop() {
  double temp = max6675_read_celsius();

  if (isnan(temp)) {
    Serial.println("Error: Thermocouple is not connected.");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
  }

  delay(1000);
}