// File: src/main.cpp
// PlatformIO: board = uno, framework = arduino
// Wiring (hardware SPI):
//   MAX6675 VCC -> 5V
//   MAX6675 GND -> GND
//   MAX6675 CS  -> Arduino digital pin 10 (you can change)
//   MAX6675 SCK -> Arduino SCK (D13)
//   MAX6675 SO  -> Arduino MISO (D12)
//
// For Software SPI:
//   MAX6675 SCK -> D8 (example), SO -> D9, CS -> D10

#include <Arduino.h>
#include "max6675.h"

#define CS_PIN 10  // UNO default SS pin
// Uncomment to use software SPI and comment hw init below
//#define USE_SW_SPI

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("MAX6675 Arduino UNO example");

#ifdef USE_SW_SPI
  int ret = max6675_init_sw(CS_PIN, 8, 9); // cs, sck, miso pins
  if (ret != 0) {
    Serial.println("SW SPI init failed");
  } else {
    Serial.println("SW SPI initialized");
  }
#else
  int ret = max6675_init_hw(CS_PIN, 400000); // cs, clock_hz
  if (ret != 0) {
    Serial.println("HW SPI init failed");
  } else {
    Serial.println("HW SPI initialized");
  }
#endif
}

void loop() {
  float c = max6675_read_celsius_float(2);
  float f = max6675_read_fahrenheit_float(2);
  long c_int_0 = max6675_read_celsius_int(0);
  long c_int_2 = max6675_read_celsius_int(2);
  bool open = max6675_is_open();

  if (isnan(c)) {
    Serial.println("Error reading temperature (NAN) or thermocouple open");
  } else {
    Serial.print("Temperature: ");
    Serial.print(c, 2);
    Serial.print(" °C  | ");
    Serial.print(f, 2);
    Serial.println(" °F");
    Serial.print("As int (0 dec): ");
    Serial.print(c_int_0);
    Serial.print("   As int scaled (x100): ");
    Serial.println(c_int_2);
  }

  if (open) {
    Serial.println("Thermocouple open/fault detected!");
  }

  delay(1000);
}
