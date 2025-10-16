// Arduino UNO example for MAX31865 (PlatformIO)
// Wiring:
//   MAX31865 VIN -> 5V (if breakout supports 5V) or 3.3V (preferred)
//   MAX31865 GND -> GND
//   MAX31865 SCK -> D13 (SCK)
//   MAX31865 MOSI -> D11
//   MAX31865 MISO -> D12
//   MAX31865 CS -> D10
//
// Place max31865.h/.cpp into lib/max31865/src/ or src/

#include <Arduino.h>
#include "max31865.h"

#define CS_PIN 10

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("MAX31865 Arduino UNO example");

  /* Initialize HW SPI */
  if (max31865_init_hw(CS_PIN, 1000000) != 0) {
    Serial.println("max31865_init_hw failed");
    while (1) delay(1000);
  }

  /* Configure for PT100, 3-wire, set reference resistor (example) */
  max31865_set_rtd_nominal(100.0f, 430.0f);
  max31865_set_wire_mode(MAX31865_WIRES_3);
  max31865_set_filter_50hz(false); // use 60Hz (USA)
  delay(10);
}

void loop() {
  /* Start one-shot conversion */
  max31865_enable_bias(true);
  delay(10); // bias settle
  max31865_start_one_shot();
  delay(200); // conversion time (200ms safe for 50Hz)
  max31865_enable_bias(false);

  float c = max31865_read_temperature_c_float(2);
  long ci = max31865_read_temperature_c_int(2);

  if (!isnan(c)) {
    Serial.print("Temp C: ");
    Serial.print(c, 2);
    Serial.print("  int x100: ");
    Serial.println(ci);
  } else {
    Serial.println("Read failed or fault!");
    uint8_t fault = max31865_last_fault();
    Serial.print("Fault: 0x");
    Serial.println(fault, HEX);
  }

  delay(1000);
}
