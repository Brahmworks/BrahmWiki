#include <Arduino.h>
#include <math.h> // For isnan()

// Make sure to include the C-style header
extern "C" {
  #include "max6675.h"
}

// Define the Chip Select pin
#define CS_PIN 10

// --- Software SPI Pin Definitions (Optional) ---
// If you want to use Software SPI, uncomment these
// and change the init function in setup()
// #define SW_SCK_PIN  13
// #define SW_MISO_PIN 12

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { /* wait for serial */ }

  Serial.println("Initializing MAX6675...");
  
  // --- Initialize using Hardware SPI ---
  // Format: max6675_init_hw(CS_PIN, CLOCK_SPEED_HZ)
  int init_result = max6675_init_hw(CS_PIN, 1000000); // 1MHz clock
  
  // --- Alternative: Initialize using Software SPI ---
  // int init_result = max6675_init_sw(CS_PIN, SW_SCK_PIN, SW_MISO_PIN);

  if (init_result != 0) {
    Serial.println("MAX6675 initialization failed!");
  }
}

void loop() {
  Serial.println("--- Reading All Values ---");

  // 1. Read all the different temperature formats
  float temp_c_float = max6675_read_celsius_float(2);   // °C with 2 decimals
  float temp_f_float = max6675_read_fahrenheit_float(2); // °F with 2 decimals
  long  temp_c_int   = max6675_read_celsius_int(2);     // °C scaled by 100 (e.g., 2525)
  long  temp_f_int   = max6675_read_fahrenheit_int(2);   // °F scaled by 100 (e.g., 7745)

  // 2. Read the raw 16-bit data
  uint16_t raw_data;
  int raw_read_result = max6675_read_raw(&raw_data);

  // 3. Check for errors
  // We can check any of the float/int results. If one fails, they all fail.
  if (temp_c_int == MAX6675_ERROR || raw_read_result != 0) {
    
    Serial.println("ERROR: Read failed.");

    // 4. Use is_open() to find out why
    if (max6675_is_open()) {
      Serial.println("Cause: Thermocouple is open (disconnected).");
    } else {
      Serial.println("Cause: SPI communication error.");
    }

  } else {
    // 5. Print all the successful readings
    Serial.print("  Celsius (float):    "); Serial.print(temp_c_float, 2); Serial.println(" C");
    Serial.print("  Fahrenheit (float): "); Serial.print(temp_f_float, 2); Serial.println(" F");
    Serial.print("  Celsius (int x100): "); Serial.println(temp_c_int);
    Serial.print("  Fahrenheit (int x100):"); Serial.println(temp_f_int);
    Serial.print("  Raw 16-bit Value:   0x"); Serial.println(raw_data, HEX);
  }
  
  // Note: max6675_deinit() is available but rarely used in Arduino 
  // unless you are dynamically re-allocating pins.
  
  delay(2000); // Wait two seconds
}