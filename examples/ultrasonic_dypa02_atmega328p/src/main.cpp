#include <Arduino.h>
#include <SoftwareSerial.h> 
#include "ultrasonic_dypa02.h"

// Sensor Pin Configuration (for SoftwareSerial)
#define SENSOR_RX_PIN 10 // MCU RX (Sensor TX)
#define SENSOR_TX_PIN 11 // MCU TX (Sensor RX)

// Configuration for filtering and detection
#define SAMPLES_REQUIRED    15      // Get 15 valid samples
#define DIST_MIN_MM         1       // Minimum valid distance
#define DIST_MAX_MM         1000    // Maximum valid distance (1m)
#define PER_READ_TIMEOUT_MS 200     // Timeout for one packet
#define OVERALL_TIMEOUT_MS  5000    // Give up after 5 seconds
#define OBJECT_THRESHOLD_MM 80      // 8.0 cm

// --- Define a sensor handle ---
dyp_sensor_t sensor_1 = DYP_SENSOR_INITIALIZER;


void setup() {
  // Start debug serial
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Arduino DYP-A02 Sensor Test (Robust)");
  Serial.print("Object threshold: ");
  Serial.print(OBJECT_THRESHOLD_MM);
  Serial.println(" mm");

  Serial.println("Initializing sensor with SoftwareSerial...");
  
  // --- NEW: Check dyp_err_t return code ---
  dyp_err_t err = dyp_a02_init(
      &sensor_1, // <-- Pass sensor handle
      SENSOR_RX_PIN, 
      SENSOR_TX_PIN
  );
  
  if (err != DYP_OK) {
    // --- NEW: Print descriptive error string ---
    Serial.print("Failed to initialize sensor library: ");
    Serial.println(dyp_a02_strerror(err));
    while(1); // Halt
  }

  Serial.println("Sensor initialized.");
}

void loop() {
  // --- Pass the sensor handle to the read function ---
  long dist_mm = dyp_a02_read_average_filtered_int(
      &sensor_1, // <-- Pass sensor handle
      SAMPLES_REQUIRED,
      DIST_MIN_MM,
      DIST_MAX_MM,
      PER_READ_TIMEOUT_MS,
      OVERALL_TIMEOUT_MS
  );

  if (dist_mm != DYP_A02_READ_ERROR) {
    Serial.print("Average distance: ");
    Serial.print(dist_mm);
    Serial.print(" mm  (");
    Serial.print((float)dist_mm / 10.0f, 2);
    Serial.println(" cm)");

    bool detected = dyp_a02_is_object_detected(dist_mm, OBJECT_THRESHOLD_MM);
    if (detected) {
      Serial.println("--> Object DETECTED!");
    } else {
      Serial.println("--> No object detected.");
    }

  } else {
    // --- NEW: Get and print the specific error ---
    dyp_err_t last_err = dyp_a02_get_last_error(&sensor_1);
    Serial.print("Failed to read sensor: ");
    Serial.println(dyp_a02_strerror(last_err));
  }
  
  delay(500); // Wait 500ms before next read
}