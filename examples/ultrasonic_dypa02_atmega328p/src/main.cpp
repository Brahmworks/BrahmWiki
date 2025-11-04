#include <Arduino.h>
#include <ultrasonic_dypa02.h>

/* Fixed PCB pins */
#define RX_PIN A4  // Sensor TX -> MCU RX
#define TX_PIN A5  // Sensor RX <- MCU TX

/* Detection threshold in cm */
const float THRESHOLD_CM = 8.0f;

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  Serial.println("========================================");
  Serial.println("  DYP-A02YYGDW Multi-Unit Test");
  Serial.println("  Using library functions only");
  Serial.println("  Threshold: < 8 cm = DETECTED");
  Serial.println("========================================");
  Serial.println();

  if (dyp_uart_init(RX_PIN, TX_PIN, 9600) != 0) {
    Serial.println("ERROR: dyp_uart_init failed!");
    while (1) delay(1000);
  }

  Serial.println("Sensor initialized successfully!");
  Serial.println("Reading distance every 500 ms...");
  Serial.println();
  delay(1000);
}

void loop() {
  float distance_mm = 0.0f;
  float distance_cm = 0.0f;
  float distance_m  = 0.0f;
  float distance_ft = 0.0f;

  int res_mm = dyp_uart_read_distance_mm_float(&distance_mm, 100);
  int res_cm = dyp_uart_read_distance_cm_float(&distance_cm, 100);
  int res_m  = dyp_uart_read_distance_meter_float(&distance_m, 100);
  int res_ft = dyp_uart_read_distance_feet_float(&distance_ft, 100);

  // If any one of them succeeds, print
  if (res_cm == 0) {
    Serial.println("----------------------------------------");
    Serial.print("Distance: ");
    Serial.print(distance_mm, 1); Serial.print(" mm  |  ");
    Serial.print(distance_cm, 2); Serial.print(" cm  |  ");
    Serial.print(distance_m, 3);  Serial.print(" m  |  ");
    Serial.print(distance_ft, 3); Serial.println(" ft");

    bool detected = (distance_cm > 0.0f && distance_cm <= THRESHOLD_CM);

    if (detected)
      Serial.println("🟢 Object DETECTED within 8 cm!");
    else
      Serial.println("⚪ No object detected (beyond 8 cm)");
  } 
  else {
    Serial.println("TIMEOUT/ERROR - No valid frame");
  }

  delay(500);
}
