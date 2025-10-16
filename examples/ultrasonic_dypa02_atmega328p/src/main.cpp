#include <Arduino.h>
#include <ultrasonic_dypa02.h>

#define RX_PIN A4  // sensor TX -> MCU RX (SoftwareSerial RX)
#define TX_PIN A5  // sensor RX <- MCU TX (SoftwareSerial TX)

/* Threshold */
const float DISTANCE_THRESHOLD_CM = 9.0f;

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }
  Serial.println("DYP-A02YYGDW UART example (library)");

  if (dyp_uart_init(RX_PIN, TX_PIN, 9600) != 0) {
    Serial.println("dyp_uart_init failed");
    while (1) delay(1000);
  }
}

void loop() {
  float distance_cm = 0.0f;
  int r = dyp_uart_read_distance_cm_float(&distance_cm, 200); // 200 ms timeout
  if (r == 0) {
    bool present = (distance_cm > 0.0f && distance_cm <= DISTANCE_THRESHOLD_CM);
    Serial.print("Distance: ");
    Serial.print(distance_cm, 1);
    Serial.print(" cm  | Object: ");
    Serial.println(present ? "YES" : "NO");
  } else {
    Serial.println("Timeout / No valid frame");
  }
  delay(100);
}
