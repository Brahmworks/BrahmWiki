#include <Arduino.h>
#include <SoftwareSerial.h>

//Pin Configuration
#define RX_PIN A4  
#define TX_PIN A5  
SoftwareSerial dypSerial(RX_PIN, TX_PIN);

//Constants
const float DISTANCE_THRESHOLD_CM = 8.0;  // threshold in cm

//Variables
bool objectDetected = false;

void setup() {
  Serial.begin(9600);
  dypSerial.begin(9600);

  Serial.println("DYP-A02YYGDW Object Detection Started");
  delay(500);
}

void loop() {
  static uint8_t buf[4] = {0,0,0,0};

  while (dypSerial.available() > 0) {
    // shift old bytes
    buf[0] = buf[1];
    buf[1] = buf[2];
    buf[2] = buf[3];
    buf[3] = (uint8_t)dypSerial.read();

    // check for valid frame
    if (buf[0] == 0xFF) {
      uint16_t raw = ((uint16_t)buf[1] << 8) | buf[2];
      uint8_t sum = buf[3];
      uint8_t check = (0xFF + buf[1] + buf[2]) & 0xFF;

      if (check == sum) {
        float distance_cm = raw / 10.0f;

        //Presence Logic
        objectDetected = (distance_cm <= DISTANCE_THRESHOLD_CM && distance_cm > 0);

        //Debug Output (optional)
        Serial.print("Distance: ");
        Serial.print(distance_cm, 1);
        Serial.print(" cm  |  Object Detected: ");
        Serial.println(objectDetected ? "YES" : "NO");

        // reset buffer for next frame
        buf[0] = buf[1] = buf[2] = buf[3] = 0;
      }
    }
  }

  delay(10); // small delay to avoid serial flooding
}
