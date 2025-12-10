#include <Arduino.h>
#include "servo_control.h"

// Instantiate ServoControl object
ServoControl servos;

// Use UART 1 for Servo Communication
HardwareSerial ServoSerial(1);

// Pin Definitions (Modify as needed)
#define S_RXD 22
#define S_TXD 21

void setup() {
    // Initialize Debug Serial
    Serial.begin(115200);
    // Wait for serial port to connect (optional, for debugging)
    delay(1000); 
    Serial.println("--- Starting Servo Control Example ---");

    // Initialize Servo Library
    // This sets up the serial port (1Mbps, 8N1) and links it to the SCServo instance
    servos.begin(&ServoSerial, S_RXD, S_TXD);
    Serial.println("Servo Serial Initialized.");

    // Initial sequence: Move all to center
    Serial.println("Centering all motors...");
    servos.moveServo(SERVO_ID_TORSO, 180, 1000, 20);
    delay(100);
    servos.moveServo(SERVO_ID_NECK, 180, 1000, 20);
    delay(100);
    servos.moveServo(SERVO_ID_HEAD, 180, 1000, 20);
    
    Serial.println("Setup Complete. Starting Loop...");
    delay(2000);
}

void loop() {
    // ---------------------------------------------------------
    // Scenario 1: Direct Control (Programmatic Sequence)
    // ---------------------------------------------------------
    Serial.println("Scenario 1: Moving Head Left and Right");
    
    // Move Head to 90 degrees
    servos.moveServo(SERVO_ID_HEAD, 90, 800, 40);
    delay(1500);

    // Move Head to 270 degrees
    servos.moveServo(SERVO_ID_HEAD, 270, 800, 40);
    delay(1500);

    // Return Head to Center
    servos.moveServo(SERVO_ID_HEAD, 180, 800, 40);
    delay(1000);

    // ---------------------------------------------------------
    // Scenario 2: Processing JSON Commands (Simulated)
    // ---------------------------------------------------------
    Serial.println("Scenario 2: Simulating JSON Command execution");

    // Command to move Torso slowly
    const char* cmd_torso = "{\"angle\":220,\"speed\":200,\"accel\":10}";
    Serial.printf("Processing Command for Torso: %s\n", cmd_torso);
    if (servos.processCommand(cmd_torso, SERVO_ID_TORSO)) {
        Serial.println("Command Executed Successfully.");
    } else {
        Serial.println("Command Failed (Invalid JSON?).");
    }
    delay(2000);

    // Command to move Neck quickly
    const char* cmd_neck = "{\"angle\":45,\"speed\":1500,\"accel\":50}";
    Serial.printf("Processing Command for Neck: %s\n", cmd_neck);
    servos.processCommand(cmd_neck, SERVO_ID_NECK);
    delay(2000);

    // ---------------------------------------------------------
    // Reset Sequence
    // ---------------------------------------------------------
    Serial.println("Resetting positions...");
    servos.moveServo(SERVO_ID_TORSO, 180, 1000, 20);
    servos.moveServo(SERVO_ID_NECK, 180, 1000, 20);
    servos.moveServo(SERVO_ID_HEAD, 180, 1000, 20);
    
    Serial.println("Loop Finished. Waiting...");
    delay(3000);
}
