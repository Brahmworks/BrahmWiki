/**
 * @file main.cpp
 * @brief ATmega328P SoftwareSerial Client Example
 * 
 * This example demonstrates using SoftwareSerial for RobustUART communication.
 * Useful when hardware UART is already in use for debugging.
 * 
 * Wiring (for SoftwareSerial):
 *   Arduino Pin 10 (RX) <---> Host TX
 *   Arduino Pin 11 (TX) <---> Host RX
 *   GND <---> GND
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RobustUART.h>
#include <RobustUART_client.h>

// Create SoftwareSerial instance
// RX on pin 10, TX on pin 11
SoftwareSerial softSerial(10, 11); // RX, TX

// Create Client instance using SoftwareSerial
RobustClient<SoftwareSerial> client(&softSerial);

void setup() {
    // Debug serial (uses hardware UART)
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== ATmega328P SoftwareSerial RobustUART Client ===");
    Serial.println("Using SoftwareSerial on pins 10 (RX) and 11 (TX)");
    
    // Initialize Client with SoftwareSerial
    // baud, rx_pin, tx_pin
    client.begin(115200, 10, 11);
    
    Serial.println("Client initialized. Waiting for commands...");
}

void loop() {
    // Process incoming commands
    client.process();
    
    delay(10);
}
