/**
 * @file main.cpp
 * @brief ESP32 Host Example - Sends commands to ATmega client
 * 
 * Wiring:
 *   ESP32 TX (GPIO17) <---> ATmega RX (Pin 2)
 *   ESP32 RX (GPIO16) <---> ATmega TX (Pin 3)
 *   GND <---> GND
 */

#include <Arduino.h>
#include <RobustUART.h>
#include <RobustUART_host.h>

// Create Host instance using Serial2
// RX pin 16, TX pin 17, baud 115200
RobustHost<HardwareSerial> host(&Serial2);

void setup() {
    // Debug serial
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== ESP32 RobustUART Host ===");
    Serial.println("Wiring: ESP32 TX->ATmega RX, ESP32 RX->ATmega TX");
    
    // Initialize Host UART
    // Using Serial2: RX=16, TX=17
    host.begin(115200, 16, 17);
    
    Serial.println("Host initialized. Starting ping test...");
}

void loop() {
    static uint32_t last_ping = 0;
    static bool first_ping = true;
    
    // Send ping every 5 seconds
    if (millis() - last_ping > 5000 || first_ping) {
        first_ping = false;
        last_ping = millis();
        
        Serial.println("\n--- Sending Ping ---");
        
        if (host.ping(500)) {
            Serial.println("Ping successful!");
            Serial.print("Response: ");
            Serial.write(host.get_response(), host.get_response_length());
            Serial.println();
        } else {
            Serial.println("Ping failed - no response");
        }
        
        // Also get status
        Serial.println("\n--- Getting Status ---");
        uint8_t status[32];
        size_t len = sizeof(status);
        
        if (host.get_status(status, len, 500)) {
            Serial.print("Status received (");
            Serial.print(len);
            Serial.println(" bytes):");
            for (size_t i = 0; i < len; i++) {
                Serial.printf("%02X ", status[i]);
            }
            Serial.println();
        } else {
            Serial.println("Status request failed");
        }
        
        // Get firmware version
        Serial.println("\n--- Getting Firmware ---");
        uint8_t version[32];
        len = sizeof(version);
        
        if (host.get_firmware_version(version, len, 500)) {
            Serial.print("Firmware: ");
            Serial.write(version, len);
            Serial.println();
        } else {
            Serial.println("Firmware request failed");
        }
        
        // Print statistics
        auto stats = host.get_stats();
        Serial.println("\n--- Statistics ---");
        Serial.printf("Packets sent: %u\n", stats.packets_sent);
        Serial.printf("Packets received: %u\n", stats.packets_received);
        Serial.printf("Errors: %u\n", stats.packets_error);
        Serial.printf("Retries: %u\n", stats.retries);
        Serial.printf("CRC errors: %u\n", stats.crc_errors);
        Serial.printf("Timeouts: %u\n", stats.timeouts);
    }
    
    delay(100);
}
