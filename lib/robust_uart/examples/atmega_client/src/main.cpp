/**
 * @file main.cpp
 * @brief ATmega328P Client Example - Responds to Host commands
 * 
 * Wiring:
 *   ATmega RX (Pin 2) <---> ESP32 TX (GPIO17)
 *   ATmega TX (Pin 3) <---> ESP32 RX (GPIO16)
 *   GND <---> GND
 */

#include <Arduino.h>
#include <RobustUART.h>
#include <RobustUART_client.h>

// Create Client instance using Serial (UART0)
// On ATmega328P: RX=Pin 0, TX=Pin 1
RobustClient<HardwareSerial> client(&Serial);

void setup() {
    // Debug serial
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== ATmega328P RobustUART Client ===");
    Serial.println("Wiring: ATmega RX->ESP32 TX, ATmega TX->ESP32 RX");
    
    // Initialize Client UART
    // Using Serial: RX=0, TX=1 (default pins)
    client.begin(115200);
    
    Serial.println("Client initialized. Waiting for commands...");
    
    // Register custom command handlers if needed
    // client.register_handler(0x10, my_handler);
}

void loop() {
    // Process incoming commands
    client.process();
    
    // Add your own code here
    delay(10);
}
