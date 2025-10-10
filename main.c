/*
 * Embedded Systems Project Template
 * Supports: ESP32, ATmega328P, ATmega2560
 *
 * Modify this file according to your project requirements
 */

#ifdef ESP32
#include <Arduino.h>
// ESP32 specific includes
#elif defined(__AVR_ATmega328P__)
#include <avr/io.h>
#include <util/delay.h>
// ATmega328P specific includes
#elif defined(__AVR_ATmega2560__)
#include <avr/io.h>
#include <util/delay.h>
// ATmega2560 specific includes
#endif

// Include your library headers here
// #include "lib/sensors/temperature_ds18b20/src/ds18b20.h"

void setup()
{
// Initialization code
#ifdef ESP32
    Serial.begin(115200);
#else
    // AVR initialization
#endif

    // Initialize your libraries here
}

void loop()
{
    // Main program logic

#ifdef ESP32
    delay(1000);
#else
    _delay_ms(1000);
#endif
}

#ifndef ESP32
int main()
{
    setup();
    while (1)
    {
        loop();
    }
    return 0;
}
#endif