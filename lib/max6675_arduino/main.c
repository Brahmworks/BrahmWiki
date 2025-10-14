#include "max6675.h"
#include <Arduino.h> 

static uint8_t _sck_pin;
static uint8_t _cs_pin;
static uint8_t _so_pin;

void max6675_init(uint8_t sck_pin, uint8_t cs_pin, uint8_t so_pin) {
    _sck_pin = sck_pin;
    _cs_pin = cs_pin;
    _so_pin = so_pin;

    // Configure pins using Arduino functions
    pinMode(_cs_pin, OUTPUT);
    pinMode(_sck_pin, OUTPUT);
    pinMode(_so_pin, INPUT);

    // Start with chip select high (inactive)
    digitalWrite(_cs_pin, HIGH);
}

// Private function to read 16 bits from the sensor
static uint16_t spiread(void) {
    uint16_t value = 0;

    for (int i = 15; i >= 0; i--) {
        digitalWrite(_sck_pin, LOW);
        delayMicroseconds(1);
        if (digitalRead(_so_pin)) {
            value |= (1 << i);
        }
        digitalWrite(_sck_pin, HIGH);
        delayMicroseconds(1);
    }
    return value;
}

double max6675_read_celsius(void) {
    // Select the chip
    digitalWrite(_cs_pin, LOW);
    delayMicroseconds(1);

    uint16_t raw_value = spiread();

    // Deselect the chip
    digitalWrite(_cs_pin, HIGH);
    delayMicroseconds(1);

    // Check for open circuit (bit 2 should be 0)
    if (raw_value & 0x4) {
        return NAN; // Not a Number, indicates an error
    }

    // Shift right by 3 to discard the status bits.
    raw_value >>= 3;

    // The temperature is the 12-bit value multiplied by the resolution (0.25°C).
    return raw_value * 0.25;
}