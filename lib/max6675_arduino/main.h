#ifndef MAX6675_H
#define MAX6675_H

#include <stdint.h> 

/**
 * @brief Initializes the MAX6675 sensor pins for the Arduino framework.
 *
 * @param sck_pin The pin connected to the SCK (Serial Clock) input.
 * @param cs_pin The pin connected to the CS (Chip Select) input.
 * @param so_pin The pin connected to the SO (Serial Data Out) input.
 */
void max6675_init(uint8_t sck_pin, uint8_t cs_pin, uint8_t so_pin);

/**
 * @brief Reads the temperature from the MAX6675.
 *
 * @return The temperature in degrees Celsius. Returns NAN if the thermocouple is open.
 */
double max6675_read_celsius(void);

#endif // MAX6675_H