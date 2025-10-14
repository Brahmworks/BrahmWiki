#ifndef MAX6675_H
#define MAX6675_H

#include "driver/gpio.h"

/**
 * @brief Initializes the MAX6675 sensor pins for ESP-IDF.
 *
 * @param sck_pin The GPIO number connected to the SCK (Serial Clock) input.
 * @param cs_pin The GPIO number connected to the CS (Chip Select) input.
 * @param so_pin The GPIO number connected to the SO (Serial Data Out) input.
 */
void max6675_init(gpio_num_t sck_pin, gpio_num_t cs_pin, gpio_num_t so_pin);

/**
 * @brief Reads the temperature from the MAX6675.
 *
 * @return The temperature in degrees Celsius. Returns NAN if the thermocouple is open.
 */
double max6675_read_celsius(void);

#endif // MAX6675_H