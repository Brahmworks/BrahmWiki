#include "max6675.h"
#include <math.h>
#include "rom/ets_sys.h"

static gpio_num_t _sck_pin;
static gpio_num_t _cs_pin;
static gpio_num_t _so_pin;

void max6675_init(gpio_num_t sck_pin, gpio_num_t cs_pin, gpio_num_t so_pin) {
    _sck_pin = sck_pin;
    _cs_pin = cs_pin;
    _so_pin = so_pin;

    // Configure pins
    gpio_set_direction(_cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(_sck_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(_so_pin, GPIO_MODE_INPUT);

    // Start with chip select high (inactive)
    gpio_set_level(_cs_pin, 1);
}

// Private function to read 16 bits from the sensor
static uint16_t spiread(void) {
    uint16_t value = 0;

    for (int i = 15; i >= 0; i--) {
        gpio_set_level(_sck_pin, 0);
        ets_delay_us(1);
        if (gpio_get_level(_so_pin)) {
            value |= (1 << i);
        }
        gpio_set_level(_sck_pin, 1);
        ets_delay_us(1);
    }
    return value;
}

double max6675_read_celsius(void) {
    // Select the chip
    gpio_set_level(_cs_pin, 0);
    ets_delay_us(1);

    uint16_t raw_value = spiread();

    // Deselect the chip
    gpio_set_level(_cs_pin, 1);
    ets_delay_us(1);

    // Check for open circuit (bit 2 should be 0)
    if (raw_value & 0x4) {
        return NAN; // Not a Number, indicates an error
    }

    // The actual temperature data is in the top 12 bits.
    // Shift right by 3 to discard the status bits.
    raw_value >>= 3;

    // The temperature is the 12-bit value multiplied by the resolution (0.25°C).
    return raw_value * 0.25;
}