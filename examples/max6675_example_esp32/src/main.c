#include <stdio.h>
#include <math.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max6675.h"

// Define GPIO pins for the sensor connection
#define MAX_CS_PIN   GPIO_NUM_5
#define MAX_SCK_PIN  GPIO_NUM_18
#define MAX_SO_PIN   GPIO_NUM_19

void app_main(void)
{
    printf("MAX6675 ESP-IDF Example\n");

    // Initialize the sensor library with the defined pins
    max6675_init(MAX_SCK_PIN, MAX_CS_PIN, MAX_SO_PIN);
    
    // Allow sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(500)); 

    while (1) {
        double temp = max6675_read_celsius();

        if (isnan(temp)) {
            printf("Error: Thermocouple is not connected or faulty.\n");
        } else {
            printf("Temperature: %.2f C\n", temp);
        }
        
        // Wait for 1 second before the next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}