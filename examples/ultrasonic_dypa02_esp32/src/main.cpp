#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic_dypa02.h"

#define TRIGGER_PIN 25
#define ECHO_PIN 26

extern "C" void app_main(void);

void app_main(void)
{
    ultrasonic_init(TRIGGER_PIN, ECHO_PIN);

    while (1) {
        float distance_cm = measureDistanceInCentimeters();
        printf("Distance: %.2f cm\n", distance_cm);

        if (isObstacleDetected(UNIT_CM, 20)) {
            printf("Obstacle detected within 20 cm!\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
