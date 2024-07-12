/
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define IR_SENSOR_PIN GPIO_NUM_26 // Define the GPIO pin connected to the IR sensor

void app_main(void) {
    // Configure the IR sensor pin as input
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;  // Disable interrupt
    io_conf.mode = GPIO_MODE_INPUT;         // Set as input mode
    io_conf.pin_bit_mask = (1ULL << IR_SENSOR_PIN); // Pin bit mask
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // Disable pull-up mode
    gpio_config(&io_conf);

    while (1) {
        int sensorValue = gpio_get_level(IR_SENSOR_PIN); // Read the IR sensor value

        if (sensorValue == 1) {
            printf("Object is near\n");
        } else {
            printf("Object is not near\n");
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Delay to avoid spamming the output
    }
}