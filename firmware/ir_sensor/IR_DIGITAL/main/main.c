#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/dac.h"

#define LEFTMOST_IR_PIN GPIO_NUM_17
#define RIGHTMOST_IR_PIN GPIO_NUM_15
#define CENTER_IR_PIN GPIO_NUM_4
#define LEFT45_IR_PIN GPIO_NUM_16
#define RIGHT45_IR_PIN GPIO_NUM_2

#define pin1 

void app_main(void)
{
    gpio_set_direction(LEFT45_IR_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LEFTMOST_IR_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(RIGHT45_IR_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(RIGHTMOST_IR_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(CENTER_IR_PIN, GPIO_MODE_INPUT);

    while (1)
    {
    int read = gpio_get_level(RIGHTMOST_IR_PIN);
    printf("%d \n", read);
    vTaskDelay(10);
    }    
}