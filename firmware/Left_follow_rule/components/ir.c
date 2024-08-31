#include "ir.h"
#include <stdio.h>

void IR_Enable() {
    gpio_set_direction(IR_LEFTMOST, GPIO_MODE_INPUT);
    gpio_set_direction(IR_RIGHTMOST, GPIO_MODE_INPUT);
    gpio_set_direction(IR_CENTER, GPIO_MODE_INPUT);
}