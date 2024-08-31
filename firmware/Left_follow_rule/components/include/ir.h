#ifdef IR_H
#define IR_H

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"

#define IR_LEFTMOST GPIO_NUM_17
#define IR_RIGHTMOST GPIO_NUM_15
#define IR_CENTER GPIO_NUM_4

void IR_Enable();

#endif