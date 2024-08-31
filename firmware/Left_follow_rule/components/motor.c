#include "motor.h"
#include <stdio.h>


void EnableMotor () {
    gpio_set_direction(MotorPin_A1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MotorPin_A2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MotorPin_B1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MotorPin_B2, GPIO_MODE_OUTPUT);
}

void SET_MOTOR_DIRECTION(int direction) {
    if (direction == 1) {
        gpio_set_level(MotorPin_A1, 1);
        gpio_set_level(MotorPin_A2, 0);
        gpio_set_level(MotorPin_B1, 1);
        gpio_set_level(MotorPin_B2, 0);
    } else if (direction == 0) {
        gpio_set_level(MotorPin_A1, 0);
        gpio_set_level(MotorPin_A2, 0);
        gpio_set_level(MotorPin_B1, 0);
        gpio_set_level(MotorPin_B2, 0);
    } else if (direction == 2) {
        gpio_set_level(MotorPin_A1, 1);
        gpio_set_level(MotorPin_A2, 0);
        gpio_set_level(MotorPin_B1, 0);
        gpio_set_level(MotorPin_B2, 0);
    } else if (direction == 3) {
        gpio_set_level(MotorPin_A1, 0);
        gpio_set_level(MotorPin_A2, 0);
        gpio_set_level(MotorPin_B1, 1);
        gpio_set_level(MotorPin_B2, 0);
    }
}