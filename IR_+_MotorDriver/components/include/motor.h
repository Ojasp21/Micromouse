
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"

#define MotorPin_A1 GPIO_NUM_12
#define MotorPin_A2 GPIO_NUM_25
#define MotorPin_B1 GPIO_NUM_13
#define MotorPin_B2 GPIO_NUM_26

void EnableMotor();

void SET_MOTOR_DIRECTION(int direction);