#include <stdio.h>
#include "ir.h"
#include "motor.h"

void app_main(void)
{
    IR_Enable();
    EnableMotor();

    int a = 0;
    int LeftMost_IR, RightMost_IR, Center_IR;

    while(a == 0) {
        LeftMost_IR = gpio_get_value(IR_LEFTMOST);
        RightMost_IR = gpio_get_value(IR_RIGHTMOST);
        Center_IR = gpio_get_value(IR_CENTER);
        
        if (Center_IR == 0 && LeftMost_IR == 0 && RightMost_IR == 0) {
            SET_MOTOR_DIRECTION(0);
            a = 1;
        }

        if (Center_IR == 0 && LeftMost_IR == 1) {
            SET_MOTOR_DIRECTION(2);
        }

        if (Center_IR == 0 && RightMost_IR == 1) {
            SET_MOTOR_DIRECTION(3);
        }

        if (Center_IR == 1) {
            SET_MOTOR_DIRECTION(1);
        }

        // if (LeftMost_IR == 1) {
        //     SET_MOTOR_DIRECTION("left");
        // } 
        
        // if (RightMost_IR == 1) {
        //     SET_MOTOR_DIRECTION("right");
        // }
    }
    
}