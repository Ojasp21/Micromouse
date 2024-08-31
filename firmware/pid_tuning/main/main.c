#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#define LEFTMOST_IR   ADC2_CHANNEL_5
#define RIGHTMOST_IR   ADC2_CHANNEL_6

#define LEFT_MOTOR_A   32
#define LEFT_MOTOR_B   33
#define RIGHT_MOTOR_B   25
#define RIGHT_MOTOR_A   26

#define Standard_Speed  60

ledc_timer_config_t pwm_timer;
ledc_channel_config_t pwm_channel[2];

int left_error_previous;
int right_error_previous;

int left_error_new;
int right_error_new;

float Kp = 1;
float Kd = 0.1;

int left_pwm_output;
int right_pwm_output;

static void init_pwm() {
    adc2_config_channel_atten(LEFTMOST_IR, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(RIGHTMOST_IR, ADC_ATTEN_DB_11);

    pwm_timer.duty_resolution = LEDC_TIMER_10_BIT;
    pwm_timer.freq_hz = 1000;
    pwm_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    pwm_timer.timer_num = LEDC_TIMER_0;
    pwm_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&pwm_timer);

    pwm_channel[0].channel = LEDC_CHANNEL_0;
    pwm_channel[0].duty = 0;
    pwm_channel[0].gpio_num = LEFT_MOTOR_A;
    pwm_channel[0].speed_mode = LEDC_HIGH_SPEED_MODE;
    pwm_channel[0].hpoint = 0;
    pwm_channel[0].timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&pwm_channel[0]);

    pwm_channel[1].channel = LEDC_CHANNEL_1;
    pwm_channel[1].duty = 0;
    pwm_channel[1].gpio_num = RIGHT_MOTOR_A;
    pwm_channel[1].speed_mode = LEDC_HIGH_SPEED_MODE;
    pwm_channel[1].hpoint = 0;
    pwm_channel[1].timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&pwm_channel[1]);
}

int left_ir_error() {
    int new_read_left;
    int expected_read_left = 3000;  // here the readings of the left ir sensor to be achieved will be given
    int left_error;
    adc2_get_raw(LEFTMOST_IR, ADC_WIDTH_BIT_DEFAULT, &new_read_left);
    if (new_read_left >= expected_read_left) {
        left_error = -(expected_read_left - new_read_left);
    } else {
        left_error = 0;
    }
    return left_error;
}

int right_ir_error() {
    int new_read_right;
    int expected_read_right = 3000;  // here the readings of the right ir sensor to be achieved will be given
    int right_error;
    adc2_get_raw(RIGHTMOST_IR, ADC_WIDTH_BIT_DEFAULT, &new_read_right);
    if (new_read_right >= expected_read_right) {
        right_error = -(expected_read_right - new_read_right);
    } else {
        right_error = 0;
    }
    return right_error;
}

void app_main(void)
{
    init_pwm();

    while(1) {
        left_error_new = left_ir_error();
        right_error_new = right_ir_error();


        if (left_error_new == 0) {
            left_pwm_output = 0;
        } else {
            left_pwm_output = round(Kp*left_error_new + Kd*(left_error_previous - left_error_new));   
        }

        if (right_error_new == 0) {
            right_pwm_output = 0;
        } else {
            right_pwm_output = round(Kp*right_error_new + Kd*(right_error_previous - right_error_new));
        }
        

        printf("left_pwm_output = %d \n", left_pwm_output);
        printf("right_pwm_output = %d \n", right_pwm_output);
        
        // mapping of updated left and right pid pwm output

        ledc_set_duty(pwm_channel[0].speed_mode, pwm_channel[0].channel, Standard_Speed + left_pwm_output);
        ledc_update_duty(pwm_channel[0].speed_mode, pwm_channel[0].channel);
        ledc_set_duty(pwm_channel[1].speed_mode, pwm_channel[1].channel, Standard_Speed + right_pwm_output);
        ledc_update_duty(pwm_channel[1].speed_mode, pwm_channel[1].channel);

        left_error_previous = left_error_new;
        right_error_previous = right_error_new;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}