#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define LEFTMOST_IR  ADC2_CHANNEL_5
#define RIGHTMOST_IR  ADC2_CHANNEL_6

void app_main(void)
{
    adc2_config_channel_atten(LEFTMOST_IR, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(RIGHTMOST_IR, ADC_ATTEN_DB_11);
    
    while (1)
    {
        int LEFTMOST_READING;
        adc2_get_raw(LEFTMOST_IR, ADC_WIDTH_BIT_DEFAULT, &LEFTMOST_READING);
        printf("LEFTMOST_READING: %d \n", LEFTMOST_READING);

        int RIGHTMOST_READING;
        adc2_get_raw(RIGHTMOST_IR, ADC_WIDTH_BIT_DEFAULT, &RIGHTMOST_READING);
        printf("RIGHTMOST_READING: %d \n", RIGHTMOST_READING);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
}
 