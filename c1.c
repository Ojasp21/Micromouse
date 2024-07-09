// C Header file having Standard Input Output Functions
#include <stdio.h>

// Header files for Free RTOS - Real Time Operating Systems
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// SRA's custom header file including additional functions
#include "sra_board.h"

// threshold value for object detection
#define OBJECT_DETECTION_THRESHOLD 500

// pointer to a character array
static const char *TAG = "IR_READINGS";

// main driver function
void app_main(void)
{
    // enable IR sensor after checking optimal working state of ESP
    adc_handle_t ir_sensor;
    ESP_ERROR_CHECK(enable_ir_sensor(&ir_sensor));

    // infinite loop to get IR readings continuously
    while (1)
    {
        // get IR sensor reading
        int ir_reading = read_ir_sensor(ir_sensor);

        // constrain ir reading between 0 and 4095
        ir_reading = bound(ir_reading, 0, 4095);
        // map readings from (0, 4095) to (0, 1000)
        ir_reading = map(ir_reading, 0, 4095, 0, 1000);
        
        // check if the object is near based on the threshold
        if (ir_reading >= OBJECT_DETECTION_THRESHOLD)
        {
            ESP_LOGI(TAG, "Object is near");
        }
        else
        {
            ESP_LOGI(TAG, "Object is not near");
        }

        // delay of 10 ms after each log
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}