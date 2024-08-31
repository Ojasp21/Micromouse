/**
 * @file vl53l0x.h
 * @author JanG175
 * @brief ESP-IDF component for VL53L0X ToF distance sensor
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

// #define VL53L0X_I2C_INIT        1 // uncomment to initialize I2C driver

#define VL53L0X_MAX_I2C_FREQ    400000
#define VL53L0X_I2C_ADDR        0x29

typedef struct vl53l0x_conf_t
{
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
    gpio_num_t xshut_pin;
    gpio_num_t gpio1_pin;
}
vl53l0x_conf_t;


void vl53l0x_init(vl53l0x_conf_t vl53l0x_conf);

void vl53l0x_deinit(vl53l0x_conf_t vl53l0x_conf);

void vl53l0x_hardware_reset(vl53l0x_conf_t vl53l0x_conf);

void vl53l0x_software_reset();

void vl53l0x_set_timing_budget(uint32_t timing_budget_ms);

bool vl53l0x_read(vl53l0x_conf_t vl53l0x_conf, uint16_t* p_range_mm);

bool vl53l0x_read_single_with_polling(uint16_t* p_range_mm);

bool vl53l0x_read_single_with_delay(uint16_t* p_range_mm);

bool vl53l0x_read_single_with_interrupt(vl53l0x_conf_t vl53l0x_conf, uint16_t* p_range_mm);

void vl53l0x_set_device_address(uint8_t new_address);
