/**
 * @file vl53l0x.c
 * @author JanG175
 * @brief ESP-IDF component for VL53L0X ToF distance sensor
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "vl53l0x.h"

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t xSemaphore;

static const char* TAG = "vl53l0x";
static VL53L0X_Dev_t vl53l0x_dev;
static uint32_t vl53l0x_timing_budget_ms;


/**
 * @brief print the last error message from the API
 * 
 * @param status API error code
 * @param function API function name 
*/
static void print_pal_error(VL53L0X_Error status, const char* function)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(status, buf);
    ESP_LOGE(TAG, "%s API status: %i : %s\n", function, status, buf);
}


/**
 * @brief GPIO1 interrupt handler
 * 
 * @param arg interrupt handler arguments
*/
static void IRAM_ATTR gpio1_isr_handler(void* arg)
{
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}


/**
 * @brief initialize the VL53L0X sensor
 * 
 * @param vl53l0x_conf VL53L0X configuration struct
*/
void vl53l0x_init(vl53l0x_conf_t vl53l0x_conf)
{
    vSemaphoreCreateBinary(xSemaphore);
    uint32_t freq = VL53L0X_MAX_I2C_FREQ;

    // init xshut pin
    if (vl53l0x_conf.xshut_pin != -1)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << vl53l0x_conf.xshut_pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        gpio_set_level(vl53l0x_conf.xshut_pin, 1);
    }

    // init gpio1 pin
    if (vl53l0x_conf.gpio1_pin != -1)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << vl53l0x_conf.gpio1_pin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_POSEDGE
        };
        gpio_config(&io_conf);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(vl53l0x_conf.gpio1_pin, gpio1_isr_handler, NULL);
    }

    // check i2c frequency
    if (vl53l0x_conf.i2c_freq == 0)
        ESP_LOGW(TAG, "I2C frequency is 0, using default frequency instead");
    else if (vl53l0x_conf.i2c_freq <= VL53L0X_MAX_I2C_FREQ)
        freq = vl53l0x_conf.i2c_freq;
    else
        ESP_LOGW(TAG, "I2C frequency is too high, using max frequency instead");

#ifdef VL53L0X_I2C_INIT
    // init i2c connection
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = vl53l0x_conf.sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = vl53l0x_conf.scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(vl53l0x_conf.i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(vl53l0x_conf.i2c_port, conf.mode, 0, 0, 0));
#endif

    // api init
    portENTER_CRITICAL(&spinlock);
    vl53l0x_dev.i2c_address = VL53L0X_I2C_ADDR;
    vl53l0x_dev.i2c_port = vl53l0x_conf.i2c_port;
    vl53l0x_dev.i2c_freq = freq;
    portEXIT_CRITICAL(&spinlock);

    if (vl53l0x_conf.xshut_pin != -1)
        vl53l0x_hardware_reset(vl53l0x_conf);
    else
        vl53l0x_software_reset();

    // device init
    VL53L0X_Error status = VL53L0X_DataInit(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_DataInit");

    status = VL53L0X_StaticInit(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_StaticInit");

    // SPADs calibration
    uint8_t is_aperture_spads;
    uint32_t ref_spad_count;
    status = VL53L0X_PerformRefSpadManagement(&vl53l0x_dev, &ref_spad_count, &is_aperture_spads);
    ESP_LOGI(TAG, "ref_spad_count = %" PRIu32 ", is_aperture_spads = %" PRIu8 "\n", ref_spad_count, is_aperture_spads);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_PerformRefSpadManagement");

    // temperature calibration
    uint8_t vhv_settings;
    uint8_t phase_cal;
    status = VL53L0X_PerformRefCalibration(&vl53l0x_dev, &vhv_settings, &phase_cal);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_PerformRefCalibration");

    // setup in single ranging mode
    status = VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
        return print_pal_error(status, "VL53L0X_SetDeviceMode");

    VL53L0X_SetGpioConfig(&vl53l0x_dev, 0, VL53L0X_DEVICEMODE_SINGLE_RANGING, VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY, VL53L0X_INTERRUPTPOLARITY_LOW);

    vl53l0x_set_timing_budget(33000);
}


/**
 * @brief deinitialize the VL53L0X sensor
 * 
 * @param vl53l0x_conf VL53L0X configuration struct
*/
void vl53l0x_deinit(vl53l0x_conf_t vl53l0x_conf)
{
    ESP_ERROR_CHECK(i2c_driver_delete(vl53l0x_conf.i2c_port));
}


/**
 * @brief hardware reset the VL53L0X sensor
 * 
 * @param vl53l0x_conf VL53L0X configuration struct
*/
void vl53l0x_hardware_reset(vl53l0x_conf_t vl53l0x_conf)
{
    if (vl53l0x_conf.xshut_pin == -1)
    {
        ESP_LOGE(TAG, "xshut pin is not configured, hardware reset is not available");
        return;
    }

    gpio_set_level(vl53l0x_conf.xshut_pin, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(vl53l0x_conf.xshut_pin, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}


/**
 * @brief software reset the VL53L0X sensor
 * 
*/
void vl53l0x_software_reset()
{
    VL53L0X_Error status = VL53L0X_ResetDevice(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
        print_pal_error(status, "VL53L0X_ResetDevice");
}


/**
 * @brief set the timing budget
 * 
 * @param timing_budget_ms timing budget in ms
*/
void vl53l0x_set_timing_budget(uint32_t timing_budget_ms)
{
    VL53L0X_Error status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&vl53l0x_dev, timing_budget_ms);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_SetMeasurementTimingBudgetMicroSeconds");
        return;
    }

    portENTER_CRITICAL(&spinlock);
    vl53l0x_timing_budget_ms = timing_budget_ms;
    portEXIT_CRITICAL(&spinlock);
}


/**
 * @brief read the distance from the VL53L0X sensor
 * 
 * @param vl53l0x_conf VL53L0X configuration struct
 * @param p_range_mm pointer to the meassured distance value in mm
 * 
 * @return true - successful meassurment, false - failed meassurment
*/
bool vl53l0x_read(vl53l0x_conf_t vl53l0x_conf, uint16_t* p_range_mm)
{
    if (vl53l0x_conf.gpio1_pin != -1)
        return vl53l0x_read_single_with_interrupt(vl53l0x_conf, p_range_mm);

    return vl53l0x_read_single_with_polling(p_range_mm);
}


/**
 * @brief read the distance from the VL53L0X sensor with polling
 * 
 * @param p_range_mm pointer to the meassured distance value in mm
 * 
 * @return true - successful meassurment, false - failed meassurment
*/
bool vl53l0x_read_single_with_polling(uint16_t* p_range_mm)
{
    VL53L0X_RangingMeasurementData_t measurement_data;
    VL53L0X_Error status = VL53L0X_PerformSingleRangingMeasurement(&vl53l0x_dev, &measurement_data);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_PerformSingleRangingMeasurement");
        return false;
    }

    *p_range_mm = measurement_data.RangeMilliMeter;
    if (measurement_data.RangeStatus != 0)
        return false;

    return true;
}


/**
 * @brief read the distance from the VL53L0X sensor with delay
 * 
 * @param p_range_mm pointer to the meassured distance value in mm
 * 
 * @return true - successful meassurment, false - failed meassurment
*/
bool vl53l0x_read_single_with_delay(uint16_t* p_range_mm)
{
    TickType_t xTicksToWait = vl53l0x_timing_budget_ms / 1000 / portTICK_PERIOD_MS;
    VL53L0X_Error status;

    status = VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_SetDeviceMode");
        return false;
    }

    // start measurement
    status = VL53L0X_StartMeasurement(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_StartMeasurement");
        return false;
    }

    // wait
    vTaskDelay(xTicksToWait);

    // get data
    VL53L0X_RangingMeasurementData_t measurement_data;
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &measurement_data);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_GetRangingMeasurementData");
        return false;
    }

    *p_range_mm = measurement_data.RangeMilliMeter;
    if (measurement_data.RangeStatus != 0)
        return false;

    // clear interrupt
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE)
        return false;

    return true;
}


/**
 * @brief read the distance from the VL53L0X sensor with interrupt
 * 
 * @param vl53l0x_conf VL53L0X configuration struct
 * @param p_range_mm pointer to the meassured distance value in mm
 * 
 * @return true - successful meassurment, false - failed meassurment
*/
bool vl53l0x_read_single_with_interrupt(vl53l0x_conf_t vl53l0x_conf, uint16_t* p_range_mm)
{
    if (vl53l0x_conf.gpio1_pin == -1)
        return false;

    VL53L0X_Error status;

    // set mode
    status = VL53L0X_SetDeviceMode(&vl53l0x_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_SetDeviceMode");
        return false;
    }

    // clear semphr
    xSemaphoreTake(xSemaphore, 0);

    // start measurement
    status = VL53L0X_StartMeasurement(&vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_StartMeasurement");
        return false;
    }

    // wait for interrupt
    xSemaphoreTake(xSemaphore, 1000 / portMAX_DELAY);

    // get data
    VL53L0X_RangingMeasurementData_t measurement_data;
    status = VL53L0X_GetRangingMeasurementData(&vl53l0x_dev, &measurement_data);
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status, "VL53L0X_GetRangingMeasurementData");
        return false;
    }

    *p_range_mm = measurement_data.RangeMilliMeter;
    if (measurement_data.RangeStatus != 0)
        return false;

    // clear interrupt
    VL53L0X_ClearInterruptMask(&vl53l0x_dev, 0);
    if (status != VL53L0X_ERROR_NONE)
        return false;

    return true;
}


/**
 * @brief set the device address
 * 
 * @param new_address new device address
*/
void vl53l0x_set_device_address(uint8_t new_address)
{
    // VL53L0X_SetDeviceAddress expects the address to be left-aligned
    VL53L0X_Error status = VL53L0X_SetDeviceAddress(&vl53l0x_dev, new_address << 1);
    if (status != VL53L0X_ERROR_NONE)
        print_pal_error(status, "VL53L0X_PerformSingleRangingMeasurement");

    portENTER_CRITICAL(&spinlock);
    vl53l0x_dev.i2c_address = new_address;
    portEXIT_CRITICAL(&spinlock);
}
