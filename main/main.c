#include <stdio.h>                 // Standard input/output library for general-purpose functions like printf.
#include "sdkconfig.h"             // Configuration-specific macros and settings for the ESP-IDF SDK.
#include "freertos/FreeRTOS.h"     // FreeRTOS library for task management and real-time operating system features.
#include "freertos/task.h"         // Provides functions to manage and manipulate tasks in FreeRTOS.
#include "esp_log.h"               // Logging library for ESP-IDF, used for debugging and informational messages.
#include "driver/i2c_master.h"     // I2C master driver for ESP32, used to communicate with the AD5933.
#include "driver/i2c_types.h"      // Definitions for I2C types used by the driver.
#include "AD5933.h"                // Header file for the AD5933, defining constants, macros, and function prototypes.

// Define GPIO pins for the I2C master clock (SCL) and data (SDA) lines.
// These pins are connected to the AD5933 for communication.
#define I2C_MASTER_SCL_IO 9  // GPIO number for the I2C SCL (clock) line.
#define I2C_MASTER_SDA_IO 8  // GPIO number for the I2C SDA (data) line.

// Define clock frequency constants for testing and configuration purposes.
// MHz_6 is a test value, while INTERNAL_CLOCK_FREQ corresponds to the AD5933's internal clock frequency.
#define MHz_6 6000000           // Test value: 6 MHz.
#define INTERNAL_CLOCK_FREQ 16000000  // AD5933 internal clock frequency: 16 MHz.

// I2C master bus configuration structure.
// This structure specifies the settings used to initialize the I2C bus.
i2c_master_bus_config_t i2c_master_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,          // Use the default clock source for the I2C peripheral.
    .i2c_port = I2C_NUM_0,                      // Use I2C port 0 on the ESP32.
    .scl_io_num = I2C_MASTER_SCL_IO,           // Assign the GPIO pin defined for SCL.
    .sda_io_num = I2C_MASTER_SDA_IO,           // Assign the GPIO pin defined for SDA.
    .glitch_ignore_cnt = 7,                    // Number of clock glitches to ignore for stability.
    .flags.enable_internal_pullup = true,      // Enable internal pull-up resistors for the SCL and SDA lines.
};

// Number of frequency increments for the AD5933 sweep.
// This value is used to test the configuration of frequency sweep parameters.
int num_incr = 3;

// Function to perform basic tests of AD5933 register read and write operations.
// This function validates single register access and block read/write operations.
void AD5933_basic_test() {

    // Step 1: Test writing to and reading from a single register.
    // Write the value 0x11 to the starting frequency register (AD5933_REG_FREQ_START).
    AD5933_set_reg_value(AD5933_REG_FREQ_START, 0x11);

    // Set the pointer register to point to the starting frequency register for reading.
    AD5933_set_ptr_reg(AD5933_REG_FREQ_START);

    // Read back the value from the register.
    uint8_t single_reg = 0;
    AD5933_read_reg(&single_reg);

    // Validate the read value.
    if (single_reg == 0x11) 
        ESP_LOGI("test", "1st test passed");  // Log success if the value matches.
    else 
        ESP_LOGI("test", "1st test FAILED!");  // Log failure if the value does not match.

    // Step 2: Test writing to and reading from a block of registers.
    // Write a 2-byte block to the increment number registers (AD5933_REG_INC_NUM).
    uint8_t test_data[2] = {0x01, 0x01};
    AD5933_write_block(AD5933_REG_INC_NUM, test_data, 2);

    // Set the pointer register to the increment number register for reading.
    AD5933_set_ptr_reg(AD5933_REG_INC_NUM);
    uint8_t test_read[2];  // Buffer to store the read data.

    // Read back the 2-byte block from the register.
    AD5933_read_reg_block(test_read, 2);

    // Validate the read values.
    if (test_read[0] == 0x01 && test_read[1] == 0x01) 
        ESP_LOGI("test", "2nd test passed");  // Log success if both bytes match.
    else 
        ESP_LOGI("test", "2nd test FAILED!");  // Log failure if any byte does not match.
}

// Utility function to print the contents of an unsigned 8-bit integer array.
// Parameters:
// arr - Pointer to the array to be printed.
// n - Number of elements in the array.
void print_uarr(uint8_t* arr, uint8_t n) {
    // Loop through the array and log each element with its index and value.
    for (int i = 0; i < n; i++) {
        ESP_LOGI("arr contents: ", "[%d] = %x", i, arr[i]);
    }
}

// Utility function to print the contents of a signed 16-bit integer array.
// Parameters:
// arr - Pointer to the array to be printed.
// n - Number of elements in the array.
void print_arr(signed short* arr, uint8_t n) {
    // Loop through the array and log each element with its index and value.
    for (int i = 0; i < n; i++) {
        ESP_LOGI("arr contents: ", "[%d] = %d", i, arr[i]);
    }
}
// TESTING function for more important functionality
//  - init settings
//  - start freq sweep
void app_main(void)
{
    ESP_LOGI("log", "starting up");
    // init master bus
    i2c_master_bus_handle_t bus_handle;

    //ESP_ERROR_CHECK((&i2c_master_config, bus_handle));
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &bus_handle));

    // init AD5933

    AD5933_init_i2c_device(bus_handle);

    // reset
    AD5933_set_reg_value(AD5933_REG_CONTROL_LB, 0x18);


    AD5933_init_settings(30000, INTERNAL_CLOCK_FREQ, 1000, num_incr, AD5933_RANGE_2000mVpp, AD5933_PGA_1, 25);
    
    // check to make sure theyre set
    // uint8_t settings[12];
    // AD5933_set_ptr_reg(AD5933_REG_CONTROL_HB);
    // AD5933_read_reg_block(settings, 12);
    // ESP_LOGI("settings", "\n");
    // print_uarr(settings, 12);

    signed short real_arr[num_incr];
    signed short imag_arr[num_incr];


    AD5933_start_freq_sweep(real_arr, imag_arr);
    double gain_factor = gain_factor_calibration(330, calc_magnitude(real_arr[0], imag_arr[0]));
    while (1) {
        AD5933_start_freq_sweep(real_arr, imag_arr);

        // print_arr(real_arr, num_incr);
        // print_arr(imag_arr, num_incr);

        // do gain factor calculation with first point:
        

        for (int i = 0; i < num_incr; i ++){
            ESP_LOGI("log", "real: %d, imag: %d", real_arr[i], imag_arr[i]);
            double impedance = AD5933_calculate_impedance(gain_factor, real_arr[i], imag_arr[i]);
            ESP_LOGI("log", "impedance: %f", impedance);
        }
        ESP_LOGI("log",  "Pausing");
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    // logic analyzer test code
    // while (1) {
    //     AD5933_set_reg_value(0x80, 0xAA);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    //AD5933_init_settings(10,10,10,10);

}

// TODO (separate drivers):
//  - status LEDS
//  - check for good contact
