#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "AD5933.h"

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8

#define MHz_6 6000000
#define INTERNAL_CLOCK_FREQ 16000000

i2c_master_bus_config_t i2c_master_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

int num_incr = 3;

// TESTING function for read/write + block read/write
void AD5933_basic_test() {

    // set + read single register
    AD5933_set_reg_value(AD5933_REG_FREQ_START, 0x11);

    AD5933_set_ptr_reg(AD5933_REG_FREQ_START);

    uint8_t single_reg = 0;
    AD5933_read_reg(&single_reg);

    if (single_reg == 0x11) ESP_LOGI("test", "1st test passed");
    else ESP_LOGI("test", "1st test FAILED!");

    // set + read block register
    uint8_t test_data[2] = {0x01,0x01};
    AD5933_write_block(AD5933_REG_INC_NUM, test_data, 2);

    AD5933_set_ptr_reg(AD5933_REG_INC_NUM);
    uint8_t test_read[2];

    AD5933_read_reg_block(test_read, 2);

    if (test_read[0] == 0x01 && test_read[1] == 0x01) ESP_LOGI("test", "2nd test passed");
    else ESP_LOGI("test", "2nd test FAILED!");
}

void print_uarr(uint8_t* arr, uint8_t n) {
    for (int i = 0; i < n; i++) {
        ESP_LOGI("arr contents: ", "[%d] = %x", i, arr[i]);
    }
}

void print_arr(signed short* arr, uint8_t n) {
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
