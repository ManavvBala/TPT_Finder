/**
 * @file
 * @author Manav B., Eun Be Cha, Blake Hannaford
 * @version 0.1
 *
 * @section DESCRIPTION
 *
 * This app will control sensors for the TPT-Finder surgical
 * sensing pen.   IT is developed for the ESP32 series of processors.
 * Flexible configurations are provided to drive one sensor only,
 * a subset of sensors, or (eventually) perform the full 3-sensor
 * application.
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/usb_serial_jtag.h"
#include "AD5933.h"
#include "LogWriter.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8

#define MHz_6 6000000
#define INTERNAL_CLOCK_FREQ 16000000

// AD5933 Impedance Measurement Setups
#define START_FREQ 30000
#define NUM_INCR 3
#define FREQ_INCR 1000

#define CALIBRATION_NUM_INCR 3

#define BUFFER_SIZE 1024

//   Params for FreeRTOS tasks created
#define DEFAULT_STACK     4096
#define TASK_PRIO_3         3
#define TASK_PRIO_2         2


double gain_factor;
//double system_phase;
double gain_factor_range[CALIBRATION_NUM_INCR];
double system_phase_range[CALIBRATION_NUM_INCR];
// int16_t real[NUM_INCR];
// int16_t imag[NUM_INCR];

void HandleSerialInput();

/**
 * @brief Initial i2c bus configuration for TPT_finder
 */

i2c_master_bus_config_t i2c_master_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

/**
 * @brief tiny wrapper function to clean up FreeRTOS vTaskDelay for easier
 * typing and reading
 * @author Blake Hannaford
 */
void delayMS(int ms)  // just cleaner, easier to type
{
        vTaskDelay(ms/portTICK_PERIOD_MS);
}

/**
 * @brief print an array of uint8_t's
 */
void print_uarr(uint8_t* arr, uint8_t n) {
    for (int i = 0; i < n; i++) {
        //ESP_LOGI("arr contents: ", "[%d] = %x", i, arr[i]);
        printf("arr contents: [%d] = %x\n", i, arr[i]);
    }
}

/**
 * @brief print an array of signed short's
 */
void print_arr(signed short* arr, uint8_t n) {
    for (int i = 0; i < n; i++) {
        //ESP_LOGI("arr contents: ", "[%d] = %d", i, arr[i]);
        printf("arr contents: [%d] = %d\n", i, arr[i]);
    }
}


/**
 * @brief print an array of AD5933 frequencies (uint16_t)
 */
void init_freq_arr(uint8_t n, uint16_t start, uint16_t step, uint16_t* arr) {
    for (int i = 0; i < n; i++) {
        arr[i] = start + step * i;
    }
}

/**
 * @brief print an array of double precision floats
 */
void print_double_arr(double* arr, int n) {
    for (int i = 0; i < n; i++) {
        ESP_LOGI("log", "arr contents: [%d] = %f", i, arr[i]);
        //printf("arr contents: [%d] = %x\n", i, arr[i]);
    }
}

/**
 * @brief Initialize the AD5933 Impedance measurement chip
 */
void chip_init_AD5933(i2c_master_bus_handle_t bhand)
{   ESP_LOGI("log", "Starting AD5933 Chip Initialization");
    delayMS(500);

    // init AD5933
    ESP_ERROR_CHECK(AD5933_init_i2c_device(bhand));

    // reset
    AD5933_set_reg_value(AD5933_REG_CONTROL_LB, 0x18);

    AD5933_init_settings(START_FREQ, INTERNAL_CLOCK_FREQ, FREQ_INCR, CALIBRATION_NUM_INCR, AD5933_RANGE_2000mVpp, AD5933_PGA_1, 25) ;
}


/**
 * @brief Calibrate the AD5933 Impedance measurement chip
 */
void  chip_calibrate_AD5933()
{
    int16_t calib_real[CALIBRATION_NUM_INCR];
    int16_t calib_imag[CALIBRATION_NUM_INCR];

    ESP_LOGI("log", "Starting AD5933 Impedance Calibration");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // start frequency sweep
    AD5933_start_freq_sweep(calib_real, calib_imag);
    // calculate gain_factor (a global) based on first point recorded
    gain_factor = gain_factor_calibration(330, calc_magnitude(calib_real[0], calib_imag[0]));
    // print the gain_factor
    ESP_LOGI("        gain factor", "gainfactor: %f", gain_factor);

    // system phase calibration (system_phase_range, global)
    system_phase_calibration(system_phase_range, calib_real, calib_imag, CALIBRATION_NUM_INCR);
    ESP_LOGI("log", "        system phases listed below");
    print_double_arr(system_phase_range, CALIBRATION_NUM_INCR);
}


/**
 * @brief A basic "hello world" task.
 */
static void hello_task(void *arg)
{
    int i=0;
    while(1) {
        delayMS(5000);
        printf("\n\n\n");
        i++;
        printf("-----------------------------------  Hello world! (TPT-Finder Hello) (task rep: %d) \n", i);
        printf("\n\n\n");
    }
}

/**
 * @brief Take measurement with AD5933 Impedance measurement chip:
 * re-initialize once, then
 * loop forever collecting values and logging them.
 */
static void impedance_task(void *arg)
{

    // reinitialize after calibration(?)
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("log", "re-initializing AD5933");
    AD5933_init_settings(START_FREQ, INTERNAL_CLOCK_FREQ, FREQ_INCR, NUM_INCR, AD5933_RANGE_2000mVpp, AD5933_PGA_1, 25);
    vTaskDelay(1000 / portTICK_PERIOD_MS);


    int16_t real_arr[NUM_INCR];
    int16_t imag_arr[NUM_INCR];
     while (1) {
        AD5933_start_freq_sweep(real_arr, imag_arr);
        for (int i = 0; i < NUM_INCR; i ++){
            ESP_LOGI("log", "Uncompensated real: %d, imag: %d", real_arr[i], imag_arr[i]);
            double impedance = AD5933_calculate_impedance(gain_factor, real_arr[i], imag_arr[i]);
            ESP_LOGI("log", "impedance magnitude: %f", impedance);

            // phase calculations
            bool phase_error;
            double phase = arctan_phase_angle(real_arr[i], imag_arr[i], &phase_error);
            double comp_real, comp_imag;
            compensated_real_and_imag(impedance, phase, system_phase_range[i], &comp_real, &comp_imag);

            ESP_LOGI("log", "Calculated phase: %f", phase);
            ESP_LOGI("log", "System phase compensated real: %f, imag: %f", comp_real, comp_imag);
        }
        // ESP_LOGI("log",  "Pausing");
        delayMS(100);
    }
}

// TESTING function for more important functionality
//  - init settings
//  - start freq sweep


/**
 * @brief  Entry point for system start
 */
void app_main(void)
{
    ESP_LOGI("log", " Pausing for user monitor connection: ");
    delayMS(3000);
    printf("starting up\n");
    // init master bus
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &bus_handle));

    ESP_LOGI("log","     Starting chip init");
    chip_init_AD5933(bus_handle);
    delayMS(2000);


    ESP_LOGI("log","     Starting chip calibrate");
    chip_calibrate_AD5933();
    delayMS(1000);

    // create/launch the FreeRTOS tasks
    xTaskCreatePinnedToCore(hello_task, "Hello World Task",
                            DEFAULT_STACK,
                            NULL,
                            TASK_PRIO_3,
                            NULL,
                            tskNO_AFFINITY);

    xTaskCreatePinnedToCore(impedance_task, "Impedance Task",
                            DEFAULT_STACK,
                            NULL,
                            TASK_PRIO_2,
                            NULL,
                            tskNO_AFFINITY);
}


