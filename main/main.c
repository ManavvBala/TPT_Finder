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

// A demo task to transition through RGB colors on esp32-C6 board (Waveshare)
#include "LEDColorTask.h"
#include "LCD_setup.h"

/* Config for i2c pinout
 *      (one i2c setup for all
 *        sensors )
 */
#define I2C_SENSORS_SCL_IO 0    //   (Embedded Team 9)
#define I2C_SENSORS_SDA_IO 1    //   (Embedded Team 8)
#define I2C_SENSORS_Peripheral_Port     I2C_NUM_0



#define MHz_6                6000000
#define INTERNAL_CLOCK_FREQ 16000000

// AD5933 Impedance Measurement Setups
#define START_FREQ   30000
#define NUM_INCR         3
#define FREQ_INCR     1000

#define CALIBRATION_NUM_INCR 3

#define BUFFER_SIZE 1024

//   Params for FreeRTOS tasks created
#define DEFAULT_STACK     4096
#define TASK_PRIO_3         3
#define TASK_PRIO_2         2

/****************************************
 *
 *      Configure for hardware Setups
 *
 */

#define RUN_BioZ         false
#define RUN_rgbLED       true
#define RUN_HELLO_WORLD  false
#define RUN_LCD_TASK     true

// Configure Logging prefix:
static const char* TAG = "app_main";
static const char* tag = "app_main - debug w Claude";


double gain_factor;
//double system_phase;
double gain_factor_range[CALIBRATION_NUM_INCR];
double system_phase_range[CALIBRATION_NUM_INCR];
// int16_t real[NUM_INCR];
// int16_t imag[NUM_INCR];

void HandleSerialInput();

/**
 * @brief i2c bus configuration for all TPT_finder i2c sensors.
 * One "port" (i.e. the I2C peripheral (not LP I2C))
 *
 *    Select the right sensor using i2c address
 *
 */
i2c_master_bus_config_t i2c_master_config_BioZ = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port =   I2C_SENSORS_Peripheral_Port,
    .scl_io_num = I2C_SENSORS_SCL_IO,
    .sda_io_num = I2C_SENSORS_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = 1
};


/*
 * @brief tiny wrapper function to clean up FreeRTOS vTaskDelay for easier
 * typing and reading
 * @author Blake Hannaford
 */
void delayMS(int ms) {
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
        ESP_LOGI(TAG, "arr contents: [%d] = %f", i, arr[i]);
        //printf("arr contents: [%d] = %x\n", i, arr[i]);
    }
}

/**
 * @brief Initialize the AD5933 Impedance measurement chip
 */
void chip_init_AD5933(i2c_master_bus_handle_t bhand)
{   ESP_LOGI(TAG, "Starting AD5933 Chip Initialization");
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

    ESP_LOGI(TAG, "Starting AD5933 Impedance Calibration");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // start frequency sweep
    AD5933_start_freq_sweep(calib_real, calib_imag);
    // calculate gain_factor (a global) based on first point recorded
    gain_factor = gain_factor_calibration(330, calc_magnitude(calib_real[0], calib_imag[0]));
    // print the gain_factor
    ESP_LOGI("        gain factor", "gainfactor: %f", gain_factor);

    // system phase calibration (system_phase_range, global)
    system_phase_calibration(system_phase_range, calib_real, calib_imag, CALIBRATION_NUM_INCR);
    ESP_LOGI(TAG, "        system phases listed below");
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
    ESP_LOGI(TAG, "re-initializing AD5933");
    AD5933_init_settings(START_FREQ, INTERNAL_CLOCK_FREQ, FREQ_INCR, NUM_INCR, AD5933_RANGE_2000mVpp, AD5933_PGA_1, 25);
    vTaskDelay(1000 / portTICK_PERIOD_MS);


    int16_t real_arr[NUM_INCR];
    int16_t imag_arr[NUM_INCR];
     while (1) {
        AD5933_start_freq_sweep(real_arr, imag_arr);
        for (int i = 0; i < NUM_INCR; i ++){
            ESP_LOGI(TAG, "Uncompensated real: %d, imag: %d", real_arr[i], imag_arr[i]);
            double impedance = AD5933_calculate_impedance(gain_factor, real_arr[i], imag_arr[i]);
            ESP_LOGI(TAG, "impedance magnitude: %f", impedance);

            // phase calculations
            bool phase_error;
            double phase = arctan_phase_angle(real_arr[i], imag_arr[i], &phase_error);
            double comp_real, comp_imag;
            compensated_real_and_imag(impedance, phase, system_phase_range[i], &comp_real, &comp_imag);

            ESP_LOGI(TAG, "Calculated phase: %f", phase);
            ESP_LOGI(TAG, "System phase compensated real: %f, imag: %f", comp_real, comp_imag);
        }
        // ESP_LOGI(TAG,  "Pausing");
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
    // ESP_LOGI(TAG, " Pausing for user monitor connection: ");
    // delayMS(3000);
    printf("starting up\n");

    /*
     *
     * create/launch the FreeRTOS tasks
     *
     */

    if (RUN_LCD_TASK) {
        // init master bus (LCD Version)    ** this is done by the HD44780 code
        // i2c_master_bus_handle_t bh_LCD;
        // ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config_LCD, &bh_LCD));

        ESP_LOGI(TAG, " Configuring / initing LCD display");
        LCD_init(LCD_ADDR, I2C_SENSORS_SDA_IO, I2C_SENSORS_SCL_IO, LCD_COLS, LCD_ROWS);


        delayMS(500);

        xTaskCreate(&LCD_DemoTask, "LCD Task", 2048, NULL, 5, NULL);
        }



    if (RUN_HELLO_WORLD){
        xTaskCreatePinnedToCore(hello_task, "Hello World Task",
                                DEFAULT_STACK,
                                NULL,
                                TASK_PRIO_3,
                                NULL,
                                tskNO_AFFINITY);
    }

    if (RUN_rgbLED){
        ESP_LOGI(TAG," Configuring RGB LED interface");
        configure_led();
        delayMS(500);
        // create/launch color fade rgbLED task
        xTaskCreatePinnedToCore(LED_task, "rgbLED color fade Task",
                                DEFAULT_STACK,
                                NULL,
                                TASK_PRIO_3,
                                NULL,
                                tskNO_AFFINITY);
    }

    if (RUN_BioZ) {
        // init master bus (BioZ Version)
        i2c_master_bus_handle_t bus_handle;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config_BioZ, &bus_handle));

        ESP_LOGI(TAG,"     Starting Impedance (AD5933) chip init");
        chip_init_AD5933(bus_handle);
        delayMS(2000);


        ESP_LOGI(TAG,"     Starting chip calibrate");
        chip_calibrate_AD5933();
        delayMS(1000);

        xTaskCreatePinnedToCore(impedance_task, "Impedance Task",
                                DEFAULT_STACK,
                                NULL,
                                TASK_PRIO_2,
                                NULL,
                                tskNO_AFFINITY);
    }

    //

}


