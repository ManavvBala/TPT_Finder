#ifndef AD5933_H
#define AD5933_H

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

// Define the I2C address for the AD5933 device
// This is the 7-bit address used to communicate with the AD5933
#define AD5933_ADDRESS ((uint8_t)0x0D)

// Voltage output range options for the AD5933
// These values represent the peak-to-peak excitation voltages the device can output
#define AD5933_RANGE_2000mVpp ((uint8_t)0x0)  // 2.0V peak-to-peak output range
#define AD5933_RANGE_200mVpp  ((uint8_t)0x1)  // 200mV peak-to-peak output range
#define AD5933_RANGE_400mVpp  ((uint8_t)0x2)  // 400mV peak-to-peak output range
#define AD5933_RANGE_1000mVpp ((uint8_t)0x3)  // 1.0V peak-to-peak output range

// PGA (Programmable Gain Amplifier) settings for the AD5933
// Used to adjust the gain of the input signal for better accuracy
#define AD5933_PGA_1 ((uint8_t)0x1)  // PGA gain of 1
#define AD5933_PGA_5 ((uint8_t)0x0)  // PGA gain of 5

// Register addresses for the AD5933
// These addresses correspond to different configuration and data registers
#define AD5933_REG_CONTROL_HB ((uint8_t)0x80)  // High byte of the control register
#define AD5933_REG_CONTROL_LB ((uint8_t)0x81)  // Low byte of the control register
#define AD5933_REG_FREQ_START ((uint8_t)0x82)  // Starting frequency register
#define AD5933_REG_FREQ_INC   ((uint8_t)0x85)  // Frequency increment register
#define AD5933_REG_INC_NUM    ((uint8_t)0x88)  // Number of frequency increments register
#define AD5933_REG_SETTLING_CYCLES ((uint8_t)0x8A)  // Number of settling time cycles register
#define AD5933_REG_STATUS     ((uint8_t)0x8F)  // Status register
#define AD5933_REG_TEMP_DATA  ((uint8_t)0x92)  // Temperature data register
#define AD5933_REG_REAL_DATA  ((uint8_t)0x94)  // Real part of impedance data
#define AD5933_REG_REAL_DATA_2 ((uint8_t)0x95) // Low byte of real part data
#define AD5933_REG_IMAG_DATA  ((uint8_t)0x96)  // Imaginary part of impedance data
#define AD5933_REG_IMAG_DATA_2 ((uint8_t)0x97) // Low byte of imaginary part data

// Block command options for the AD5933
// Used for efficient data transfer via I2C
#define AD5933_BLOCK_WRITE ((uint8_t)0xA0)  // Block write command
#define AD5933_BLOCK_READ  ((uint8_t)0xA1)  // Block read command
#define AD5933_ADDR_POINTER ((uint8_t)0xB0) // Pointer register command

// Control register function options
// These options control various operational modes of the device
#define AD5933_FUNCTION_INIT_START_FREQ ((uint8_t)0x1)  // Initialize with start frequency
#define AD5933_FUNCTION_START_SWEEP     ((uint8_t)0x2)  // Start frequency sweep
#define AD5933_FUNCTION_INC_FREQ        ((uint8_t)0x3)  // Increment frequency
#define AD5933_FUNCTION_REPEAT_FREQ     ((uint8_t)0x4)  // Repeat current frequency
#define AD5933_FUNCTION_MEASURE_TEMP    ((uint8_t)0x9)  // Measure temperature
#define AD5933_FUNCTION_POWER_DOWN      ((uint8_t)0xA)  // Enter power-down mode
#define AD5933_FUNCTION_STANDBY         ((uint8_t)0xB)  // Enter standby mode

// Control register bit manipulation macros
// Used to set specific control bits in the control register
#define AD5933_CONTROL_FUNCTION(x) ((x) << 4)  // Set the function bits
#define AD5933_CONTROL_RANGE(x)    ((x) << 1)  // Set the output range bits

// Status register bit masks
// Used to check the status of various device operations
#define AD5933_STAT_TEMP_VALID (0x1 << 0)  // Temperature measurement valid
#define AD5933_STAT_DATA_VALID (0x1 << 1)  // Data measurement valid
#define AD5933_STAT_SWEEP_DONE (0x1 << 2)  // Frequency sweep complete

// Device clock frequency placeholder
#define CLOCK_FREQ 

// Extern declarations for device configuration and handle
extern i2c_device_config_t dev_cfg;
extern i2c_master_dev_handle_t AD5933_dev_handle;

// Function prototypes for AD5933 operations
// Initialize the I2C device with the specified bus handle
esp_err_t AD5933_init_i2c_device(i2c_master_bus_handle_t bus_handle);

// Set a single register value
void AD5933_set_reg_value(uint8_t reg_addr, uint8_t reg_val);

// Set the pointer register to a specific address
void AD5933_set_ptr_reg(uint8_t reg_addr);

// Write a block of data to a specified register
void AD5933_set_reg_block(uint8_t reg_addr, uint8_t* data, uint8_t size);

// Write a block of data to the device
void AD5933_write_block(uint8_t reg_addr, uint8_t* data, uint8_t size);

// Read a single register value
void AD5933_read_reg(uint8_t* read_buff);

// Read a block of register values
void AD5933_read_reg_block(uint8_t* read_buff, uint8_t num_bytes);

// Start a frequency sweep and store real and imaginary data in arrays
void AD5933_start_freq_sweep(signed short* real_arr, signed short* imag_arr);

// Initialize device settings, including frequency and gain configurations
void AD5933_init_settings(double start_freq, double clock_freq, double freq_incr, uint16_t num_incr, uint8_t range, uint8_t PGA, uint16_t num_cycles);

// Calculate impedance using a gain factor and measured data
double AD5933_calculate_impedance(double gainFactor, signed short real, signed short imag);

// Perform gain factor calibration using a known impedance
double gain_factor_calibration(double Z_calibration, double magnitude);

// Calculate the magnitude of an impedance from real and imaginary parts
double calc_magnitude(int16_t real, int16_t imag);

#endif
