#include "AD5933.h"  # Includes the header file defining constants, macros, and function prototypes for AD5933.
#include "driver/i2c_master.h"  # Includes the I2C master driver for communication with the AD5933.
#include "driver/i2c_types.h"   # Includes definitions of I2C types used in the ESP-IDF framework.
#include "esp_log.h"  # ESP-IDF logging utility, used for debugging and displaying information.
#include "math.h"     # Standard math library for functions like sqrt().

# I2C device configuration structure for the AD5933 device.
# This structure specifies how the AD5933 is set up on the I2C bus.
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,  # Address length is 7 bits, as required by the AD5933 datasheet.
    .device_address = 0x0D,                # The I2C slave address for the AD5933, fixed in the hardware.
    .scl_speed_hz = 10000,                 # I2C clock frequency in Hz. A lower speed is used to ensure reliable communication.
    .scl_wait_us = 100,                    # Wait time in microseconds for clock line stabilization. Default value for stability.
    .flags.disable_ack_check = 0           # ACK checking is enabled to verify successful data transfers.
};

# Handle for the AD5933 device on the I2C bus.
# This handle is created when the device is added to the I2C bus and used in all subsequent operations.
i2c_master_dev_handle_t AD5933_dev_handle;

# Function to initialize the I2C device for AD5933 communication.
# This function registers the AD5933 device on the specified I2C bus, setting up the communication parameters.
# Parameters:
# bus_handle - The handle to the I2C bus where the AD5933 is connected.
# Returns:
# esp_err_t - Returns ESP_OK if the device was successfully initialized; otherwise, an error code.
esp_err_t AD5933_init_i2c_device(i2c_master_bus_handle_t bus_handle) {
    # The dev_cfg structure contains all the settings required for the AD5933 device.
    # These settings are passed to the I2C master bus driver to configure the device.
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, &AD5933_dev_handle);
}

# Function to set a single register value in the AD5933.
# This function writes a value to a specific register address in the AD5933.
# Registers control various device operations, such as starting a sweep or setting the output voltage range.
# Parameters:
# reg_addr - The address of the register to write to. The address is obtained from the datasheet register map.
# reg_val - The value to write to the register. This value configures the device behavior as per the datasheet.
void AD5933_set_reg_value(uint8_t reg_addr, uint8_t reg_val) {
    # A buffer is prepared containing the register address and the value to be written.
    # The first byte is the address, and the second byte is the value.
    uint8_t write_buff[2] = {reg_addr, reg_val};
    # The I2C transmission writes the two bytes to the AD5933 device.
    i2c_master_transmit(AD5933_dev_handle, write_buff, 2, -1);
}

# Function to set the pointer register of the AD5933.
# The pointer register determines which memory location subsequent read or write operations will target.
# For example, to read data from the real part register, the pointer must first be set to 0x94.
# Parameters:
# reg_addr - The address to set in the pointer register. This is used to target specific registers for I2C communication.
void AD5933_set_ptr_reg(uint8_t reg_addr) {
    # Prepare a command buffer with the pointer register address and the desired target address.
    uint8_t pointer_cmd[2] = {AD5933_ADDR_POINTER, reg_addr};
    # Transmit the command via I2C to set the pointer register.
    i2c_master_transmit(AD5933_dev_handle, pointer_cmd, 2, -1);
}

# Function to write a block of data to sequential registers in the AD5933.
# This function is typically used to configure multi-byte settings like the starting frequency or frequency increment.
# Parameters:
# reg_addr - The starting register address for the block write. This address is specified in the datasheet.
# data - Pointer to an array containing the data to write. Each byte corresponds to one register.
# size - Number of bytes in the data array. Determines how many registers will be written.
void AD5933_set_reg_block(uint8_t reg_addr, uint8_t* data, uint8_t size) {
    # Temporary buffer for transmitting each register's address and value.
    uint8_t write_buff[2] = {0, 0};

    # Loop through the data array, writing each byte to the corresponding register.
    for (int i = 0; i < size; i++) {
        # Calculate the current register address by adding the offset to the starting address.
        write_buff[0] = reg_addr + i;
        # Get the data byte from the array.
        write_buff[1] = data[i];
        # Write the current address and value pair to the AD5933 via I2C.
        i2c_master_transmit(AD5933_dev_handle, write_buff, 2, -1);
    }
}

# Function to write a block of data to the AD5933 using the block write command.
# The block write is a more efficient way to send multiple bytes at once, reducing the overhead of individual register writes.
# Parameters:
# reg_addr - The starting register address for the block write. This address is provided in the datasheet.
# data - Pointer to the data array containing the values to be written.
# size - Number of bytes in the data array.
void AD5933_write_block(uint8_t reg_addr, uint8_t* data, uint8_t size) {
    # Total write size includes the block command, size byte, and data bytes.
    uint8_t write_size = 2 + size;
    # Dynamically allocate memory for the write buffer.
    uint8_t* write_buff = calloc(write_size, sizeof(uint8_t));

    # Initialize the buffer with the block write command and the size of the data block.
    write_buff[0] = AD5933_BLOCK_WRITE;  # Block write command (defined in the header file).
    write_buff[1] = size;                # Number of data bytes to write.

    # Copy the data array into the buffer, starting at the third position.
    for (int i = 0; i < size; i++) {
        write_buff[i + 2] = data[i];
    }

    # Set the pointer register to the starting address for the block write.
    uint8_t pointer_cmd[2] = {AD5933_ADDR_POINTER, reg_addr};
    i2c_master_transmit(AD5933_dev_handle, pointer_cmd, 2, -1);

    # Transmit the block write command along with the data buffer.
    i2c_master_transmit(AD5933_dev_handle, write_buff, write_size, -1);

    # Free the dynamically allocated memory after the transmission.
    free(write_buff);
}

# Function to read a single register value from the AD5933.
# This function retrieves one byte of data from the current pointer register.
# Parameters:
# read_buff - Pointer to a buffer where the read data will be stored.
void AD5933_read_reg(uint8_t* read_buff) {
    # Perform an I2C receive operation to read one byte from the AD5933.
    i2c_master_receive(AD5933_dev_handle, read_buff, 1, -1);
}


# Function to read a block of data from sequential registers in the AD5933.
# This is typically used to retrieve multi-byte data such as real and imaginary components or calibration values.
# Parameters:
# read_buff - Pointer to the buffer where the read data will be stored.
# num_bytes - Number of bytes to read from the AD5933 registers.
void AD5933_read_reg_block(uint8_t* read_buff, uint8_t num_bytes) {
    # Prepare a command to request a block read from the AD5933.
    # The first byte is the block read command, and the second byte is the number of bytes to read.
    uint8_t write_buff[2] = {AD5933_BLOCK_READ, num_bytes};
    
    # Perform an I2C transaction that first writes the block read command
    # and then reads the specified number of bytes into the provided buffer.
    i2c_master_transmit_receive(AD5933_dev_handle, write_buff, 2, read_buff, num_bytes, -1);
}

# Function to initialize the AD5933 for a frequency sweep.
# Configures key parameters such as start frequency, frequency increment, number of increments,
# output range, programmable gain amplifier (PGA) gain, and settling time cycles.
# Parameters:
# start_freq - Starting frequency of the sweep in Hz.
# clock_freq - Master clock frequency supplied to the AD5933 in Hz.
# freq_incr - Frequency increment step size in Hz.
# num_incr - Number of frequency increments (1 to 511, per datasheet).
# range - Output voltage range selection (e.g., AD5933_RANGE_2000mVpp for 2.0V peak-to-peak).
# PGA - Programmable Gain Amplifier setting (e.g., AD5933_PGA_1 for x1 gain or AD5933_PGA_5 for x5 gain).
# num_cycles - Number of settling time cycles before taking measurements.
void AD5933_init_settings(double start_freq, double clock_freq, double freq_incr, uint16_t num_incr, uint8_t range, uint8_t PGA, uint16_t num_cycles) {
    # Convert the start frequency to a 24-bit code based on the AD5933 formula:
    # Code = (start_freq / (clock_freq / 4)) * 2^27
    uint8_t start_freq_bytes[3] = {0};
    uint32_t start_freq_code = (uint32_t)((start_freq / (clock_freq / 4)) * (1 << 27));

    # Split the 24-bit start frequency code into three 8-bit bytes.
    start_freq_bytes[0] = (start_freq_code >> 16) & 0xFF;  # Most significant byte.
    start_freq_bytes[1] = (start_freq_code >> 8) & 0xFF;   # Middle byte.
    start_freq_bytes[2] = start_freq_code & 0xFF;          # Least significant byte.

    # Write the start frequency to the AD5933's registers (0x82, 0x83, 0x84).
    AD5933_write_block(AD5933_REG_FREQ_START, start_freq_bytes, 3);

    # Convert the frequency increment to a 24-bit code using the same formula.
    uint8_t freq_incr_bytes[3] = {0};
    uint32_t freq_incr_code = (uint32_t)((freq_incr / (clock_freq / 4)) * (1 << 27));

    # Split the increment code into three bytes.
    freq_incr_bytes[0] = (freq_incr_code >> 16) & 0xFF;
    freq_incr_bytes[1] = (freq_incr_code >> 8) & 0xFF;
    freq_incr_bytes[2] = freq_incr_code & 0xFF;

    # Write the frequency increment to the AD5933's registers (0x85, 0x86, 0x87).
    AD5933_write_block(AD5933_REG_FREQ_INC, freq_incr_bytes, 3);

    # Write the number of increments as a 16-bit value to the registers (0x88, 0x89).
    uint8_t num_incr_bytes[2] = {0};
    num_incr_bytes[0] = (num_incr >> 8) & 0xFF;  # High byte.
    num_incr_bytes[1] = num_incr & 0xFF;         # Low byte.
    AD5933_write_block(AD5933_REG_INC_NUM, num_incr_bytes, 2);

    # Write the number of settling time cycles as a 16-bit value to the registers (0x8A, 0x8B).
    uint8_t num_cycles_bytes[2] = {0};
    num_cycles_bytes[0] = (num_cycles >> 8) & 0xFF;  # High byte.
    num_cycles_bytes[1] = num_cycles & 0xFF;         # Low byte.
    AD5933_write_block(AD5933_REG_SETTLING_CYCLES, num_cycles_bytes, 2);

    # Configure the control register for standby mode, output voltage range, and PGA gain.
    # The high byte of the control register is set first, including range and PGA bits.
    AD5933_set_reg_value(AD5933_REG_CONTROL_HB, AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_STANDBY) | AD5933_CONTROL_RANGE(range) | PGA);

    # The low byte of the control register is set to 0 for default behavior.
    AD5933_set_reg_value(AD5933_REG_CONTROL_LB, 0x00);
}

# Function to calculate the impedance magnitude from real and imaginary components.
# Parameters:
# real - Real part of the impedance data (16-bit signed integer from the AD5933).
# imag - Imaginary part of the impedance data (16-bit signed integer from the AD5933).
# Returns:
# The magnitude of the impedance, calculated as sqrt(real^2 + imag^2).
double calc_magnitude(int16_t real, int16_t imag) {
    # Use the Pythagorean theorem to compute the magnitude.
    return sqrt((double)((real * real) + (imag * imag)));
}

# Function to calculate the impedance value from the gain factor and measured data.
# Parameters:
# gainFactor - Calibration factor obtained from a known impedance.
# real - Real part of the impedance data.
# imag - Imaginary part of the impedance data.
# Returns:
# The calculated impedance in ohms.
double AD5933_calculate_impedance(double gainFactor, signed short real, signed short imag) {
    # Calculate the magnitude of the impedance from real and imaginary components.
    double magnitude = sqrt((double)(real * real) + (double)(imag * imag));

    # Impedance is the inverse of the product of magnitude and gain factor.
    return 1.0 / (magnitude * gainFactor);
}

# Function to perform a single-point calibration to determine the gain factor.
# The gain factor relates raw magnitude values to known impedance values.
# Parameters:
# Z_calibration - The known calibration impedance in ohms.
# magnitude - The magnitude of the raw DFT output for the calibration impedance.
# Returns:
# Gain factor, which is 1 / (Z_calibration * magnitude).
double gain_factor_calibration(double Z_calibration, double magnitude) {
    # Calculate the gain factor using the formula provided in the datasheet.
    return (1 / Z_calibration) / magnitude;
}
# Function to start a frequency sweep using the AD5933.
# It performs the sweep, collects real and imaginary data for each frequency step, and stores them in arrays.
# Parameters:
# real_arr - Array to store the real part of the impedance at each frequency step.
# imag_arr - Array to store the imaginary part of the impedance at each frequency step.
void AD5933_start_freq_sweep(signed short* real_arr, signed short* imag_arr) {
    uint8_t ctrl_hb_low;  # Stores the lower nibble of the control register high byte.
    uint8_t status = 0;   # Used to track the device status from the status register.
    int freq_index = 0;   # Index of the current frequency step during the sweep.

    signed short real;    # Temporary variable for storing the real part of the impedance.
    signed short imag;    # Temporary variable for storing the imaginary part of the impedance.

    # Step 1: Read the lower nibble of the control high byte (preserves the range and PGA settings).
    AD5933_set_ptr_reg(AD5933_REG_CONTROL_HB);
    AD5933_read_reg(&ctrl_hb_low);
    ctrl_hb_low &= 0x0F;  # Mask the lower 4 bits, keeping only the range and gain settings.

    # Step 2: Initialize the AD5933 to the starting frequency.
    # This prepares the device to begin the sweep from the defined start frequency.
    AD5933_set_reg_value(AD5933_REG_CONTROL_HB, AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_INIT_START_FREQ) | ctrl_hb_low);

    # Step 3: Set the AD5933 to start sweep mode.
    # This instructs the device to begin the frequency sweep as configured in the initialization.
    AD5933_set_reg_value(AD5933_REG_CONTROL_HB, AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_START_SWEEP) | ctrl_hb_low);

    # Step 4: Poll the status register to monitor the sweep progress.
    # Continue reading the status register until the sweep completes.
    AD5933_set_ptr_reg(AD5933_REG_STATUS);
    do {
        # Wait for valid data (DFT conversion complete) at the current frequency step.
        while ((status & AD5933_STAT_DATA_VALID) == 0) {
            AD5933_read_reg(&status);
        }

        # Step 5: Read the real part of the impedance data.
        uint8_t data[2] = {0, 0};
        AD5933_set_ptr_reg(AD5933_REG_REAL_DATA);  # Set pointer to the real data register.
        AD5933_read_reg_block(data, 2);            # Read 2 bytes (high and low byte of real data).
        real = (signed short)((data[0] << 8) | data[1]);  # Combine the two bytes into a signed 16-bit value.

        # Step 6: Read the imaginary part of the impedance data.
        AD5933_set_ptr_reg(AD5933_REG_IMAG_DATA);  # Set pointer to the imaginary data register.
        AD5933_read_reg_block(data, 2);            # Read 2 bytes (high and low byte of imaginary data).
        imag = (signed short)((data[0] << 8) | data[1]);  # Combine the two bytes into a signed 16-bit value.

        # Step 7: Store the real and imaginary data in their respective arrays.
        real_arr[freq_index] = real;
        imag_arr[freq_index] = imag;

        # Step 8: Increment the frequency to the next step in the sweep.
        AD5933_set_reg_value(AD5933_REG_CONTROL_HB, AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_INC_FREQ) | ctrl_hb_low);

        # Step 9: Check the status register to see if the sweep is complete.
        AD5933_set_ptr_reg(AD5933_REG_STATUS);
        AD5933_read_reg(&status);

        # Move to the next frequency index.
        freq_index++;
    } while ((status & AD5933_STAT_SWEEP_DONE) == 0);  # Loop until the sweep is marked as done.
}

# Function to perform a single-point gain factor calibration.
# This is used to calibrate the AD5933 system using a known impedance.
# Parameters:
# Z_calibration - The known impedance value in ohms used for calibration.
# magnitude - The measured magnitude of the impedance (from calc_magnitude).
# Returns:
# Gain factor used for impedance calculations.
double gain_factor_calibration(double Z_calibration, double magnitude) {
    # Gain factor is calculated as the inverse of the product of the known impedance and the measured magnitude.
    return (1 / Z_calibration) / magnitude;
}

# Function to calculate the magnitude of the impedance from real and imaginary components.
# The magnitude represents the absolute value of the complex impedance.
# Parameters:
# real - Real part of the impedance data (from the AD5933).
# imag - Imaginary part of the impedance data (from the AD5933).
# Returns:
# The magnitude of the impedance.
double calc_magnitude(int16_t real, int16_t imag) {
    # Use the Pythagorean theorem to compute the magnitude of the complex impedance.
    return sqrt((double)((real * real) + (imag * imag)));
}

# Function to calculate the impedance from the gain factor and measured data.
# This uses the gain factor derived during calibration to convert raw DFT results into impedance values.
# Parameters:
# gainFactor - The calibration factor obtained using gain_factor_calibration().
# real - Real part of the impedance data (from the AD5933).
# imag - Imaginary part of the impedance data (from the AD5933).
# Returns:
# The impedance value in ohms.
double AD5933_calculate_impedance(double gainFactor, signed short real, signed short imag) {
    # Step 1: Calculate the magnitude of the impedance from real and imaginary parts.
    double magnitude = calc_magnitude(real, imag);

    # Step 2: Compute the impedance using the formula Z = 1 / (magnitude * gainFactor).
    return 1.0 / (magnitude * gainFactor);
}
