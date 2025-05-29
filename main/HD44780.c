#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include <esp_log.h>

static const char* TAG = "HD44780_LCD";
static const char* TAG2 = "HD44780_LCD_details";


// LCD module defines
#define LCD_LINEONE             0x00        // start of line 1
#define LCD_LINETWO             0x40        // start of line 2
#define LCD_LINETHREE           0x14        // start of line 3
#define LCD_LINEFOUR            0x54        // start of line 4

#define LCD_BACKLIGHT           0x08
#define LCD_ENABLE              0x04               
#define LCD_COMMAND             0x00
#define LCD_WRITE               0x01

#define LCD_SET_DDRAM_ADDR      0x80
#define LCD_READ_BF             0x40

// LCD instructions
#define LCD_CLEAR               0x01        // replace all characters with ASCII 'space'
#define LCD_HOME                0x02        // return cursor to first position on first line
#define LCD_ENTRY_MODE          0x06        // shift cursor from left to right on read/write
#define LCD_DISPLAY_OFF         0x08        // turn display off
#define LCD_DISPLAY_ON          0x0C        // display on, cursor off, don't blink character
#define LCD_FUNCTION_RESET      0x30        // reset the LCD
#define LCD_FUNCTION_SET_4BIT   0x28        // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR          0x80        // set cursor position

// Pin mappings
// P0 -> RS
// P1 -> RW
// P2 -> E
// P3 -> Backlight
// P4 -> D4
// P5 -> D5
// P6 -> D6
// P7 -> D7

static char tag[] = "LCD Driver";
static uint8_t LCD_addr;
static uint8_t SDA_pin;
static uint8_t SCL_pin;
static uint8_t LCD_cols;
static uint8_t LCD_rows;

// New I2C handles for ESP-IDF v5+
static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t lcd_dev_handle;

static void LCD_writeNibble(uint8_t nibble, uint8_t mode);
static void LCD_writeByte(uint8_t data, uint8_t mode);
static void LCD_pulseEnable(uint8_t nibble);

static esp_err_t I2C_init(void)
{
    // New I2C master bus configuration
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_pin,
        .scl_io_num = SCL_pin,
        .glitch_ignore_cnt = 15,
        .flags.enable_internal_pullup = true,
        .intr_priority = 0, // Claude.ai
        .trans_queue_depth = 0,
    };
    
    // Create new I2C master bus
    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to initialize I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Device configuration for LCD
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_addr,
        .scl_speed_hz = 5000,
    };

    // Add LCD as a device on the bus
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &lcd_dev_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to add LCD device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

void LCD_init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows)
{
    LCD_addr = addr;
    SDA_pin = dataPin;
    SCL_pin = clockPin;
    LCD_cols = cols;
    LCD_rows = rows;
    esp_err_t initResult = I2C_init();

    vTaskDelay(100 / portTICK_PERIOD_MS);                                 // Initial 40 mSec delay

    if (initResult == ESP_OK)
        ESP_LOGI(tag, " Completed I2C_init()");
    else
        ESP_LOGI(tag, "I2C_init() FAIL");

    // Add this right after I2C_init() succeeds, before any LCD commands:
        ESP_LOGI(tag, "Testing basic I2C communication...");
        uint8_t dummy_data = 0x00;
        esp_err_t probe_ret = i2c_master_transmit(lcd_dev_handle, &dummy_data, 1, 100);
        ESP_LOGI(tag, "Basic probe result: %s", esp_err_to_name(probe_ret));

        if (probe_ret == ESP_OK) {
            ESP_LOGI(tag, "PCF8574 is responding");
        } else {
            ESP_LOGE(tag, "PCF8574 not responding properly");
        }

    // Reset the LCD controller
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // First part of reset sequence
    vTaskDelay(10 / portTICK_PERIOD_MS);                                  // 4.1 mS delay (min)

    ESP_LOGI(TAG, " Completed LCD reset sequence: Part 1");

    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // second part of reset sequence
    ets_delay_us(200);                                                  // 100 uS delay (min)
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // Third time's a charm
    LCD_writeNibble(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                // Activate 4-bit mode
    ets_delay_us(80);
    // 40 uS delay (min)
    ESP_LOGI(TAG, " Completed LCD reset sequence: Part 2");

    // --- Busy flag now available ---
    // Function Set instruction
    LCD_writeByte(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                  // Set mode, lines, and font
    ets_delay_us(80); 
    ESP_LOGI(TAG, " Completed LCD setting mode/lines/font");

    // Clear Display instruction
    LCD_writeByte(LCD_CLEAR, LCD_COMMAND);                              // clear display RAM
    vTaskDelay(2 / portTICK_PERIOD_MS);                                   // Clearing memory takes a bit longer
    ESP_LOGI(TAG, " Completed LCD clear");


    // Entry Mode Set instruction
    LCD_writeByte(LCD_ENTRY_MODE, LCD_COMMAND);                         // Set desired shift characteristics
    ets_delay_us(80); 

    LCD_writeByte(LCD_DISPLAY_ON, LCD_COMMAND);                         // Ensure LCD is set to on
    ESP_LOGI(TAG, " Completed ALL LCD setups");

}

void LCD_setCursor(uint8_t col, uint8_t row)
{
    if (row > LCD_rows - 1) {
        ESP_LOGE(tag, "Cannot write to row %d. Please select a row in the range (0, %d)", row, LCD_rows-1);
        row = LCD_rows - 1;
    }
    uint8_t row_offsets[] = {LCD_LINEONE, LCD_LINETWO, LCD_LINETHREE, LCD_LINEFOUR};
    LCD_writeByte(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]), LCD_COMMAND);
    ESP_LOGI(TAG2, " Completed LCD_setCursor");
}

void LCD_writeChar(char c)
{
    LCD_writeByte(c, LCD_WRITE);                                        // Write data to DDRAM
}

void LCD_writeStr(char* str)
{
    while (*str) {
        LCD_writeChar(*str++);
    }
}

void LCD_home(void)
{
    LCD_writeByte(LCD_HOME, LCD_COMMAND);
    vTaskDelay(2 / portTICK_PERIOD_MS);                                   // This command takes a while to complete
    ESP_LOGI(TAG2, " Completed LCD_home()  " );
}

void LCD_clearScreen(void)
{
    LCD_writeByte(LCD_CLEAR, LCD_COMMAND);
    vTaskDelay(2 / portTICK_PERIOD_MS);                                   // This command takes a while to complete
    ESP_LOGI(TAG2, " Completed LCD_clearScreen");
}

static void LCD_writeNibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;

    ESP_LOGI(TAG2, "About to transmit: 0x%02X to device at 0x%02X", data, LCD_addr);

    // Use the new I2C transmit function
    // esp_err_t ret = i2c_master_transmit(lcd_dev_handle, &data, 1, -1);
    esp_err_t ret = i2c_master_transmit(lcd_dev_handle, &data, 1, 1000);  // Claude
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to transmit data: %s", esp_err_to_name(ret));
        return;
    }
    else
        ESP_LOGI(TAG2, "transmitted a nibble successfully to LCD");
    ets_delay_us(500);
    LCD_pulseEnable(data);                                              // Clock data into LCD
}

static void LCD_writeByte(uint8_t data, uint8_t mode)
{
    LCD_writeNibble(data & 0xF0, mode);
    LCD_writeNibble((data << 4) & 0xF0, mode);
}

static void LCD_pulseEnable(uint8_t data)
{
    // Set enable high
    uint8_t enable_high = data | LCD_ENABLE;
    // esp_err_t ret = i2c_master_transmit(lcd_dev_handle, &enable_high, 1, -1);
    esp_err_t ret = i2c_master_transmit(lcd_dev_handle, &enable_high, 1, 1000); // Claude
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to transmit enable high: %s", esp_err_to_name(ret));
    }
    ets_delay_us(10);

    // Set enable low
    uint8_t enable_low = data & ~LCD_ENABLE;
    ret = i2c_master_transmit(lcd_dev_handle, &enable_low, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to transmit enable low: %s", esp_err_to_name(ret));
    }
    ets_delay_us(500);
}

// Clean up function to release I2C resources
void LCD_deinit(void)
{
    if (lcd_dev_handle != NULL) {
        i2c_master_bus_rm_device(lcd_dev_handle);
    }
    
    if (i2c_bus_handle != NULL) {
        i2c_del_master_bus(i2c_bus_handle);
    }
}
