//  Setup LCD for ESP32 testing
//    (BH pin connections)

#include "driver/i2c.h"
#include "HD44780.h"

#define LCD_ADDR 0x27
#define LCD_SDA_PIN 0
#define LCD_SCL_PIN 1
#define LCD_COLS 16
#define LCD_ROWS 2

void LCD_DemoTask(void* param);
