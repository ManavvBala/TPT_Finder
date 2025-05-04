#include "LCD_setup.h"

void LCD_DemoTask(void* param) {
    char num;
    while (true) {
        LCD_home();
        LCD_clearScreen();
        // LCD_writeStr("16x2 I2C LCD");
        // vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        LCD_writeStr("Lets Count 0-10!");
        vTaskDelay(3000 / portTICK_RATE_MS);
        LCD_clearScreen();
        for (int i = 0; i <= 10; i++) {
            LCD_setCursor(8, 2);
            sprintf(num, "%d", i);
            LCD_writeStr(num);
            vTaskDelay(500 / portTICK_RATE_MS);
        }
    }
}
