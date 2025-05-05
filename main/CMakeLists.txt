idf_component_register(
                    SRCS "LogWriter.c" "AD5933.c"
                    "main.c" "LEDColorTask.c" "LCD_2x16.c" "HD44780.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "esp_driver_i2c")

                   #
