#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Define pin numbers
#define BUTTON_PIN      GPIO_NUM_6 // Pin for the push button
#define LED_RED         GPIO_NUM_9 
#define LED_GREEN       GPIO_NUM_10
#define LED_YELLOW      GPIO_NUM_21
#define LED_BLUE        GPIO_NUM_20
#define LED_RED_RING    GPIO_NUM_3
#define READ_VAL        GPIO_NUM_7
#define READ_VAL2       GPIO_NUM_2

// Simulated testing values
#define VALUE_CORRECT   1
#define VALUE_WRONG     0
#define HIGH_STATE      1
#define LOW_STATE       0

void app_main(void) {
    // Initializes all the input GPIO pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN) | (1ULL << READ_VAL) | (1ULL << READ_VAL2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    // Configure LED pins as outputs
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_RED) | (1ULL << LED_GREEN) | (1ULL << LED_YELLOW) | (1ULL << LED_BLUE) | (1ULL << LED_RED_RING),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&led_conf);

    // Turn off all LEDs initially
    gpio_set_level(LED_RED, LOW_STATE);
    gpio_set_level(LED_GREEN, LOW_STATE);
    gpio_set_level(LED_YELLOW, LOW_STATE);
    gpio_set_level(LED_BLUE, LOW_STATE);
    gpio_set_level(LED_RED_RING, LOW_STATE);

    // Initializes the button press state to 0
    // And the last button state to high
    int button_press = 0;
    int last_button_state = HIGH_STATE;

    while (1) {
        // Read the button state and input values
        int button_state = gpio_get_level(BUTTON_PIN);
        int read_val = gpio_get_level(READ_VAL);
        int read_val2 = gpio_get_level(READ_VAL2);

        // Button press is incremented to indicate that it has been pressed
        if (button_state == LOW_STATE && last_button_state == HIGH_STATE) {
            button_press++;
            printf("Number of times button pressed: %d\n", button_press); // For debugging purposes
        }

        // If the button is not pressed or reset to 0
        if (button_press == 0) {
            // All the LEDs are off
            gpio_set_level(LED_GREEN, LOW_STATE);
            gpio_set_level(LED_YELLOW, LOW_STATE);
            gpio_set_level(LED_RED, LOW_STATE);
            gpio_set_level(LED_RED_RING, LOW_STATE);
            gpio_set_level(LED_BLUE, LOW_STATE);
        }

        // If the LED is pressed once
        if (button_press == 1) {
            // If the value is correct, green LED turns on
            if (read_val == VALUE_CORRECT) {
                gpio_set_level(LED_GREEN, HIGH_STATE);
                gpio_set_level(LED_RED, LOW_STATE);
            } else if (read_val == VALUE_WRONG) { // If the value is incorrect, red LED turns on
                gpio_set_level(LED_RED, HIGH_STATE);
                gpio_set_level(LED_GREEN, LOW_STATE);
                button_press = 0; // Return back to 0 button press if value is incorrect
            }
        }

        // If the button is pressed twice and the green LED is on
        if (button_press == 2 && read_val == VALUE_CORRECT) {
            if (read_val2 == LOW_STATE) { // Low possibility
                gpio_set_level(LED_RED_RING, HIGH_STATE);
            } else if (read_val2 == HIGH_STATE) { // High possiblity
                gpio_set_level(LED_BLUE, HIGH_STATE);
            } else {
                gpio_set_level(LED_YELLOW, HIGH_STATE);
            }
        }

        if (button_press == 3) { // Press button one more time to reset
            button_press = 0;
        }

        last_button_state = button_state;
        vTaskDelay(pdMS_TO_TICKS(50)); // Debounce delay
    }
}
