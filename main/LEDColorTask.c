/*
 *    LEDColorTask.c
 *    smoothly change colors on the Waveshare ESP32C6 board
 *    builtin RGB LED  (Author BH, Spr 2025)
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "led_strip.h"

#include "LEDColorTask.h"

// #define LEDC_TIMER              LEDC_TIMER_0
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          BLINK_GPIO // Define the output GPIO
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
// #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
// #define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

// logging tag
static const char *TAG = "LEDColorTask";

//  Code for the RGB combo led on pin 8

static led_strip_handle_t led_strip;


/*
 * @brief a lookup table to make PWM into an exponential response
 *  (compensates for human visual log response)
 */
static uint32_t logtable[] = {  // xfactor =  2.5
 0.0,  0.0,  0.0,  0.0,  0.0,  1.0,  1.0,  1.0,
 1.0,  2.0,  2.0,  2.0,  2.0,  3.0,  3.0,  3.0,
 3.0,  4.0,  4.0,  4.0,  4.0,  5.0,  5.0,  5.0,
 6.0,  6.0,  6.0,  6.0,  7.0,  7.0,  7.0,  8.0,
 8.0,  8.0,  9.0,  9.0,  9.0, 10.0, 10.0, 10.0,
10.0, 11.0, 11.0, 12.0, 12.0, 12.0, 13.0, 13.0,
13.0, 14.0, 14.0, 14.0, 15.0, 15.0, 15.0, 16.0,
16.0, 17.0, 17.0, 17.0, 18.0, 18.0, 19.0, 19.0,
19.0, 20.0, 20.0, 21.0, 21.0, 22.0, 22.0, 23.0,
23.0, 23.0, 24.0, 24.0, 25.0, 25.0, 26.0, 26.0,
27.0, 27.0, 28.0, 28.0, 29.0, 29.0, 30.0, 30.0,
31.0, 31.0, 32.0, 32.0, 33.0, 34.0, 34.0, 35.0,
35.0, 36.0, 36.0, 37.0, 38.0, 38.0, 39.0, 39.0,
40.0, 41.0, 41.0, 42.0, 43.0, 43.0, 44.0, 45.0,
45.0, 46.0, 47.0, 47.0, 48.0, 49.0, 49.0, 50.0,
51.0, 52.0, 52.0, 53.0, 54.0, 55.0, 55.0, 56.0,
57.0, 58.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0,
63.0, 64.0, 65.0, 66.0, 67.0, 68.0, 69.0, 70.0,
71.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0,
78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0,
86.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0,
95.0, 97.0, 98.0, 99.0, 100.0, 101.0, 103.0, 104.0,
105.0, 106.0, 108.0, 109.0, 110.0, 112.0, 113.0, 114.0,
116.0, 117.0, 118.0, 120.0, 121.0, 123.0, 124.0, 126.0,
127.0, 128.0, 130.0, 131.0, 133.0, 135.0, 136.0, 138.0,
139.0, 141.0, 142.0, 144.0, 146.0, 147.0, 149.0, 151.0,
153.0, 154.0, 156.0, 158.0, 160.0, 161.0, 163.0, 165.0,
167.0, 169.0, 171.0, 173.0, 174.0, 176.0, 178.0, 180.0,
182.0, 184.0, 186.0, 189.0, 191.0, 193.0, 195.0, 197.0,
199.0, 201.0, 204.0, 206.0, 208.0, 210.0, 213.0, 215.0,
217.0, 220.0, 222.0, 225.0, 227.0, 229.0, 232.0, 234.0,
237.0, 240.0, 242.0, 245.0, 247.0, 250.0, 253.0, 256.0,
};



void LED_task(void *)
{   static uint32_t iter = 0;
    while (1) {
        iter++;
        /* generate offset indices for the R,G,B PWM */
        int ofst = (int)(256/3);
        int ix1 = (iter+ofst)%256;
        int ix2 = (iter+2*ofst)%256;
        int ix3 = (iter)%256;

        /*
         *  Human visual perception is logarithmic in light intensity.
         *  map the 8-bit brightness value to an exponential --
         *  subjectively this looks more linear!!
         */
        const int scale = 1;
        uint32_t ir = logtable[ ix1 ] >> scale;
        uint32_t ig = logtable[ ix2 ] >> scale;
        uint32_t ib = logtable[ ix3 ] >> scale;
        led_strip_set_pixel(led_strip, 0, ir,ig,ib);

        ESP_LOGI(TAG, "LEDs (RGB): %d, %d, %d", (unsigned int)ir, (unsigned int)ig, (unsigned int)ib );

        /* (we refer to the single RGB unit as a "strip" of length 1)
         *  Refresh the "strip" to send data
         */
        led_strip_refresh(led_strip);

        // vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        vTaskDelay(100/ portTICK_PERIOD_MS);
        }
}


void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

