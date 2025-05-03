/*  Includes for LEDColorTask
 *
 */

#define BLINK_GPIO   8

void configure_led(void);
// RTOS task function
void LED_task(void * arg);
