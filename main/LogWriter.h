#ifndef LOGWRITER_H_
#define LOGWRITER_H_

#include "driver/uart.h"

#define UART_PORT UART_NUM_0
#define UART_BUFFER_SIZE (1024*2)

extern uart_config_t uart_config;

void UART_init();

void write_values(uint8_t n, double gain_factor, int16_t* real, int16_t* imag, uint16_t* freq);

#endif