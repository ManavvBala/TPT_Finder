#include "LogWriter.h"
#include "driver/uart.h"
#include <string.h>

uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

QueueHandle_t uart_queue;

void UART_init() {
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    // don't need to set pins since we are writing over serial
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE, \
                    UART_BUFFER_SIZE, 10, &uart_queue, 0));
}

void write_values(uint8_t n, double gain_factor, int16_t* real, int16_t* imag, uint16_t* freq) {
    // output format:
    // | sync byte (0xFA) | num_incr | tuples basically for (freq, real, imag)
    // total length = 1 + 1 + 8 + n*3*2
    uint8_t* msg = (uint8_t*)malloc(sizeof(uint8_t) * (2 + n * 6));
    msg[0] = 0xFA;
    msg[1] = n;
    memcpy(&msg[2], &gain_factor, 8);
    for (int i = 0; i < n; i ++) {
        msg[6 * i + 10] = freq[i] >> 8;
        msg[6 * i + 11] = freq[i] & 0xFF;
        msg[6 * i + 12] = real[i] >> 8;
        msg[6 * i + 13] = real[i] & 0xFF;
        msg[6 * i + 14] = imag[i] >> 8;
        msg[6 * i + 15] = imag[i] & 0xFF;
    }

    uart_write_bytes(UART_PORT, msg, 10 + n * 6);
}