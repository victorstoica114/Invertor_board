#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef int uart_port_t;

#define UART_NUM_0 0
#define UART_NUM_1 1
#define UART_NUM_2 2

#define UART_SIGNAL_INV_DISABLE 0u

static inline int uart_read_bytes(uart_port_t uart, void *buf, size_t len, int ticks_to_wait)
{
    (void)uart;
    (void)buf;
    (void)len;
    (void)ticks_to_wait;
    return 0;
}

static inline void uart_flush_input(uart_port_t uart)
{
    (void)uart;
}

static inline int uart_write_bytes(uart_port_t uart, const char *src, size_t size)
{
    (void)uart;
    (void)src;
    return (int)size;
}

static inline esp_err_t uart_wait_tx_done(uart_port_t uart, int ticks_to_wait)
{
    (void)uart;
    (void)ticks_to_wait;
    return ESP_OK;
}

static inline esp_err_t uart_set_line_inverse(uart_port_t uart, uint32_t inverse_mask)
{
    (void)uart;
    (void)inverse_mask;
    return ESP_OK;
}
