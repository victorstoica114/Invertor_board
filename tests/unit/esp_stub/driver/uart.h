#pragma once

#include <stddef.h>
#include <stdint.h>

typedef int uart_port_t;

static inline int uart_read_bytes(uart_port_t uart, void *buf, size_t len, int ticks_to_wait)
{
    (void)uart;
    (void)buf;
    (void)len;
    (void)ticks_to_wait;
    return 0;
}

