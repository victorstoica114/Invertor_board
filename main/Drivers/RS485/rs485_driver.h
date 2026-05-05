#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

void rs485Init(void);
void rs485Reinit(void);
uint32_t rs485GetBaudRate(uart_port_t uart);
uart_port_t rs485GetUart1(void);
uart_port_t rs485GetUart2(void);
gpio_num_t rs485GetDir1(void);
gpio_num_t rs485GetDir2(void);
void rs485DriverSetTx(gpio_num_t dirPin, bool txEnable);
void rs485DriverWriteFrame(uart_port_t uart, gpio_num_t dirPin, const uint8_t *frame, int len);
void rs485ResetPorts(void);

#ifdef __cplusplus
}
#endif
