#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

void rs485Init(void);
void rs485Reinit(void);
uint32_t rs485GetBaudRate(uart_port_t uart);
uart_port_t rs485GetUart1(void);  /* RS485_1_UART */
uart_port_t rs485GetUart2(void);  /* RS485_2_UART */
gpio_num_t rs485GetDir1(void);    /* RS485_1_DIR */
gpio_num_t rs485GetDir2(void);    /* RS485_2_DIR */
void rs485SetDirection(gpio_num_t dirPin, bool txEnable);
esp_err_t rs485WriteBytes(uart_port_t uart,
                          gpio_num_t dirPin,
                          const uint8_t *data,
                          int len,
                          TickType_t txTimeoutTicks);

#ifdef __cplusplus
}
#endif
