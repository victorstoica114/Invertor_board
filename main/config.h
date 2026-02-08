#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/twai.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* Tag comun de log */
#define EXAMPLE_TAG "SNIFFER_BRIDGE"

/* --- PINI CAN --- */
#define CAN1_TX  19
#define CAN1_RX  20
#define CAN2_TX  21
#define CAN2_RX  0

/* --- PIN LED --- */
#define LED_GPIO 2

/* --- PINI RS485 --- */
#define RS485_1_RX   1
#define RS485_1_TX   18
#define RS485_1_DIR  11

#define RS485_2_RX   22
#define RS485_2_TX   23
#define RS485_2_DIR  10

/* --- UART-uri folosite pentru RS485 --- */
#define RS485_1_UART  UART_NUM_1
#define RS485_2_UART  UART_NUM_0

/* --- Setări UART --- */
#define RS485_BAUDRATE     9600
#define RS485_BUF_SIZE     512

#ifdef __cplusplus
extern "C" {
#endif

/* Init/config */
void rs485Init(void);
void canInit(void);

/* Accessori pentru resurse */
twai_handle_t canGetBus0(void);   // CAN1
twai_handle_t canGetBus1(void);   // CAN2

uart_port_t rs485GetUart1(void);  // RS485_1_UART
uart_port_t rs485GetUart2(void);  // RS485_2_UART

gpio_num_t rs485GetDir1(void);    // RS485_1_DIR
gpio_num_t rs485GetDir2(void);    // RS485_2_DIR

/* Task LED (cerut explicit să rămână apelat din main) */
void led_blink_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
