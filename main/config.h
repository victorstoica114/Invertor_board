#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "Growatt_regs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/twai.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* Common log tag */
#define EXAMPLE_TAG "SNIFFER_BRIDGE"

/* --- CAN pins --- */
#define CAN1_TX  19
#define CAN1_RX  20
#define CAN2_TX  21
#define CAN2_RX  0

/* --- LED pin --- */
#define LED_GPIO 2

/* --- RS485 pins --- */
#define RS485_1_RX   1
#define RS485_1_TX   18
#define RS485_1_DIR  11

#define RS485_2_RX   22
#define RS485_2_TX   23
#define RS485_2_DIR  10

/* --- UARTs used for RS485 --- */
#define RS485_1_UART  UART_NUM_1
#define RS485_2_UART  UART_NUM_0

/* --- UART settings --- */
#define RS485_BAUDRATE     9600
#define RS485_BUF_SIZE     512

/* --- Decoder / logging compile-time switches --- */
#define CAN_DECODER_SHOW_RAW_FRAMES 0
#define REG_RAW_VALUES 0
#define MODBUS_DECODER_SNAPSHOT_ONLY 1
#define RS485_FORWARD_VERBOSE_LOGS 0

#ifdef __cplusplus
extern "C" {
#endif

/* Init/config */
void rs485Init(void);
void canInit(void);

/* Resource accessors */
twai_handle_t canGetBus0(void);   /* CAN1 */
twai_handle_t canGetBus1(void);   /* CAN2 */

uart_port_t rs485GetUart1(void);  /* RS485_1_UART */
uart_port_t rs485GetUart2(void);  /* RS485_2_UART */

gpio_num_t rs485GetDir1(void);    /* RS485_1_DIR */
gpio_num_t rs485GetDir2(void);    /* RS485_2_DIR */

/* Bridge filtering lists (configuration) */
extern const uint32_t g_can1ToCan2ExcludeIds[];
extern const size_t g_can1ToCan2ExcludeIdsCount;

extern const uint16_t g_rs485ForwardExcludeRegs[];
extern const size_t g_rs485ForwardExcludeRegsCount;

/* LED task */
void led_blink_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
