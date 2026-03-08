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

/* --- Bridge mode switches --- */
#define CAN_FORWARD_ENABLE 0
#define RS485_FORWARD_ENABLE 0

/* --- Runtime context selection (BMS / Inverter wiring and protocol) --- */
#define LINE_CAN 1
#define LINE_RS485 2

#define PROTOCOL_CAN_GROWATT 1
#define PROTOCOL_RS485_GROWATT 2

/* Requested user-facing config macros */
#define BMS_line LINE_CAN
#define Inverter_line LINE_RS485
#define BMS_protocol PROTOCOL_CAN_GROWATT
#define Inverter_protocol PROTOCOL_RS485_GROWATT
#define BMS_PORT 1
#define Inverter_PORT 2

/* Compile-time validation */
#if ((BMS_line != LINE_CAN) && (BMS_line != LINE_RS485))
#error "BMS_line must be LINE_CAN or LINE_RS485"
#endif
#if ((Inverter_line != LINE_CAN) && (Inverter_line != LINE_RS485))
#error "Inverter_line must be LINE_CAN or LINE_RS485"
#endif
#if ((BMS_protocol != PROTOCOL_CAN_GROWATT) && (BMS_protocol != PROTOCOL_RS485_GROWATT))
#error "BMS_protocol must be PROTOCOL_CAN_GROWATT or PROTOCOL_RS485_GROWATT"
#endif
#if ((Inverter_protocol != PROTOCOL_CAN_GROWATT) && (Inverter_protocol != PROTOCOL_RS485_GROWATT))
#error "Inverter_protocol must be PROTOCOL_CAN_GROWATT or PROTOCOL_RS485_GROWATT"
#endif
#if ((BMS_PORT < 1) || (BMS_PORT > 2))
#error "BMS_PORT must be 1 or 2"
#endif
#if ((Inverter_PORT < 1) || (Inverter_PORT > 2))
#error "Inverter_PORT must be 1 or 2"
#endif
#if ((BMS_line == LINE_CAN) && (BMS_protocol != PROTOCOL_CAN_GROWATT))
#error "If BMS_line is LINE_CAN, BMS_protocol must be PROTOCOL_CAN_GROWATT"
#endif
#if ((BMS_line == LINE_RS485) && (BMS_protocol != PROTOCOL_RS485_GROWATT))
#error "If BMS_line is LINE_RS485, BMS_protocol must be PROTOCOL_RS485_GROWATT"
#endif
#if ((Inverter_line == LINE_CAN) && (Inverter_protocol != PROTOCOL_CAN_GROWATT))
#error "If Inverter_line is LINE_CAN, Inverter_protocol must be PROTOCOL_CAN_GROWATT"
#endif
#if ((Inverter_line == LINE_RS485) && (Inverter_protocol != PROTOCOL_RS485_GROWATT))
#error "If Inverter_line is LINE_RS485, Inverter_protocol must be PROTOCOL_RS485_GROWATT"
#endif

/* --- RS485 -> CAN translator (synthesizes Growatt CAN telemetry from RS485 cache) --- */
#define RS485_CAN_322_TRANSLATOR_ENABLE 1
#define RS485_CAN_322_TX_PERIOD_MS 200
#define RS485_CAN_BRIDGE_TX_LOG_EVERY_N 10
/* --- CAN -> RS485 translator (minimal Modbus slave on inverter RS485) --- */
#define CAN_RS485_SOC_TRANSLATOR_ENABLE 1
#define CAN_RS485_SOC_SLAVE_ID 1u
#define CAN_RS485_SOC_FAKE_PCT 99u
#define CAN_RS485_SOC_RX_GAP_US 5000u
#define CAN_RS485_SOC_LOG_EVERY_N 20u

/* Active RS485 BMS poller: requests Modbus blocks directly from BMS on RS485_1. */
#define RS485_BMS_POLLER_ENABLE 1
#define RS485_BMS_SLAVE_ID 1u
#define RS485_BMS_POLL_PERIOD_MS 200
#define RS485_BMS_POLL_LOG_EVERY_N 25

/* Fallback values used when RS485 cache is missing/partial. */
#define RS485_CAN_BRIDGE_USE_FALLBACK 0
#define RS485_CAN_BRIDGE_FALLBACK_SOC_PCT       99u
#define RS485_CAN_BRIDGE_FALLBACK_TEMP_C        25
#define RS485_CAN_BRIDGE_FALLBACK_PACK_V_CV     6909u
#define RS485_CAN_BRIDGE_FALLBACK_PACK_I_0P1    0
#define RS485_CAN_BRIDGE_FALLBACK_SOH_PCT       100u
#define RS485_CAN_BRIDGE_FALLBACK_RM_10MAH      3950u
#define RS485_CAN_BRIDGE_FALLBACK_FCC_10MAH     4000u
#define RS485_CAN_BRIDGE_FALLBACK_CYCLE_COUNT   0u
#define RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_MV   4388u
#define RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_MV   4253u
#define RS485_CAN_BRIDGE_FALLBACK_CELL_MAX_IDX  3u
#define RS485_CAN_BRIDGE_FALLBACK_CELL_MIN_IDX  12u
#define RS485_CAN_BRIDGE_FALLBACK_FLAGS_319     0x9Cu
#define RS485_CAN_BRIDGE_FALLBACK_ADDR_319      0u
#define RS485_CAN_BRIDGE_FALLBACK_TEMP_SENSOR_MAX 1u
#define RS485_CAN_BRIDGE_FALLBACK_TEMP_SENSOR_MIN 1u
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

