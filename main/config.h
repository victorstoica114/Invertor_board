#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocols/growatt/growatt_register_map.h"

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

/* --- RS485 -> CAN translator (uses RS485 SOC/TEMP to synthesize CAN 0x322) --- */
#define RS485_CAN_322_TRANSLATOR_ENABLE 1
#define RS485_CAN_322_TX_PERIOD_MS 200

/* --- Global working mode --- */
#define ACTIVE_WORKING_MODE 0 /* 0=bridge, 1=forward, 2=sniffer */

/* --- Active protocol selection --- */
#define ACTIVE_BMS_PROTOCOL      0 /* 0=Growatt, 1=Pylon */
#define ACTIVE_INVERTER_PROTOCOL 0 /* 0=Growatt, 1=Pylon */

/* --- Orchestrator runtime --- */
#define ORCHESTRATOR_TASK_STACK        4096
#define ORCHESTRATOR_TASK_PRIORITY     11
#define ORCHESTRATOR_BMS_QUEUE_LEN     1
#define ORCHESTRATOR_INVERTER_QUEUE_LEN 1
#define ORCHESTRATOR_FORCE_FORWARD_MS  1000

/* --- Growatt tasks --- */
#define GROWATT_BMS_MODBUS_SLAVE_ADDR  GROWATT_MODBUS_DEFAULT_SLAVE_ADDR
#define GROWATT_BMS_MODBUS_GAP_US      5000
#define GROWATT_BMS_QUERY_PERIOD_MS    250
#define GROWATT_BMS_PUBLISH_PERIOD_MS  250
#define GROWATT_BMS_TASK_STACK         4096
#define GROWATT_BMS_TASK_PRIORITY      10

#define GROWATT_INVERTER_TX_PERIOD_MS  200
#define GROWATT_INVERTER_TASK_STACK    4096
#define GROWATT_INVERTER_TASK_PRIORITY 9

/* --- Pylon placeholders --- */
#define PYLON_BMS_TASK_STACK           3072
#define PYLON_BMS_TASK_PRIORITY        8
#define PYLON_INVERTER_TASK_STACK      3072
#define PYLON_INVERTER_TASK_PRIORITY   8
#define PYLON_PLACEHOLDER_TASK_PERIOD_MS 1000

/* --- Working mode task settings --- */
#define FORWARD_CAN_DECODE_ENABLE      1
#define FORWARD_RS485_DECODE_ENABLE    1
#define FORWARD_CAN_TASK_STACK         4096
#define FORWARD_CAN_TASK_PRIORITY      10
#define FORWARD_RS485_TASK_STACK       4096
#define FORWARD_RS485_TASK_PRIORITY    9
#define FORWARD_RS485_GAP_US           5000

#define SNIFFER_CAN_DECODE_ENABLE      1
#define SNIFFER_RS485_DECODE_ENABLE    1
#define SNIFFER_CAN_TASK_STACK         4096
#define SNIFFER_CAN_TASK_PRIORITY      9
#define SNIFFER_RS485_TASK_STACK       4096
#define SNIFFER_RS485_TASK_PRIORITY    9
#define SNIFFER_RS485_GAP_US           5000

#define WORKING_MODE_HEX_PRINT_LIMIT   64
#define WORKING_MODE_SNAPSHOT_PERIOD_MS 5000
#define WORKING_MODE_SNAPSHOT_TASK_STACK 4096
#define WORKING_MODE_SNAPSHOT_TASK_PRIORITY 7

#ifdef __cplusplus
extern "C" {
#endif

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

