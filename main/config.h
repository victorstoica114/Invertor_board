#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocols/growatt/growatt_register_map.h"
#include "protocols/jkbms_modbus/jkbms_modbus_register_map.h"

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
#define CAN_DECODER_SHOW_RAW_FRAMES 1
#define REG_RAW_VALUES 0
#define MODBUS_DECODER_SNAPSHOT_ONLY 1
#define RS485_FORWARD_VERBOSE_LOGS 0

/* --- Bridge mode switches --- */
#define CAN_FORWARD_ENABLE 0
#define RS485_FORWARD_ENABLE 0

/* --- RS485 -> CAN translator (uses RS485 SOC/TEMP to synthesize CAN 0x322) --- */
#define RS485_CAN_322_TRANSLATOR_ENABLE 1
#define RS485_CAN_322_TX_PERIOD_MS 200

/* --- CAN -> RS485 Growatt translator (answers inverter Modbus polls from CAN cache) --- */
#define CAN_RS485_SOC_TRANSLATOR_ENABLE 1
#define CAN_RS485_SOC_SLAVE_ID 1u
#define CAN_RS485_SOC_FAKE_PCT 99u
#define CAN_RS485_SOC_RX_GAP_US 5000u
/* Source freshness window for fail-safe (stop answering if source is stale). */
#define BRIDGE_SOURCE_STALE_MS 2000u

/* --- Global working mode --- */
#define ACTIVE_WORKING_MODE 0 /* 0=bridge, 1=forward, 2=sniffer */

/* --- Active protocol selection --- */
#define ACTIVE_BMS_PROTOCOL      0 /* 0=Growatt, 1=Pylon */
#define ACTIVE_INVERTER_PROTOCOL 0 /* 0=Growatt, 1=Pylon */

/* --- Runtime/web compatibility IDs --- */
#define MODE_SNIFFER 1
#define MODE_FORWARD 2
#define MODE_BRIDGE 3

#define SYSTEM_MODE (((ACTIVE_WORKING_MODE) == 2) ? MODE_SNIFFER : \
                     (((ACTIVE_WORKING_MODE) == 1) ? MODE_FORWARD : MODE_BRIDGE))

#define LINE_CAN 1
#define LINE_RS485 2

#define PROTOCOL_CAN_GROWATT 1
#define PROTOCOL_RS485_GROWATT 2
#define PROTOCOL_RS485_PYLON 3
#define PROTOCOL_CAN_PYLON 4
#define PROTOCOL_CAN_DEYE 5
#define PROTOCOL_RS485_JKBMS 6
#define PROTOCOL_CAN_GOODWE 7
#define PROTOCOL_CAN_SOFAR 8
#define PROTOCOL_CAN_SMA 9

#define BMS_line LINE_RS485
#define Inverter_line LINE_CAN
#define BMS_protocol PROTOCOL_RS485_GROWATT
#define Inverter_protocol PROTOCOL_CAN_GROWATT
#define BMS_PORT 1
#define Inverter_PORT 2

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

/* --- JKBMS Modbus task --- */
#define JKBMS_BMS_MODBUS_SLAVE_ADDR    JKBMS_MODBUS_DEFAULT_SLAVE_ADDR
#define JKBMS_BMS_MODBUS_GAP_US        5000
#define JKBMS_BMS_QUERY_PERIOD_MS      250
#define JKBMS_BMS_PUBLISH_PERIOD_MS    250
#define JKBMS_BMS_TASK_STACK           4096
#define JKBMS_BMS_TASK_PRIORITY        10

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

/* --- Web interface --- */
#define WEB_INTERFACE_ENABLE            1
#define WEB_INTERFACE_PORT              80
#define WEB_INTERFACE_TASK_STACK        8192
#define WEB_INTERFACE_TASK_PRIO         5
#define WIFI_STA_SSID                   "ED423"
#define WIFI_STA_PASSWORD               "electr0n!ca"
#define WIFI_STA_MAX_RETRY              10
#define WIFI_STA_HOSTNAME               "esp32-bridge"

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

