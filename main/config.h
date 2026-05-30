#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocols/growatt/growatt_registers_map.h"
#include "protocols/jkbms_modbus/jkbms_modbus_registers_map.h"

#if defined(__has_include)
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#endif

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
#define RS485_DEFAULT_BAUDRATE 9600u
#define RS485_FAST_BAUDRATE    115200u
#define RS485_BAUDRATE         RS485_DEFAULT_BAUDRATE
#define RS485_BUF_SIZE         512

/* --- CAN settings --- */
#define CAN_DEFAULT_BITRATE    500000u
#define CAN_JKBMS_250K_BITRATE 250000u

/* --- RS485 line-control compatibility (used by Pylon RS485 bridge) --- */
#define RS485_USE_HALF_DUPLEX     1
#define RS485_1_USE_HALF_DUPLEX   1
#define RS485_2_USE_HALF_DUPLEX   1
#define RS485_1_DIR_TX_LEVEL      1
#define RS485_2_DIR_TX_LEVEL      1
#define RS485_1_TX_PRE_DELAY_MS   0
#define RS485_1_TX_POST_DELAY_MS  0
#define RS485_2_TX_PRE_DELAY_MS   0
#define RS485_2_TX_POST_DELAY_MS  0

/* --- Decoder / logging compile-time switches --- */
#define CAN_DECODER_SHOW_RAW_FRAMES 1
#define REG_RAW_VALUES 0
#define MODBUS_DECODER_SNAPSHOT_ONLY 1
#define RS485_FORWARD_VERBOSE_LOGS 0

/* --- Bridge mode switches --- */
#define CAN_FORWARD_ENABLE 0
#define RS485_FORWARD_ENABLE 0
#define CAN_EXCLUDE_LIST_ENABLE 0
#define CAN_FORWARD_PYLON_16S_TO_8S_ENABLE 0

/* --- RS485 -> CAN translator (uses RS485 SOC/TEMP to synthesize CAN 0x322) --- */
#define RS485_CAN_322_TRANSLATOR_ENABLE 1
#define RS485_CAN_322_TX_PERIOD_MS 200

/* --- CAN -> RS485 Growatt translator (answers inverter Modbus polls from CAN cache) --- */
#define CAN_RS485_SOC_TRANSLATOR_ENABLE 1
#define CAN_RS485_SOC_SLAVE_ID 1u
#define CAN_RS485_SOC_FAKE_PCT 99u
#define CAN_RS485_SOC_RX_GAP_US 5000u

/* --- Diagnostic step: force fake replies for CAN_PYLON -> RS485_PYLON route --- */
#define PYLON_CAN_RS485_FORCE_FAKE_ENABLE 0
#define PYLON_RS485_ACTIVE_PROBE_ENABLE 1
#define PYLON_RS485_BMS_UART_INVERT 0
#define PYLON_RS485_SOC_FLOOR_ENABLE 1
#define PYLON_RS485_SOC_FLOOR_PCT 20u
#define PYLON_RS485_PACK_ID_FALLBACK_ENABLE 0

/* Diagnostic: ignore persisted NVS route and boot with the config.h route below. */
#define RUNTIME_SETTINGS_FORCE_DEFAULTS 1

/* Source freshness window for fail-safe (stop answering if source is stale). */
#define BRIDGE_SOURCE_STALE_MS 2000u

/* Telemetry cache max age before UI hides stale values. */
#define WEB_TELEMETRY_STALE_MS 10000u

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
#define PROTOCOL_CAN_VICTRON 10
#define PROTOCOL_RS485_PACE 11
#define PROTOCOL_RS485_JKBMS_NATIVE 12
#define PROTOCOL_RS485_VOLTRONIC 13
#define PROTOCOL_RS485_CHINA_TOWER 14
#define PROTOCOL_RS485_WOW 15
#define PROTOCOL_RS485_JKBMS_115200 16
#define PROTOCOL_RS485_PYLON_115200 17
#define PROTOCOL_CAN_JKBMS_250K 18
#define PROTOCOL_RS485_SEPLOS 19
#define PROTOCOL_RS485_SEPLOS_19200 20
#define PROTOCOL_RS485_DALY 21
#define PROTOCOL_CAN_DALY 22
#define PROTOCOL_ID_MAX PROTOCOL_CAN_DALY

static inline uint8_t bridgeProtocolCanonical(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_RS485_JKBMS_115200:
            return PROTOCOL_RS485_JKBMS;
        case PROTOCOL_RS485_PYLON_115200:
            return PROTOCOL_RS485_PYLON;
        default:
            return protocol;
    }
}

static inline bool bridgeProtocolIsRs485JkbmsModbus(uint8_t protocol)
{
    return bridgeProtocolCanonical(protocol) == PROTOCOL_RS485_JKBMS;
}

static inline bool bridgeProtocolIsRs485Pylon(uint8_t protocol)
{
    return bridgeProtocolCanonical(protocol) == PROTOCOL_RS485_PYLON;
}

static inline uint32_t bridgeProtocolRs485Baudrate(uint8_t protocol)
{
    switch (protocol) {
        case PROTOCOL_RS485_JKBMS_115200:
        case PROTOCOL_RS485_PYLON_115200:
            return RS485_FAST_BAUDRATE;
        case PROTOCOL_RS485_SEPLOS_19200:
            return 19200u;
        default:
            return RS485_DEFAULT_BAUDRATE;
    }
}

static inline bool bridgeProtocolIsCanJkbms250k(uint8_t protocol)
{
    return protocol == PROTOCOL_CAN_JKBMS_250K;
}

static inline uint32_t bridgeProtocolCanBitrate(uint8_t protocol)
{
    if (protocol == PROTOCOL_CAN_DALY) {
        return CAN_JKBMS_250K_BITRATE;
    }
    return bridgeProtocolIsCanJkbms250k(protocol) ? CAN_JKBMS_250K_BITRATE : CAN_DEFAULT_BITRATE;
}

#define BMS_line LINE_RS485
#define Inverter_line LINE_CAN
#define BMS_protocol PROTOCOL_RS485_DALY
#define Inverter_protocol PROTOCOL_CAN_PYLON
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
#define JKBMS_BMS_TASK_STACK           6144
#define JKBMS_BMS_TASK_PRIORITY        10

/* --- JKBMS native RS485 task --- */
#define JKBMS_RS485_NATIVE_QUERY_PERIOD_MS 250
#define JKBMS_RS485_NATIVE_TASK_STACK      6144
#define JKBMS_RS485_NATIVE_TASK_PRIORITY   10

/* --- PACE RS485 Modbus task --- */
#define PACE_BMS_MODBUS_SLAVE_ADDR     0x01u
#define PACE_BMS_MODBUS_GAP_US         5000
#define PACE_BMS_QUERY_PERIOD_MS       250
#define PACE_BMS_PUBLISH_PERIOD_MS     250
#define PACE_BMS_TASK_STACK            4096
#define PACE_BMS_TASK_PRIORITY         10

/* --- Voltronic Inverter and BMS 485 protocol task (JK UART protocol 007) --- */
#define VOLTRONIC_BMS_MODBUS_SLAVE_ADDR    0x01u
#define VOLTRONIC_BMS_MODBUS_GAP_US        5000
#define VOLTRONIC_BMS_QUERY_PERIOD_MS      250
#define VOLTRONIC_BMS_PUBLISH_PERIOD_MS    250
#define VOLTRONIC_BMS_TASK_STACK           4096
#define VOLTRONIC_BMS_TASK_PRIORITY        10

/* --- China Tower shared battery cabinet protocol task (JK UART protocol 008) --- */
#define CHINA_TOWER_BMS_MODBUS_SLAVE_ADDR  0x01u
#define CHINA_TOWER_BMS_MODBUS_GAP_US      5000
#define CHINA_TOWER_BMS_QUERY_PERIOD_MS    250
#define CHINA_TOWER_BMS_PUBLISH_PERIOD_MS  250
#define CHINA_TOWER_BMS_TASK_STACK         4096
#define CHINA_TOWER_BMS_TASK_PRIORITY      10

/* --- WOW RS485 Modbus protocol task (JK UART protocol 009 / SRNE-WOW profile) --- */
#define WOW_BMS_MODBUS_SLAVE_ADDR          0x01u
#define WOW_BMS_MODBUS_GAP_US              5000
#define WOW_BMS_QUERY_PERIOD_MS            250
#define WOW_BMS_PUBLISH_PERIOD_MS          250
#define WOW_BMS_TASK_STACK                 4096
#define WOW_BMS_TASK_PRIORITY              10

/* --- Seplos RS485 ASCII protocol task --- */
#define SEPLOS_BMS_ADDRESS                 0x00u
#define SEPLOS_BMS_REQUEST_INFO            0x00u
#define SEPLOS_BMS_REQUEST_CANDIDATE_PERIOD 2u
#define SEPLOS_BMS_RAW_LOGS                1
#define SEPLOS_BMS_QUERY_PERIOD_MS         500
#define SEPLOS_BMS_TASK_STACK              6144
#define SEPLOS_BMS_TASK_PRIORITY           10

/* --- Daly RS485 proprietary protocol task --- */
#define DALY_RS485_BMS_ID                  1u
#define DALY_RS485_QUERY_PERIOD_MS         120
#define DALY_RS485_RESPONSE_TIMEOUT_MS     220
#define DALY_RS485_PUBLISH_PERIOD_MS       500
#define DALY_RS485_SOURCE_STALE_MS         10000
#define DALY_RS485_TASK_STACK              6144
#define DALY_RS485_TASK_PRIORITY           10

/* --- Daly CAN proprietary protocol task --- */
#define DALY_CAN_BMS_ID                    1u
#define DALY_CAN_QUERY_PERIOD_MS           120
#define DALY_CAN_RESPONSE_TIMEOUT_MS       220
#define DALY_CAN_TX_TIMEOUT_MS             50
#define DALY_CAN_PUBLISH_PERIOD_MS         500
#define DALY_CAN_SOURCE_STALE_MS           10000
#define DALY_CAN_TASK_STACK                6144
#define DALY_CAN_TASK_PRIORITY             10

/* --- Pylon placeholders --- */
#define PYLON_BMS_TASK_STACK           3072
#define PYLON_BMS_TASK_PRIORITY        8
#define PYLON_INVERTER_TASK_STACK      3072
#define PYLON_INVERTER_TASK_PRIORITY   8
#define PYLON_PLACEHOLDER_TASK_PERIOD_MS 1000

/* --- EASUN Pylon CAN 24V diagnostic sender ---
 * Sends a controlled 8S/24V Pylon frame set on CAN2 while sniffer mode is active.
 * This is intentionally not a CAN1->CAN2 forwarder, because JK Pylon CAN is 16S/48V.
 */
#define EASUN_PYLON_24V_DIAG_SENDER_ENABLE 0
#define EASUN_PYLON_24V_DIAG_TX_PERIOD_MS  1000
#define EASUN_PYLON_24V_DIAG_TASK_STACK    3072
#define EASUN_PYLON_24V_DIAG_TASK_PRIORITY 8

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
#define SNIFFER_RS485_GAP_US           30000

#define WORKING_MODE_HEX_PRINT_LIMIT   64
#define WORKING_MODE_SNAPSHOT_PERIOD_MS 5000
#define WORKING_MODE_SNAPSHOT_TASK_STACK 8192
#define WORKING_MODE_SNAPSHOT_TASK_PRIORITY 7

/* --- Web interface --- */
#define WEB_INTERFACE_ENABLE            1
#define WEB_INTERFACE_PORT              80
#define WEB_INTERFACE_TASK_STACK        8192
#define WEB_INTERFACE_TASK_PRIO         5
#define WIFI_STA_MAX_RETRY              10

#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID                   "CHANGE_ME_WIFI_SSID"
#endif

#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD               "CHANGE_ME_WIFI_PASSWORD"
#endif

#ifndef WIFI_STA_HOSTNAME
#define WIFI_STA_HOSTNAME               "esp32-bridge"
#endif

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
