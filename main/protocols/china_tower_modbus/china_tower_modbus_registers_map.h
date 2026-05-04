#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t start;
    uint16_t count;
} china_tower_modbus_poll_block_t;

/*
 * China Tower shared battery cabinet RS485 profile used by JK UART protocol
 * 008. Public material for V2.0 is sparse; this map follows the live JK app
 * layout observed on the bench: summary registers at 0x0000 and per-cell
 * millivolts starting at 0x0009.
 */
#define CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV      0x0000u
#define CHINA_TOWER_MB_REG_CELL_COUNT             0x0001u
#define CHINA_TOWER_MB_REG_SOC_PCT                0x0002u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_0003       0x0003u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_0004       0x0004u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_0005       0x0005u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_0006       0x0006u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_0007       0x0007u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_0008       0x0008u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_0009       0x0009u
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_000A       0x000Au
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_000B       0x000Bu
#define CHINA_TOWER_MB_REG_RUNTIME_RAW_000C       0x000Cu

#define CHINA_TOWER_MB_STATUS_CHG                 0x0100u
#define CHINA_TOWER_MB_STATUS_DCHG                0x0200u
#define CHINA_TOWER_MB_STATUS_MOSFET_CHG          0x0400u
#define CHINA_TOWER_MB_STATUS_MOSFET_DCHG         0x0800u

#define CHINA_TOWER_MB_REG_CELL01_MV              0x0009u
#define CHINA_TOWER_MB_REG_CELL16_MV              0x0018u
#define CHINA_TOWER_MB_CELL_COUNT                 16u

#define CHINA_TOWER_MB_REG_TEMP1_DECIC            0x001Fu
#define CHINA_TOWER_MB_REG_TEMP2_DECIC            0x0020u
#define CHINA_TOWER_MB_REG_TEMP3_DECIC            0x0021u
#define CHINA_TOWER_MB_REG_TEMP4_DECIC            0x0022u
#define CHINA_TOWER_MB_REG_MOS_TEMP_DECIC         0x0023u
#define CHINA_TOWER_MB_REG_ENV_TEMP_DECIC         0x0024u
#define CHINA_TOWER_MB_TEMP_REG_COUNT             6u

extern const china_tower_modbus_poll_block_t g_chinaTowerModbusPollBlocks[];
extern const size_t g_chinaTowerModbusPollBlocksCount;
