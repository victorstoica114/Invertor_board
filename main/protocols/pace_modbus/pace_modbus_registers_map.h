#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t start;
    uint16_t count;
} pace_modbus_poll_block_t;

/* PACE BMS Modbus Protocol for RS485 V1.3 (2017-06-27). */
#define PACE_MB_REG_CURRENT_10MA           0x0000u
#define PACE_MB_REG_PACK_VOLTAGE_10MV      0x0001u
#define PACE_MB_REG_SOC_PCT                0x0002u
#define PACE_MB_REG_SOH_PCT                0x0003u
#define PACE_MB_REG_REMAIN_CAP_10MAH       0x0004u
#define PACE_MB_REG_FULL_CAP_10MAH         0x0005u
#define PACE_MB_REG_DESIGN_CAP_10MAH       0x0006u
#define PACE_MB_REG_CYCLE_COUNT            0x0007u
#define PACE_MB_REG_WARNING_FLAGS          0x0009u
#define PACE_MB_REG_PROTECTION_FLAGS       0x000Au
#define PACE_MB_REG_STATUS_FLAGS           0x000Bu
#define PACE_MB_REG_BALANCE_STATUS         0x000Cu

#define PACE_MB_STATUS_CHG                 0x0100u
#define PACE_MB_STATUS_DCHG                0x0200u
#define PACE_MB_STATUS_MOSFET_CHG          0x0400u
#define PACE_MB_STATUS_MOSFET_DCHG         0x0800u

#define PACE_MB_REG_CELL01_MV              0x000Fu
#define PACE_MB_REG_CELL16_MV              0x001Eu
#define PACE_MB_CELL_COUNT                 16u

#define PACE_MB_REG_TEMP1_DECIC            0x001Fu
#define PACE_MB_REG_TEMP2_DECIC            0x0020u
#define PACE_MB_REG_TEMP3_DECIC            0x0021u
#define PACE_MB_REG_TEMP4_DECIC            0x0022u
#define PACE_MB_REG_MOS_TEMP_DECIC         0x0023u
#define PACE_MB_REG_ENV_TEMP_DECIC         0x0024u
#define PACE_MB_TEMP_REG_COUNT             6u

extern const pace_modbus_poll_block_t g_paceModbusPollBlocks[];
extern const size_t g_paceModbusPollBlocksCount;
