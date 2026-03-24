#pragma once

#include <stdint.h>

/*
 * Placeholder map for Pylon protocol integration.
 * Values are intentionally conservative and should be replaced once
 * protocol details are provided.
 */

#define PYLON_MODBUS_DEFAULT_SLAVE_ADDR   0x01u

#define PYLON_MB_REG_SOC_PCT              0x0000u
#define PYLON_MB_REG_TEMP_C               0x0001u
#define PYLON_MB_REG_PACK_V_CV            0x0002u
#define PYLON_MB_REG_CELL_MIN_MV          0x0003u
#define PYLON_MB_REG_CELL_MAX_MV          0x0004u
#define PYLON_MB_REG_CELL_MIN_IDX         0x0005u
#define PYLON_MB_REG_CELL_MAX_IDX         0x0006u
