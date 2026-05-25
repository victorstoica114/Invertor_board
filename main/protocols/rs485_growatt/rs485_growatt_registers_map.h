#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t start;
    uint16_t count;
} rs485_growatt_poll_block_t;

#define RS485_GROWATT_MODBUS_DEFAULT_SLAVE_ADDR 0x01u

/* Device info block: start 0x0001, count 0x000F */
#define RS485_GROWATT_MB_REG_INFO_0001 0x0001u
#define RS485_GROWATT_MB_REG_INFO_0002 0x0002u
#define RS485_GROWATT_MB_REG_INFO_0003 0x0003u
#define RS485_GROWATT_MB_REG_INFO_0004 0x0004u

/* Main status block: start 0x0010, count 0x001B (up to 0x002A) */
#define RS485_GROWATT_MB_REG_MAIN_START 0x0010u
#define RS485_GROWATT_MB_REG_MAIN_END   0x002Au

#define RS485_GROWATT_MB_REG_STATUS_FLAGS            0x0013u
#define RS485_GROWATT_MB_REG_ERROR_CODE              0x0014u
#define RS485_GROWATT_MB_REG_SOC_PCT                 0x0015u
#define RS485_GROWATT_MB_REG_PACK_V_CV               0x0016u
#define RS485_GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE 0x0017u
#define RS485_GROWATT_MB_REG_TEMP_C                  0x0018u
#define RS485_GROWATT_MB_REG_MAX_CHG_DISCHG_CA       0x0019u
#define RS485_GROWATT_MB_REG_REMAIN_CAP_CAH          0x001Au
#define RS485_GROWATT_MB_REG_FULL_CAP_CAH            0x001Bu
#define RS485_GROWATT_MB_REG_CYCLE_COUNT             0x001Eu
#define RS485_GROWATT_MB_REG_SOH_PCT                 0x0020u
#define RS485_GROWATT_MB_REG_CV_TARGET_CV            0x0021u
#define RS485_GROWATT_MB_REG_WARNING_CODE            0x0022u
#define RS485_GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE   0x0023u
#define RS485_GROWATT_MB_REG_EXTENDED_ERROR_CODE     0x0024u
#define RS485_GROWATT_MB_REG_CELL_MAX_MV             0x0025u
#define RS485_GROWATT_MB_REG_CELL_MIN_MV             0x0026u
#define RS485_GROWATT_MB_REG_CELL_MAX_IDX            0x0027u
#define RS485_GROWATT_MB_REG_CELL_MIN_IDX            0x0028u

#define RS485_GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE   RS485_GROWATT_MB_REG_CYCLE_COUNT
#define RS485_GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE   RS485_GROWATT_MB_REG_MAX_CHG_DISCHG_CA

/* Cell voltage block: 0x0070 is a header/reserved word; cells are 0x0071..0x0080. */
#define RS485_GROWATT_MB_REG_CELL_HEADER 0x0070u
#define RS485_GROWATT_MB_REG_CELL_BASE  0x0071u
#define RS485_GROWATT_MB_CELL_COUNT     16u
#define RS485_GROWATT_MB_REG_CELL_LAST  0x0080u

extern const rs485_growatt_poll_block_t g_rs485GrowattPollBlocks[];
extern const size_t g_rs485GrowattPollBlocksCount;
