#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t start;
    uint16_t count;
} growatt_modbus_poll_block_t;

/* Default slave address used for polling the BMS over Modbus RTU. */
#define GROWATT_MODBUS_DEFAULT_SLAVE_ADDR      0x01u

/* ---------------- CAN IDs ---------------- */
#define GROWATT_CAN_ID_CTRL_EVENT_TRIGGER      0x211u
#define GROWATT_CAN_ID_CTRL_QUERY              0x212u

#define GROWATT_CAN_ID_311_STATUS_LIMITS       0x311u
#define GROWATT_CAN_ID_312_PROT_ALM            0x312u
#define GROWATT_CAN_ID_313_V_I_SOC_SOH         0x313u
#define GROWATT_CAN_ID_314_RM_FCC_DV_CYCLES    0x314u

/* Optional/triggered frames (cells or extended telemetry on some BMS variants) */
#define GROWATT_CAN_ID_315_CELL_GRP1           0x315u
#define GROWATT_CAN_ID_316_CELL_GRP2           0x316u
#define GROWATT_CAN_ID_317_CELL_GRP3           0x317u
#define GROWATT_CAN_ID_318_CELL_GRP4           0x318u

/* Reference/metadata frames */
#define GROWATT_CAN_ID_319_CELL_REF_FLAGS      0x319u
#define GROWATT_CAN_ID_320_MAKER_SW            0x320u
#define GROWATT_CAN_ID_321_UPGRADE_INFO        0x321u
#define GROWATT_CAN_ID_322_TEMP_SOC_MIN_MAX    0x322u
#define GROWATT_CAN_ID_323_CELLCOUNT_PROT_WARN 0x323u

/* Optional/triggered status extension */
#define GROWATT_CAN_ID_324_EXT1                0x324u
#define GROWATT_CAN_ID_325_EXT2                0x325u

/* Cache window currently tracked for periodic CAN snapshot */
#define GROWATT_CAN_CACHE_ID_MIN               GROWATT_CAN_ID_311_STATUS_LIMITS
#define GROWATT_CAN_CACHE_ID_MAX               GROWATT_CAN_ID_323_CELLCOUNT_PROT_WARN

/* ---------------- RS485 (Modbus) Registers ---------------- */

/* Device info block: start 0x0001, count 0x000F */
#define GROWATT_MB_REG_INFO_0001               0x0001u
#define GROWATT_MB_REG_INFO_0002               0x0002u
#define GROWATT_MB_REG_INFO_0003               0x0003u
#define GROWATT_MB_REG_INFO_0004               0x0004u

/* Main status block: start 0x0010, count 0x001B (up to 0x002A) */
#define GROWATT_MB_REG_MAIN_START              0x0010u
#define GROWATT_MB_REG_MAIN_END                0x002Au

#define GROWATT_MB_REG_STATUS_FLAGS            0x0013u /* raw status/flags (vendor specific) */
#define GROWATT_MB_REG_SOC_PCT                 0x0015u /* SOC (%) */
#define GROWATT_MB_REG_PACK_V_CV               0x0016u /* pack voltage (centivolts) */
#define GROWATT_MB_REG_PACK_I_ABS_CA_TENTATIVE 0x0017u /* tentative: |pack current| (0.01A) */
#define GROWATT_MB_REG_TEMP_C                  0x0018u /* pack temperature (C) */
#define GROWATT_MB_REG_CYCLE_COUNT_TENTATIVE   0x0019u /* tentative: cycle count */
#define GROWATT_MB_REG_REMAIN_CAP_CAH          0x001Au /* remaining capacity (0.01Ah) */
#define GROWATT_MB_REG_FULL_CAP_CAH            0x001Bu /* full capacity / FCC (0.01Ah) */
#define GROWATT_MB_REG_SOH_PCT                 0x0020u /* SOH (%) */
#define GROWATT_MB_REG_CV_TARGET_CV            0x0021u /* charge-voltage target (centivolts) */
#define GROWATT_MB_REG_ICHG_LIM_CA_TENTATIVE   0x0022u /* tentative: charge current limit (0.01A) */
#define GROWATT_MB_REG_IDIS_LIM_CA_TENTATIVE   0x0023u /* tentative: discharge current limit (0.01A) */
#define GROWATT_MB_REG_CELL_MAX_MV             0x0025u /* max cell voltage (mV) */
#define GROWATT_MB_REG_CELL_MIN_MV             0x0026u /* min cell voltage (mV) */
#define GROWATT_MB_REG_CELL_MAX_IDX            0x0027u /* cell index of max voltage */
#define GROWATT_MB_REG_CELL_MIN_IDX            0x0028u /* cell index of min voltage */

/* Backward compatibility name kept for existing code/config references. */
#define GROWATT_MB_REG_MAIN_RAW_0013           GROWATT_MB_REG_STATUS_FLAGS

/* Cell voltage block: 0x0070 is a header/reserved word; cells are 0x0071..0x0080. */
#define GROWATT_MB_REG_CELL_HEADER             0x0070u
#define GROWATT_MB_REG_CELL_BASE               0x0071u
#define GROWATT_MB_CELL_COUNT                  16u
#define GROWATT_MB_REG_CELL_LAST               0x0080u
#define GROWATT_MB_REG_CELL_EXTRA              GROWATT_MB_REG_CELL_HEADER /* legacy alias */

/* Helper macro: cell N register (N: 1..16) */
#define GROWATT_MB_REG_CELL_N(n)               (GROWATT_MB_REG_CELL_BASE + ((n) - 1u))

#define GROWATT_MB_REG_CELL01_MV               GROWATT_MB_REG_CELL_N(1u)
#define GROWATT_MB_REG_CELL02_MV               GROWATT_MB_REG_CELL_N(2u)
#define GROWATT_MB_REG_CELL03_MV               GROWATT_MB_REG_CELL_N(3u)
#define GROWATT_MB_REG_CELL04_MV               GROWATT_MB_REG_CELL_N(4u)
#define GROWATT_MB_REG_CELL05_MV               GROWATT_MB_REG_CELL_N(5u)
#define GROWATT_MB_REG_CELL06_MV               GROWATT_MB_REG_CELL_N(6u)
#define GROWATT_MB_REG_CELL07_MV               GROWATT_MB_REG_CELL_N(7u)
#define GROWATT_MB_REG_CELL08_MV               GROWATT_MB_REG_CELL_N(8u)
#define GROWATT_MB_REG_CELL09_MV               GROWATT_MB_REG_CELL_N(9u)
#define GROWATT_MB_REG_CELL10_MV               GROWATT_MB_REG_CELL_N(10u)
#define GROWATT_MB_REG_CELL11_MV               GROWATT_MB_REG_CELL_N(11u)
#define GROWATT_MB_REG_CELL12_MV               GROWATT_MB_REG_CELL_N(12u)
#define GROWATT_MB_REG_CELL13_MV               GROWATT_MB_REG_CELL_N(13u)
#define GROWATT_MB_REG_CELL14_MV               GROWATT_MB_REG_CELL_N(14u)
#define GROWATT_MB_REG_CELL15_MV               GROWATT_MB_REG_CELL_N(15u)
#define GROWATT_MB_REG_CELL16_MV               GROWATT_MB_REG_CELL_N(16u)

extern const growatt_modbus_poll_block_t g_growattModbusPollBlocks[];
extern const size_t g_growattModbusPollBlocksCount;
