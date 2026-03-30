#pragma once

#include <stdint.h>

/*
 * Sofar CAN frame map used by Seplos BMS (Sofar-compatible profile).
 * Observed payload is little-endian.
 *
 * Notes:
 * - 0x351 / 0x355 / 0x356 / 0x35C are aligned with the common low-voltage
 *   battery CAN profile used by multiple inverter vendors.
 * - 0x370 / 0x371 carry temperature/cell-state style payloads on this profile.
 */

#define SOFAR_CAN_ID_LIMITS_351         0x351u
#define SOFAR_CAN_ID_SOC_SOH_355        0x355u
#define SOFAR_CAN_ID_PACK_356           0x356u
#define SOFAR_CAN_ID_MODULE_INFO_359    0x359u
#define SOFAR_CAN_ID_STATUS_35C         0x35Cu
#define SOFAR_CAN_ID_BRAND_35E          0x35Eu
#define SOFAR_CAN_ID_MODULE_35F         0x35Fu
#define SOFAR_CAN_ID_MISC_370           0x370u
#define SOFAR_CAN_ID_MISC_371           0x371u

/* 0x351 offsets */
#define SOFAR_CAN_351_OFF_CHG_VLIM_DV   0u
#define SOFAR_CAN_351_OFF_CHG_ILIM_DA   2u
#define SOFAR_CAN_351_OFF_DIS_ILIM_DA   4u
#define SOFAR_CAN_351_OFF_DIS_VLIM_DV   6u

/* 0x355 offsets */
#define SOFAR_CAN_355_OFF_SOC_PCT       0u
#define SOFAR_CAN_355_OFF_SOH_PCT       2u

/* 0x356 offsets */
#define SOFAR_CAN_356_OFF_PACK_V_CV     0u
#define SOFAR_CAN_356_OFF_PACK_I_DA     2u
#define SOFAR_CAN_356_OFF_TEMP_DECIC    4u
