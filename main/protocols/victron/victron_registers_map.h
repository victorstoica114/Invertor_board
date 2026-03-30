#pragma once

#include <stdint.h>

/*
 * Victron CAN frame map (as emitted by Seplos in Victron profile).
 * Observed payload is little-endian and follows the same base layout
 * used by Pylon/SMA-like low-voltage battery profiles.
 *
 * Units:
 * - 0x351 limits in 0.1 V / 0.1 A
 * - 0x355 SOC/SOH in %
 * - 0x356 pack voltage in 0.01 V, current in 0.1 A, temperature in 0.1 C
 */

#define VICTRON_CAN_ID_LIMITS_351       0x351u
#define VICTRON_CAN_ID_SOC_SOH_355      0x355u
#define VICTRON_CAN_ID_PACK_356         0x356u
#define VICTRON_CAN_ID_ALARMS_359       0x359u
#define VICTRON_CAN_ID_VENDOR_35A       0x35Au
#define VICTRON_CAN_ID_MANUFACTURER_35E 0x35Eu
#define VICTRON_CAN_ID_BATTERY_35F      0x35Fu

/* 0x351 offsets */
#define VICTRON_CAN_351_OFF_CHG_VLIM_DV 0u
#define VICTRON_CAN_351_OFF_CHG_ILIM_DA 2u
#define VICTRON_CAN_351_OFF_DIS_ILIM_DA 4u
#define VICTRON_CAN_351_OFF_DIS_VLIM_DV 6u

/* 0x355 offsets */
#define VICTRON_CAN_355_OFF_SOC_PCT     0u
#define VICTRON_CAN_355_OFF_SOH_PCT     2u

/* 0x356 offsets */
#define VICTRON_CAN_356_OFF_PACK_V_CV   0u
#define VICTRON_CAN_356_OFF_PACK_I_DA   2u
#define VICTRON_CAN_356_OFF_TEMP_DECIC  4u
