#pragma once

#include <stdint.h>

/*
 * GoodWe CAN frame map (as implemented by Seplos/GoodWe-compatible BMS stacks).
 * Payload is little-endian.
 *
 * Units:
 * - 0x456: limits in 0.1 V / 0.1 A
 * - 0x457: SOC/SOH in 0.1 %
 * - 0x458: pack voltage in 0.1 V, current in 0.1 A, temp in 0.1 C
 */

#define GOODWE_CAN_ID_MODULES_453        0x453u
#define GOODWE_CAN_ID_ALARMS_455         0x455u
#define GOODWE_CAN_ID_LIMITS_456         0x456u
#define GOODWE_CAN_ID_SOC_SOH_457        0x457u
#define GOODWE_CAN_ID_PACK_458           0x458u

/* 0x456 offsets */
#define GOODWE_CAN_456_OFF_CHG_VLIM_DV   0u
#define GOODWE_CAN_456_OFF_CHG_ILIM_DA   2u
#define GOODWE_CAN_456_OFF_DIS_ILIM_DA   4u
#define GOODWE_CAN_456_OFF_DIS_VLIM_DV   6u

/* 0x457 offsets */
#define GOODWE_CAN_457_OFF_SOC_DECIPCT   0u
#define GOODWE_CAN_457_OFF_SOH_DECIPCT   2u

/* 0x458 offsets */
#define GOODWE_CAN_458_OFF_PACK_V_DV     0u
#define GOODWE_CAN_458_OFF_PACK_I_DA     2u
#define GOODWE_CAN_458_OFF_TEMP_DECIC    4u
