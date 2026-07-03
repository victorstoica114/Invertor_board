#pragma once

#include <stdint.h>

/*
 * Pylon CAN frame map (as observed on Seplos/Pylon-compatible batteries).
 * Units:
 * - voltage/current limits in 0.1 V / 0.1 A (little-endian u16)
 * - SOC/SOH in %
 * - pack voltage in 0.01 V (some variants may send half-pack)
 * - temperatures in 0.1 C
 * - cell voltages in mV
 */

#define PYLON_CAN_ID_LIMITS_351         0x351u
#define PYLON_CAN_ID_SOC_SOH_355        0x355u
#define PYLON_CAN_ID_PACK_356           0x356u
#define PYLON_CAN_ID_MODULE_INFO_359    0x359u
#define PYLON_CAN_ID_VENDOR_INFO_35A    0x35Au
#define PYLON_CAN_ID_STATUS_35C         0x35Cu
#define PYLON_CAN_ID_ASCII_ID_35E       0x35Eu
#define PYLON_CAN_ID_JK_EXT_CELL_370    0x370u
#define PYLON_CAN_ID_JK_EXT_INDEX_371   0x371u
#define PYLON_CAN_ID_MISC_372           0x372u
#define PYLON_CAN_ID_CELL_TEMP_373      0x373u
#define PYLON_CAN_ID_ASCII_374          0x374u
#define PYLON_CAN_ID_ASCII_375          0x375u
#define PYLON_CAN_ID_ASCII_376          0x376u
#define PYLON_CAN_ID_ASCII_377          0x377u
#define PYLON_CAN_ID_MISC_379           0x379u

#define PYLON_CAN_ID_MIN                PYLON_CAN_ID_LIMITS_351
#define PYLON_CAN_ID_MAX                PYLON_CAN_ID_MISC_379
#define PYLON_CAN_CACHE_COUNT           (PYLON_CAN_ID_MAX - PYLON_CAN_ID_MIN + 1u)

/* 0x351 offsets */
#define PYLON_CAN_351_OFF_CHG_VLIM_DV   0u
#define PYLON_CAN_351_OFF_CHG_ILIM_DA   2u
#define PYLON_CAN_351_OFF_DIS_ILIM_DA   4u
#define PYLON_CAN_351_OFF_DIS_VLIM_DV   6u

/* 0x355 offsets */
#define PYLON_CAN_355_OFF_SOC_PCT       0u
#define PYLON_CAN_355_OFF_SOH_PCT       2u

/* 0x356 offsets */
#define PYLON_CAN_356_OFF_PACK_V_CV     0u
#define PYLON_CAN_356_OFF_PACK_I_DA     2u
#define PYLON_CAN_356_OFF_TEMP_DECIC    4u

/* 0x373 offsets */
#define PYLON_CAN_373_OFF_CELL_MIN_MV   0u
#define PYLON_CAN_373_OFF_CELL_MAX_MV   2u
#define PYLON_CAN_373_OFF_TEMP1_DECIC   4u
#define PYLON_CAN_373_OFF_TEMP2_DECIC   6u

/*
 * Optional JK/Pylon extension observed on JK-BMS CAN Pylon profile.
 * 0x370 uses raw degrees C for small temperature values and mV for cell
 * extremes. 0x371 carries the related sensor/cell indices.
 */
#define PYLON_CAN_370_OFF_TEMP_MAX_RAW  0u
#define PYLON_CAN_370_OFF_TEMP_MIN_RAW  2u
#define PYLON_CAN_370_OFF_CELL_MAX_MV   4u
#define PYLON_CAN_370_OFF_CELL_MIN_MV   6u

#define PYLON_CAN_371_OFF_TEMP_MAX_SENS 0u
#define PYLON_CAN_371_OFF_TEMP_MIN_SENS 2u
#define PYLON_CAN_371_OFF_CELL_MAX_IDX  4u
#define PYLON_CAN_371_OFF_CELL_MIN_IDX  6u

/*
 * Pylon RS485 is an ASCII CID2 payload protocol in this codebase, not a
 * Modbus register map. Keep RS485 payload structures in pylon_rs485_protocol.h
 * and keep this file focused on CAN frame IDs/offsets.
 */
