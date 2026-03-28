#pragma once

#include <stdint.h>

/*
 * Deye CAN mapping (observed JK-BMS compatible dialect).
 * Many frames overlap Pylon CAN IDs but semantics can differ by bytes.
 */

#define DEYE_CAN_ID_LIMITS_351          0x351u
#define DEYE_CAN_ID_SOC_SOH_355         0x355u
#define DEYE_CAN_ID_PACK_356            0x356u
#define DEYE_CAN_ID_MODULE_INFO_359     0x359u
#define DEYE_CAN_ID_STATUS_35C          0x35Cu
#define DEYE_CAN_ID_ASCII_ID_35E        0x35Eu
#define DEYE_CAN_ID_TEMP_CELL_370       0x370u
#define DEYE_CAN_ID_SENSOR_INDEX_371    0x371u

#define DEYE_CAN_351_OFF_CHG_VLIM_DV    0u
#define DEYE_CAN_351_OFF_CHG_ILIM_DA    2u
#define DEYE_CAN_351_OFF_DIS_ILIM_DA    4u
#define DEYE_CAN_351_OFF_DIS_VLIM_DV    6u

#define DEYE_CAN_355_OFF_SOC_PCT        0u
#define DEYE_CAN_355_OFF_SOH_PCT        2u

#define DEYE_CAN_356_OFF_PACK_V_CV      0u
#define DEYE_CAN_356_OFF_PACK_I_DA      2u
#define DEYE_CAN_356_OFF_TEMP_DECIC     4u

#define DEYE_CAN_359_OFF_MODULE_COUNT   4u
#define DEYE_CAN_359_OFF_TAG            5u

#define DEYE_CAN_370_OFF_TEMP_MAX_RAW   0u
#define DEYE_CAN_370_OFF_TEMP_MIN_RAW   2u
#define DEYE_CAN_370_OFF_CELL_MAX_MV    4u
#define DEYE_CAN_370_OFF_CELL_MIN_MV    6u

#define DEYE_CAN_371_OFF_TEMP_MAX_SENS  0u
#define DEYE_CAN_371_OFF_TEMP_MIN_SENS  2u
#define DEYE_CAN_371_OFF_CELL_MAX_IDX   4u
#define DEYE_CAN_371_OFF_CELL_MIN_IDX   6u
