#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JKBMS_CAN_ID_BATT_ST   0x02F4u
#define JKBMS_CAN_ID_CELL_VOLT 0x04F4u
#define JKBMS_CAN_ID_CELL_TEMP 0x05F4u
#define JKBMS_CAN_ID_ALM_INFO  0x07F4u
#define JKBMS_CAN_ID_CELL_VOLT_EXT_BASE 0x18E028F4u
#define JKBMS_CAN_ID_CELL_VOLT_EXT_LAST 0x18E628F4u
#define JKBMS_CAN_CELL_VOLT_EXT_FRAMES 7u
#define JKBMS_CAN_ID_CELL_TEMP_EXT 0x18F228F4u
#define JKBMS_CAN_CACHE_COUNT  12u

typedef struct {
    bool valid;
    uint32_t id;
    uint32_t updatedMs;
    uint8_t dlc;
    uint8_t data[8];
} jkbms_can_frame_t;

int jkbmsCanCacheIndex(uint32_t id);
bool jkbmsCanAnyValid(const jkbms_can_frame_t *cache, size_t count);
bool jkbmsCanTryGetSocPct(const jkbms_can_frame_t *cache, size_t count, uint8_t *socOut);
void jkbmsCanUpdateBatteryModel(const char *ifname, const jkbms_can_frame_t *cache, size_t count);
void jkbmsCanDecodeSnapshot(const char *ifname, const jkbms_can_frame_t *cache, size_t count);

#ifdef __cplusplus
}
#endif
