#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PYLON_CAN_ID_MIN 0x351u
#define PYLON_CAN_ID_MAX 0x379u
#define PYLON_CAN_CACHE_COUNT (PYLON_CAN_ID_MAX - PYLON_CAN_ID_MIN + 1u)

typedef struct {
    bool valid;
    uint32_t id;
    uint32_t updatedMs;
    uint8_t dlc;
    uint8_t data[8];
} pylon_can_frame_t;

bool pylonCanAnyValid(const pylon_can_frame_t *cache, size_t count);
void pylonCanDecodeSnapshot(const char *ifname, const pylon_can_frame_t *cache, size_t count);

#ifdef __cplusplus
}
#endif
