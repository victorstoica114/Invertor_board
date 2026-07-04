#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "pylon_registers_map.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    uint32_t id;
    uint32_t updatedMs;
    uint8_t dlc;
    uint8_t data[8];
} pylon_can_frame_t;

bool pylonCanAnyValid(const pylon_can_frame_t *cache, size_t count);
void pylonCanDecodeSnapshot(const char *ifname, const pylon_can_frame_t *cache, size_t count);
void pylonCanDecodeSnapshotWithProtocol(const char *ifname,
                                        const pylon_can_frame_t *cache,
                                        size_t count,
                                        const char *protocolLabel);

#ifdef __cplusplus
}
#endif
