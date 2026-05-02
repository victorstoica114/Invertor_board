#pragma once

#include <stdint.h>

typedef void *twai_handle_t;

typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
} twai_message_t;

static inline int twai_clear_receive_queue_v2(twai_handle_t handle)
{
    (void)handle;
    return 0;
}
