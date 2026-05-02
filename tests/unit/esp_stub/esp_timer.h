#pragma once

#include <stdint.h>
#include <time.h>

// In ESP-IDF this returns time since boot in microseconds.
static inline int64_t esp_timer_get_time(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)(ts.tv_nsec / 1000);
}
