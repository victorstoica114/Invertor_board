#pragma once

#include <stddef.h>

#include "freertos/FreeRTOS.h"

typedef void *QueueHandle_t;
typedef int BaseType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY ((TickType_t)-1)

static inline QueueHandle_t xQueueCreate(size_t queue_length, size_t item_size)
{
    (void)queue_length;
    (void)item_size;
    return (QueueHandle_t)1;
}

static inline BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks_to_wait)
{
    (void)queue;
    (void)item;
    (void)ticks_to_wait;
    return pdFALSE;
}

static inline BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item)
{
    (void)queue;
    (void)item;
    return pdPASS;
}

static inline void vQueueDelete(QueueHandle_t queue)
{
    (void)queue;
}
