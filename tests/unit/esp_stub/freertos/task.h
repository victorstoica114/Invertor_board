#pragma once

typedef void *TaskHandle_t;

static inline void vTaskDelay(int ticks) { (void)ticks; }

static inline int xTaskCreate(void (*task)(void *),
                              const char *name,
                              unsigned stack_depth,
                              void *param,
                              unsigned priority,
                              TaskHandle_t *task_handle)
{
    (void)task;
    (void)name;
    (void)stack_depth;
    (void)param;
    (void)priority;
    if (task_handle != 0) {
        *task_handle = (TaskHandle_t)1;
    }
    return 1;
}

static inline void vTaskDelete(TaskHandle_t task)
{
    (void)task;
}
