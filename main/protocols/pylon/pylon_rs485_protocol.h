#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PYLON_RS485_MAX_CELLS 32u

typedef struct {
    bool valid42;
    bool valid61;
    bool valid62;
    bool valid63;
    char info42[256];
    char info61[128];
    char info62[32];
    char info63[64];
} pylon_rs485_cache_t;

typedef struct {
    bool valid;
    float current_a;
    uint16_t pack_voltage_cv;
    uint16_t cycles;
    uint8_t soc_pct;
    uint8_t soh_pct;
    uint16_t raw_word0;
    uint16_t max_cell_mv;
    uint16_t min_cell_mv;
    uint8_t max_cell_idx;
    uint8_t min_cell_idx;
    uint8_t cell_count;
    uint16_t cell_mv[PYLON_RS485_MAX_CELLS];
    int16_t temp_mos_c10;
    int16_t temp_t1_c10;
    int16_t temp_t2_c10;
    int16_t temp_t4_c10;
    int16_t temp_t5_c10;
    uint8_t status_63;
} pylon_rs485_summary_t;
