#include "protocols/growatt/growatt_registers_map.h"

const growatt_modbus_poll_block_t g_growattModbusPollBlocks[] = {
    { .start = GROWATT_MB_REG_INFO_0001,  .count = 0x000Fu },
    { .start = GROWATT_MB_REG_MAIN_START, .count = (GROWATT_MB_REG_MAIN_END - GROWATT_MB_REG_MAIN_START + 1u) },
    { .start = GROWATT_MB_REG_CELL_BASE,  .count = GROWATT_MB_CELL_COUNT },
};

const size_t g_growattModbusPollBlocksCount =
    sizeof(g_growattModbusPollBlocks) / sizeof(g_growattModbusPollBlocks[0]);
