#include "protocols/rs485_growatt/rs485_growatt_registers_map.h"

const rs485_growatt_poll_block_t g_rs485GrowattPollBlocks[] = {
    { .start = RS485_GROWATT_MB_REG_INFO_0001,  .count = 0x000Fu },
    { .start = RS485_GROWATT_MB_REG_MAIN_START, .count = (RS485_GROWATT_MB_REG_MAIN_END - RS485_GROWATT_MB_REG_MAIN_START + 1u) },
    { .start = RS485_GROWATT_MB_REG_CELL_BASE,  .count = 0x0011u },
};

const size_t g_rs485GrowattPollBlocksCount =
    sizeof(g_rs485GrowattPollBlocks) / sizeof(g_rs485GrowattPollBlocks[0]);
