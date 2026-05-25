#include "protocols/rs485_growatt/rs485_growatt_registers_map.h"

const rs485_growatt_poll_block_t g_rs485GrowattPollBlocks[] = {
    { .start = RS485_GROWATT_MB_REG_INFO_0001,  .count = 0x000Fu },
    /* Seplos Growatt485 rejects the original 0x0010..0x002A bulk read. */
    { .start = 0x0010u, .count = 0x0008u },
    { .start = 0x0018u, .count = 0x0008u },
    { .start = 0x0020u, .count = 0x000Bu },
    { .start = RS485_GROWATT_MB_REG_CELL_BASE,  .count = RS485_GROWATT_MB_CELL_COUNT },
};

const size_t g_rs485GrowattPollBlocksCount =
    sizeof(g_rs485GrowattPollBlocks) / sizeof(g_rs485GrowattPollBlocks[0]);
