#include "protocols/rs485_growatt/rs485_growatt_registers_map.h"

const rs485_growatt_poll_block_t g_rs485GrowattPollBlocks[] = {
    /*
     * Keep the fast poll cycle to telemetry blocks that are required by the
     * bridge and have been observed answering reliably on JK/Growatt485.
     * Device-info reads are intentionally left out of the runtime cycle:
     * several BMS firmwares ignore them, which creates persistent unanswered
     * Modbus requests and can starve useful telemetry on a half-duplex bus.
     */
    { .start = 0x0010u, .count = 0x000Fu },
    { .start = 0x0020u, .count = 0x000Bu },
    { .start = RS485_GROWATT_MB_REG_CELL_BASE,  .count = RS485_GROWATT_MB_CELL_COUNT },
};

const size_t g_rs485GrowattPollBlocksCount =
    sizeof(g_rs485GrowattPollBlocks) / sizeof(g_rs485GrowattPollBlocks[0]);
