#include "protocols/china_tower_modbus/china_tower_modbus_registers_map.h"

const china_tower_modbus_poll_block_t g_chinaTowerModbusPollBlocks[] = {
    { .start = CHINA_TOWER_MB_REG_PACK_VOLTAGE_10MV, .count = 0x000Du },
    { .start = CHINA_TOWER_MB_REG_CELL01_MV, .count = CHINA_TOWER_MB_CELL_COUNT },
    { .start = CHINA_TOWER_MB_REG_WARNING_FLAGS, .count = 3u },
};

const size_t g_chinaTowerModbusPollBlocksCount =
    sizeof(g_chinaTowerModbusPollBlocks) / sizeof(g_chinaTowerModbusPollBlocks[0]);
