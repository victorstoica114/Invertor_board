#include "protocols/jkbms_modbus/jkbms_modbus_register_map.h"

/*
 * Poll window #1:
 *   0x1200..0x123F includes cell voltage area from table (0x0000..0x003E).
 *
 * Poll window #2:
 *   0x128A..0x12B1 includes temp/pack voltage/current/SOC/cycles area.
 */
const jkbms_modbus_poll_block_t g_jkbmsModbusPollBlocks[] = {
    { .start = JKBMS_RT_REG_CELL0_MV,       .count = 0x0040u },
    { .start = JKBMS_RT_REG_TEMP_MOS_DECIC, .count = 0x0028u },
};

const size_t g_jkbmsModbusPollBlocksCount =
    sizeof(g_jkbmsModbusPollBlocks) / sizeof(g_jkbmsModbusPollBlocks[0]);
