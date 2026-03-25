#include "protocols/jkbms_modbus/jkbms_modbus_register_map.h"

/*
 * Poll window #1:
 *   0x1200..0x1249 includes:
 *   - cell voltage area (0x0000..0x003E)
 *   - cell avg / max diff / min-max index pair (0x0044..0x0048)
 *
 * Poll window #2:
 *   0x128A..0x12B9 includes temp/pack voltage/current/alarm/SOC/cycles/SOH area.
 */
const jkbms_modbus_poll_block_t g_jkbmsModbusPollBlocks[] = {
    { .start = JKBMS_RT_REG_CELL0_MV,       .count = 0x004Au },
    { .start = JKBMS_RT_REG_TEMP_MOS_DECIC, .count = 0x0030u },
};

const size_t g_jkbmsModbusPollBlocksCount =
    sizeof(g_jkbmsModbusPollBlocks) / sizeof(g_jkbmsModbusPollBlocks[0]);
