#include "protocols/pace_modbus/pace_modbus_registers_map.h"

const pace_modbus_poll_block_t g_paceModbusPollBlocks[] = {
    /* Runtime summary: current, voltage, SOC/SOH, capacity, flags. */
    { .start = PACE_MB_REG_CURRENT_10MA, .count = 0x000Du },
    /* Cell voltages plus battery/MOS/environment temperatures. */
    { .start = PACE_MB_REG_CELL01_MV, .count = 0x0016u },
};

const size_t g_paceModbusPollBlocksCount =
    sizeof(g_paceModbusPollBlocks) / sizeof(g_paceModbusPollBlocks[0]);
