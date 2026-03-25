#include "protocols/jkbms_modbus/jkbms_modbus_register_map.h"

/*
 * Keep poll blocks compact and explicit. Some JK firmwares are sensitive to large
 * ranges that include sparse/undocumented addresses.
 */
const jkbms_modbus_poll_block_t g_jkbmsModbusPollBlocks[] = {
    /* Cell table: 0x1200..0x123F (contains 0x1200,0x1202,...,0x123E) */
    { .start = JKBMS_RT_REG_CELL0_MV,              .count = 0x0040u },
    /* Cell summary area: avg/diff/max-min indexes */
    { .start = JKBMS_RT_REG_CELL_AVG_MV,           .count = 0x0006u },
    /* Runtime electrical + thermal + alarm + capacity + cycles */
    { .start = JKBMS_RT_REG_TEMP_MOS_DECIC,        .count = 0x0028u },
    /* SOC/SOH/precharge pair explicitly */
    { .start = JKBMS_RT_REG_SOH_PRECHARGE_U8X2,    .count = 0x0002u },
};

const size_t g_jkbmsModbusPollBlocksCount =
    sizeof(g_jkbmsModbusPollBlocks) / sizeof(g_jkbmsModbusPollBlocks[0]);
