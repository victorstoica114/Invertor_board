#include "protocols/jkbms_modbus/jkbms_modbus_registers_map.h"

/*
 * Keep poll blocks compact and explicit. Some JK firmwares are sensitive to large
 * ranges that include sparse/undocumented addresses.
 */
const jkbms_modbus_poll_block_t g_jkbmsModbusPollBlocks[] = {
    /*
     * Cell table split into smaller chunks. Some JK firmwares/transceivers are
     * unstable on larger FC03 ranges and return partial/empty replies.
     */
    { .start = (uint16_t)(JKBMS_RT_REG_CELL0_MV + 0x0000u), .count = 0x0010u },
    { .start = (uint16_t)(JKBMS_RT_REG_CELL0_MV + 0x0010u), .count = 0x0010u },
    { .start = (uint16_t)(JKBMS_RT_REG_CELL0_MV + 0x0020u), .count = 0x0010u },
    { .start = (uint16_t)(JKBMS_RT_REG_CELL0_MV + 0x0030u), .count = 0x0010u },
    { .start = (uint16_t)(JKBMS_RT_REG_CELL0_MV + 0x0040u), .count = 0x0020u },
    /* Cell summary area: avg/diff/max-min indexes */
    { .start = JKBMS_RT_REG_CELL_AVG_MV,           .count = 0x0006u },
    /* Runtime electrical + thermal + alarm + capacity + cycles */
    { .start = JKBMS_RT_REG_TEMP_MOS_DECIC,        .count = 0x0028u },
    /* SOC/SOH/precharge pair explicitly */
    { .start = JKBMS_RT_REG_SOH_PRECHARGE_U8X2,    .count = 0x0002u },
};

const size_t g_jkbmsModbusPollBlocksCount =
    sizeof(g_jkbmsModbusPollBlocks) / sizeof(g_jkbmsModbusPollBlocks[0]);
