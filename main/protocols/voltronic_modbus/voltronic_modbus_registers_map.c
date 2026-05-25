#include "protocols/voltronic_modbus/voltronic_modbus_registers_map.h"

const voltronic_modbus_poll_block_t g_voltronicModbusPollBlocks[] = {
    /*
     * Some Seplos Voltronic firmwares reject broad public-map reads, but accept
     * classic 0x03 single-register reads and return a 16-bit byte-count frame.
     * Keep the core public registers in a fast enough cycle for source freshness.
     */
    { .start = VOLTRONIC_MB_REG_PROTOCOL_TYPE,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_CHARGE_CURRENT_DA,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_DISCHARGE_CURRENT_DA,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_MODULE_VOLTAGE_DV,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_SOC_PCT,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = (VOLTRONIC_MB_REG_TOTAL_CAPACITY_MAH + 1u),
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_LIMITS_START,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_DISCHARGE_V_LIMIT_DV,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_CHARGE_I_LIMIT_DA,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_DISCHARGE_I_LIMIT_DA,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
    { .start = VOLTRONIC_MB_REG_CHG_DSG_STATUS,
      .count = 1u,
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC,
      .functionCode = VOLTRONIC_MB_READ_HOLDING_REGS },
};

const size_t g_voltronicModbusPollBlocksCount =
    sizeof(g_voltronicModbusPollBlocks) / sizeof(g_voltronicModbusPollBlocks[0]);
