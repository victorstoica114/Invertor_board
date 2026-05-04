#include "protocols/voltronic_modbus/voltronic_modbus_registers_map.h"

const voltronic_modbus_poll_block_t g_voltronicModbusPollBlocks[] = {
    /* Public Voltronic Power "Inverter and BMS 485" map. JK protocol 007
     * answers this block first on the field unit, including cell voltages. */
    { .start = VOLTRONIC_MB_REG_STATUS_START,
      .count = (VOLTRONIC_MB_REG_STATUS_END - VOLTRONIC_MB_REG_STATUS_START + 1u),
      .frameOrder = VOLTRONIC_MB_FRAME_CLASSIC },
};

const size_t g_voltronicModbusPollBlocksCount =
    sizeof(g_voltronicModbusPollBlocks) / sizeof(g_voltronicModbusPollBlocks[0]);
