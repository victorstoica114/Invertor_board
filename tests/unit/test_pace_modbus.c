#include "unity.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "decoders/modbusDecoder.h"
#include "protocols/pace_modbus/pace_modbus_bms_task.h"
#include "protocols/pace_modbus/pace_modbus_poller.h"
#include "protocols/pace_modbus/pace_modbus_registers_map.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static uint16_t crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

static int buildResp03(uint8_t slave,
                       const uint16_t *regs,
                       uint8_t count,
                       uint8_t *out,
                       int outCap)
{
    int len = 3 + ((int)count * 2) + 2;
    TEST_ASSERT_TRUE(len <= outCap);

    out[0] = slave;
    out[1] = 0x03u;
    out[2] = (uint8_t)(count * 2u);
    for (uint8_t i = 0; i < count; i++) {
        out[3 + (i * 2)] = (uint8_t)((regs[i] >> 8) & 0xFFu);
        out[4 + (i * 2)] = (uint8_t)(regs[i] & 0xFFu);
    }
    uint16_t crc = crc16(out, len - 2);
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)((crc >> 8) & 0xFFu);
    return len;
}

static void feedPaceResponse(modbusDecoder_t *decoder,
                             uint16_t start,
                             const uint16_t *regs,
                             uint8_t count)
{
    uint8_t frame[80];
    int len = buildResp03(0x01u, regs, count, frame, sizeof(frame));

    modbusDecoderRecordRequest(decoder, 0x01u, 0x03u, start, count, 1000);
    modbusDecoderFeed(decoder, frame, len, 2000);
    modbusDecoderFlush(decoder);
}

void test_pace_modbus_decodes_summary_and_cell_extremes(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;

    uint16_t summary[13] = {
        0xFE0Cu, /* -500 * 10mA = -5A */
        5120u,   /* 51.20V */
        80u,
        98u,
        12000u,
        20000u,
        20000u,
        42u,
        0u,
        0x0001u,
        0x0040u,
        0x0821u,
        0x0004u,
    };
    uint16_t cellsAndTemps[22] = {
        3200u, 3201u, 3202u, 3203u,
        3204u, 3205u, 3206u, 3207u,
        3208u, 3209u, 3210u, 3211u,
        3212u, 3213u, 3214u, 3215u,
        250u, 260u, 270u, 280u, 300u, 240u,
    };

    modbusDecoderInit(&decoder, "PACE_TEST", 5000);
    feedPaceResponse(&decoder, PACE_MB_REG_CURRENT_10MA, summary, 13u);
    feedPaceResponse(&decoder, PACE_MB_REG_CELL01_MV, cellsAndTemps, 22u);

    TEST_ASSERT_TRUE(paceModbusBuildDecodedPacket(&decoder, 7u, &packet));
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_ID_PACE, packet.sourceProtocol);
    TEST_ASSERT_EQUAL_UINT32(7u, packet.sequence);
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(80u, packet.socPct);
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(5120u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasWarningFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0001u, packet.warningFlags);
    TEST_ASSERT_TRUE(packet.hasProtectionFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0040u, packet.protectionFlags);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0821u, packet.statusFlags);
    TEST_ASSERT_TRUE(packet.hasBalanceFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0004u, packet.balanceFlags);
    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_UINT16(26u, (uint16_t)packet.temperatureC);
    TEST_ASSERT_EQUAL_UINT8(6u, packet.tempCount);
    TEST_ASSERT_EQUAL_INT16(250, packet.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(270, packet.tempDeciC[2]);
    TEST_ASSERT_EQUAL_INT16(300, packet.tempDeciC[4]);
    TEST_ASSERT_EQUAL_INT16(240, packet.tempDeciC[5]);
    TEST_ASSERT_TRUE(packet.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(3200u, packet.minCellMv);
    TEST_ASSERT_EQUAL_UINT16(3215u, packet.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(1u, packet.minCellIndex);
    TEST_ASSERT_EQUAL_UINT8(16u, packet.maxCellIndex);
    TEST_ASSERT_EQUAL_UINT8(16u, packet.cellCount);
    TEST_ASSERT_EQUAL_UINT16(3200u, packet.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(3207u, packet.cellMv[7]);
    TEST_ASSERT_EQUAL_UINT16(3215u, packet.cellMv[15]);
}

void test_pace_modbus_register_map_matches_v13_poll_plan(void)
{
    TEST_ASSERT_EQUAL_UINT32(2u, (uint32_t)g_paceModbusPollBlocksCount);

    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CURRENT_10MA, g_paceModbusPollBlocks[0].start);
    TEST_ASSERT_EQUAL_UINT16(13u, g_paceModbusPollBlocks[0].count);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_WARNING_FLAGS, PACE_MB_REG_CURRENT_10MA + 9u);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_PROTECTION_FLAGS, PACE_MB_REG_CURRENT_10MA + 10u);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_STATUS_FLAGS, PACE_MB_REG_CURRENT_10MA + 11u);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_BALANCE_STATUS, PACE_MB_REG_CURRENT_10MA + 12u);

    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CELL01_MV, g_paceModbusPollBlocks[1].start);
    TEST_ASSERT_EQUAL_UINT16(22u, g_paceModbusPollBlocks[1].count);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CELL01_MV + 15u, PACE_MB_REG_CELL16_MV);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CELL01_MV + 16u, PACE_MB_REG_TEMP1_DECIC);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_TEMP1_DECIC + 4u, PACE_MB_REG_MOS_TEMP_DECIC);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_TEMP1_DECIC + 5u, PACE_MB_REG_ENV_TEMP_DECIC);
    TEST_ASSERT_EQUAL_UINT8(6u, PACE_MB_TEMP_REG_COUNT);

    TEST_ASSERT_EQUAL_UINT16(0x0100u, PACE_MB_STATUS_CHG);
    TEST_ASSERT_EQUAL_UINT16(0x0200u, PACE_MB_STATUS_DCHG);
    TEST_ASSERT_EQUAL_UINT16(0x0400u, PACE_MB_STATUS_MOSFET_CHG);
    TEST_ASSERT_EQUAL_UINT16(0x0800u, PACE_MB_STATUS_MOSFET_DCHG);
}

void test_pace_modbus_accepts_field_high_voltage_cells_and_distinct_temps(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;

    uint16_t summary[13] = {
        0u,
        7270u, /* 72.70V */
        100u,
        100u,
        0u,
        0u,
        0u,
        0u,
        0u,
        0x0001u,
        0x0001u,
        PACE_MB_STATUS_MOSFET_DCHG,
        0x0000u,
    };
    uint16_t cellsAndTemps[22] = {
        4528u, 4560u, 4617u, 4509u,
        4516u, 4524u, 4555u, 4530u,
        4589u, 4573u, 4576u, 4476u,
        4519u, 4519u, 4517u, 4584u,
        276u, 280u, 273u, 272u, 288u, 270u,
    };

    modbusDecoderInit(&decoder, "PACE_FIELD", 5000);
    feedPaceResponse(&decoder, PACE_MB_REG_CURRENT_10MA, summary, 13u);
    feedPaceResponse(&decoder, PACE_MB_REG_CELL01_MV, cellsAndTemps, 22u);

    TEST_ASSERT_TRUE(paceModbusBuildDecodedPacket(&decoder, 42u, &packet));
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(7270u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, packet.socPct);

    TEST_ASSERT_TRUE(packet.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(4476u, packet.minCellMv);
    TEST_ASSERT_EQUAL_UINT8(12u, packet.minCellIndex);
    TEST_ASSERT_EQUAL_UINT16(4617u, packet.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(3u, packet.maxCellIndex);
    TEST_ASSERT_EQUAL_UINT8(16u, packet.cellCount);
    TEST_ASSERT_EQUAL_UINT16(4528u, packet.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(4476u, packet.cellMv[11]);
    TEST_ASSERT_EQUAL_UINT16(4584u, packet.cellMv[15]);

    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_UINT8(6u, packet.tempCount);
    TEST_ASSERT_EQUAL_INT16(276, packet.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(280, packet.tempDeciC[1]);
    TEST_ASSERT_EQUAL_INT16(273, packet.tempDeciC[2]);
    TEST_ASSERT_EQUAL_INT16(272, packet.tempDeciC[3]);
    TEST_ASSERT_EQUAL_INT16(288, packet.tempDeciC[4]);
    TEST_ASSERT_EQUAL_INT16(270, packet.tempDeciC[5]);

    TEST_ASSERT_TRUE(packet.hasWarningFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0001u, packet.warningFlags);
    TEST_ASSERT_TRUE(packet.hasProtectionFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0001u, packet.protectionFlags);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_STATUS_MOSFET_DCHG, packet.statusFlags);
    TEST_ASSERT_TRUE(packet.hasBalanceFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0000u, packet.balanceFlags);
}

void test_pace_modbus_keeps_flags_when_cell_registers_are_invalid(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;

    uint16_t summary[13] = {
        0u, 5120u, 80u, 98u, 0u, 0u, 0u, 0u, 0u,
        0x8000u, 0x0040u, 0x0021u, 0x0002u,
    };
    uint16_t cellsAndTemps[22] = {
        999u, 6001u, 999u, 6001u,
        999u, 6001u, 999u, 6001u,
        999u, 6001u, 999u, 6001u,
        999u, 6001u, 999u, 6001u,
        250u, 251u, 252u, 253u, 254u, 255u,
    };

    modbusDecoderInit(&decoder, "PACE_INVALID_CELLS", 5000);
    feedPaceResponse(&decoder, PACE_MB_REG_CURRENT_10MA, summary, 13u);
    feedPaceResponse(&decoder, PACE_MB_REG_CELL01_MV, cellsAndTemps, 22u);

    TEST_ASSERT_TRUE(paceModbusBuildDecodedPacket(&decoder, 3u, &packet));
    TEST_ASSERT_FALSE(packet.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT8(0u, packet.cellCount);
    TEST_ASSERT_TRUE(packet.hasWarningFlags);
    TEST_ASSERT_EQUAL_UINT16(0x8000u, packet.warningFlags);
    TEST_ASSERT_TRUE(packet.hasProtectionFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0040u, packet.protectionFlags);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0021u, packet.statusFlags);
    TEST_ASSERT_TRUE(packet.hasBalanceFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0002u, packet.balanceFlags);
    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_UINT8(6u, packet.tempCount);
}

void test_pace_modbus_poller_cycles_runtime_and_cell_blocks(void)
{
    pace_modbus_poller_t poller;

    paceModbusPollerInit(&poller, 0, 0, 0x01u);

    TEST_ASSERT_EQUAL(ESP_OK, paceModbusPollerTick(&poller, 1000000LL, 250u));
    TEST_ASSERT_TRUE(poller.lastReqValid);
    TEST_ASSERT_EQUAL_UINT8(0x01u, poller.lastReqSlave);
    TEST_ASSERT_EQUAL_UINT8(0x03u, poller.lastReqFunc);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CURRENT_10MA, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(13u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, paceModbusPollerTick(&poller, 1100000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CURRENT_10MA, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(13u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, paceModbusPollerTick(&poller, 1300000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CELL01_MV, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(22u, poller.lastReqCount);

    TEST_ASSERT_EQUAL(ESP_OK, paceModbusPollerTick(&poller, 1600000LL, 250u));
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CURRENT_10MA, poller.lastReqStart);
    TEST_ASSERT_EQUAL_UINT16(13u, poller.lastReqCount);
}

void test_pace_modbus_accepts_high_byte_soc_layout(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;
    uint16_t summary[13] = {
        0u, 5120u, 0x5500u, 0x6300u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
    };

    modbusDecoderInit(&decoder, "PACE_TEST", 5000);
    feedPaceResponse(&decoder, PACE_MB_REG_CURRENT_10MA, summary, 13u);

    TEST_ASSERT_TRUE(paceModbusBuildDecodedPacket(&decoder, 1u, &packet));
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(85u, packet.socPct);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_pace_modbus_decodes_summary_and_cell_extremes);
    RUN_TEST(test_pace_modbus_register_map_matches_v13_poll_plan);
    RUN_TEST(test_pace_modbus_accepts_field_high_voltage_cells_and_distinct_temps);
    RUN_TEST(test_pace_modbus_keeps_flags_when_cell_registers_are_invalid);
    RUN_TEST(test_pace_modbus_poller_cycles_runtime_and_cell_blocks);
    RUN_TEST(test_pace_modbus_accepts_high_byte_soc_layout);

    return UNITY_END();
}
