#include "unity.h"

#include <stdint.h>

#include "decoders/modbusDecoder.h"
#include "protocols/pace_modbus/pace_modbus_registers_map.h"
#include "protocols/wow_modbus/wow_modbus_bms_task.h"

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
            if ((crc & 1u) != 0u) {
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
    for (uint8_t i = 0u; i < count; i++) {
        out[3 + (i * 2)] = (uint8_t)((regs[i] >> 8) & 0xFFu);
        out[4 + (i * 2)] = (uint8_t)(regs[i] & 0xFFu);
    }
    uint16_t crc = crc16(out, len - 2);
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)((crc >> 8) & 0xFFu);
    return len;
}

static void feedResponse(modbusDecoder_t *decoder,
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

void test_wow_modbus_decodes_pace_compatible_runtime_map(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;
    uint16_t summary[13] = {
        0u,
        7270u,
        100u,
        100u,
        4000u,
        4000u,
        4000u,
        12u,
        0u,
        0x0001u,
        0x0040u,
        PACE_MB_STATUS_MOSFET_DCHG,
        0x0000u,
    };
    uint16_t cellsAndTemps[22] = {
        4528u, 4559u, 4618u, 4509u,
        4517u, 4526u, 4557u, 4532u,
        4593u, 4576u, 4578u, 4476u,
        4521u, 4520u, 4520u, 4584u,
        260u, 270u, 0u, 0u, 290u, 0u,
    };

    modbusDecoderInit(&decoder, "WOW_TEST", 5000);
    feedResponse(&decoder, PACE_MB_REG_CURRENT_10MA, summary, 13u);
    feedResponse(&decoder, PACE_MB_REG_CELL01_MV, cellsAndTemps, 22u);

    TEST_ASSERT_TRUE(wowModbusBuildDecodedPacket(&decoder, 9u, &packet));
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_ID_WOW, packet.sourceProtocol);
    TEST_ASSERT_EQUAL_UINT32(9u, packet.sequence);
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(100u, packet.socPct);
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(7270u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasWarningFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0001u, packet.warningFlags);
    TEST_ASSERT_TRUE(packet.hasProtectionFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0040u, packet.protectionFlags);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_STATUS_MOSFET_DCHG, packet.statusFlags);
    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_UINT8(6u, packet.tempCount);
    TEST_ASSERT_EQUAL_INT16(260, packet.tempDeciC[0]);
    TEST_ASSERT_EQUAL_INT16(270, packet.tempDeciC[1]);
    TEST_ASSERT_EQUAL_INT16(290, packet.tempDeciC[4]);
    TEST_ASSERT_TRUE(packet.hasCellExtremes);
    TEST_ASSERT_EQUAL_UINT16(4476u, packet.minCellMv);
    TEST_ASSERT_EQUAL_UINT16(4618u, packet.maxCellMv);
    TEST_ASSERT_EQUAL_UINT8(12u, packet.minCellIndex);
    TEST_ASSERT_EQUAL_UINT8(3u, packet.maxCellIndex);
    TEST_ASSERT_EQUAL_UINT8(16u, packet.cellCount);
    TEST_ASSERT_EQUAL_UINT16(4528u, packet.cellMv[0]);
    TEST_ASSERT_EQUAL_UINT16(4618u, packet.cellMv[2]);
    TEST_ASSERT_EQUAL_UINT16(4584u, packet.cellMv[15]);
}

void test_wow_modbus_keeps_pace_poll_map_as_initial_live_probe(void)
{
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CURRENT_10MA, g_paceModbusPollBlocks[0].start);
    TEST_ASSERT_EQUAL_UINT16(13u, g_paceModbusPollBlocks[0].count);
    TEST_ASSERT_EQUAL_UINT16(PACE_MB_REG_CELL01_MV, g_paceModbusPollBlocks[1].start);
    TEST_ASSERT_EQUAL_UINT16(22u, g_paceModbusPollBlocks[1].count);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_wow_modbus_decodes_pace_compatible_runtime_map);
    RUN_TEST(test_wow_modbus_keeps_pace_poll_map_as_initial_live_probe);
    return UNITY_END();
}
