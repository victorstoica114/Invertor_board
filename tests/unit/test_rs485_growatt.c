#include "unity.h"

#include <stdint.h>

#include "decoders/modbusDecoder.h"
#include "protocols/rs485_growatt/rs485_growatt_bms_task.h"
#include "protocols/rs485_growatt/rs485_growatt_registers_map.h"

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

static void feedGrowattResponse(modbusDecoder_t *decoder,
                                uint16_t start,
                                const uint16_t *regs,
                                uint8_t count)
{
    uint8_t frame[96];
    int len = buildResp03(0x01u, regs, count, frame, sizeof(frame));

    modbusDecoderRecordRequest(decoder, 0x01u, 0x03u, start, count, 1000);
    modbusDecoderFeed(decoder, frame, len, 2000);
    modbusDecoderFlush(decoder);
}

void test_rs485_growatt_decodes_error_and_warning_registers(void)
{
    modbusDecoder_t decoder;
    bms_decoded_packet_t packet;
    enum {
        MAIN_COUNT = RS485_GROWATT_MB_REG_MAIN_END - RS485_GROWATT_MB_REG_MAIN_START + 1u,
    };
    uint16_t mainRegs[MAIN_COUNT] = {0};

    mainRegs[RS485_GROWATT_MB_REG_STATUS_FLAGS - RS485_GROWATT_MB_REG_MAIN_START] = 0x00CBu;
    mainRegs[RS485_GROWATT_MB_REG_ERROR_CODE - RS485_GROWATT_MB_REG_MAIN_START] = 0x1424u;
    mainRegs[RS485_GROWATT_MB_REG_SOC_PCT - RS485_GROWATT_MB_REG_MAIN_START] = 80u;
    mainRegs[RS485_GROWATT_MB_REG_PACK_V_CV - RS485_GROWATT_MB_REG_MAIN_START] = 5120u;
    mainRegs[RS485_GROWATT_MB_REG_TEMP_C - RS485_GROWATT_MB_REG_MAIN_START] = 27u;
    mainRegs[RS485_GROWATT_MB_REG_WARNING_CODE - RS485_GROWATT_MB_REG_MAIN_START] = 0xC441u;

    modbusDecoderInit(&decoder, "RS485_GROWATT_TEST", 5000);
    feedGrowattResponse(&decoder, RS485_GROWATT_MB_REG_MAIN_START, mainRegs, MAIN_COUNT);

    TEST_ASSERT_TRUE(rs485GrowattBuildDecodedPacket(&decoder, 9u, &packet));
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_ID_GROWATT, packet.sourceProtocol);
    TEST_ASSERT_EQUAL_UINT32(9u, packet.sequence);
    TEST_ASSERT_TRUE(packet.hasSoc);
    TEST_ASSERT_EQUAL_UINT8(80u, packet.socPct);
    TEST_ASSERT_TRUE(packet.hasPackVoltageCv);
    TEST_ASSERT_EQUAL_UINT16(5120u, packet.packVoltageCv);
    TEST_ASSERT_TRUE(packet.hasTemperatureC);
    TEST_ASSERT_EQUAL_INT16(27, packet.temperatureC);
    TEST_ASSERT_TRUE(packet.hasStatusFlags);
    TEST_ASSERT_EQUAL_UINT16(0x00CBu, packet.statusFlags);
    TEST_ASSERT_TRUE(packet.hasProtectionFlags);
    TEST_ASSERT_EQUAL_UINT16(0x1424u, packet.protectionFlags);
    TEST_ASSERT_TRUE(packet.hasWarningFlags);
    TEST_ASSERT_EQUAL_UINT16(0x0441u, packet.warningFlags);
}

void test_rs485_growatt_decodes_signed_centiamp_current(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -2.94f, rs485GrowattPackCurrentRawToA(0xFEDAu));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.94f, rs485GrowattPackCurrentRawToA(0x0126u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_rs485_growatt_decodes_error_and_warning_registers);
    RUN_TEST(test_rs485_growatt_decodes_signed_centiamp_current);
    return UNITY_END();
}
