#include "unity.h"

#include <stdint.h>

#include "config.h"
#include "decoders/modbusDecoder.h"
#include "protocols/jkbms_modbus/jkbms_modbus_freshness.h"

static modbusDecoder_t g_decoder;

static uint16_t test_modbus_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFFu;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }

    return crc;
}

static void feed_one_register_response(uint16_t startReg,
                                       uint16_t value,
                                       int64_t requestUs,
                                       int64_t responseUs)
{
    uint8_t response[] = {
        0x01u,
        0x03u,
        0x02u,
        (uint8_t)((value >> 8) & 0xFFu),
        (uint8_t)(value & 0xFFu),
        0x00u,
        0x00u
    };
    const uint16_t crc = test_modbus_crc16(response, 5);

    response[5] = (uint8_t)(crc & 0xFFu);
    response[6] = (uint8_t)((crc >> 8) & 0xFFu);

    modbusDecoderRecordRequest(&g_decoder, 0x01u, 0x03u, startReg, 1u, requestUs);
    modbusDecoderFeed(&g_decoder, response, (int)sizeof(response), responseUs);
    modbusDecoderFlush(&g_decoder);
}

void setUp(void)
{
    modbusDecoderInit(&g_decoder, "JKBMS_TEST", 5000u);
}

void tearDown(void)
{
    modbusDecoderFlush(&g_decoder);
}

void test_jkbms_cache_without_bms_response_is_not_fresh(void)
{
    int64_t newestUs = -1;

    TEST_ASSERT_FALSE(jkbmsModbusDecoderCacheFresh(&g_decoder, 1000000LL, &newestUs));
    TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)newestUs);
}

void test_jkbms_cache_is_fresh_inside_source_window(void)
{
    const int64_t sourceUs = 500000LL;
    const int64_t maxAgeUs = (int64_t)JKBMS_BMS_SOURCE_STALE_MS * 1000LL;
    int64_t newestUs = 0;

    feed_one_register_response(0x1200u, 0x1234u, sourceUs - 1000LL, sourceUs);

    TEST_ASSERT_TRUE(jkbmsModbusDecoderCacheFresh(&g_decoder,
                                                  sourceUs + maxAgeUs,
                                                  &newestUs));
    TEST_ASSERT_EQUAL_UINT32((uint32_t)sourceUs, (uint32_t)newestUs);
}

void test_jkbms_cache_expires_after_source_window(void)
{
    const int64_t sourceUs = 700000LL;
    const int64_t maxAgeUs = (int64_t)JKBMS_BMS_SOURCE_STALE_MS * 1000LL;
    int64_t newestUs = 0;

    feed_one_register_response(0x1200u, 0x5678u, sourceUs - 1000LL, sourceUs);

    TEST_ASSERT_FALSE(jkbmsModbusDecoderCacheFresh(&g_decoder,
                                                   sourceUs + maxAgeUs + 1LL,
                                                   &newestUs));
    TEST_ASSERT_EQUAL_UINT32((uint32_t)sourceUs, (uint32_t)newestUs);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_jkbms_cache_without_bms_response_is_not_fresh);
    RUN_TEST(test_jkbms_cache_is_fresh_inside_source_window);
    RUN_TEST(test_jkbms_cache_expires_after_source_window);

    return UNITY_END();
}
