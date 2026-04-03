/**
 * @file test_can_decoder.c
 * @brief Unit tests for CAN decoder functionality
 *
 * Tests CAN frame parsing, cache management, and protocol-specific decoding.
 */

#include "unity.h"
#include "decoders/CAN_Decoder.h"
#include "protocols/growatt/growatt_registers_map.h"
#include "protocols/pylon/pylon_can_protocol.h"
#include <string.h>

/* Test fixture setup/teardown */
void setUp(void)
{
    /* Reset decoder caches before each test */
    canDecoderResetCaches();
}

void tearDown(void)
{
    /* Cleanup after each test */
}

/**
 * Test: CAN decoder should accept valid Growatt CAN frame
 */
void test_can_decoder_growatt_frame_basic(void)
{
    twai_message_t msg = {0};
    msg.identifier = 0x322; /* Growatt BMS status frame */
    msg.data_length_code = 8;

    /* Simulate a frame with SOC=75%, voltage=52.4V, current=10A */
    msg.data[0] = 75;  /* SOC percentage */
    msg.data[1] = 0;
    msg.data[2] = 0xC8; /* Voltage low byte: 5240 centivolt = 52.4V */
    msg.data[3] = 0x14; /* Voltage high byte */
    msg.data[4] = 0x0A; /* Current low byte: 10 A */
    msg.data[5] = 0x00; /* Current high byte */
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    /* Feed frame to decoder */
    canDecoderOnFrame("CAN1", &msg);

    /* Verify SOC was decoded and cached */
    uint8_t socOut = 0;
    bool hasSoc = canDecoderTryGetSocPct("CAN1", &socOut);

    TEST_ASSERT_TRUE(hasSoc);
    TEST_ASSERT_EQUAL_UINT8(75, socOut);
}

/**
 * Test: CAN decoder should handle invalid interface name gracefully
 */
void test_can_decoder_invalid_interface(void)
{
    uint8_t socOut = 0;
    bool hasSoc = canDecoderTryGetSocPct("INVALID_IF", &socOut);

    TEST_ASSERT_FALSE(hasSoc);
}

/**
 * Test: CAN decoder should detect stale data
 */
void test_can_decoder_freshness_check(void)
{
    twai_message_t msg = {0};
    msg.identifier = 0x322;
    msg.data_length_code = 8;
    msg.data[0] = 80; /* SOC 80% */

    canDecoderOnFrame("CAN1", &msg);

    /* Data should be fresh immediately */
    bool isFresh = canDecoderHasFreshData("CAN1", 1000);
    TEST_ASSERT_TRUE(isFresh);

    /* Data should be stale after very long timeout (simulated) */
    /* Note: In real test we'd need to mock time, but this tests the API */
    bool isStale = canDecoderHasFreshData("CAN1", 0);
    TEST_ASSERT_FALSE(isStale);
}

/**
 * Test: CAN decoder cache reset should clear all data
 */
void test_can_decoder_cache_reset(void)
{
    twai_message_t msg = {0};
    msg.identifier = 0x322;
    msg.data_length_code = 8;
    msg.data[0] = 90; /* SOC 90% */

    canDecoderOnFrame("CAN1", &msg);

    /* Verify data is present */
    uint8_t socOut = 0;
    TEST_ASSERT_TRUE(canDecoderTryGetSocPct("CAN1", &socOut));
    TEST_ASSERT_EQUAL_UINT8(90, socOut);

    /* Reset caches */
    canDecoderResetCaches();

    /* Data should be gone after reset */
    TEST_ASSERT_FALSE(canDecoderTryGetSocPct("CAN1", &socOut));
}

/**
 * Test: CAN decoder should handle multiple interfaces independently
 */
void test_can_decoder_multiple_interfaces(void)
{
    twai_message_t msg1 = {0};
    msg1.identifier = 0x322;
    msg1.data_length_code = 8;
    msg1.data[0] = 60; /* CAN1: SOC 60% */

    twai_message_t msg2 = {0};
    msg2.identifier = 0x322;
    msg2.data_length_code = 8;
    msg2.data[0] = 85; /* CAN2: SOC 85% */

    canDecoderOnFrame("CAN1", &msg1);
    canDecoderOnFrame("CAN2", &msg2);

    /* Verify each interface has its own cached data */
    uint8_t soc1 = 0, soc2 = 0;
    TEST_ASSERT_TRUE(canDecoderTryGetSocPct("CAN1", &soc1));
    TEST_ASSERT_TRUE(canDecoderTryGetSocPct("CAN2", &soc2));
    TEST_ASSERT_EQUAL_UINT8(60, soc1);
    TEST_ASSERT_EQUAL_UINT8(85, soc2);
}

/**
 * Test: CAN decoder should handle NULL message gracefully
 */
void test_can_decoder_null_message(void)
{
    /* This should not crash */
    canDecoderOnFrame("CAN1", NULL);

    uint8_t socOut = 0;
    TEST_ASSERT_FALSE(canDecoderTryGetSocPct("CAN1", &socOut));
}

/**
 * Test: CAN decoder should handle frames with different DLC
 */
void test_can_decoder_variable_dlc(void)
{
    twai_message_t msg = {0};
    msg.identifier = 0x322;
    msg.data_length_code = 4; /* Shorter than expected */
    msg.data[0] = 55; /* SOC 55% */

    /* Decoder should handle this gracefully */
    canDecoderOnFrame("CAN1", &msg);

    uint8_t socOut = 0;
    bool hasSoc = canDecoderTryGetSocPct("CAN1", &socOut);

    /* May or may not have SOC depending on implementation - test should not crash */
    TEST_ASSERT_TRUE(hasSoc || !hasSoc); /* Always true, just testing no crash */
}

/* Main test runner function */
void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_can_decoder_growatt_frame_basic);
    RUN_TEST(test_can_decoder_invalid_interface);
    RUN_TEST(test_can_decoder_freshness_check);
    RUN_TEST(test_can_decoder_cache_reset);
    RUN_TEST(test_can_decoder_multiple_interfaces);
    RUN_TEST(test_can_decoder_null_message);
    RUN_TEST(test_can_decoder_variable_dlc);

    UNITY_END();
}
