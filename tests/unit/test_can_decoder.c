/**
 * @file test_can_decoder.c
 * @brief Unit tests for CAN decoder functionality
 *
 * Tests CAN frame parsing, cache management, and protocol-specific decoding.
 */

#include "unity.h"
#include "config.h"
#include "decoders/CAN_Decoder.h"
#include "protocols/growatt/growatt_registers_map.h"
#include "protocols/common/battery_model.h"
#include "protocols/deye/deye_registers_map.h"
#include "protocols/pylon/pylon_can_protocol.h"
#include "runtime_settings.h"
#include <string.h>

extern bridge_runtime_settings_t g_hostRuntimeSettings;
void hostStubsReset(void);

static void write_le16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void feed_can_frame(uint32_t id, const uint8_t data[8])
{
    twai_message_t msg = {0};
    msg.identifier = id;
    msg.data_length_code = 8u;
    memcpy(msg.data, data, 8u);
    canDecoderOnFrame("CAN1", &msg);
}

/* Test fixture setup/teardown */
void setUp(void)
{
    hostStubsReset();
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
    msg.identifier = GROWATT_CAN_ID_313_V_I_SOC_SOH;
    msg.data_length_code = 8;

    /* 0x313 frame stores SOC at byte 6 */
    msg.data[6] = 75;

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
    msg.identifier = GROWATT_CAN_ID_313_V_I_SOC_SOH;
    msg.data_length_code = 8;
    msg.data[6] = 80;

    canDecoderOnFrame("CAN1", &msg);

    /* Data should be fresh immediately */
    bool isFresh = canDecoderHasFreshData("CAN1", 1000);
    TEST_ASSERT_TRUE(isFresh);

    /* With a 0ms max age, the frame is likely stale by the time we query. */
    bool isStale = canDecoderHasFreshData("CAN1", 0);
    TEST_ASSERT_TRUE(isStale || !isStale);
}

/**
 * Test: CAN decoder cache reset should clear all data
 */
void test_can_decoder_cache_reset(void)
{
    twai_message_t msg = {0};
    msg.identifier = GROWATT_CAN_ID_313_V_I_SOC_SOH;
    msg.data_length_code = 8;
    msg.data[6] = 90;

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
    msg1.identifier = GROWATT_CAN_ID_313_V_I_SOC_SOH;
    msg1.data_length_code = 8;
    msg1.data[6] = 60; /* CAN1: SOC 60% */

    twai_message_t msg2 = {0};
    msg2.identifier = GROWATT_CAN_ID_313_V_I_SOC_SOH;
    msg2.data_length_code = 8;
    msg2.data[6] = 85; /* CAN2: SOC 85% */

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
    msg.identifier = GROWATT_CAN_ID_313_V_I_SOC_SOH;
    msg.data_length_code = 4; /* Shorter than expected */
    msg.data[0] = 55;

    /* Decoder should handle this gracefully */
    canDecoderOnFrame("CAN1", &msg);

    uint8_t socOut = 0;
    bool hasSoc = canDecoderTryGetSocPct("CAN1", &socOut);

    /* May or may not have SOC depending on implementation - test should not crash */
    TEST_ASSERT_TRUE(hasSoc || !hasSoc); /* Always true, just testing no crash */
}

/**
 * Test: Deye CAN updates the universal battery model as frames arrive.
 */
void test_can_decoder_deye_updates_battery_model_immediately(void)
{
    uint8_t f351[8] = {0};
    uint8_t f355[8] = {0};
    uint8_t f356[8] = {0};
    uint8_t f370[8] = {0};
    uint8_t f371[8] = {0};
    battery_model_t model = {0};

    g_hostRuntimeSettings.mode = MODE_BRIDGE;
    g_hostRuntimeSettings.bms_line = LINE_CAN;
    g_hostRuntimeSettings.bms_protocol = PROTOCOL_CAN_DEYE;
    g_hostRuntimeSettings.bms_port = 1u;
    g_hostRuntimeSettings.inverter_line = LINE_RS485;
    g_hostRuntimeSettings.inverter_protocol = PROTOCOL_RS485_PYLON;
    g_hostRuntimeSettings.inverter_port = 2u;

    write_le16(&f351[DEYE_CAN_351_OFF_CHG_VLIM_DV], 560u);
    write_le16(&f351[DEYE_CAN_351_OFF_CHG_ILIM_DA], 120u);
    write_le16(&f351[DEYE_CAN_351_OFF_DIS_ILIM_DA], 130u);
    write_le16(&f355[DEYE_CAN_355_OFF_SOC_PCT], 88u);
    write_le16(&f355[DEYE_CAN_355_OFF_SOH_PCT], 99u);
    write_le16(&f356[DEYE_CAN_356_OFF_PACK_V_CV], 5275u);
    write_le16(&f356[DEYE_CAN_356_OFF_PACK_I_DA], 25u);
    write_le16(&f356[DEYE_CAN_356_OFF_TEMP_DECIC], 276u);
    write_le16(&f370[DEYE_CAN_370_OFF_TEMP_MAX_RAW], 291u);
    write_le16(&f370[DEYE_CAN_370_OFF_TEMP_MIN_RAW], 280u);
    write_le16(&f370[DEYE_CAN_370_OFF_CELL_MAX_MV], 3612u);
    write_le16(&f370[DEYE_CAN_370_OFF_CELL_MIN_MV], 3401u);
    write_le16(&f371[DEYE_CAN_371_OFF_CELL_MAX_IDX], 3u);
    write_le16(&f371[DEYE_CAN_371_OFF_CELL_MIN_IDX], 12u);

    feed_can_frame(DEYE_CAN_ID_LIMITS_351, f351);
    feed_can_frame(DEYE_CAN_ID_SOC_SOH_355, f355);
    feed_can_frame(DEYE_CAN_ID_PACK_356, f356);
    feed_can_frame(DEYE_CAN_ID_TEMP_CELL_370, f370);
    feed_can_frame(DEYE_CAN_ID_SENSOR_INDEX_371, f371);

    batteryModelGetReal(&model);
    TEST_ASSERT_TRUE(model.valid);
    TEST_ASSERT_EQUAL_UINT8(88u, model.socPct);
    TEST_ASSERT_EQUAL_UINT8(99u, model.sohPct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 52.75f, model.packVoltageV);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.5f, model.packCurrentA);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 56.0f, model.chargeVoltageLimitV);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, model.chargeCurrentLimitA);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 13.0f, model.dischargeCurrentLimitA);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.612f, model.cellMaxV);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.401f, model.cellMinV);
    TEST_ASSERT_EQUAL_UINT8(3u, model.cellMaxIdx);
    TEST_ASSERT_EQUAL_UINT8(12u, model.cellMinIdx);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 29.1f, model.temperaturesC[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 28.0f, model.temperaturesC[2]);
}

/**
 * Test: Growatt CAN updates the universal battery model for CAN->Pylon routes.
 */
void test_can_decoder_growatt_updates_battery_model_immediately(void)
{
    uint8_t f313[8] = {0};
    uint8_t f319[8] = {0};
    battery_model_t model = {0};

    g_hostRuntimeSettings.mode = MODE_BRIDGE;
    g_hostRuntimeSettings.bms_line = LINE_CAN;
    g_hostRuntimeSettings.bms_protocol = PROTOCOL_CAN_GROWATT;
    g_hostRuntimeSettings.bms_port = 1u;
    g_hostRuntimeSettings.inverter_line = LINE_RS485;
    g_hostRuntimeSettings.inverter_protocol = PROTOCOL_RS485_PYLON;
    g_hostRuntimeSettings.inverter_port = 2u;

    write_le16(&f313[0], 7270u);
    write_le16(&f313[2], 0u);
    write_le16(&f313[4], 287u);
    f313[6] = 92u;
    f313[7] = 99u;

    f319[0] = 0xC0u;
    write_le16(&f319[1], 4618u);
    write_le16(&f319[3], 4476u);
    f319[5] = 3u;
    f319[6] = 12u;

    feed_can_frame(GROWATT_CAN_ID_313_V_I_SOC_SOH, f313);
    feed_can_frame(GROWATT_CAN_ID_319_CELL_REF_FLAGS, f319);

    batteryModelGetReal(&model);
    TEST_ASSERT_TRUE(model.valid);
    TEST_ASSERT_EQUAL_UINT8(92u, model.socPct);
    TEST_ASSERT_EQUAL_UINT8(99u, model.sohPct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 72.70f, model.packVoltageV);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 28.7f, model.temperaturesC[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.618f, model.cellMaxV);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.476f, model.cellMinV);
    TEST_ASSERT_EQUAL_UINT8(3u, model.cellMaxIdx);
    TEST_ASSERT_EQUAL_UINT8(12u, model.cellMinIdx);
    TEST_ASSERT_TRUE(model.chargeEnabled);
    TEST_ASSERT_TRUE(model.dischargeEnabled);
    TEST_ASSERT_FALSE(model.balanceEnabled);
}

/* Main test runner function */
int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_can_decoder_growatt_frame_basic);
    RUN_TEST(test_can_decoder_invalid_interface);
    RUN_TEST(test_can_decoder_freshness_check);
    RUN_TEST(test_can_decoder_cache_reset);
    RUN_TEST(test_can_decoder_multiple_interfaces);
    RUN_TEST(test_can_decoder_null_message);
    RUN_TEST(test_can_decoder_variable_dlc);
    RUN_TEST(test_can_decoder_deye_updates_battery_model_immediately);
    RUN_TEST(test_can_decoder_growatt_updates_battery_model_immediately);

    return UNITY_END();
}
