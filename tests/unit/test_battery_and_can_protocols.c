#include "unity.h"

#include <stdio.h>
#include <string.h>

#include "Web_interface/web_bridge_api.h"
#include "config.h"
#include "protocols/deye/deye_registers_map.h"
#include "protocols/pylon/pylon_registers_map.h"
#include "esp_timer.h"
#include "protocols/common/battery_model.h"
#include "protocols/deye/deye_can_protocol.h"
#include "protocols/pylon/pylon_can_protocol.h"

#define TEST_LOG_BUFFER_SIZE 2048u
#define BATTERY_STALE_OFFSET_MS 5u

static bridgeTelemetrySnapshot_t g_lastSnapshot;
static bool g_snapshotSet;
static char g_lastLog[TEST_LOG_BUFFER_SIZE];
static bool g_logSet;

void bridgeSetTelemetrySnapshot(const bridgeTelemetrySnapshot_t *in)
{
    if (in != NULL) {
        g_lastSnapshot = *in;
    } else {
        memset(&g_lastSnapshot, 0, sizeof(g_lastSnapshot));
    }
    g_snapshotSet = true;
}

void bridgeSetDecodedLogSnapshot(const char *text)
{
    g_logSet = true;
    if (text != NULL) {
        snprintf(g_lastLog, sizeof(g_lastLog), "%s", text);
    } else {
        g_lastLog[0] = '\0';
    }
}

static void write_le16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)(value >> 8);
}

static void write_le16s(uint8_t *out, int16_t value)
{
    write_le16(out, (uint16_t)value);
}

static void set_cache_frame(pylon_can_frame_t *cache,
                            size_t count,
                            uint32_t id,
                            const uint8_t *data,
                            uint8_t dlc)
{
    const size_t idx = (size_t)(id - PYLON_CAN_ID_MIN);
    TEST_ASSERT_TRUE(idx < count);
    cache[idx].valid = true;
    cache[idx].id = id;
    cache[idx].dlc = dlc;
    memset(cache[idx].data, 0, sizeof(cache[idx].data));
    if (data != NULL && dlc > 0u) {
        memcpy(cache[idx].data, data, dlc);
    }
}

static void assert_float_within(float tolerance, float expected, float actual)
{
    float diff = actual - expected;
    if (diff < 0.0f) {
        diff = -diff;
    }
    TEST_ASSERT_TRUE(diff <= tolerance);
}

void setUp(void)
{
    memset(&g_lastSnapshot, 0, sizeof(g_lastSnapshot));
    memset(g_lastLog, 0, sizeof(g_lastLog));
    g_snapshotSet = false;
    g_logSet = false;

    batteryModelClear();
    batteryModelSetDebugOverride(NULL);
    batteryModelSetDebugOverrideEnabled(false);
}

void tearDown(void)
{
    batteryModelClear();
    batteryModelSetDebugOverride(NULL);
    batteryModelSetDebugOverrideEnabled(false);
}

void test_pylon_can_any_valid_handles_empty_cache(void)
{
    pylon_can_frame_t cache[PYLON_CAN_CACHE_COUNT] = {0};

    TEST_ASSERT_FALSE(pylonCanAnyValid(NULL, 0u));
    TEST_ASSERT_FALSE(pylonCanAnyValid(cache, PYLON_CAN_CACHE_COUNT));

    cache[0].valid = true;
    cache[0].id = PYLON_CAN_ID_MIN;
    cache[0].dlc = 1u;

    TEST_ASSERT_TRUE(pylonCanAnyValid(cache, PYLON_CAN_CACHE_COUNT));
}

void test_pylon_can_decode_snapshot_populates_snapshot_and_model(void)
{
    pylon_can_frame_t cache[PYLON_CAN_CACHE_COUNT] = {0};
    uint8_t f351[8] = {0};
    uint8_t f355[4] = {0};
    uint8_t f356[6] = {0};
    uint8_t f35C[1] = {0xE0};
    uint8_t f373[8] = {0};

    write_le16(&f351[0], 5400u);
    write_le16(&f351[2], 1000u);
    write_le16(&f351[4], 1200u);
    write_le16(&f351[6], 4000u);

    write_le16(&f355[0], 75u);
    write_le16(&f355[2], 98u);

    write_le16(&f356[0], 50000u);
    write_le16s(&f356[2], 250);
    write_le16(&f356[4], 250u);

    write_le16(&f373[0], 3100u);
    write_le16(&f373[2], 3450u);
    write_le16(&f373[4], 220u);
    write_le16(&f373[6], 300u);

    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, PYLON_CAN_ID_LIMITS_351, f351, sizeof(f351));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, PYLON_CAN_ID_SOC_SOH_355, f355, sizeof(f355));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, PYLON_CAN_ID_PACK_356, f356, sizeof(f356));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, PYLON_CAN_ID_STATUS_35C, f35C, sizeof(f35C));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, PYLON_CAN_ID_CELL_TEMP_373, f373, sizeof(f373));

    pylonCanDecodeSnapshot("CAN2", cache, PYLON_CAN_CACHE_COUNT);

    TEST_ASSERT_TRUE(g_snapshotSet);
    TEST_ASSERT_TRUE(g_logSet);
    TEST_ASSERT_TRUE(g_lastSnapshot.valid);
    TEST_ASSERT_EQUAL_STRING("CAN2", g_lastSnapshot.source);
    TEST_ASSERT_EQUAL_STRING("CAN_PYLON", g_lastSnapshot.protocol);
    TEST_ASSERT_EQUAL_UINT8(75u, g_lastSnapshot.socPct);
    TEST_ASSERT_EQUAL_UINT8(98u, g_lastSnapshot.sohPct);
    TEST_ASSERT_EQUAL_UINT8(0xE0u, g_lastSnapshot.pylonStatus63);
    assert_float_within(0.01f, 500.0f, g_lastSnapshot.packVoltageV);
    assert_float_within(0.01f, 25.0f, g_lastSnapshot.currentA);
    assert_float_within(0.001f, 3.45f, g_lastSnapshot.cellMaxV);
    assert_float_within(0.001f, 3.1f, g_lastSnapshot.cellMinV);
    assert_float_within(0.001f, 0.35f, g_lastSnapshot.deltaV);
    TEST_ASSERT_TRUE(strstr(g_lastLog, "CAN Pylon") != NULL);

    battery_model_t model = {0};
    batteryModelGetReal(&model);
    TEST_ASSERT_TRUE(model.valid);
    TEST_ASSERT_EQUAL_UINT8(75u, model.socPct);
    TEST_ASSERT_EQUAL_UINT8(98u, model.sohPct);
    assert_float_within(0.01f, 500.0f, model.packVoltageV);
    assert_float_within(0.01f, 25.0f, model.packCurrentA);
    TEST_ASSERT_TRUE(model.chargeEnabled);
    TEST_ASSERT_TRUE(model.dischargeEnabled);
    TEST_ASSERT_TRUE(model.balanceEnabled);
}

void test_deye_can_decode_snapshot_populates_snapshot_and_model(void)
{
    pylon_can_frame_t cache[PYLON_CAN_CACHE_COUNT] = {0};
    uint8_t f351[8] = {0};
    uint8_t f355[4] = {0};
    uint8_t f356[6] = {0};
    uint8_t f359[8] = {0};
    uint8_t f35C[1] = {0xA0};
    uint8_t f35E[6] = { 'J', 'K', '-', 'B', 'M', 'S' };
    uint8_t f370[8] = {0};
    uint8_t f371[8] = {0};

    write_le16(&f351[0], 5400u);
    write_le16(&f351[2], 900u);
    write_le16(&f351[4], 1100u);
    write_le16(&f351[6], 3800u);

    write_le16(&f355[0], 90u);
    write_le16(&f355[2], 95u);

    write_le16(&f356[0], 48000u);
    write_le16s(&f356[2], (int16_t)-125);
    write_le16(&f356[4], 285u);

    f359[DEYE_CAN_359_OFF_MODULE_COUNT] = 3u;
    f359[DEYE_CAN_359_OFF_TAG] = 'J';
    f359[DEYE_CAN_359_OFF_TAG + 1u] = 'K';

    write_le16(&f370[DEYE_CAN_370_OFF_TEMP_MAX_RAW], 250u);
    write_le16(&f370[DEYE_CAN_370_OFF_TEMP_MIN_RAW], 180u);
    write_le16(&f370[DEYE_CAN_370_OFF_CELL_MAX_MV], 3500u);
    write_le16(&f370[DEYE_CAN_370_OFF_CELL_MIN_MV], 3000u);

    write_le16(&f371[DEYE_CAN_371_OFF_TEMP_MAX_SENS], 2u);
    write_le16(&f371[DEYE_CAN_371_OFF_TEMP_MIN_SENS], 4u);
    write_le16(&f371[DEYE_CAN_371_OFF_CELL_MAX_IDX], 5u);
    write_le16(&f371[DEYE_CAN_371_OFF_CELL_MIN_IDX], 10u);

    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_LIMITS_351, f351, sizeof(f351));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_SOC_SOH_355, f355, sizeof(f355));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_PACK_356, f356, sizeof(f356));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_MODULE_INFO_359, f359, sizeof(f359));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_STATUS_35C, f35C, sizeof(f35C));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_ASCII_ID_35E, f35E, sizeof(f35E));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_TEMP_CELL_370, f370, sizeof(f370));
    set_cache_frame(cache, PYLON_CAN_CACHE_COUNT, DEYE_CAN_ID_SENSOR_INDEX_371, f371, sizeof(f371));

    deyeCanDecodeSnapshot("CAN1", cache, PYLON_CAN_CACHE_COUNT);

    TEST_ASSERT_TRUE(g_snapshotSet);
    TEST_ASSERT_TRUE(g_logSet);
    TEST_ASSERT_TRUE(g_lastSnapshot.valid);
    TEST_ASSERT_EQUAL_STRING("CAN1", g_lastSnapshot.source);
    TEST_ASSERT_EQUAL_STRING("CAN_DEYE", g_lastSnapshot.protocol);
    TEST_ASSERT_EQUAL_UINT8(90u, g_lastSnapshot.socPct);
    TEST_ASSERT_EQUAL_UINT8(95u, g_lastSnapshot.sohPct);
    TEST_ASSERT_EQUAL_UINT8(0xA0u, g_lastSnapshot.deyeStatus35C);
    TEST_ASSERT_EQUAL_UINT8(2u, g_lastSnapshot.deyeTempMaxSensor);
    TEST_ASSERT_EQUAL_UINT8(4u, g_lastSnapshot.deyeTempMinSensor);
    TEST_ASSERT_EQUAL_UINT8(5u, g_lastSnapshot.cellMaxIdx);
    TEST_ASSERT_EQUAL_UINT8(10u, g_lastSnapshot.cellMinIdx);
    assert_float_within(0.01f, -12.5f, g_lastSnapshot.currentA);
    assert_float_within(0.001f, 3.5f, g_lastSnapshot.cellMaxV);
    assert_float_within(0.001f, 3.0f, g_lastSnapshot.cellMinV);
    assert_float_within(0.001f, 0.5f, g_lastSnapshot.deltaV);
    TEST_ASSERT_TRUE(strstr(g_lastSnapshot.stateFlags, "charge=ON") != NULL);
    TEST_ASSERT_TRUE(strstr(g_lastSnapshot.stateFlags, "discharge=OFF") != NULL);
    TEST_ASSERT_TRUE(strstr(g_lastSnapshot.stateFlags, "balance=ON") != NULL);
    TEST_ASSERT_TRUE(strstr(g_lastLog, "CAN Deye") != NULL);

    battery_model_t model = {0};
    batteryModelGetReal(&model);
    TEST_ASSERT_TRUE(model.valid);
    TEST_ASSERT_EQUAL_UINT8(90u, model.socPct);
    TEST_ASSERT_EQUAL_UINT8(95u, model.sohPct);
    TEST_ASSERT_TRUE(model.chargeEnabled);
    TEST_ASSERT_FALSE(model.dischargeEnabled);
    TEST_ASSERT_TRUE(model.balanceEnabled);
}

void test_battery_model_debug_override_controls_output(void)
{
    battery_model_t model = {0};
    model.valid = true;
    model.socPct = 10u;
    batteryModelSet(&model);

    battery_model_t overrideModel = {0};
    overrideModel.valid = true;
    overrideModel.socPct = 42u;
    batteryModelSetDebugOverride(&overrideModel);
    batteryModelSetDebugOverrideEnabled(true);

    battery_model_t out = {0};
    batteryModelGet(&out);
    TEST_ASSERT_TRUE(batteryModelIsDebugOverrideEnabled());
    TEST_ASSERT_EQUAL_UINT8(42u, out.socPct);

    batteryModelSetDebugOverrideEnabled(false);
    batteryModelGet(&out);
    TEST_ASSERT_FALSE(batteryModelIsDebugOverrideEnabled());
    TEST_ASSERT_EQUAL_UINT8(10u, out.socPct);
}

void test_battery_model_stale_data_is_cleared(void)
{
    battery_model_t model = {0};
    const uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000LL);
    model.valid = true;
    model.socPct = 55u;
    model.updatedMs = nowMs - (BRIDGE_SOURCE_STALE_MS + BATTERY_STALE_OFFSET_MS);
    batteryModelSet(&model);

    battery_model_t out = {0};
    batteryModelGet(&out);
    TEST_ASSERT_FALSE(out.valid);
    TEST_ASSERT_EQUAL_UINT8(0u, out.socPct);
}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_pylon_can_any_valid_handles_empty_cache);
    RUN_TEST(test_pylon_can_decode_snapshot_populates_snapshot_and_model);
    RUN_TEST(test_deye_can_decode_snapshot_populates_snapshot_and_model);
    RUN_TEST(test_battery_model_debug_override_controls_output);
    RUN_TEST(test_battery_model_stale_data_is_cleared);

    return UNITY_END();
}
