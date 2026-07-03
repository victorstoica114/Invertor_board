#include "unity.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "Web_interface/web_bridge_api.h"
#include "protocols/common/battery_model.h"
#include "protocols/pylon/pylon_rs485_bridge.h"
#include "runtime_settings.h"

extern bridge_runtime_settings_t g_pylonBridgeTestSettings;
extern battery_model_t g_pylonBridgeTestModel;
extern bool g_pylonBridgeTestModelValid;
extern bridgeTelemetrySnapshot_t g_pylonBridgeLastTelemetry;
extern char g_pylonBridgeLastDecodedLog[2048];
void pylonBridgeStubReset(void);

void setUp(void)
{
    pylonBridgeStubReset();
    pylonRs485BridgeResetForTest();
}

void tearDown(void)
{
}

static void configureSyntheticRoute(uint8_t bmsProtocol)
{
    g_pylonBridgeTestSettings.mode = MODE_BRIDGE;
    g_pylonBridgeTestSettings.bms_line = LINE_RS485;
    g_pylonBridgeTestSettings.inverter_line = LINE_RS485;
    g_pylonBridgeTestSettings.bms_protocol = bmsProtocol;
    g_pylonBridgeTestSettings.inverter_protocol = PROTOCOL_RS485_PYLON;
    g_pylonBridgeTestSettings.bms_port = 1u;
    g_pylonBridgeTestSettings.inverter_port = 2u;
}

static void configureNativeCanRoute(uint8_t bmsProtocol)
{
    g_pylonBridgeTestSettings.mode = MODE_BRIDGE;
    g_pylonBridgeTestSettings.bms_line = LINE_CAN;
    g_pylonBridgeTestSettings.inverter_line = LINE_RS485;
    g_pylonBridgeTestSettings.bms_protocol = bmsProtocol;
    g_pylonBridgeTestSettings.inverter_protocol = PROTOCOL_RS485_PYLON;
    g_pylonBridgeTestSettings.bms_port = 1u;
    g_pylonBridgeTestSettings.inverter_port = 2u;
}

static void configurePylonRs485ToCanRoute(void)
{
    g_pylonBridgeTestSettings.mode = MODE_BRIDGE;
    g_pylonBridgeTestSettings.bms_line = LINE_RS485;
    g_pylonBridgeTestSettings.inverter_line = LINE_CAN;
    g_pylonBridgeTestSettings.bms_protocol = PROTOCOL_RS485_PYLON;
    g_pylonBridgeTestSettings.inverter_protocol = PROTOCOL_CAN_PYLON;
    g_pylonBridgeTestSettings.bms_port = 1u;
    g_pylonBridgeTestSettings.inverter_port = 2u;
}

static void configurePylonRs485ToPylonRs485Route(void)
{
    g_pylonBridgeTestSettings.mode = MODE_BRIDGE;
    g_pylonBridgeTestSettings.bms_line = LINE_RS485;
    g_pylonBridgeTestSettings.inverter_line = LINE_RS485;
    g_pylonBridgeTestSettings.bms_protocol = PROTOCOL_RS485_PYLON;
    g_pylonBridgeTestSettings.inverter_protocol = PROTOCOL_RS485_PYLON;
    g_pylonBridgeTestSettings.bms_port = 1u;
    g_pylonBridgeTestSettings.inverter_port = 2u;
}

static void setValidModel(void)
{
    memset(&g_pylonBridgeTestModel, 0, sizeof(g_pylonBridgeTestModel));
    g_pylonBridgeTestModel.valid = true;
    g_pylonBridgeTestModel.updatedMs = 1000u;
    g_pylonBridgeTestModel.packVoltageV = 72.7f;
    g_pylonBridgeTestModel.packCurrentA = 12.3f;
    g_pylonBridgeTestModel.socPct = 87u;
    g_pylonBridgeTestModel.sohPct = 99u;
    g_pylonBridgeTestModel.cycleCount = 321u;
    g_pylonBridgeTestModel.cellMaxV = 4.618f;
    g_pylonBridgeTestModel.cellMinV = 4.476f;
    g_pylonBridgeTestModel.cellMaxIdx = 3u;
    g_pylonBridgeTestModel.cellMinIdx = 12u;
    g_pylonBridgeTestModel.temperaturesC[0] = 29.0f;
    g_pylonBridgeTestModel.temperaturesC[1] = 28.0f;
    g_pylonBridgeTestModel.temperaturesC[2] = 27.5f;
    g_pylonBridgeTestModel.temperaturesC[3] = 27.0f;
    g_pylonBridgeTestModel.temperaturesC[4] = 26.5f;
    g_pylonBridgeTestModelValid = true;
}

static uint8_t hexByteAt(const char *hex, int byteIndex)
{
    uint8_t value = 0u;

    for (int i = 0; i < 2; i++) {
        char c = hex[(byteIndex * 2) + i];
        uint8_t nibble = 0u;
        if (c >= '0' && c <= '9') {
            nibble = (uint8_t)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            nibble = (uint8_t)(10 + (c - 'A'));
        } else if (c >= 'a' && c <= 'f') {
            nibble = (uint8_t)(10 + (c - 'a'));
        } else {
            TEST_ASSERT_TRUE(false);
        }
        value = (uint8_t)((value << 4) | nibble);
    }

    return value;
}

static uint16_t hexBe16At(const char *hex, int byteIndex)
{
    return (uint16_t)(((uint16_t)hexByteAt(hex, byteIndex) << 8) |
                      (uint16_t)hexByteAt(hex, byteIndex + 1));
}

void test_pylon_synthetic_63_generic_sources_default_to_c0_and_ignore_balance(void)
{
    char info63[32] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_WOW);
    setValidModel();
    g_pylonBridgeTestModel.balanceEnabled = true;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_STRING("05E0B1800000076CC0", info63);
    TEST_ASSERT_EQUAL_UINT8(0xC0u, hexByteAt(info63, 8));
}

void test_pylon_synthetic_63_uses_explicit_generic_charge_discharge_flags(void)
{
    char info63[32] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_PACE);
    setValidModel();
    g_pylonBridgeTestModel.dischargeEnabled = true;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_UINT8(0x40u, hexByteAt(info63, 8));

    g_pylonBridgeTestModel.chargeEnabled = true;
    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_UINT8(0xC0u, hexByteAt(info63, 8));
}

void test_pylon_synthetic_63_native_sources_preserve_protocol_status_byte(void)
{
    char info63[32] = {0};

    configureNativeCanRoute(PROTOCOL_CAN_DEYE);
    setValidModel();
    g_pylonBridgeTestModel.protocolState = 0xA5u;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_UINT8(0xA5u, hexByteAt(info63, 8));
}

void test_pylon_synthetic_63_preserves_victron_protocol_status_byte(void)
{
    char info63[32] = {0};

    configureNativeCanRoute(PROTOCOL_CAN_VICTRON);
    setValidModel();
    g_pylonBridgeTestModel.protocolState = 0xC0u;
    g_pylonBridgeTestModel.chargeEnabled = true;
    g_pylonBridgeTestModel.dischargeEnabled = true;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
    TEST_ASSERT_EQUAL_UINT8(0xC0u, hexByteAt(info63, 8));
}

void test_pylon_synthetic_telemetry_preserves_deye_state_flags_from_model(void)
{
    configureNativeCanRoute(PROTOCOL_CAN_DEYE);
    setValidModel();
    g_pylonBridgeTestModel.protocolState = 0x40u;
    g_pylonBridgeTestModel.chargeEnabled = false;
    g_pylonBridgeTestModel.dischargeEnabled = true;
    g_pylonBridgeTestModel.balanceEnabled = false;

    pylonRs485BridgeRefreshSyntheticCacheForTest();

    TEST_ASSERT_TRUE(g_pylonBridgeLastTelemetry.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 72.7f, g_pylonBridgeLastTelemetry.packVoltageV);
    TEST_ASSERT_EQUAL_UINT8(0x40u, g_pylonBridgeLastTelemetry.pylonStatus63);
    TEST_ASSERT_EQUAL_UINT8(0x40u, g_pylonBridgeLastTelemetry.deyeStatus35C);
    TEST_ASSERT_TRUE(strstr(g_pylonBridgeLastTelemetry.stateFlags, "charge=OFF") != NULL);
    TEST_ASSERT_TRUE(strstr(g_pylonBridgeLastTelemetry.stateFlags, "discharge=ON") != NULL);
    TEST_ASSERT_TRUE(strstr(g_pylonBridgeLastTelemetry.stateFlags, "balance=OFF") != NULL);
}

void test_pylon_synthetic_deye_route_does_not_overwrite_source_decoded_log(void)
{
    configureNativeCanRoute(PROTOCOL_CAN_DEYE);
    setValidModel();
    snprintf(g_pylonBridgeLastDecodedLog, sizeof(g_pylonBridgeLastDecodedLog), "%s", "CAN Deye\nexisting");

    pylonRs485BridgeRefreshSyntheticCacheForTest();

    TEST_ASSERT_EQUAL_STRING("CAN Deye\nexisting", g_pylonBridgeLastDecodedLog);
}

void test_pylon_synthetic_jkbms_can_route_does_not_overwrite_source_telemetry(void)
{
    configureNativeCanRoute(PROTOCOL_CAN_JKBMS_250K);
    setValidModel();

    g_pylonBridgeLastTelemetry.valid = true;
    snprintf(g_pylonBridgeLastTelemetry.protocol,
             sizeof(g_pylonBridgeLastTelemetry.protocol),
             "%s",
             "JKBMS_CAN_250K");
    g_pylonBridgeLastTelemetry.alarmRaw = 0x0301u;
    snprintf(g_pylonBridgeLastTelemetry.protections,
             sizeof(g_pylonBridgeLastTelemetry.protections),
             "%s",
             "Cell overvoltage (L1)");

    pylonRs485BridgeRefreshSyntheticCacheForTest();

    TEST_ASSERT_TRUE(g_pylonBridgeLastTelemetry.valid);
    TEST_ASSERT_EQUAL_STRING("JKBMS_CAN_250K", g_pylonBridgeLastTelemetry.protocol);
    TEST_ASSERT_EQUAL_UINT32(0x0301u, g_pylonBridgeLastTelemetry.alarmRaw);
    TEST_ASSERT_EQUAL_STRING("Cell overvoltage (L1)", g_pylonBridgeLastTelemetry.protections);
}

void test_pylon_native_source_still_publishes_pylon_decoded_log(void)
{
    configureNativeCanRoute(PROTOCOL_CAN_PYLON);
    setValidModel();

    pylonRs485BridgeRefreshSyntheticCacheForTest();

    TEST_ASSERT_TRUE(strstr(g_pylonBridgeLastDecodedLog, "BMS Decoded Logs") != NULL);
    TEST_ASSERT_TRUE(strstr(g_pylonBridgeLastDecodedLog, "Pylon 0x61") != NULL);
}

void test_pylon_rs485_to_rs485_bridge_uses_transparent_passthrough(void)
{
    configurePylonRs485ToPylonRs485Route();

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&g_pylonBridgeTestSettings));
    TEST_ASSERT_FALSE(pylonRs485BridgeUsesCachedResponderForTest());
}

void test_pylon_cached_responder_builds_supplemental_handshake_payloads(void)
{
    char info[384] = {0};

    configurePylonRs485ToPylonRs485Route();
    setValidModel();
    g_pylonBridgeTestModel.protocolState = 0xC0u;

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSupplementalInfoForTest(0x92u, info, sizeof(info)));
    TEST_ASSERT_EQUAL_UINT8(0xC0u, hexByteAt(info, 8));

    memset(info, 0, sizeof(info));
    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSupplementalInfoForTest(0x51u, info, sizeof(info)));
    TEST_ASSERT_TRUE(strlen(info) > 0u);
}

void test_pylon_synthetic_cache_clear_removes_telemetry_when_model_not_fresh(void)
{
    configureNativeCanRoute(PROTOCOL_CAN_PYLON);
    setValidModel();

    pylonRs485BridgeRefreshSyntheticCacheForTest();

    TEST_ASSERT_TRUE(g_pylonBridgeLastTelemetry.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 72.7f, g_pylonBridgeLastTelemetry.packVoltageV);

    g_pylonBridgeTestModelValid = false;
    pylonRs485BridgeRefreshSyntheticCacheForTest();

    TEST_ASSERT_FALSE(g_pylonBridgeLastTelemetry.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, g_pylonBridgeLastTelemetry.packVoltageV);
}

void test_pylon_info42_cell_information_updates_all_cell_telemetry(void)
{
    const char *info61 =
        "DF360000640000000064640DF400010DF2000B"
        "0BD90BDC00040BD700030BC30BC300000BC300000BC30BC300000BC30000";
    const char *info42 =
        "0110"
        "0DF40DF30DF30DF30DF30DF30DF30DF3"
        "0DF30DF30DF20DF30DF30DF30DF30DF3"
        "02"
        "0BD90BDC"
        "0000DF360000000000000000";

    configurePylonRs485ToCanRoute();

    TEST_ASSERT_TRUE(pylonRs485BridgeCacheInfoForTest(0x61u, info61));
    TEST_ASSERT_TRUE(pylonRs485BridgeCacheInfoForTest(0x42u, info42));

    TEST_ASSERT_TRUE(g_pylonBridgeLastTelemetry.valid);
    TEST_ASSERT_EQUAL_UINT8(16u, g_pylonBridgeLastTelemetry.cellCount);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.572f, g_pylonBridgeLastTelemetry.cellVoltagesV[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.570f, g_pylonBridgeLastTelemetry.cellVoltagesV[10]);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.572f, g_pylonBridgeLastTelemetry.cellMaxV);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.570f, g_pylonBridgeLastTelemetry.cellMinV);
    TEST_ASSERT_EQUAL_UINT8(1u, g_pylonBridgeLastTelemetry.cellMaxIdx);
    TEST_ASSERT_EQUAL_UINT8(11u, g_pylonBridgeLastTelemetry.cellMinIdx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.002f, g_pylonBridgeLastTelemetry.cellDiffV);
    TEST_ASSERT_TRUE(strstr(g_pylonBridgeLastDecodedLog, "Pylon 0x42") != NULL);
    TEST_ASSERT_TRUE(strstr(g_pylonBridgeLastDecodedLog, "#11=3.570V") != NULL);
}

void test_pylon_synthetic_can_route_does_not_reuse_stale_rs485_cell_list(void)
{
    const char *info61 =
        "DF360000640000000064640DF400010DF2000B"
        "0BD90BDC00040BD700030BC30BC300000BC300000BC30BC30000";
    const char *info42 =
        "0110"
        "0DF40DF30DF30DF30DF30DF30DF30DF3"
        "0DF30DF30DF20DF30DF30DF30DF30DF3"
        "02"
        "0BD90BDC"
        "0000DF360000000000000000";

    configurePylonRs485ToCanRoute();
    TEST_ASSERT_TRUE(pylonRs485BridgeCacheInfoForTest(0x61u, info61));
    TEST_ASSERT_TRUE(pylonRs485BridgeCacheInfoForTest(0x42u, info42));
    TEST_ASSERT_EQUAL_UINT8(16u, g_pylonBridgeLastTelemetry.cellCount);

    configureNativeCanRoute(PROTOCOL_CAN_PYLON);
    setValidModel();
    g_pylonBridgeTestModel.packVoltageV = 57.08f;
    g_pylonBridgeTestModel.packCurrentA = 0.0f;
    g_pylonBridgeTestModel.socPct = 99u;
    g_pylonBridgeTestModel.cellMaxV = 3.573f;
    g_pylonBridgeTestModel.cellMinV = 3.569f;
    g_pylonBridgeTestModel.cellMaxIdx = 2u;
    g_pylonBridgeTestModel.cellMinIdx = 1u;
    g_pylonBridgeTestModel.cellDeltaV = 0.004f;

    pylonRs485BridgeRefreshSyntheticCacheForTest();

    TEST_ASSERT_TRUE(g_pylonBridgeLastTelemetry.valid);
    TEST_ASSERT_EQUAL_STRING("CAN_PYLON", g_pylonBridgeLastTelemetry.protocol);
    TEST_ASSERT_EQUAL_UINT8(0u, g_pylonBridgeLastTelemetry.cellCount);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.573f, g_pylonBridgeLastTelemetry.cellMaxV);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.569f, g_pylonBridgeLastTelemetry.cellMinV);
    TEST_ASSERT_EQUAL_UINT8(2u, g_pylonBridgeLastTelemetry.cellMaxIdx);
    TEST_ASSERT_EQUAL_UINT8(1u, g_pylonBridgeLastTelemetry.cellMinIdx);
}

void test_pylon_synthetic_61_generic_sources_project_percentages_only(void)
{
    char info61[128] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_WOW);
    setValidModel();

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo61ForTest(info61, sizeof(info61)));
    TEST_ASSERT_EQUAL_UINT8(87u, hexByteAt(info61, 4));
    TEST_ASSERT_EQUAL_UINT8(99u, hexByteAt(info61, 9));
    TEST_ASSERT_EQUAL_UINT16(0x1265u, hexBe16At(info61, 11));
    TEST_ASSERT_EQUAL_UINT16(0x11D4u, hexBe16At(info61, 15));
}

void test_pylon_synthetic_61_native_can_sources_project_pack_voltage_and_current(void)
{
    char info61[128] = {0};

    configureNativeCanRoute(PROTOCOL_CAN_PYLON);
    setValidModel();

    TEST_ASSERT_TRUE(pylonRs485BridgeBuildSyntheticInfo61ForTest(info61, sizeof(info61)));
    TEST_ASSERT_EQUAL_UINT16(7270u, hexBe16At(info61, 0));
    TEST_ASSERT_EQUAL_UINT16(123u, hexBe16At(info61, 2));
    TEST_ASSERT_EQUAL_UINT8(87u, hexByteAt(info61, 4));
}

void test_pylon_synthetic_payloads_reject_missing_model(void)
{
    char info61[128] = {0};
    char info63[32] = {0};

    configureSyntheticRoute(PROTOCOL_RS485_WOW);

    TEST_ASSERT_FALSE(pylonRs485BridgeBuildSyntheticInfo61ForTest(info61, sizeof(info61)));
    TEST_ASSERT_FALSE(pylonRs485BridgeBuildSyntheticInfo63ForTest(info63, sizeof(info63)));
}

void test_pylon_route_supports_115200_variants(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_PYLON_115200;
    settings.bms_port = 1u;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2u;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));

    settings.inverter_line = LINE_CAN;
    settings.inverter_protocol = PROTOCOL_CAN_PYLON;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_PYLON;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));

    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON_115200;
    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_PYLON;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));

    settings.bms_protocol = PROTOCOL_CAN_JKBMS_250K;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));

    settings.bms_protocol = PROTOCOL_CAN_DEYE;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));

    settings.bms_protocol = PROTOCOL_CAN_GROWATT;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));

    settings.bms_protocol = PROTOCOL_CAN_GOODWE;

    TEST_ASSERT_TRUE(pylonRs485BridgeSupportsRoute(&settings));
}

void test_pylon_probe_in_bridge_mode_waits_for_recent_inverter_traffic(void)
{
    const int64_t nowUs = 10000000LL;

    TEST_ASSERT_TRUE(pylonRs485BridgeProbeShouldWaitForQuietForTest(MODE_BRIDGE,
                                                                    nowUs,
                                                                    0,
                                                                    nowUs - 100000LL));
    TEST_ASSERT_TRUE(pylonRs485BridgeProbeShouldWaitForQuietForTest(MODE_BRIDGE,
                                                                    nowUs,
                                                                    nowUs - 100000LL,
                                                                    0));
}

void test_pylon_probe_in_forward_mode_waits_for_recent_inverter_traffic(void)
{
    const int64_t nowUs = 10000000LL;

    TEST_ASSERT_TRUE(pylonRs485BridgeProbeShouldWaitForQuietForTest(MODE_FORWARD,
                                                                    nowUs,
                                                                    0,
                                                                    nowUs - 100000LL));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pylon_synthetic_63_generic_sources_default_to_c0_and_ignore_balance);
    RUN_TEST(test_pylon_synthetic_63_uses_explicit_generic_charge_discharge_flags);
    RUN_TEST(test_pylon_synthetic_63_native_sources_preserve_protocol_status_byte);
    RUN_TEST(test_pylon_synthetic_63_preserves_victron_protocol_status_byte);
    RUN_TEST(test_pylon_synthetic_telemetry_preserves_deye_state_flags_from_model);
    RUN_TEST(test_pylon_synthetic_deye_route_does_not_overwrite_source_decoded_log);
    RUN_TEST(test_pylon_synthetic_jkbms_can_route_does_not_overwrite_source_telemetry);
    RUN_TEST(test_pylon_native_source_still_publishes_pylon_decoded_log);
    RUN_TEST(test_pylon_rs485_to_rs485_bridge_uses_transparent_passthrough);
    RUN_TEST(test_pylon_cached_responder_builds_supplemental_handshake_payloads);
    RUN_TEST(test_pylon_synthetic_cache_clear_removes_telemetry_when_model_not_fresh);
    RUN_TEST(test_pylon_info42_cell_information_updates_all_cell_telemetry);
    RUN_TEST(test_pylon_synthetic_can_route_does_not_reuse_stale_rs485_cell_list);
    RUN_TEST(test_pylon_synthetic_61_generic_sources_project_percentages_only);
    RUN_TEST(test_pylon_synthetic_61_native_can_sources_project_pack_voltage_and_current);
    RUN_TEST(test_pylon_synthetic_payloads_reject_missing_model);
    RUN_TEST(test_pylon_route_supports_115200_variants);
    RUN_TEST(test_pylon_probe_in_bridge_mode_waits_for_recent_inverter_traffic);
    RUN_TEST(test_pylon_probe_in_forward_mode_waits_for_recent_inverter_traffic);
    return UNITY_END();
}
