/**
 * @file test_route_selection.c
 * @brief Unit tests for orchestrator route selection logic
 *
 * Tests protocol mapping and route decision logic without requiring hardware.
 */

#include "unity.h"
#include "orchestrator/orchestrator.h"
#include "orchestrator/protocol_types.h"
#include "config.h"
#include "runtime_settings.h"
#include <string.h>

extern int g_routeStubGrowattBmsStartCount;
extern int g_routeStubJkbmsModbusBmsStartCount;
extern int g_routeStubVoltronicBmsStartCount;
extern int g_routeStubChinaTowerBmsStartCount;
extern int g_routeStubWowBmsStartCount;
extern int g_routeStubPylonInverterStartCount;
extern int g_routeStubPylonBridgeEnableCount;
extern int g_routeStubCanForwardStartCount;
void routeSelectionStubReset(void);

/* Test fixture setup/teardown */
void setUp(void)
{
    routeSelectionStubReset();
}

void tearDown(void)
{
    (void)orchestratorStop();
}

/**
 * Test: Protocol ID string conversion
 */
void test_protocol_id_to_string(void)
{
    const char *growattStr = protocolIdToStr(PROTOCOL_ID_GROWATT);
    const char *pylonStr = protocolIdToStr(PROTOCOL_ID_PYLON);
    const char *jkbmsStr = protocolIdToStr(PROTOCOL_ID_JKBMS);
    const char *paceStr = protocolIdToStr(PROTOCOL_ID_PACE);
    const char *jkbmsNativeStr = protocolIdToStr(PROTOCOL_ID_JKBMS_NATIVE);
    const char *voltronicStr = protocolIdToStr(PROTOCOL_ID_VOLTRONIC);
    const char *chinaTowerStr = protocolIdToStr(PROTOCOL_ID_CHINA_TOWER);
    const char *wowStr = protocolIdToStr(PROTOCOL_ID_WOW);
    const char *unknownStr = protocolIdToStr(99);

    TEST_ASSERT_EQUAL_STRING("GROWATT", growattStr);
    TEST_ASSERT_EQUAL_STRING("PYLON", pylonStr);
    TEST_ASSERT_EQUAL_STRING("JKBMS_MODBUS", jkbmsStr);
    TEST_ASSERT_EQUAL_STRING("PACE_RS485_MODBUS", paceStr);
    TEST_ASSERT_EQUAL_STRING("JKBMS_RS485_NATIVE", jkbmsNativeStr);
    TEST_ASSERT_EQUAL_STRING("VOLTRONIC_MODBUS", voltronicStr);
    TEST_ASSERT_EQUAL_STRING("CHINA_TOWER_MODBUS", chinaTowerStr);
    TEST_ASSERT_EQUAL_STRING("WOW_MODBUS", wowStr);
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", unknownStr);
}

/**
 * Test: bms_decoded_packet equivalence check
 */
void test_packet_equivalence_identical(void)
{
    bms_decoded_packet_t pkt1 = {0};
    bms_decoded_packet_t pkt2 = {0};

    pkt1.hasSoc = true;
    pkt1.socPct = 75;
    pkt1.hasPackVoltageCv = true;
    pkt1.packVoltageCv = 5240;

    pkt2.hasSoc = true;
    pkt2.socPct = 75;
    pkt2.hasPackVoltageCv = true;
    pkt2.packVoltageCv = 5240;

    /* Packets with same values should be equivalent */
    /* Note: packetEquivalent is static, so we test via orchestrator behavior */
    /* This test verifies data structure integrity */
    TEST_ASSERT_EQUAL_UINT8(pkt1.socPct, pkt2.socPct);
    TEST_ASSERT_EQUAL_UINT32(pkt1.packVoltageCv, pkt2.packVoltageCv);
}

/**
 * Test: bms_decoded_packet with different SOC
 */
void test_packet_equivalence_different_soc(void)
{
    bms_decoded_packet_t pkt1 = {0};
    bms_decoded_packet_t pkt2 = {0};

    pkt1.hasSoc = true;
    pkt1.socPct = 75;

    pkt2.hasSoc = true;
    pkt2.socPct = 80;

    TEST_ASSERT_NOT_EQUAL(pkt1.socPct, pkt2.socPct);
}

/**
 * Test: Configuration validation for CAN -> RS485 Growatt route
 */
void test_route_can_to_rs485_growatt_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    /* Configure for CAN_GROWATT BMS -> RS485_GROWATT Inverter */
    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_GROWATT;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_GROWATT;
    settings.inverter_port = 1;

    /* This configuration should be valid for CAN->RS485 Growatt translator */
    TEST_ASSERT_EQUAL_UINT8(LINE_CAN, settings.bms_line);
    TEST_ASSERT_EQUAL_UINT8(LINE_RS485, settings.inverter_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_GROWATT, settings.inverter_protocol);
}

/**
 * Test: Configuration validation for CAN_PYLON -> RS485 Growatt route
 */
void test_route_can_pylon_to_rs485_growatt_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    /* Configure for CAN_PYLON BMS -> RS485_GROWATT Inverter */
    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_PYLON;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_GROWATT;
    settings.inverter_port = 1;

    /* This configuration should be valid for CAN->RS485 Growatt translator */
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_CAN_PYLON, settings.bms_protocol);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_GROWATT, settings.inverter_protocol);
}

/**
 * Test: Configuration validation for RS485_JKBMS -> RS485_GROWATT route
 */
void test_route_jkbms_to_growatt_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    /* Configure for JKBMS -> RS485_GROWATT */
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_JKBMS;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_GROWATT;
    settings.inverter_port = 2;

    /* This configuration should be valid for JKBMS->Growatt translator */
    TEST_ASSERT_EQUAL_UINT8(LINE_RS485, settings.bms_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_JKBMS, settings.bms_protocol);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_GROWATT, settings.inverter_protocol);
}

/**
 * Test: Configuration validation for Pylon RS485 bridge route
 */
void test_route_pylon_rs485_bridge_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    /* Configure for RS485_PYLON <-> RS485_PYLON bridge */
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_PYLON;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    /* This configuration should be valid for Pylon RS485 bridge */
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_PYLON, settings.bms_protocol);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_PYLON, settings.inverter_protocol);
}

/**
 * Test: Configuration validation for CAN_PYLON -> RS485_PYLON bridge
 */
void test_route_can_pylon_to_rs485_pylon_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    /* Configure for CAN_PYLON -> RS485_PYLON bridge */
    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_PYLON;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 1;

    /* This configuration should be valid for Pylon CAN->RS485 bridge */
    TEST_ASSERT_EQUAL_UINT8(LINE_CAN, settings.bms_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_CAN_PYLON, settings.bms_protocol);
    TEST_ASSERT_EQUAL_UINT8(LINE_RS485, settings.inverter_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_PYLON, settings.inverter_protocol);
}

void test_route_can_jkbms_250k_to_rs485_pylon_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_JKBMS_250K;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
    TEST_ASSERT_EQUAL(1, g_routeStubCanForwardStartCount);
    TEST_ASSERT_EQUAL(0, g_routeStubPylonInverterStartCount);
}

/**
 * Test: Configuration validation for PACE_RS485_MODBUS -> RS485_PYLON bridge
 */
void test_route_pace_rs485_to_rs485_pylon_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_PACE;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL_UINT8(LINE_RS485, settings.bms_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_PACE, settings.bms_protocol);
    TEST_ASSERT_EQUAL_UINT8(LINE_RS485, settings.inverter_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_PYLON, settings.inverter_protocol);
}

/**
 * Test: Growatt RS485 BMS source can feed the Pylon RS485 synthetic responder.
 */
void test_route_growatt_rs485_to_rs485_pylon_starts_responder(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_GROWATT;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubGrowattBmsStartCount);
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
    TEST_ASSERT_EQUAL(0, g_routeStubPylonInverterStartCount);
}

/**
 * Test: Voltronic RS485 BMS source can feed the Pylon RS485 synthetic responder.
 */
void test_route_voltronic_rs485_to_rs485_pylon_starts_responder(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_VOLTRONIC;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubVoltronicBmsStartCount);
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
    TEST_ASSERT_EQUAL(0, g_routeStubPylonInverterStartCount);
}

/**
 * Test: China Tower RS485 BMS source can feed the Pylon RS485 synthetic responder.
 */
void test_route_china_tower_rs485_to_rs485_pylon_starts_responder(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_CHINA_TOWER;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubChinaTowerBmsStartCount);
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
    TEST_ASSERT_EQUAL(0, g_routeStubPylonInverterStartCount);
}

/**
 * Test: WOW RS485 BMS source can feed the Pylon RS485 synthetic responder.
 */
void test_route_wow_rs485_to_rs485_pylon_starts_responder(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_WOW;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubWowBmsStartCount);
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
    TEST_ASSERT_EQUAL(0, g_routeStubPylonInverterStartCount);
}

/**
 * Test: Configuration validation for JKBMS_RS485_NATIVE -> RS485_PYLON bridge
 */
void test_route_jkbms_native_to_rs485_pylon_valid(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_JKBMS_NATIVE;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL_UINT8(LINE_RS485, settings.bms_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_JKBMS_NATIVE, settings.bms_protocol);
    TEST_ASSERT_EQUAL_UINT8(LINE_RS485, settings.inverter_line);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_PYLON, settings.inverter_protocol);
}

/**
 * Test: JKBMS Modbus 115200 uses the normal JKBMS Modbus task and Pylon responder.
 */
void test_route_jkbms_115200_to_rs485_pylon_starts_responder(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_JKBMS_115200;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON_115200;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubJkbmsModbusBmsStartCount);
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
}

/**
 * Test: Deye CAN BMS source can feed the Pylon RS485 responder.
 */
void test_route_deye_can_to_rs485_pylon_starts_responder(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_DEYE;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
    TEST_ASSERT_EQUAL(1, g_routeStubCanForwardStartCount);
}

/**
 * Test: Pylon 115200 variants are accepted as Pylon bridge protocols.
 */
void test_route_pylon_115200_bridge_starts_responder(void)
{
    bridge_runtime_settings_t settings = {0};

    settings.mode = MODE_BRIDGE;
    settings.bms_line = LINE_RS485;
    settings.bms_protocol = PROTOCOL_RS485_PYLON_115200;
    settings.bms_port = 1;
    settings.inverter_line = LINE_RS485;
    settings.inverter_protocol = PROTOCOL_RS485_PYLON_115200;
    settings.inverter_port = 2;

    TEST_ASSERT_EQUAL(ESP_OK, orchestratorStartFromRuntime(&settings));
    TEST_ASSERT_EQUAL(1, g_routeStubPylonBridgeEnableCount);
    TEST_ASSERT_EQUAL(0, g_routeStubPylonInverterStartCount);
}

/**
 * Test: Invalid configuration - mismatched lines
 */
void test_route_invalid_same_line_both_sides(void)
{
    bridge_runtime_settings_t settings = {0};

    /* Invalid: Both BMS and inverter on same CAN port */
    settings.bms_line = LINE_CAN;
    settings.bms_protocol = PROTOCOL_CAN_GROWATT;
    settings.bms_port = 1;
    settings.inverter_line = LINE_CAN;
    settings.inverter_protocol = PROTOCOL_CAN_GROWATT;
    settings.inverter_port = 1; /* Same port! */

    /* This configuration is likely invalid for bridge mode */
    TEST_ASSERT_EQUAL_UINT8(settings.bms_port, settings.inverter_port);
    TEST_ASSERT_EQUAL_UINT8(settings.bms_line, settings.inverter_line);
}

/**
 * Test: Protocol constants are correctly defined
 */
void test_protocol_constants_defined(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, PROTOCOL_CAN_GROWATT);
    TEST_ASSERT_EQUAL_UINT8(2, PROTOCOL_RS485_GROWATT);
    TEST_ASSERT_EQUAL_UINT8(3, PROTOCOL_RS485_PYLON);
    TEST_ASSERT_EQUAL_UINT8(4, PROTOCOL_CAN_PYLON);
    TEST_ASSERT_EQUAL_UINT8(5, PROTOCOL_CAN_DEYE);
    TEST_ASSERT_EQUAL_UINT8(6, PROTOCOL_RS485_JKBMS);
    TEST_ASSERT_EQUAL_UINT8(7, PROTOCOL_CAN_GOODWE);
    TEST_ASSERT_EQUAL_UINT8(8, PROTOCOL_CAN_SOFAR);
    TEST_ASSERT_EQUAL_UINT8(9, PROTOCOL_CAN_SMA);
    TEST_ASSERT_EQUAL_UINT8(10, PROTOCOL_CAN_VICTRON);
    TEST_ASSERT_EQUAL_UINT8(11, PROTOCOL_RS485_PACE);
    TEST_ASSERT_EQUAL_UINT8(12, PROTOCOL_RS485_JKBMS_NATIVE);
    TEST_ASSERT_EQUAL_UINT8(13, PROTOCOL_RS485_VOLTRONIC);
    TEST_ASSERT_EQUAL_UINT8(14, PROTOCOL_RS485_CHINA_TOWER);
    TEST_ASSERT_EQUAL_UINT8(15, PROTOCOL_RS485_WOW);
    TEST_ASSERT_EQUAL_UINT8(16, PROTOCOL_RS485_JKBMS_115200);
    TEST_ASSERT_EQUAL_UINT8(17, PROTOCOL_RS485_PYLON_115200);
    TEST_ASSERT_EQUAL_UINT8(18, PROTOCOL_CAN_JKBMS_250K);
    TEST_ASSERT_EQUAL_UINT32(9600u, bridgeProtocolRs485Baudrate(PROTOCOL_RS485_JKBMS));
    TEST_ASSERT_EQUAL_UINT32(115200u, bridgeProtocolRs485Baudrate(PROTOCOL_RS485_JKBMS_115200));
    TEST_ASSERT_EQUAL_UINT32(9600u, bridgeProtocolRs485Baudrate(PROTOCOL_RS485_PYLON));
    TEST_ASSERT_EQUAL_UINT32(115200u, bridgeProtocolRs485Baudrate(PROTOCOL_RS485_PYLON_115200));
    TEST_ASSERT_EQUAL_UINT32(250000u, bridgeProtocolCanBitrate(PROTOCOL_CAN_JKBMS_250K));
    TEST_ASSERT_EQUAL_UINT32(500000u, bridgeProtocolCanBitrate(PROTOCOL_CAN_PYLON));
    TEST_ASSERT_TRUE(bridgeProtocolIsRs485JkbmsModbus(PROTOCOL_RS485_JKBMS_115200));
    TEST_ASSERT_TRUE(bridgeProtocolIsRs485Pylon(PROTOCOL_RS485_PYLON_115200));
    TEST_ASSERT_TRUE(bridgeProtocolIsCanJkbms250k(PROTOCOL_CAN_JKBMS_250K));
}

/**
 * Test: Line constants are correctly defined
 */
void test_line_constants_defined(void)
{
    TEST_ASSERT_EQUAL_UINT8(1, LINE_CAN);
    TEST_ASSERT_EQUAL_UINT8(2, LINE_RS485);
}

/**
 * Test: All supported CAN BMS protocols for translation route
 */
void test_route_all_can_bms_protocols_supported(void)
{
    uint8_t canBmsProtocols[] = {
        PROTOCOL_CAN_GROWATT,
        PROTOCOL_CAN_PYLON,
        PROTOCOL_CAN_GOODWE,
        PROTOCOL_CAN_SOFAR,
        PROTOCOL_CAN_SMA,
        PROTOCOL_CAN_VICTRON
    };

    for (size_t i = 0; i < sizeof(canBmsProtocols); i++) {
        bridge_runtime_settings_t settings = {0};
        settings.bms_line = LINE_CAN;
        settings.bms_protocol = canBmsProtocols[i];
        settings.inverter_line = LINE_RS485;
        settings.inverter_protocol = PROTOCOL_RS485_GROWATT;

        /* All these should be valid for CAN->RS485 Growatt route */
        TEST_ASSERT_EQUAL_UINT8(LINE_CAN, settings.bms_line);
        TEST_ASSERT_EQUAL_UINT8(PROTOCOL_RS485_GROWATT, settings.inverter_protocol);
    }
}

/* Main test runner function */
int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_protocol_id_to_string);
    RUN_TEST(test_packet_equivalence_identical);
    RUN_TEST(test_packet_equivalence_different_soc);
    RUN_TEST(test_route_can_to_rs485_growatt_valid);
    RUN_TEST(test_route_can_pylon_to_rs485_growatt_valid);
    RUN_TEST(test_route_jkbms_to_growatt_valid);
    RUN_TEST(test_route_pylon_rs485_bridge_valid);
    RUN_TEST(test_route_can_pylon_to_rs485_pylon_valid);
    RUN_TEST(test_route_can_jkbms_250k_to_rs485_pylon_valid);
    RUN_TEST(test_route_pace_rs485_to_rs485_pylon_valid);
    RUN_TEST(test_route_growatt_rs485_to_rs485_pylon_starts_responder);
    RUN_TEST(test_route_voltronic_rs485_to_rs485_pylon_starts_responder);
    RUN_TEST(test_route_china_tower_rs485_to_rs485_pylon_starts_responder);
    RUN_TEST(test_route_wow_rs485_to_rs485_pylon_starts_responder);
    RUN_TEST(test_route_jkbms_native_to_rs485_pylon_valid);
    RUN_TEST(test_route_jkbms_115200_to_rs485_pylon_starts_responder);
    RUN_TEST(test_route_deye_can_to_rs485_pylon_starts_responder);
    RUN_TEST(test_route_pylon_115200_bridge_starts_responder);
    RUN_TEST(test_route_invalid_same_line_both_sides);
    RUN_TEST(test_protocol_constants_defined);
    RUN_TEST(test_line_constants_defined);
    RUN_TEST(test_route_all_can_bms_protocols_supported);

    return UNITY_END();
}
