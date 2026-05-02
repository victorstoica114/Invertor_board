"""
Protocol sample fixtures for integration tests.

The samples mirror the fields used by the production decoders so Python
integration tests and host C tests can share realistic frame data.
"""

# Growatt CAN frame samples (ID 0x313)
# Format: big-endian voltage cV, current dA, temperature dC, SOC%, SOH%.
GROWATT_CAN_FRAME_SAMPLES = [
    {
        "id": 0x313,
        "dlc": 8,
        "data": bytes([0x14, 0x78, 0x00, 0x64, 0x00, 0xFA, 75, 98]),
        "description": "Growatt 0x313: SOC 75%, voltage 52.40V, current 10.0A, temp 25.0C",
        "expected_soc": 75,
        "expected_voltage_cv": 5240,
        "expected_current_da": 100,
    },
    {
        "id": 0x313,
        "dlc": 8,
        "data": bytes([0x15, 0x18, 0x00, 0x00, 0x00, 0xF0, 100, 100]),
        "description": "Growatt 0x313: SOC 100%, voltage 54.00V, current 0.0A, temp 24.0C",
        "expected_soc": 100,
        "expected_voltage_cv": 5400,
        "expected_current_da": 0,
    },
    {
        "id": 0x313,
        "dlc": 8,
        "data": bytes([0x14, 0x00, 0xFF, 0x9C, 0x00, 0xF0, 25, 97]),
        "description": "Growatt 0x313: SOC 25%, voltage 51.20V, current -10.0A, temp 24.0C",
        "expected_soc": 25,
        "expected_voltage_cv": 5120,
        "expected_current_da": -100,
    },
]

# Pylon CAN frame samples
PYLON_CAN_FRAME_SAMPLES = [
    {
        "id": 0x351,  # Pylon charge/discharge limits
        "dlc": 8,
        "data": bytes([0x1C, 0x02, 0xF4, 0x01, 0xE8, 0x03, 0xB8, 0x01]),
        "description": "Pylon 0x351: charge voltage/current and discharge current limits",
    },
    {
        "id": 0x355,  # Pylon SOC/SOH
        "dlc": 8,
        "data": bytes([0x64, 0x00, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00]),
        "description": "Pylon 0x355: SOC 100%, SOH 98%",
        "expected_soc": 100,
        "expected_soh": 98,
    },
    {
        "id": 0x356,  # Pylon pack voltage/current/temperature
        "dlc": 8,
        "data": bytes([0x78, 0x14, 0x9C, 0xFF, 0xFA, 0x00, 0x00, 0x00]),
        "description": "Pylon 0x356: voltage 52.40V, current -10.0A, temp 25.0C",
        "expected_voltage_cv": 5240,
        "expected_current_da": -100,
    },
]

# Modbus (JKBMS) response samples
# Format: [slave_id, function_code, byte_count, data..., crc_low, crc_high]
JKBMS_MODBUS_SAMPLES = [
    {
        "slave_id": 0x01,
        "function": 0x03,  # Read Holding Registers
        "registers": {
            0x0064: 0x1234,
            0x0065: 0x5678,
        },
        "raw_response": bytes(
            [
                0x01,
                0x03,
                0x04,
                0x12,
                0x34,
                0x56,
                0x78,
                0x00,
                0x00,
            ]
        ),
        "description": "JKBMS read holding registers response",
    },
]

# Modbus (Growatt BMS) response samples
GROWATT_MODBUS_SAMPLES = [
    {
        "slave_id": 0x01,
        "function": 0x03,
        "registers": {
            0x0000: 5240,  # Battery voltage (cV)
            0x0001: 75,  # SOC (%)
            0x0002: 250,  # Temperature (0.1C)
        },
        "description": "Growatt BMS status registers",
    },
]


def calculate_modbus_crc(data):
    """
    Calculate Modbus RTU CRC16.

    Args:
        data: bytes object containing the data to calculate CRC for

    Returns:
        16-bit CRC value as integer
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def get_modbus_frame_with_crc(slave_id, function, data):
    """
    Build a complete Modbus frame with correct CRC.

    Args:
        slave_id: Modbus slave address
        function: Modbus function code
        data: payload data (bytes)

    Returns:
        Complete frame with CRC as bytes object
    """
    frame = bytes([slave_id, function]) + data
    crc = calculate_modbus_crc(frame)
    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return frame


def get_can_frame_dict(sample):
    """
    Convert sample dict to a TWAI-like Python dictionary.

    Args:
        sample: sample dictionary from one of the CAN sample lists

    Returns:
        Dict suitable for test frame creation
    """
    return {
        "identifier": sample["id"],
        "data_length_code": sample["dlc"],
        "data": list(sample["data"]),
        "description": sample["description"],
    }


if __name__ == "__main__":
    print("Testing protocol fixtures...")

    print(f"\nGrowatt CAN samples: {len(GROWATT_CAN_FRAME_SAMPLES)}")
    for i, sample in enumerate(GROWATT_CAN_FRAME_SAMPLES):
        print(f"  Sample {i + 1}: {sample['description']}")

    print(f"\nPylon CAN samples: {len(PYLON_CAN_FRAME_SAMPLES)}")
    for i, sample in enumerate(PYLON_CAN_FRAME_SAMPLES):
        print(f"  Sample {i + 1}: {sample['description']}")

    test_frame = bytes([0x01, 0x03, 0x04, 0x12, 0x34, 0x56, 0x78])
    crc = calculate_modbus_crc(test_frame)
    print(f"\nTest Modbus CRC: 0x{crc:04X}")

    print("\nProtocol fixtures ready for use in tests.")
