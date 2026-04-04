"""
Protocol sample fixtures for testing.

This module provides pre-captured protocol frames for use in unit and integration tests.
These samples represent real protocol data from supported BMS and inverter devices.
"""

# Growatt CAN frame samples (ID 0x322)
# Format: [SOC%, reserved, voltage_low, voltage_high, current_low, current_high, ...]
GROWATT_CAN_FRAME_SAMPLES = [
    {
        "id": 0x322,
        "dlc": 8,
        "data": bytes([75, 0x00, 0xC8, 0x14, 0x0A, 0x00, 0x00, 0x00]),
        "description": "SOC 75%, Voltage 52.4V (5240 cV), Current 10A",
        "expected_soc": 75,
        "expected_voltage_cv": 5240,
    },
    {
        "id": 0x322,
        "dlc": 8,
        "data": bytes([100, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00]),
        "description": "SOC 100%, Voltage 54.0V (5400 cV), Current 0A",
        "expected_soc": 100,
        "expected_voltage_cv": 5400,
    },
    {
        "id": 0x322,
        "dlc": 8,
        "data": bytes([25, 0x00, 0xE8, 0x13, 0xF6, 0xFF, 0x00, 0x00]),
        "description": "SOC 25%, Voltage 51.2V (5120 cV), Current -10A (discharge)",
        "expected_soc": 25,
        "expected_voltage_cv": 5120,
    },
]

# Pylon CAN frame samples
PYLON_CAN_FRAME_SAMPLES = [
    {
        "id": 0x351,  # Pylon battery voltage/current/temperature
        "dlc": 8,
        "data": bytes([0x40, 0x14, 0xF6, 0xFF, 0xE8, 0x03, 0x64, 0x00]),
        "description": "Pylon 0x351: Voltage 52.0V, Current -10A, Temp 10°C, SOC 100%",
    },
    {
        "id": 0x355,  # Pylon SOC/SOH
        "dlc": 8,
        "data": bytes([0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00]),
        "description": "Pylon 0x355: SOC 100%, SOH 100%",
    },
]

# Modbus (JKBMS) response samples
# Format: [slave_id, function_code, byte_count, data..., crc_low, crc_high]
JKBMS_MODBUS_SAMPLES = [
    {
        "slave_id": 0x01,
        "function": 0x03,  # Read Holding Registers
        "registers": {
            0x0064: 0x1234,  # Example register
            0x0065: 0x5678,
        },
        "raw_response": bytes([
            0x01, 0x03, 0x04,  # Header: slave, function, byte count
            0x12, 0x34,  # Register 0x0064 value
            0x56, 0x78,  # Register 0x0065 value
            0x00, 0x00   # CRC (to be calculated)
        ]),
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
            0x0001: 75,    # SOC (%)
            0x0002: 250,   # Temperature (0.1°C)
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
    Convert sample dict to format suitable for testing.

    Args:
        sample: Sample dictionary from above

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
    # Test the fixture utilities
    print("Testing protocol fixtures...")

    print(f"\nGrowatt CAN samples: {len(GROWATT_CAN_FRAME_SAMPLES)}")
    for i, sample in enumerate(GROWATT_CAN_FRAME_SAMPLES):
        print(f"  Sample {i+1}: {sample['description']}")

    print(f"\nPylon CAN samples: {len(PYLON_CAN_FRAME_SAMPLES)}")
    for i, sample in enumerate(PYLON_CAN_FRAME_SAMPLES):
        print(f"  Sample {i+1}: {sample['description']}")

    # Test CRC calculation
    test_frame = bytes([0x01, 0x03, 0x04, 0x12, 0x34, 0x56, 0x78])
    crc = calculate_modbus_crc(test_frame)
    print(f"\nTest Modbus CRC: 0x{crc:04X}")

    print("\nProtocol fixtures ready for use in tests!")
