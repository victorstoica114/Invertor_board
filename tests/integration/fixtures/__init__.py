"""
Test fixtures package for ESP32 CAN/RS485 Bridge tests.

This package provides protocol samples and test utilities.
"""

from .protocol_samples import (
    GROWATT_CAN_FRAME_SAMPLES,
    PYLON_CAN_FRAME_SAMPLES,
    JKBMS_MODBUS_SAMPLES,
    GROWATT_MODBUS_SAMPLES,
    calculate_modbus_crc,
    get_modbus_frame_with_crc,
    get_can_frame_dict,
)

__all__ = [
    'GROWATT_CAN_FRAME_SAMPLES',
    'PYLON_CAN_FRAME_SAMPLES',
    'JKBMS_MODBUS_SAMPLES',
    'GROWATT_MODBUS_SAMPLES',
    'calculate_modbus_crc',
    'get_modbus_frame_with_crc',
    'get_can_frame_dict',
]
