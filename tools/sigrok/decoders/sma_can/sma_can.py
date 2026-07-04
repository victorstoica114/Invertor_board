##
## SMA Sunny Island compatible CAN frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FRAME_NAMES = {
    0x351: 'limits',
    0x355: 'SOC/SOH',
    0x356: 'pack',
    0x359: 'alarms/status raw',
    0x35A: 'vendor raw',
    0x35E: 'manufacturer',
    0x35F: 'battery info raw',
}

VERSION = 'v2026.07.04a'


def le16(data, pos):
    return data[pos] | (data[pos + 1] << 8)


def le16s(data, pos):
    value = le16(data, pos)
    if value & 0x8000:
        value -= 0x10000
    return value


def format_data(data):
    return ' '.join('{:02X}'.format(value & 0xFF) for value in data)


def ascii_text(data):
    chars = []
    for value in data:
        if 32 <= value <= 126:
            chars.append(chr(value))
        elif value == 0:
            chars.append(' ')
        else:
            chars.append('.')
    return ''.join(chars).strip()


def can_data_bytes(can_packet):
    frame_type, can_id, rtr_type, dlc, payload = can_packet
    data = list(payload or [])
    if len(data) > int(dlc):
        data = data[:int(dlc)]
    return {
        'frame_type': frame_type,
        'id': int(can_id),
        'rtr_type': rtr_type,
        'dlc': int(dlc),
        'data': data,
    }


def describe_351(data):
    return '0x351 SMA limits chgV={:.1f}V chgI={:.1f}A disI={:.1f}A lowV={:.1f}V'.format(
        le16(data, 0) / 10.0,
        le16(data, 2) / 10.0,
        le16(data, 4) / 10.0,
        le16(data, 6) / 10.0,
    )


def describe_355(data):
    return '0x355 SMA SOC={}%% SOH={}%% raw_tail={}'.format(
        le16(data, 0),
        le16(data, 2),
        format_data(data[4:]),
    )


def describe_356(data):
    return '0x356 SMA pack V={:.2f}V I={:+.1f}A temp={:.1f}C raw_tail={}'.format(
        le16(data, 0) / 100.0,
        le16s(data, 2) / 10.0,
        le16s(data, 4) / 10.0,
        format_data(data[6:]),
    )


def describe_raw(can_id, label, data):
    words = ', '.join(str(le16(data, pos)) for pos in range(0, len(data) - 1, 2))
    if words:
        return '0x{:03X} SMA {} raw={} u16=[{}]'.format(can_id, label, format_data(data), words)
    return '0x{:03X} SMA {} raw={}'.format(can_id, label, format_data(data))


def describe_ascii_or_raw(can_id, label, data):
    text = ascii_text(data)
    if text:
        return "0x{:03X} SMA {} '{}' raw={}".format(can_id, label, text, format_data(data))
    return describe_raw(can_id, label, data)


def describe_packet(can_packet):
    frame = can_data_bytes(can_packet)
    can_id = frame['id']
    data = frame['data']
    dlc = frame['dlc']

    if can_id == 0x351 and dlc >= 8:
        return describe_351(data)
    if can_id == 0x355 and dlc >= 4:
        return describe_355(data)
    if can_id == 0x356 and dlc >= 6:
        return describe_356(data)
    if can_id == 0x359:
        return describe_raw(can_id, 'alarms/status', data)
    if can_id == 0x35A:
        return describe_ascii_or_raw(can_id, 'vendor', data)
    if can_id == 0x35E:
        return describe_ascii_or_raw(can_id, 'manufacturer', data)
    if can_id == 0x35F:
        return describe_ascii_or_raw(can_id, 'battery info', data)

    name = FRAME_NAMES.get(can_id, 'unknown')
    return '0x{:03X} {} raw={}'.format(can_id, name, format_data(data))


def frame_summary(can_packet):
    frame = can_data_bytes(can_packet)
    name = FRAME_NAMES.get(frame['id'], 'SMA?')
    return 'SMA CAN {} 0x{:03X} {} DLC={} [{}]'.format(
        VERSION,
        frame['id'],
        name,
        frame['dlc'],
        format_data(frame['data']),
    )
