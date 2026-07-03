##
## Victron-compatible CAN frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FRAME_NAMES = {
    0x351: 'limits',
    0x355: 'SOC/SOH',
    0x356: 'pack',
    0x359: 'alarms/raw',
    0x35A: 'vendor/raw',
    0x35E: 'manufacturer',
    0x35F: 'battery/raw',
    0x360: 'status/raw',
    0x370: 'ASCII/raw',
    0x371: 'raw',
    0x372: 'raw',
    0x373: 'Pylon-style cell/temp tentative',
    0x374: 'ASCII segment',
    0x375: 'ASCII segment',
    0x376: 'ASCII segment',
    0x377: 'ASCII segment',
    0x378: 'raw',
    0x379: 'raw',
    0x380: 'ASCII segment',
    0x381: 'ASCII segment',
}

VERSION = 'v2026.07.03a'


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


def temp_raw_to_c(raw):
    return float(raw) if raw <= 200 else raw / 10.0


def cell_extremes_valid(cell_min_mv, cell_max_mv):
    return (
        1500 <= cell_min_mv <= 5000 and
        1500 <= cell_max_mv <= 5000 and
        cell_max_mv >= cell_min_mv
    )


def on_off(value):
    return 'ON' if value else 'OFF'


def describe_351(data):
    return '0x351 limits chgV={:.1f}V chgI={:.1f}A disI={:.1f}A lowV={:.1f}V flags={}'.format(
        le16(data, 0) / 10.0,
        le16(data, 2) / 10.0,
        le16(data, 4) / 10.0,
        le16(data, 6) / 10.0,
        'charge={} discharge={}'.format(on_off(le16(data, 2) > 0), on_off(le16(data, 4) > 0)),
    )


def describe_355(data):
    return '0x355 SOC={}%% SOH={}%% raw_tail={}'.format(
        le16(data, 0),
        le16(data, 2),
        format_data(data[4:]),
    )


def describe_356(data):
    return '0x356 pack V={:.2f}V I={:+.1f}A temp={:.1f}C raw_tail={}'.format(
        le16(data, 0) / 100.0,
        le16s(data, 2) / 10.0,
        le16s(data, 4) / 10.0,
        format_data(data[6:]),
    )


def describe_359(data):
    return '0x359 alarms/status raw={}'.format(format_data(data))


def describe_35a(data):
    return '0x35A vendor/raw raw={}'.format(format_data(data))


def describe_ascii_frame(can_id, data):
    text = ascii_text(data)
    if text:
        return "0x{:03X} ASCII '{}' raw={}".format(can_id, text, format_data(data))
    return '0x{:03X} raw={}'.format(can_id, format_data(data))


def describe_35f(data):
    return '0x35F battery/raw raw={} u16=[{}]'.format(
        format_data(data),
        ', '.join(str(le16(data, pos)) for pos in range(0, len(data) - 1, 2)),
    )


def describe_373(data):
    cell_min_mv = le16(data, 0)
    cell_max_mv = le16(data, 2)
    temp1 = temp_raw_to_c(le16(data, 4))
    temp2 = temp_raw_to_c(le16(data, 6))
    if cell_extremes_valid(cell_min_mv, cell_max_mv):
        return (
            '0x373 tentative Pylon-style cell_min={:.3f}V cell_max={:.3f}V '
            't1={:.1f}C t2={:.1f}C'
        ).format(cell_min_mv / 1000.0, cell_max_mv / 1000.0, temp1, temp2)
    return '0x373 tentative/raw raw={}'.format(format_data(data))


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
        return describe_359(data)
    if can_id == 0x35A:
        return describe_35a(data)
    if can_id in (0x35E, 0x370, 0x374, 0x375, 0x376, 0x377, 0x380, 0x381):
        return describe_ascii_frame(can_id, data)
    if can_id == 0x35F:
        return describe_35f(data)
    if can_id == 0x373 and dlc >= 8:
        return describe_373(data)

    name = FRAME_NAMES.get(can_id, 'unknown')
    return '0x{:03X} {} raw={}'.format(can_id, name, format_data(data))


def frame_summary(can_packet):
    frame = can_data_bytes(can_packet)
    name = FRAME_NAMES.get(frame['id'], 'Victron?')
    return 'Victron CAN {} 0x{:03X} {} DLC={} [{}]'.format(
        VERSION,
        frame['id'],
        name,
        frame['dlc'],
        format_data(frame['data']),
    )
