##
## Pylon-compatible CAN frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FRAME_NAMES = {
    0x351: 'limits',
    0x355: 'SOC/SOH',
    0x356: 'pack',
    0x359: 'module info',
    0x35C: 'status',
    0x35E: 'ASCII name',
    0x370: 'JK ext cell/temp',
    0x371: 'JK ext indexes',
    0x373: 'cell/temp',
}


def le16(data, pos):
    return data[pos] | (data[pos + 1] << 8)


def le16s(data, pos):
    value = le16(data, pos)
    if value & 0x8000:
        value -= 0x10000
    return value


def ascii_text(data):
    chars = []
    for value in data:
        chars.append(chr(value) if 32 <= value <= 126 else '.')
    return ''.join(chars).rstrip('. ')


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


def status_flags(status):
    return 'charge={}, discharge={}, balance={}'.format(
        'ON' if (status & 0x80) else 'OFF',
        'ON' if (status & 0x40) else 'OFF',
        'ON' if (status & 0x20) else 'OFF',
    )


def temp_raw(raw):
    return float(raw) if raw <= 200 else float(raw) / 10.0


def cell_extremes_valid(cell_min_mv, cell_max_mv):
    return (
        1500 <= cell_min_mv <= 5000 and
        1500 <= cell_max_mv <= 5000 and
        cell_max_mv >= cell_min_mv
    )


def describe_packet(can_packet):
    frame = can_data_bytes(can_packet)
    can_id = frame['id']
    data = frame['data']
    dlc = frame['dlc']

    if can_id == 0x351 and dlc >= 8:
        return (
            '0x351 limits chgV={:.1f}V chgI={:.1f}A disI={:.1f}A lowV={:.1f}V'
        ).format(
            le16(data, 0) / 10.0,
            le16(data, 2) / 10.0,
            le16(data, 4) / 10.0,
            le16(data, 6) / 10.0,
        )

    if can_id == 0x355 and dlc >= 4:
        return '0x355 SOC={}%% SOH={}%%'.format(le16(data, 0), le16(data, 2))

    if can_id == 0x356 and dlc >= 6:
        return '0x356 pack V={:.2f}V I={:.1f}A temp={:.1f}C'.format(
            le16(data, 0) / 100.0,
            le16s(data, 2) / 10.0,
            le16(data, 4) / 10.0,
        )

    if can_id == 0x359:
        module_count = data[4] if dlc >= 5 else 0
        return '0x359 module info modules={} raw={}'.format(
            module_count,
            format_data(data),
        )

    if can_id == 0x35C and dlc >= 1:
        status = data[0]
        return '0x35C status=0x{:02X} ({})'.format(status, status_flags(status))

    if can_id == 0x35E:
        return "0x35E name='{}'".format(ascii_text(data))

    if can_id == 0x370 and dlc >= 8:
        cell_max_mv = le16(data, 4)
        cell_min_mv = le16(data, 6)
        if cell_extremes_valid(cell_min_mv, cell_max_mv):
            return '0x370 JK ext tMax={:.1f}C tMin={:.1f}C cell_max={:.3f}V cell_min={:.3f}V'.format(
                temp_raw(le16(data, 0)),
                temp_raw(le16(data, 2)),
                cell_max_mv / 1000.0,
                cell_min_mv / 1000.0,
            )
        return '0x370 JK ext raw={}'.format(format_data(data))

    if can_id == 0x371 and dlc >= 8:
        return '0x371 JK ext idx tMax#{} tMin#{} cellMax#{} cellMin#{}'.format(
            le16(data, 0),
            le16(data, 2),
            le16(data, 4),
            le16(data, 6),
        )

    if can_id == 0x373 and dlc >= 8:
        cell_min_mv = le16(data, 0)
        cell_max_mv = le16(data, 2)
        if cell_extremes_valid(cell_min_mv, cell_max_mv):
            return '0x373 cell_min={:.3f}V cell_max={:.3f}V t1={:.1f}C t2={:.1f}C'.format(
                cell_min_mv / 1000.0,
                cell_max_mv / 1000.0,
                le16(data, 4) / 10.0,
                le16(data, 6) / 10.0,
            )
        return '0x373 raw={}'.format(format_data(data))

    name = FRAME_NAMES.get(can_id, 'unknown')
    return '0x{:03X} {} raw={}'.format(can_id, name, format_data(data))


def frame_summary(can_packet):
    frame = can_data_bytes(can_packet)
    name = FRAME_NAMES.get(frame['id'], 'Pylon?')
    return 'Pylon CAN 0x{:03X} {} DLC={} [{}]'.format(
        frame['id'],
        name,
        frame['dlc'],
        format_data(frame['data']),
    )


def format_data(data):
    return ' '.join('{:02X}'.format(value & 0xFF) for value in data)

