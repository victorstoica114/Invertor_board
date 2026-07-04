##
## Sofar-compatible low-voltage CAN frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FRAME_NAMES = {
    0x351: 'limits',
    0x355: 'SOC/SOH',
    0x356: 'pack',
    0x359: 'module info',
    0x35C: 'status',
    0x35E: 'brand',
    0x35F: 'module raw',
    0x370: 'temperature/cell extremes',
    0x371: 'extreme indexes',
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


def on_off(value):
    return 'ON' if value else 'OFF'


def temp_word_to_c(raw):
    if raw <= 200:
        return float(raw)
    return raw / 10.0


def u16_words(data):
    words = []
    for pos in range(0, len(data) - 1, 2):
        words.append(le16(data, pos))
    return words


def describe_351(data):
    return '0x351 Sofar limits chgV={:.1f}V chgI={:.1f}A disI={:.1f}A lowV={:.1f}V'.format(
        le16(data, 0) / 10.0,
        le16(data, 2) / 10.0,
        le16(data, 4) / 10.0,
        le16(data, 6) / 10.0,
    )


def describe_355(data):
    return '0x355 Sofar SOC={}%% SOH={}%%'.format(le16(data, 0), le16(data, 2))


def describe_356(data):
    return '0x356 Sofar pack V={:.2f}V I={:+.1f}A temp={:.1f}C'.format(
        le16(data, 0) / 100.0,
        le16s(data, 2) / 10.0,
        le16s(data, 4) / 10.0,
    )


def describe_359(data):
    parts = []
    if len(data) >= 5:
        parts.append('modules={}'.format(data[4]))
    if len(data) >= 6:
        text = ascii_text(data[5:])
        if text:
            parts.append("tag='{}'".format(text))
    parts.append('raw={}'.format(format_data(data)))
    return '0x359 Sofar module info {}'.format(' '.join(parts))


def describe_35c(data):
    state = data[0] if data else 0
    return '0x35C Sofar state raw=0x{:02X} charge={} discharge={} balance={} raw={}'.format(
        state,
        on_off(state & 0x80),
        on_off(state & 0x40),
        on_off(state & 0x20),
        format_data(data),
    )


def describe_35e(data):
    text = ascii_text(data)
    if text:
        return "0x35E Sofar brand '{}' raw={}".format(text, format_data(data))
    return '0x35E Sofar brand raw={}'.format(format_data(data))


def describe_35f(data):
    words = u16_words(data)
    text = ascii_text(data)
    if text:
        return "0x35F Sofar module raw='{}' u16={} raw={}".format(text, words, format_data(data))
    return '0x35F Sofar module u16={} raw={}'.format(words, format_data(data))


def describe_370(data):
    return '0x370 Sofar temp max={:.1f}C min={:.1f}C cell_max={:.3f}V cell_min={:.3f}V'.format(
        temp_word_to_c(le16(data, 0)),
        temp_word_to_c(le16(data, 2)),
        le16(data, 4) / 1000.0,
        le16(data, 6) / 1000.0,
    )


def describe_371(data):
    return '0x371 Sofar temp_max_sensor={} temp_min_sensor={} cell_max_idx={} cell_min_idx={}'.format(
        le16(data, 0),
        le16(data, 2),
        le16(data, 4),
        le16(data, 6),
    )


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
    if can_id == 0x35C:
        return describe_35c(data)
    if can_id == 0x35E:
        return describe_35e(data)
    if can_id == 0x35F:
        return describe_35f(data)
    if can_id == 0x370 and dlc >= 8:
        return describe_370(data)
    if can_id == 0x371 and dlc >= 8:
        return describe_371(data)

    name = FRAME_NAMES.get(can_id, 'unknown')
    return '0x{:03X} {} raw={}'.format(can_id, name, format_data(data))


def frame_summary(can_packet):
    frame = can_data_bytes(can_packet)
    name = FRAME_NAMES.get(frame['id'], 'Sofar?')
    return 'Sofar CAN {} 0x{:03X} {} DLC={} [{}]'.format(
        VERSION,
        frame['id'],
        name,
        frame['dlc'],
        format_data(frame['data']),
    )
