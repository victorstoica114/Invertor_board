##
## JKBMS native RS485 binary frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

START = (0x4E, 0x57)
COMMAND_READ_ALL = 0x06
END_ID = 0x68
MAX_FRAME_LEN = 512

ID_NAMES = {
    0x79: 'cell voltages',
    0x80: 'MOS/tube temperature',
    0x81: 'box temperature',
    0x82: 'battery temperature',
    0x83: 'total voltage',
    0x84: 'total current',
    0x85: 'SOC',
    0x86: 'temperature sensor count',
    0x87: 'cycle count',
    0x89: 'total capacity',
    0x8A: 'battery string count',
    0x8B: 'alarms',
    0x8C: 'status',
    0x8E: 'pack overvoltage limit',
    0x8F: 'pack undervoltage limit',
    0x90: 'cell overvoltage limit',
    0x93: 'cell undervoltage limit',
    0x97: 'discharge current limit',
    0x99: 'charge current limit',
    0xAA: 'rated capacity',
    0xAF: 'battery type',
    0xB7: 'software version',
    0xBA: 'manufacturer',
}

FIXED_LENGTHS = {
    0x80: 2, 0x81: 2, 0x82: 2, 0x83: 2, 0x84: 2, 0x87: 2, 0x89: 4,
    0x8A: 2, 0x8B: 2, 0x8C: 2, 0x8E: 2, 0x8F: 2, 0x90: 2, 0x91: 2,
    0x92: 2, 0x93: 2, 0x94: 2, 0x95: 2, 0x96: 2, 0x97: 2, 0x98: 2,
    0x99: 2, 0x9A: 2, 0x9B: 2, 0x9C: 2, 0x9E: 2, 0x9F: 2, 0xA0: 2,
    0xA1: 2, 0xA2: 2, 0xA3: 2, 0xA4: 2, 0xA5: 2, 0xA6: 2, 0xA7: 2,
    0xA8: 2, 0xAD: 2, 0xB0: 2,
    0x85: 1, 0x86: 1, 0x9D: 1, 0xA9: 1, 0xAB: 1, 0xAC: 1, 0xAE: 1,
    0xAF: 1, 0xB1: 1, 0xB3: 1, 0xB8: 1,
    0xAA: 4, 0xB5: 4, 0xB6: 4, 0xB9: 4,
    0xB2: 10, 0xB4: 8, 0xB7: 15, 0xBA: 24, 0xC0: 5,
}

WARNING_NAMES = {
    0x0001: 'SOC low',
}

PROTECTION_NAMES = {
    0x0002: 'Module temperature high',
    0x0004: 'Charge voltage high',
    0x0008: 'Discharge voltage low',
    0x0010: 'Pack temperature high',
    0x0020: 'Charge current high',
    0x0040: 'Discharge current high',
    0x0080: 'Cell voltage delta high',
    0x0100: 'Enclosure temperature high',
    0x0200: 'Pack temperature low',
    0x0400: 'Pack voltage high',
    0x0800: 'Pack voltage low',
}

ALARM_NAMES = dict(WARNING_NAMES)
ALARM_NAMES.update(PROTECTION_NAMES)
ALARM_NAMES.update({
    0x1000: 'Other fault 1',
    0x2000: 'Other fault 2',
})


def be16(data, pos=0):
    return (data[pos] << 8) | data[pos + 1]


def be32(data, pos=0):
    return ((data[pos] << 24) | (data[pos + 1] << 16) |
            (data[pos + 2] << 8) | data[pos + 3])


def hex_bytes(data):
    return ' '.join('{:02X}'.format(byte & 0xFF) for byte in data)


def ascii_text(data):
    chars = []
    for value in data:
        chars.append(chr(value) if 32 <= value <= 126 else '.')
    return ''.join(chars).rstrip('. ')


def decode_temp_c(raw):
    return -(raw - 100) if raw > 100 else raw


def decode_current_a(raw):
    magnitude = raw & 0x7FFF
    amps = magnitude / 100.0
    return amps if raw & 0x8000 else -amps


def expected_frame_len(raw):
    raw = bytes(raw)
    if len(raw) < 4:
        return None
    if raw[0] != START[0] or raw[1] != START[1]:
        return None
    length = be16(raw, 2) + 2
    if length < 13 or length > MAX_FRAME_LEN:
        return None
    return length


def frame_complete(raw):
    length = expected_frame_len(raw)
    return length is not None and len(raw) >= length


def checksum_expected(raw):
    raw = bytes(raw)
    if len(raw) < 4:
        return 0
    return sum(raw[:-4]) & 0xFFFF


def checksum_frame(raw):
    raw = bytes(raw)
    if len(raw) < 4:
        return 0
    return be16(raw, len(raw) - 2)


def checksum_ok(raw):
    return checksum_expected(raw) == checksum_frame(raw)


def data_len_for_id(entry_id, raw, pos):
    if entry_id == 0x79:
        if pos >= len(raw):
            raise ValueError('cell-voltage entry missing length byte')
        return raw[pos], True
    if entry_id == END_ID:
        return 0, False
    if entry_id not in FIXED_LENGTHS:
        return None, False
    return FIXED_LENGTHS[entry_id], False


def parse_entries(raw, payload_start=11, payload_end=None):
    entries = []
    pos = payload_start
    if payload_end is None:
        payload_end = len(raw) - 4

    while pos < payload_end:
        entry_start = pos
        entry_id = raw[pos]
        pos += 1
        if entry_id == END_ID:
            entries.append({
                'id': entry_id,
                'name': 'end',
                'data': b'',
                'start_index': entry_start,
                'end_index': entry_start,
                'data_start_index': None,
                'data_end_index': None,
                'length_index': None,
            })
            break

        data_len, has_len_byte = data_len_for_id(entry_id, raw, pos)
        length_index = None
        if data_len is None:
            entries.append({
                'id': entry_id,
                'name': 'unknown',
                'data': b'',
                'start_index': entry_start,
                'end_index': entry_start,
                'data_start_index': None,
                'data_end_index': None,
                'length_index': None,
            })
            continue

        if has_len_byte:
            length_index = pos
            pos += 1

        if pos + data_len > payload_end:
            raise ValueError('entry 0x{:02X} exceeds frame length'.format(entry_id))

        data = bytes(raw[pos:pos + data_len])
        data_start = pos if data_len else None
        data_end = pos + data_len - 1 if data_len else None
        pos += data_len
        entries.append({
            'id': entry_id,
            'name': ID_NAMES.get(entry_id, 'id_0x{:02X}'.format(entry_id)),
            'data': data,
            'start_index': entry_start,
            'end_index': pos - 1,
            'data_start_index': data_start,
            'data_end_index': data_end,
            'length_index': length_index,
        })

    return entries


def parse_frame(raw):
    raw = bytes(raw)
    length = expected_frame_len(raw)
    if length is None:
        raise ValueError('invalid JK native start/length')
    if len(raw) < length:
        raise ValueError('incomplete frame')
    raw = raw[:length]

    if raw[8] != COMMAND_READ_ALL:
        raise ValueError('unsupported command 0x{:02X}'.format(raw[8]))

    entries = parse_entries(raw, 11, length - 4)
    return {
        'raw': raw,
        'length': length,
        'declared_length': be16(raw, 2),
        'terminal': raw[4:8],
        'command': raw[8],
        'source': raw[9],
        'frame_type': raw[10],
        'entries': entries,
        'checksum': checksum_frame(raw),
        'expected_checksum': checksum_expected(raw),
        'checksum_ok': checksum_ok(raw),
    }


def alert_list(bits, mapping, append_unknown=False):
    parts = []
    known = 0
    for mask, name in mapping.items():
        known |= mask
        if bits & mask:
            parts.append(name)
    if append_unknown:
        unknown = bits & (~known)
        for bit in range(16):
            if unknown & (1 << bit):
                parts.append('Unknown bit {}'.format(bit))
    return ', '.join(parts) if parts else 'none'


def decode_cells(data):
    cells = []
    for pos in range(0, len(data) - 2, 3):
        cell_no = data[pos]
        mv = be16(data, pos + 1)
        if cell_no and mv:
            cells.append((cell_no, mv))
    return cells


def describe_entry(entry):
    data = entry.get('data', b'')
    entry_id = entry.get('id')

    if entry_id == 0x79:
        cells = decode_cells(data)
        if not cells:
            return '0x79 cells none'
        min_cell = min(cells, key=lambda item: item[1])
        max_cell = max(cells, key=lambda item: item[1])
        return '0x79 cells count={} min={:.3f}V#{} max={:.3f}V#{} dV={}mV'.format(
            len(cells),
            min_cell[1] / 1000.0,
            min_cell[0],
            max_cell[1] / 1000.0,
            max_cell[0],
            max_cell[1] - min_cell[1],
        )
    if entry_id in (0x80, 0x81, 0x82) and len(data) >= 2:
        return '0x{:02X} {}={}C'.format(entry_id, ID_NAMES.get(entry_id), decode_temp_c(be16(data)))
    if entry_id == 0x83 and len(data) >= 2:
        return '0x83 pack_v={:.2f}V'.format(be16(data) / 100.0)
    if entry_id == 0x84 and len(data) >= 2:
        return '0x84 pack_i={:+.2f}A'.format(decode_current_a(be16(data)))
    if entry_id == 0x85 and len(data) >= 1:
        return '0x85 SOC={}%%'.format(min(data[0], 100))
    if entry_id == 0x86 and len(data) >= 1:
        return '0x86 temp_sensors={}'.format(data[0])
    if entry_id == 0x87 and len(data) >= 2:
        return '0x87 cycles={}'.format(be16(data))
    if entry_id == 0x8A and len(data) >= 2:
        return '0x8A strings={}'.format(be16(data))
    if entry_id == 0x8B and len(data) >= 2:
        bits = data[0] | (data[1] << 8)
        return '0x8B alarms=0x{:04X} {}'.format(bits, alert_list(bits, ALARM_NAMES, True))
    if entry_id == 0x8C and len(data) >= 2:
        flags = data[1]
        return '0x8C status=0x{:04X} charge={} discharge={} balance={}'.format(
            be16(data),
            'ON' if flags & 0x01 else 'OFF',
            'ON' if flags & 0x02 else 'OFF',
            'ON' if flags & 0x04 else 'OFF',
        )
    if entry_id == 0x8E and len(data) >= 2:
        return '0x8E pack_ov_limit={:.1f}V'.format(be16(data) / 10.0)
    if entry_id == 0x8F and len(data) >= 2:
        return '0x8F pack_uv_limit={:.1f}V'.format(be16(data) / 10.0)
    if entry_id == 0x90 and len(data) >= 2:
        return '0x90 cell_ov_limit={:.3f}V'.format(be16(data) / 1000.0)
    if entry_id == 0x93 and len(data) >= 2:
        return '0x93 cell_uv_limit={:.3f}V'.format(be16(data) / 1000.0)
    if entry_id == 0x97 and len(data) >= 2:
        return '0x97 discharge_limit={:.1f}A'.format(be16(data) / 10.0)
    if entry_id == 0x99 and len(data) >= 2:
        return '0x99 charge_limit={:.1f}A'.format(be16(data) / 10.0)
    if entry_id == 0xAA and len(data) >= 4:
        return '0xAA rated_capacity={}Ah'.format(be32(data))
    if entry_id in (0xB7, 0xBA):
        return '0x{:02X} {}="{}"'.format(entry_id, ID_NAMES.get(entry_id), ascii_text(data))

    return '0x{:02X} {} raw={}'.format(entry_id, entry.get('name', 'entry'), hex_bytes(data))


def frame_summary(frame, direction=''):
    prefix = (direction + ' ') if direction else ''
    crc_text = 'OK' if frame.get('checksum_ok') else 'BAD'
    source = frame.get('source', 0)
    frame_type = frame.get('frame_type', 0)
    typ = 'rsp' if frame_type == 0x01 else 'req' if source == 0x03 else 'frame'
    return '{}JKBMS native {} len={} src=0x{:02X} type=0x{:02X} chk={}'.format(
        prefix,
        typ,
        frame.get('length', 0),
        source,
        frame_type,
        crc_text,
    )


def describe_frame(frame):
    entries = [entry for entry in frame.get('entries', []) if entry.get('id') != END_ID]
    highlights = []
    for entry in entries:
        entry_id = entry.get('id')
        if entry_id in (0x79, 0x83, 0x84, 0x85, 0x8B, 0x8C):
            highlights.append(describe_entry(entry))
        if len(highlights) >= 6:
            break

    if highlights:
        return '; '.join(highlights)
    if entries:
        return '{} entries: {}'.format(
            len(entries),
            ', '.join('0x{:02X}'.format(entry['id']) for entry in entries[:12]),
        )
    return 'read-all request'
