##
## Pylon-compatible RS485 ASCII frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

VERSION = 'v2026.08.01a'

REQUEST_NAMES = {
    0x61: 'analog/telemetry',
    0x62: 'alarm/status flags',
    0x63: 'charge/discharge status',
    0x42: 'cell information',
    0x44: 'alarms',
}

PYLON_MAX_CELLS = 32


def hex_nibble(ch):
    if isinstance(ch, int):
        ch = chr(ch)
    if '0' <= ch <= '9':
        return ord(ch) - ord('0')
    if 'A' <= ch <= 'F':
        return 10 + ord(ch) - ord('A')
    if 'a' <= ch <= 'f':
        return 10 + ord(ch) - ord('a')
    return -1


def parse_hex_byte(text, pos):
    hi = hex_nibble(text[pos])
    lo = hex_nibble(text[pos + 1])
    if hi < 0 or lo < 0:
        raise ValueError('invalid hex byte at position {}'.format(pos))
    return (hi << 4) | lo


def parse_hex_word(text, pos):
    return ((parse_hex_byte(text, pos) << 8) |
            parse_hex_byte(text, pos + 2))


def ascii_checksum(body):
    total = 0
    for ch in body:
        total += ord(ch)
    return (~total + 1) & 0xffff


def length_check_nibble(length_field):
    lenid = length_field & 0x0fff
    n0 = (lenid >> 8) & 0x0f
    n1 = (lenid >> 4) & 0x0f
    n2 = lenid & 0x0f
    return (~(n0 + n1 + n2) + 1) & 0x0f


def parse_info_bytes(info_ascii):
    if len(info_ascii) % 2 != 0:
        raise ValueError('payload hex length is odd')
    out = []
    for pos in range(0, len(info_ascii), 2):
        out.append(parse_hex_byte(info_ascii, pos))
    return out


def be16(data, pos):
    return (data[pos] << 8) | data[pos + 1]


def be16s(data, pos):
    value = be16(data, pos)
    if value & 0x8000:
        value -= 0x10000
    return value


def cell_mv_valid(mv):
    return 1500 <= mv <= 5000


def fmt_cells(cells):
    return ' '.join('C{:02d}={:.3f}V'.format(i + 1, mv / 1000.0)
                    for i, mv in enumerate(cells))


def hex_bytes(data):
    return ' '.join('{:02X}'.format(b) for b in data)


def cell_stats(cells):
    if not cells:
        return None
    min_idx = min(range(len(cells)), key=lambda i: cells[i])
    max_idx = max(range(len(cells)), key=lambda i: cells[i])
    avg = sum(cells) / float(len(cells))
    return {
        'count': len(cells),
        'min_mv': cells[min_idx],
        'max_mv': cells[max_idx],
        'min_idx': min_idx + 1,
        'max_idx': max_idx + 1,
        'avg_mv': avg,
    }


def try_parse_info42_simple_cells(info, start_pos=0):
    if start_pos >= len(info):
        return None
    pos = start_pos
    count = info[pos]
    pos += 1
    if count == 0 or count > PYLON_MAX_CELLS:
        return None
    if pos + count * 2 > len(info):
        return None

    cells = []
    for _ in range(count):
        mv = be16(info, pos)
        pos += 2
        if not cell_mv_valid(mv):
            return None
        cells.append(mv)
    return cells


def try_parse_info42_pack_cells(info):
    if len(info) < 2:
        return None

    pos = 0
    pack_count = info[pos]
    pos += 1
    if pack_count == 0 or pack_count > 16:
        return None

    cells = []
    for pack in range(pack_count):
        if pos >= len(info):
            return None
        count = info[pos]
        pos += 1
        if count == 0 or count > PYLON_MAX_CELLS:
            return None
        if pos + count * 2 > len(info):
            return None

        for _ in range(count):
            mv = be16(info, pos)
            pos += 2
            if not cell_mv_valid(mv):
                return None
            if len(cells) < PYLON_MAX_CELLS:
                cells.append(mv)

        if pos >= len(info):
            break

        temp_count = info[pos]
        pos += 1
        if temp_count > 16 or pos + temp_count * 2 > len(info):
            return None
        pos += temp_count * 2

        # Pylon-style 0x42 may append per-pack current, voltage, capacity,
        # cycle count, and compatibility bytes. Some compatible BMS profiles
        # trim this tail, so only require the inter-pack fixed area.
        if pack + 1 < pack_count:
            if pos + 12 > len(info):
                return None
            pos += 12

    return cells if cells else None


def describe_info42_cells(info):
    cells = try_parse_info42_pack_cells(info)
    layout = 'pack'
    if cells is None:
        cells = try_parse_info42_simple_cells(info, 0)
        layout = 'simple'
    if cells is None:
        return None

    stats = cell_stats(cells)
    return ('0x42 cells {} layout count={} min={:.3f}V#{} max={:.3f}V#{} '
            'avg={:.3f}V {}').format(
                layout,
                stats['count'],
                stats['min_mv'] / 1000.0,
                stats['min_idx'],
                stats['max_mv'] / 1000.0,
                stats['max_idx'],
                stats['avg_mv'] / 1000.0,
                fmt_cells(cells))


def describe_request_payload(cid2, info):
    if cid2 == 0x42:
        if not info:
            return 'empty selector'
        if len(info) == 1:
            if info[0] == 0xff:
                return 'selector=0xFF aggregate/all packs'
            return 'selector=0x{:02X}'.format(info[0])
    if not info:
        return 'empty payload'
    return 'payload=' + ''.join('{:02X}'.format(b) for b in info)


def kelvin_c10_to_c(value):
    return (value - 2731) / 10.0


def describe_61(frame):
    info = frame.get('info_bytes', [])
    if len(info) < 33:
        return None

    temps = [
        ('MOS', kelvin_c10_to_c(be16(info, 19))),
        ('T1', kelvin_c10_to_c(be16(info, 21))),
        ('T2', kelvin_c10_to_c(be16(info, 25))),
        ('T4', kelvin_c10_to_c(be16(info, 29))),
        ('T5', kelvin_c10_to_c(be16(info, 31))),
    ]

    return {
        'pack_v': be16(info, 0) / 1000.0,
        'current_a': be16s(info, 2) / 100.0,
        'soc': info[4],
        'cycles': be16(info, 5),
        'soh': info[9],
        'max_mv': be16(info, 11),
        'max_idx': be16(info, 13) & 0xff,
        'min_mv': be16(info, 15),
        'min_idx': be16(info, 17) & 0xff,
        'temps': temps,
        'temp_text': ' '.join('{}={:.1f}C'.format(name, value)
                              for name, value in temps),
    }


def describe_payload_texts(frame, requested_cid2=None):
    info = frame.get('info_bytes', [])
    if not info:
        return ['INFO empty', 'empty']

    cid2 = frame.get('code')
    if cid2 == 0x00:
        cid2 = infer_response_request(frame, requested_cid2)

    raw = hex_bytes(info)
    if cid2 in REQUEST_NAMES:
        return ['0x{:02X} payload {}'.format(cid2, raw),
                '0x{:02X} payload {}B'.format(cid2, len(info)),
                'payload {}B'.format(len(info)),
                'INFO']
    return ['payload {}'.format(raw), 'payload {}B'.format(len(info)), 'INFO']


def parse_frame(raw):
    if isinstance(raw, bytes):
        text = raw.decode('ascii', errors='replace')
    else:
        text = raw

    if text.endswith('\n'):
        text = text[:-1]
    if len(text) < 18:
        raise ValueError('frame too short')
    if text[0] != '~':
        raise ValueError('missing start marker')
    if text[-1] != '\r':
        raise ValueError('missing CR terminator')

    body = text[1:-5]
    checksum_text = text[-5:-1]
    for ch in body + checksum_text:
        if hex_nibble(ch) < 0:
            raise ValueError('non-hex character in frame')

    ver = parse_hex_byte(body, 0)
    addr = parse_hex_byte(body, 2)
    cid1 = parse_hex_byte(body, 4)
    code = parse_hex_byte(body, 6)
    length_field = parse_hex_word(body, 8)
    info_ascii = body[12:]
    info_len = length_field & 0x0fff
    lchksum = (length_field >> 12) & 0x0f
    expected_lchksum = length_check_nibble(length_field)
    checksum = int(checksum_text, 16)
    expected_checksum = ascii_checksum(body)

    info_bytes = parse_info_bytes(info_ascii)

    return {
        'raw': text,
        'body': body,
        'ver': ver,
        'addr': addr,
        'cid1': cid1,
        'code': code,
        'length_field': length_field,
        'info_len': info_len,
        'info_ascii': info_ascii,
        'info_bytes': info_bytes,
        'lchksum': lchksum,
        'expected_lchksum': expected_lchksum,
        'length_ok': lchksum == expected_lchksum and info_len == len(info_ascii),
        'checksum': checksum,
        'expected_checksum': expected_checksum,
        'checksum_ok': checksum == expected_checksum,
    }


def is_request(frame):
    return frame.get('cid1') == 0x46 and frame.get('code') in REQUEST_NAMES


def infer_response_request(frame, pending_cid2=None):
    if pending_cid2 is not None:
        return pending_cid2
    info = frame.get('info_bytes', [])
    if len(info) == 9:
        return 0x63
    if len(info) >= 33:
        return 0x61
    if len(info) == 4:
        return 0x62
    return None


def status63_flags(status):
    flags = []
    flags.append('charge={}'.format('ON' if (status & 0x80) else 'OFF'))
    flags.append('discharge={}'.format('ON' if (status & 0x40) else 'OFF'))
    flags.append('balance={}'.format('ON' if (status & 0x20) else 'OFF'))
    return ', '.join(flags)


def short_request_name(cid2):
    return {
        0x42: 'cells',
        0x61: 'telem',
        0x62: 'flags',
        0x63: 'status',
        0x44: 'alarms',
    }.get(cid2, 'cmd')


def describe_info(frame, requested_cid2=None):
    info = frame.get('info_bytes', [])
    if frame.get('code') != 0x00:
        name = REQUEST_NAMES.get(frame.get('code'), 'request')
        return 'request {} {}'.format(name,
                                      describe_request_payload(frame.get('code'), info))

    cid2 = infer_response_request(frame, requested_cid2)
    if not info:
        if cid2 in REQUEST_NAMES:
            return '0x{:02X} empty response/no payload ({})'.format(
                cid2, REQUEST_NAMES[cid2])
        return 'response OK empty/no payload'

    if cid2 == 0x42:
        decoded = describe_info42_cells(info)
        if decoded is not None:
            return decoded
        return '0x42 payload present but no valid cell list ({})'.format(
            frame.get('info_ascii'))

    if cid2 == 0x63 and len(info) >= 9:
        status = info[8]
        return '0x63 status=0x{:02X} ({})'.format(status, status63_flags(status))

    if cid2 == 0x61 and len(info) >= 33:
        decoded = describe_61(frame)
        return ('0x61 V={:.3f}V I={:.2f}A SOC={}%% SOH={}%% cycles={} '
                'cell_max={:.3f}V#{} cell_min={:.3f}V#{} {}').format(
                    decoded['pack_v'], decoded['current_a'],
                    decoded['soc'], decoded['soh'], decoded['cycles'],
                    decoded['max_mv'] / 1000.0, decoded['max_idx'],
                    decoded['min_mv'] / 1000.0, decoded['min_idx'],
                    decoded['temp_text'])

    if cid2 == 0x62:
        flags = int(frame.get('info_ascii') or '0', 16)
        if flags == 0:
            return '0x62 flags=0x{} (no flags set)'.format(frame.get('info_ascii'))
        return '0x62 flags=0x{} raw={}'.format(frame.get('info_ascii'),
                                               frame.get('info_ascii'))

    if frame.get('info_ascii'):
        return 'response OK payload={}'.format(frame.get('info_ascii'))
    return 'response OK'


def describe_info_texts(frame, requested_cid2=None):
    long_text = describe_info(frame, requested_cid2)
    info = frame.get('info_bytes', [])

    if frame.get('code') != 0x00:
        cid2 = frame.get('code')
        if info:
            return [long_text,
                    'req 0x{:02X} {}'.format(cid2, frame.get('info_ascii')),
                    'req 0x{:02X}'.format(cid2),
                    'req']
        return [long_text, 'req 0x{:02X} empty'.format(cid2), 'req 0x{:02X}'.format(cid2), 'req']

    cid2 = infer_response_request(frame, requested_cid2)
    if not info:
        if cid2 is not None:
            return [long_text, '0x{:02X} empty'.format(cid2), 'empty']
        return [long_text, 'empty']

    if cid2 == 0x63 and len(info) >= 9:
        status = info[8]
        charge = 'ON' if (status & 0x80) else 'OFF'
        discharge = 'ON' if (status & 0x40) else 'OFF'
        balance = 'ON' if (status & 0x20) else 'OFF'
        return [long_text,
                '0x63 C={} D={} B={}'.format(charge, discharge, balance),
                '0x63 C/D {}'.format('ON' if (status & 0xC0) == 0xC0 else 'state'),
                '0x63']

    if cid2 == 0x61 and len(info) >= 33:
        decoded = describe_61(frame)
        return [long_text,
                '0x61 V={:.3f} SOC={} min={:.3f}#{} max={:.3f}#{}'.format(
                    decoded['pack_v'], decoded['soc'],
                    decoded['min_mv'] / 1000.0, decoded['min_idx'],
                    decoded['max_mv'] / 1000.0, decoded['max_idx']),
                '0x61 SOC={} T={:.1f}C'.format(decoded['soc'], decoded['temps'][0][1]),
                '0x61 SOC={} min/max'.format(decoded['soc']),
                '0x61']

    if cid2 == 0x62:
        flags = int(frame.get('info_ascii') or '0', 16)
        if flags == 0:
            return [long_text, '0x62 flags=0', '0x62']
        return [long_text, '0x62 flags=0x{:X}'.format(flags), '0x62']

    if cid2 == 0x42:
        decoded = describe_info42_cells(info)
        if decoded is not None:
            cells = try_parse_info42_pack_cells(info) or try_parse_info42_simple_cells(info, 0) or []
            stats = cell_stats(cells)
            return [long_text,
                    '0x42 cells={} min={:.3f} max={:.3f}'.format(
                        stats['count'], stats['min_mv'] / 1000.0, stats['max_mv'] / 1000.0),
                    '0x42 cells={}'.format(stats['count']),
                    '0x42']
        return [long_text, '0x42 no cell list', '0x42']

    return [long_text, 'decoded']


def frame_summary(frame, direction='', pending_cid2=None):
    prefix = (direction + ' ') if direction else ''
    if is_request(frame):
        name = REQUEST_NAMES.get(frame['code'], 'request')
        empty = ' empty' if not frame.get('info_ascii') else ''
        return '{}Pylon req addr=0x{:02X} cid2=0x{:02X} {} chk={} len={}{}'.format(
            prefix, frame['addr'], frame['code'], name,
            'OK' if frame['checksum_ok'] else 'BAD',
            frame.get('info_len', 0), empty)

    if frame.get('cid1') == 0x46 and frame.get('code') == 0x00:
        cid2 = infer_response_request(frame, pending_cid2)
        cid_text = ' cid2=0x{:02X}'.format(cid2) if cid2 is not None else ''
        name = REQUEST_NAMES.get(cid2, None)
        name_text = ' {}'.format(name) if name else ''
        empty = ' empty' if not frame.get('info_ascii') else ''
        return '{}Pylon rsp addr=0x{:02X} OK{} chk={}{} len={}{}'.format(
            prefix, frame['addr'], cid_text,
            'OK' if frame['checksum_ok'] else 'BAD',
            name_text, frame.get('info_len', 0), empty)

    return '{}Pylon frame addr=0x{:02X} cid1=0x{:02X} code=0x{:02X} chk={}'.format(
        prefix, frame['addr'], frame['cid1'], frame['code'],
        'OK' if frame['checksum_ok'] else 'BAD')


def frame_summary_texts(frame, direction='', pending_cid2=None):
    long_text = frame_summary(frame, direction, pending_cid2)
    prefix = (direction + ' ') if direction else ''
    chk = 'OK' if frame.get('checksum_ok') else 'BAD'

    if is_request(frame):
        cid2 = frame['code']
        return [long_text,
                '{}req 0x{:02X} {} len={} chk={}'.format(
                    prefix, cid2, short_request_name(cid2), frame.get('info_len', 0), chk),
                '{}req 0x{:02X}'.format(prefix, cid2),
                'req']

    if frame.get('cid1') == 0x46 and frame.get('code') == 0x00:
        cid2 = infer_response_request(frame, pending_cid2)
        if cid2 is not None:
            empty = ' empty' if not frame.get('info_ascii') else ''
            return [long_text,
                    '{}rsp 0x{:02X} {} len={} chk={}{}'.format(
                        prefix, cid2, short_request_name(cid2), frame.get('info_len', 0), chk, empty),
                    '{}rsp 0x{:02X}'.format(prefix, cid2),
                    'rsp']
        return [long_text, '{}rsp len={} chk={}'.format(prefix, frame.get('info_len', 0), chk), 'rsp']

    return [long_text,
            '{}frame code=0x{:02X} chk={}'.format(prefix, frame.get('code', 0), chk),
            'frame']
