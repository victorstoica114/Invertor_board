##
## Pylon-compatible RS485 ASCII frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

REQUEST_NAMES = {
    0x61: 'analog/telemetry',
    0x62: 'system/info',
    0x63: 'charge/discharge status',
    0x42: 'telemetry',
    0x44: 'alarms',
}


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


def describe_info(frame, requested_cid2=None):
    info = frame.get('info_bytes', [])
    if frame.get('code') != 0x00:
        name = REQUEST_NAMES.get(frame.get('code'), 'request')
        if info:
            return 'request {} payload={}'.format(name, frame.get('info_ascii'))
        return 'request {}'.format(name)

    cid2 = infer_response_request(frame, requested_cid2)
    if cid2 == 0x63 and len(info) >= 9:
        status = info[8]
        return '0x63 status=0x{:02X} ({})'.format(status, status63_flags(status))

    if cid2 == 0x61 and len(info) >= 33:
        pack_v = be16(info, 0) / 1000.0
        current_a = be16s(info, 2) / 10.0
        soc = info[4]
        cycles = be16(info, 5)
        soh = info[9]
        max_mv = be16(info, 11)
        max_idx = be16(info, 13) & 0xff
        min_mv = be16(info, 15)
        min_idx = be16(info, 17) & 0xff
        return ('0x61 V={:.3f}V I={:.1f}A SOC={}%% SOH={}%% cycles={} '
                'cell_max={:.3f}V#{} cell_min={:.3f}V#{}').format(
                    pack_v, current_a, soc, soh, cycles,
                    max_mv / 1000.0, max_idx, min_mv / 1000.0, min_idx)

    if cid2 == 0x62:
        return '0x62 payload={}'.format(frame.get('info_ascii'))

    if frame.get('info_ascii'):
        return 'response OK payload={}'.format(frame.get('info_ascii'))
    return 'response OK'


def frame_summary(frame, direction='', pending_cid2=None):
    prefix = (direction + ' ') if direction else ''
    if is_request(frame):
        name = REQUEST_NAMES.get(frame['code'], 'request')
        return '{}Pylon req addr=0x{:02X} cid2=0x{:02X} {} chk={}'.format(
            prefix, frame['addr'], frame['code'], name,
            'OK' if frame['checksum_ok'] else 'BAD')

    if frame.get('cid1') == 0x46 and frame.get('code') == 0x00:
        cid2 = infer_response_request(frame, pending_cid2)
        cid_text = ' cid2=0x{:02X}'.format(cid2) if cid2 is not None else ''
        return '{}Pylon rsp addr=0x{:02X} OK{} chk={}'.format(
            prefix, frame['addr'], cid_text,
            'OK' if frame['checksum_ok'] else 'BAD')

    return '{}Pylon frame addr=0x{:02X} cid1=0x{:02X} code=0x{:02X} chk={}'.format(
        prefix, frame['addr'], frame['cid1'], frame['code'],
        'OK' if frame['checksum_ok'] else 'BAD')
