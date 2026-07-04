##
## Daly RS485 native and Modbus frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

VERSION = 'v2026.07.04a'

DALY_START_BYTE = 0xA5
DALY_FRAME_LEN = 13
DALY_PAYLOAD_LEN = 0x08
DALY_DEFAULT_BMS_ID = 0x01
DALY_RS485_REQUEST_ADDR = 0x40
DALY_UART_REQUEST_ADDR = 0x80

DALY_MODBUS_REQUEST_ADDR = 0x81
DALY_MODBUS_ALT_REQUEST_ADDR = 0xD2
DALY_MODBUS_OBSERVED_RESPONSE_ADDR = 0x51

DALY_MODBUS_READ_CELLS_START = 0x0000
DALY_MODBUS_READ_CELLS_COUNT = 0x007F
DALY_MODBUS_READ_INFO_START = 0x0100
DALY_MODBUS_READ_INFO_COUNT = 0x0078
DALY_MODBUS_SOC_DECI_REG_INDEX = 58
DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX = 48
DALY_MODBUS_TEMP_SENSOR_REG_COUNT = 4
DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX = 61
DALY_MODBUS_MOS_TEMP_RAW_REG_INDEX = 90

COMMAND_NAMES = {
    0x50: 'rated capacity/cell voltage',
    0x53: 'battery type info',
    0x5A: 'min/max pack voltage',
    0x5B: 'max discharge/charge current',
    0x90: 'pack voltage/current/SOC',
    0x91: 'cell voltage extremes',
    0x92: 'temperature extremes',
    0x93: 'MOS status/capacity',
    0x94: 'status info',
    0x95: 'cell voltages',
    0x96: 'cell temperatures',
    0x97: 'cell balance state',
    0x98: 'failure codes',
}

FUNCTION_NAMES = {
    0x03: 'read holding registers',
}


def be16(data, pos):
    return (data[pos] << 8) | data[pos + 1]


def be32(data, pos):
    return ((data[pos] << 24) | (data[pos + 1] << 16) |
            (data[pos + 2] << 8) | data[pos + 3])


def hex_bytes(data):
    return ' '.join('{:02X}'.format(byte & 0xFF) for byte in data)


def native_checksum(raw):
    return sum(raw[:DALY_FRAME_LEN - 1]) & 0xFF


def modbus_crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xffff


def frame_crc(raw):
    return raw[-2] | (raw[-1] << 8)


def current_a_from_raw(raw):
    return (raw - 30000) / 10.0


def temp_c_from_offset(raw):
    return raw - 40


def on_off(value):
    return 'ON' if value else 'OFF'


def looks_like_cell_mv(value):
    return 1800 <= value <= 4500


def decode_offset_temp(raw):
    if raw == 0xFFFF or raw < 40 or raw > 160:
        return None
    return raw - 40


def date_pattern(word0, word1):
    year = (word0 >> 8) & 0xFF
    month = word0 & 0xFF
    day = (word1 >> 8) & 0xFF
    hour = word1 & 0xFF
    return 20 <= year <= 40 and 1 <= month <= 12 and 1 <= day <= 31 and hour <= 23


def native_role(addr):
    if addr in (DALY_RS485_REQUEST_ADDR, DALY_UART_REQUEST_ADDR):
        return 'req'
    if addr == DALY_DEFAULT_BMS_ID:
        return 'rsp'
    return 'frame'


def describe_native_payload(cmd, data, role='frame'):
    data = list(data or [])
    if role == 'req' and all((value & 0xFF) == 0 for value in data):
        return '0x{:02X} request {}'.format(cmd, COMMAND_NAMES.get(cmd, 'unknown'))

    if len(data) < 8:
        return '0x{:02X} {} raw={}'.format(cmd, COMMAND_NAMES.get(cmd, 'unknown'), hex_bytes(data))

    if cmd == 0x50:
        return '0x50 rated_capacity={:.3f}Ah raw={}'.format(
            be32(data, 0) / 1000.0,
            hex_bytes(data),
        )

    if cmd == 0x5A:
        return '0x5A pack_voltage_extremes max={:.1f}V min={:.1f}V raw={}'.format(
            be16(data, 0) / 10.0,
            be16(data, 4) / 10.0,
            hex_bytes(data),
        )

    if cmd == 0x5B:
        return '0x5B current_limits raw={}'.format(hex_bytes(data))

    if cmd == 0x90:
        return '0x90 pack V={:.1f}V I={:+.1f}A SOC={:.1f}%'.format(
            be16(data, 0) / 10.0,
            current_a_from_raw(be16(data, 4)),
            be16(data, 6) / 10.0,
        )

    if cmd == 0x91:
        return '0x91 cell_max={:.3f}V#{} cell_min={:.3f}V#{} dV={}mV'.format(
            be16(data, 0) / 1000.0,
            data[2],
            be16(data, 3) / 1000.0,
            data[5],
            max(0, be16(data, 0) - be16(data, 3)),
        )

    if cmd == 0x92:
        return '0x92 temp_max={:.1f}C#{} temp_min={:.1f}C#{}'.format(
            temp_c_from_offset(data[0]),
            data[1],
            temp_c_from_offset(data[2]),
            data[3],
        )

    if cmd == 0x93:
        return '0x93 state=0x{:02X} charge={} discharge={} remaining={:.3f}Ah'.format(
            data[0],
            on_off(data[1] == 1),
            on_off(data[2] == 1),
            be32(data, 4) / 1000.0,
        )

    if cmd == 0x94:
        return '0x94 cells={} temps={} charge={} discharge={} cycles={}'.format(
            data[0],
            data[1],
            on_off(data[2] == 1),
            on_off(data[3] == 1),
            be16(data, 5),
        )

    if cmd == 0x95:
        frame_no = data[0]
        base = (frame_no - 1) * 3 + 1 if frame_no else 0
        cells = []
        for idx in range(3):
            pos = 1 + (idx * 2)
            mv = be16(data, pos)
            if frame_no:
                cells.append('C{:02d}={:.3f}V'.format(base + idx, mv / 1000.0))
            else:
                cells.append('cell{}={:.3f}V'.format(idx + 1, mv / 1000.0))
        return '0x95 frame={} {}'.format(frame_no, ' '.join(cells))

    if cmd == 0x96:
        frame_no = data[0]
        base = (frame_no - 1) * 7 + 1 if frame_no else 0
        temps = []
        for idx in range(7):
            label = 'T{:02d}'.format(base + idx) if frame_no else 'T{}'.format(idx + 1)
            temps.append('{}={:.1f}C'.format(label, temp_c_from_offset(data[1 + idx])))
        return '0x96 frame={} {}'.format(frame_no, ' '.join(temps))

    if cmd == 0x97:
        mask = be32(data, 0)
        mask2 = be16(data, 4)
        return '0x97 balance mask=0x{:08X}{:04X} active={}'.format(
            mask,
            mask2,
            on_off(mask != 0 or mask2 != 0),
        )

    if cmd == 0x98:
        alarm = be32(data, 0)
        warning = (data[4] << 16) | (data[5] << 8) | data[6]
        return '0x98 alarm=0x{:08X} warning=0x{:06X}'.format(alarm, warning)

    return '0x{:02X} {} raw={}'.format(cmd, COMMAND_NAMES.get(cmd, 'unknown'), hex_bytes(data))


def parse_native_frame(raw):
    raw = bytes(raw)
    if len(raw) != DALY_FRAME_LEN:
        raise ValueError('native frame must be 13 bytes')
    if raw[0] != DALY_START_BYTE:
        raise ValueError('native frame missing 0xA5 start byte')

    checksum = raw[-1]
    expected = native_checksum(raw)
    return {
        'raw': raw,
        'protocol': 'native',
        'type': native_role(raw[1]),
        'addr': raw[1],
        'cmd': raw[2],
        'cmd_name': COMMAND_NAMES.get(raw[2], 'unknown'),
        'payload_len': raw[3],
        'data': raw[4:12],
        'checksum': checksum,
        'expected_checksum': expected,
        'checksum_ok': checksum == expected,
        'payload_len_ok': raw[3] == DALY_PAYLOAD_LEN,
    }


def parse_modbus_request(raw, crc, expected_crc, crc_ok):
    return {
        'raw': bytes(raw),
        'protocol': 'modbus',
        'type': 'request',
        'slave': raw[0],
        'func': raw[1],
        'start': be16(raw, 2),
        'count': be16(raw, 4),
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def parse_modbus_response(raw, request, crc, expected_crc, crc_ok):
    byte_count = raw[2]
    start = request.get('start') if request else None
    regs = []
    for idx in range(byte_count // 2):
        addr = start + idx if start is not None else None
        regs.append({
            'addr': addr,
            'index': idx,
            'value': be16(raw, 3 + (idx * 2)),
            'data_index': 3 + (idx * 2),
        })
    return {
        'raw': bytes(raw),
        'protocol': 'modbus',
        'type': 'response',
        'slave': raw[0],
        'func': raw[1],
        'byte_count': byte_count,
        'start': start,
        'count': len(regs),
        'registers': regs,
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def parse_modbus_exception(raw, crc, expected_crc, crc_ok):
    return {
        'raw': bytes(raw),
        'protocol': 'modbus',
        'type': 'exception',
        'slave': raw[0],
        'func': raw[1] & 0x7F,
        'exception_func': raw[1],
        'exception_code': raw[2],
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def parse_modbus_frame(raw, request=None):
    raw = bytes(raw)
    if len(raw) < 5:
        raise ValueError('Modbus frame too short')

    crc = frame_crc(raw)
    expected_crc = modbus_crc16(raw[:-2])
    crc_ok = crc == expected_crc
    func = raw[1]

    if func & 0x80:
        if len(raw) != 5:
            raise ValueError('invalid Modbus exception length')
        return parse_modbus_exception(raw, crc, expected_crc, crc_ok)

    if func not in FUNCTION_NAMES:
        raise ValueError('unsupported Modbus function 0x{:02X}'.format(func))

    if len(raw) == 8:
        return parse_modbus_request(raw, crc, expected_crc, crc_ok)

    byte_count = raw[2]
    expected_len = 3 + byte_count + 2
    if byte_count >= 2 and byte_count % 2 == 0 and len(raw) == expected_len:
        return parse_modbus_response(raw, request, crc, expected_crc, crc_ok)

    raise ValueError('could not classify Daly Modbus frame')


def parse_frame(raw, request=None):
    raw = bytes(raw)
    if raw and raw[0] == DALY_START_BYTE:
        return parse_native_frame(raw)
    return parse_modbus_frame(raw, request)


def frame_complete(raw):
    raw = bytes(raw)
    if not raw:
        return False

    if raw[0] == DALY_START_BYTE:
        return len(raw) >= DALY_FRAME_LEN

    if len(raw) < 5:
        return False

    func = raw[1]
    if func & 0x80:
        return len(raw) >= 5

    if func not in FUNCTION_NAMES:
        return len(raw) >= 8

    if len(raw) >= 3:
        byte_count = raw[2]
        if byte_count >= 2 and byte_count % 2 == 0:
            expected_len = 3 + byte_count + 2
            if len(raw) < expected_len:
                if len(raw) == 8 and modbus_crc16(raw[:6]) == frame_crc(raw[:8]):
                    return True
                return False
            return True

    return len(raw) >= 8


def modbus_register_name(addr, value=None):
    if addr is None:
        return 'word'
    if DALY_MODBUS_READ_CELLS_START <= addr < DALY_MODBUS_READ_CELLS_START + 32:
        if value is None or looks_like_cell_mv(value):
            return 'cell{:02d}_mv'.format(addr + 1)
    if addr == DALY_MODBUS_SOC_DECI_REG_INDEX:
        return 'soc_deci_pct'
    if DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX <= addr < DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX + DALY_MODBUS_TEMP_SENSOR_REG_COUNT:
        return 'temp_sensor{}_raw'.format(addr - DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX + 1)
    if addr == DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX:
        return 'temp_sensor_count'
    if addr == DALY_MODBUS_MOS_TEMP_RAW_REG_INDEX:
        return 'mos_temp_raw'
    return 'reg_0x{:04X}'.format(addr)


def describe_modbus_register(addr, value):
    name = modbus_register_name(addr, value)
    if addr is not None and name.startswith('cell') and looks_like_cell_mv(value):
        idx = addr - DALY_MODBUS_READ_CELLS_START + 1
        return 'C{:02d}={:.3f}V'.format(idx, value / 1000.0)
    if addr == DALY_MODBUS_SOC_DECI_REG_INDEX:
        return 'SOC={:.1f}% raw=0x{:04X}'.format(value / 10.0, value)
    temp = decode_offset_temp(value)
    if temp is not None and (name.startswith('temp_sensor') or name == 'mos_temp_raw'):
        return '{}={:.1f}C'.format(name, temp)
    return '{}=0x{:04X}'.format(name, value)


def describe_register_variants(addr, value):
    text = describe_modbus_register(addr, value)
    if addr is None:
        return ['word=0x{:04X}'.format(value), 'word']
    name = modbus_register_name(addr, value)
    return [
        '0x{:04X} {}'.format(addr, text),
        '{}=0x{:04X}'.format(name, value),
        name,
        '0x{:04X}'.format(addr),
    ]


def summarize_modbus_cells(registers):
    cells = []
    for reg in registers:
        addr = reg.get('addr')
        value = reg.get('value')
        if addr is None or addr >= 32 or not looks_like_cell_mv(value):
            if cells:
                break
            continue
        cells.append((addr + 1, value))

    if not cells:
        return None

    min_cell = min(cells, key=lambda item: item[1])
    max_cell = max(cells, key=lambda item: item[1])
    cell_text = ' '.join('C{:02d}={:.3f}V'.format(idx, mv / 1000.0) for idx, mv in cells)
    return 'cells count={} {} min={:.3f}V#{} max={:.3f}V#{}'.format(
        len(cells),
        cell_text,
        min_cell[1] / 1000.0,
        min_cell[0],
        max_cell[1] / 1000.0,
        max_cell[0],
    )


def modbus_values(registers):
    return {reg['addr']: reg['value'] for reg in registers if reg.get('addr') is not None}


def summarize_modbus_soc_current(registers):
    values = modbus_values(registers)
    parts = []
    if DALY_MODBUS_SOC_DECI_REG_INDEX in values and 0 <= values[DALY_MODBUS_SOC_DECI_REG_INDEX] <= 1000:
        parts.append('SOC={:.1f}%#{}'.format(
            values[DALY_MODBUS_SOC_DECI_REG_INDEX] / 10.0,
            DALY_MODBUS_SOC_DECI_REG_INDEX,
        ))

    regs = [reg.get('value') for reg in registers]
    for idx in range(0, max(0, len(regs) - 3)):
        soc = regs[idx]
        current_raw = regs[idx + 1]
        if soc is None or current_raw is None:
            continue
        if soc <= 1000 and 25000 <= current_raw <= 35000 and date_pattern(regs[idx + 2], regs[idx + 3]):
            parts.append('SOC_candidate={:.1f}%#{} I={:+.1f}A'.format(
                soc / 10.0,
                idx,
                current_a_from_raw(current_raw),
            ))
            break
    return parts


def summarize_modbus_temperatures(registers):
    values = modbus_values(registers)
    parts = []
    for idx in range(DALY_MODBUS_TEMP_SENSOR_REG_COUNT):
        addr = DALY_MODBUS_TEMP_SENSOR_BASE_REG_INDEX + idx
        if addr in values:
            temp = decode_offset_temp(values[addr])
            if temp is not None:
                parts.append('T{}={:.1f}C'.format(idx + 1, temp))
    if DALY_MODBUS_MOS_TEMP_RAW_REG_INDEX in values:
        temp = decode_offset_temp(values[DALY_MODBUS_MOS_TEMP_RAW_REG_INDEX])
        if temp is not None:
            parts.append('MOS={:.1f}C'.format(temp))
    if DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX in values:
        parts.append('temp_count={}'.format(values[DALY_MODBUS_TEMP_SENSOR_COUNT_REG_INDEX]))
    return parts


def describe_modbus_registers(registers):
    if not registers:
        return '0 register words'

    parts = []
    cells = summarize_modbus_cells(registers)
    if cells:
        parts.append(cells)
    parts.extend(summarize_modbus_soc_current(registers))
    parts.extend(summarize_modbus_temperatures(registers))

    if parts:
        return ', '.join(parts)

    start = registers[0].get('addr')
    end = registers[-1].get('addr')
    if start is not None:
        return 'regs 0x{:04X}..0x{:04X}'.format(start, end)
    return '{} register words'.format(len(registers))


def describe_modbus_registers_compact(registers):
    if not registers:
        return '0 regs'
    cells = summarize_modbus_cells(registers)
    if cells:
        tokens = cells.split()
        count = next((item for item in tokens if item.startswith('count=')), 'count=?')
        min_text = next((item for item in tokens if item.startswith('min=')), '')
        max_text = next((item for item in tokens if item.startswith('max=')), '')
        return 'cells {} {} {}'.format(count, min_text, max_text).strip()
    parts = summarize_modbus_soc_current(registers) + summarize_modbus_temperatures(registers)
    if parts:
        return ' '.join(parts[:8])
    if registers[0].get('addr') is not None:
        return 'regs 0x{:04X}..0x{:04X}'.format(registers[0]['addr'], registers[-1]['addr'])
    return '{} regs'.format(len(registers))


def describe_frame(frame):
    if frame.get('protocol') == 'native':
        return describe_native_payload(frame['cmd'], frame['data'], frame.get('type'))

    typ = frame.get('type')
    if typ == 'request':
        match = 'poll-block'
        if frame['start'] == DALY_MODBUS_READ_CELLS_START and frame['count'] == DALY_MODBUS_READ_CELLS_COUNT:
            match = 'cells/info block'
        elif frame['start'] == DALY_MODBUS_READ_INFO_START and frame['count'] == DALY_MODBUS_READ_INFO_COUNT:
            match = 'extended info block'
        return '{} start=0x{:04X} count={} {}'.format(
            FUNCTION_NAMES.get(frame['func'], 'function'),
            frame['start'],
            frame['count'],
            match,
        )
    if typ == 'response':
        regs = frame.get('registers', [])
        if frame.get('start') is not None and regs:
            return 'response regs 0x{:04X}..0x{:04X}: {}'.format(
                regs[0]['addr'], regs[-1]['addr'], describe_modbus_registers(regs))
        return 'response {} bytes'.format(frame.get('byte_count', 0))
    if typ == 'exception':
        return 'exception func=0x{:02X} code=0x{:02X}'.format(
            frame.get('func', 0), frame.get('exception_code', 0))
    return 'frame'


def describe_frame_variants(frame):
    full = describe_frame(frame)
    variants = [full]
    if frame.get('protocol') == 'native':
        variants.append('{} cmd=0x{:02X}'.format(frame.get('type'), frame.get('cmd')))
        if not frame.get('checksum_ok'):
            variants.append('checksum BAD')
    elif frame.get('type') == 'response':
        regs = frame.get('registers', [])
        if frame.get('start') is not None and regs:
            variants.append('0x{:04X}..0x{:04X} {}'.format(
                regs[0]['addr'], regs[-1]['addr'], describe_modbus_registers_compact(regs)))
    elif frame.get('type') == 'request':
        variants.append('read 0x{:04X} count {}'.format(frame['start'], frame['count']))
    variants.append('decoded')

    unique = []
    for text in variants:
        if text and text not in unique:
            unique.append(text)
    return unique


def frame_summary(frame, direction=''):
    prefix = (direction + ' ') if direction else ''
    if frame.get('protocol') == 'native':
        ok = 'OK' if frame.get('checksum_ok') and frame.get('payload_len_ok') else 'BAD'
        return '{}Daly native {} addr=0x{:02X} cmd=0x{:02X} {} chk={}'.format(
            prefix,
            frame.get('type', 'frame'),
            frame['addr'],
            frame['cmd'],
            frame.get('cmd_name', 'unknown'),
            ok,
        )

    crc_text = 'OK' if frame.get('crc_ok') else 'BAD'
    typ = frame.get('type')
    if typ == 'request':
        return '{}Daly Modbus req slave=0x{:02X} func=0x{:02X} start=0x{:04X} count={} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['start'], frame['count'], crc_text)
    if typ == 'response':
        regs = frame.get('registers', [])
        reg_text = ''
        if frame.get('start') is not None and regs:
            reg_text = ' regs=0x{:04X}..0x{:04X}'.format(regs[0]['addr'], regs[-1]['addr'])
        return '{}Daly Modbus rsp slave=0x{:02X} func=0x{:02X}{} bytes={} crc={}'.format(
            prefix, frame['slave'], frame['func'], reg_text, frame.get('byte_count', 0), crc_text)
    if typ == 'exception':
        return '{}Daly Modbus exception slave=0x{:02X} func=0x{:02X} code=0x{:02X} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['exception_code'], crc_text)
    return '{}Daly frame'.format(prefix)
