##
## PACE RS485 Modbus V1.3 frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FUNCTION_NAMES = {
    0x03: 'read holding registers',
    0x04: 'read input registers',
}

REG_CURRENT_10MA = 0x0000
REG_PACK_VOLTAGE_10MV = 0x0001
REG_SOC_PCT = 0x0002
REG_SOH_PCT = 0x0003
REG_REMAIN_CAP_10MAH = 0x0004
REG_FULL_CAP_10MAH = 0x0005
REG_DESIGN_CAP_10MAH = 0x0006
REG_CYCLE_COUNT = 0x0007
REG_WARNING_FLAGS = 0x0009
REG_PROTECTION_FLAGS = 0x000A
REG_STATUS_FLAGS = 0x000B
REG_BALANCE_STATUS = 0x000C

REG_CELL01_MV = 0x000F
REG_CELL16_MV = 0x001E
CELL_COUNT = 16

REG_TEMP1_DECIC = 0x001F
REG_TEMP2_DECIC = 0x0020
REG_TEMP3_DECIC = 0x0021
REG_TEMP4_DECIC = 0x0022
REG_MOS_TEMP_DECIC = 0x0023
REG_ENV_TEMP_DECIC = 0x0024

STATUS_FLAGS = (
    (0x0100, 'charge'),
    (0x0200, 'discharge'),
    (0x0400, 'charge MOS'),
    (0x0800, 'discharge MOS'),
)

POLL_BLOCKS = (
    (REG_CURRENT_10MA, 0x000D),
    (REG_CELL01_MV, 0x0016),
)

REGISTER_NAMES = {
    REG_CURRENT_10MA: 'pack_current',
    REG_PACK_VOLTAGE_10MV: 'pack_voltage',
    REG_SOC_PCT: 'soc',
    REG_SOH_PCT: 'soh',
    REG_REMAIN_CAP_10MAH: 'remain_capacity',
    REG_FULL_CAP_10MAH: 'full_capacity',
    REG_DESIGN_CAP_10MAH: 'design_capacity',
    REG_CYCLE_COUNT: 'cycles',
    0x0008: 'runtime_raw_0008',
    REG_WARNING_FLAGS: 'warning_flags',
    REG_PROTECTION_FLAGS: 'protection_flags',
    REG_STATUS_FLAGS: 'status_flags',
    REG_BALANCE_STATUS: 'balance_status',
    REG_TEMP1_DECIC: 'temp1',
    REG_TEMP2_DECIC: 'temp2',
    REG_TEMP3_DECIC: 'temp3',
    REG_TEMP4_DECIC: 'temp4',
    REG_MOS_TEMP_DECIC: 'mos_temp',
    REG_ENV_TEMP_DECIC: 'env_temp',
}


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


def hex_bytes(data):
    return ' '.join('{:02X}'.format(byte & 0xFF) for byte in data)


def be16(data, pos):
    return (data[pos] << 8) | data[pos + 1]


def signed16(value):
    value &= 0xFFFF
    if value & 0x8000:
        value -= 0x10000
    return value


def flag_list(value, flags):
    names = [name for mask, name in flags if value & mask]
    return ', '.join(names) if names else 'none'


def bit_mask_text(value):
    if value == 0:
        return 'none'
    bits = ['b{}'.format(i) for i in range(16) if value & (1 << i)]
    return ','.join(bits)


def decode_packed_pct(raw):
    lo = raw & 0xFF
    hi = (raw >> 8) & 0xFF
    if hi <= 100 and lo == 0 and hi > 0:
        return hi
    if lo <= 100:
        return lo
    if hi <= 100:
        return hi
    return min(raw, 100)


def valid_cell_mv(value):
    return 1000 <= value <= 6000


def register_name(addr):
    if REG_CELL01_MV <= addr <= REG_CELL16_MV:
        return 'cell{:02d}'.format(addr - REG_CELL01_MV + 1)
    return REGISTER_NAMES.get(addr, 'reg_0x{:04X}'.format(addr))


def capacity_ah(value):
    return value / 100.0


def temp_c(value):
    return signed16(value) / 10.0


def describe_register(addr, value):
    if REG_CELL01_MV <= addr <= REG_CELL16_MV:
        idx = addr - REG_CELL01_MV + 1
        if valid_cell_mv(value):
            return 'cell{:02d}={:.3f}V'.format(idx, value / 1000.0)
        return 'cell{:02d}_raw=0x{:04X}'.format(idx, value)

    if addr == REG_CURRENT_10MA:
        return 'pack_i={:+.2f}A'.format(signed16(value) / 100.0)
    if addr == REG_PACK_VOLTAGE_10MV:
        return 'pack_v={:.2f}V'.format(value / 100.0)
    if addr == REG_SOC_PCT:
        return 'SOC={}% raw=0x{:04X}'.format(decode_packed_pct(value), value)
    if addr == REG_SOH_PCT:
        return 'SOH={}% raw=0x{:04X}'.format(decode_packed_pct(value), value)
    if addr == REG_REMAIN_CAP_10MAH:
        return 'remain={:.2f}Ah'.format(capacity_ah(value))
    if addr == REG_FULL_CAP_10MAH:
        return 'full={:.2f}Ah'.format(capacity_ah(value))
    if addr == REG_DESIGN_CAP_10MAH:
        return 'design={:.2f}Ah'.format(capacity_ah(value))
    if addr == REG_CYCLE_COUNT:
        return 'cycles={}'.format(value)
    if addr == REG_WARNING_FLAGS:
        return 'warning=0x{:04X} ({})'.format(value, bit_mask_text(value))
    if addr == REG_PROTECTION_FLAGS:
        return 'protection=0x{:04X} ({})'.format(value, bit_mask_text(value))
    if addr == REG_STATUS_FLAGS:
        return 'status=0x{:04X} ({})'.format(value, flag_list(value, STATUS_FLAGS))
    if addr == REG_BALANCE_STATUS:
        return 'balance=0x{:04X} ({})'.format(value, bit_mask_text(value))
    if addr in (REG_TEMP1_DECIC, REG_TEMP2_DECIC, REG_TEMP3_DECIC,
                REG_TEMP4_DECIC, REG_MOS_TEMP_DECIC, REG_ENV_TEMP_DECIC):
        return '{}={:.1f}C'.format(register_name(addr), temp_c(value))

    return '{}=0x{:04X}'.format(register_name(addr), value)


def describe_register_variants(addr, value):
    full = '0x{:04X} {}'.format(addr, describe_register(addr, value))
    name = register_name(addr)
    if name == 'reg_0x{:04X}'.format(addr):
        return [full, '0x{:04X}=0x{:04X}'.format(addr, value), '0x{:04X}'.format(addr)]
    return [full, '{}=0x{:04X}'.format(name, value), name, '0x{:04X}'.format(addr)]


def parse_request(raw, crc, expected_crc, crc_ok):
    return {
        'raw': bytes(raw),
        'type': 'request',
        'slave': raw[0],
        'func': raw[1],
        'start': be16(raw, 2),
        'count': be16(raw, 4),
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def parse_response(raw, request, crc, expected_crc, crc_ok):
    byte_count = raw[2]
    regs = []
    start = request.get('start') if request else None
    for idx in range(byte_count // 2):
        addr = (start + idx) if start is not None else None
        regs.append({
            'addr': addr,
            'value': be16(raw, 3 + (idx * 2)),
            'data_index': 3 + (idx * 2),
        })
    return {
        'raw': bytes(raw),
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


def parse_exception(raw, crc, expected_crc, crc_ok):
    return {
        'raw': bytes(raw),
        'type': 'exception',
        'slave': raw[0],
        'func': raw[1] & 0x7f,
        'exception_func': raw[1],
        'exception_code': raw[2],
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def parse_frame(raw, request=None):
    raw = bytes(raw)
    if len(raw) < 5:
        raise ValueError('frame too short')

    crc = frame_crc(raw)
    expected_crc = modbus_crc16(raw[:-2])
    crc_ok = crc == expected_crc
    func = raw[1]

    if func & 0x80:
        if len(raw) != 5:
            raise ValueError('invalid exception frame length')
        return parse_exception(raw, crc, expected_crc, crc_ok)

    if func not in FUNCTION_NAMES:
        raise ValueError('unsupported function 0x{:02X}'.format(func))

    if len(raw) == 8:
        return parse_request(raw, crc, expected_crc, crc_ok)

    byte_count = raw[2]
    expected_len = 3 + byte_count + 2
    if byte_count >= 2 and byte_count % 2 == 0 and len(raw) == expected_len:
        return parse_response(raw, request, crc, expected_crc, crc_ok)

    raise ValueError('could not classify Modbus RTU frame')


def frame_complete(raw):
    raw = bytes(raw)
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


def summarize_cells(registers):
    cells = []
    for reg in registers:
        addr = reg.get('addr')
        value = reg.get('value')
        if addr is None or not (REG_CELL01_MV <= addr <= REG_CELL16_MV):
            continue
        if valid_cell_mv(value):
            cells.append((addr - REG_CELL01_MV + 1, value))

    if not cells:
        return None

    min_cell = min(cells, key=lambda item: item[1])
    max_cell = max(cells, key=lambda item: item[1])
    cell_list = ' '.join('C{:02d}={:.3f}V'.format(idx, mv / 1000.0)
                         for idx, mv in cells)
    return 'cells count={} {} min={:.3f}V#{} max={:.3f}V#{}'.format(
        len(cells),
        cell_list,
        min_cell[1] / 1000.0,
        min_cell[0],
        max_cell[1] / 1000.0,
        max_cell[0],
    )


def summarize_runtime(registers):
    values = {reg['addr']: reg['value'] for reg in registers if reg.get('addr') is not None}
    parts = []

    for addr in (REG_CURRENT_10MA, REG_PACK_VOLTAGE_10MV, REG_SOC_PCT,
                 REG_SOH_PCT, REG_REMAIN_CAP_10MAH, REG_FULL_CAP_10MAH,
                 REG_DESIGN_CAP_10MAH, REG_CYCLE_COUNT, REG_WARNING_FLAGS,
                 REG_PROTECTION_FLAGS, REG_STATUS_FLAGS, REG_BALANCE_STATUS):
        if addr in values:
            parts.append(describe_register(addr, values[addr]))

    return parts


def summarize_temps(registers):
    values = {reg['addr']: reg['value'] for reg in registers if reg.get('addr') is not None}
    parts = []
    for addr in (REG_TEMP1_DECIC, REG_TEMP2_DECIC, REG_TEMP3_DECIC,
                 REG_TEMP4_DECIC, REG_MOS_TEMP_DECIC, REG_ENV_TEMP_DECIC):
        if addr in values:
            parts.append(describe_register(addr, values[addr]))
    return parts


def describe_registers(registers):
    if not registers:
        return '0 register words'

    cell_text = summarize_cells(registers)
    temps = summarize_temps(registers)
    runtime = summarize_runtime(registers)
    start = registers[0].get('addr')
    end = registers[-1].get('addr')

    if start is not None and start == REG_CELL01_MV and cell_text:
        parts = [cell_text]
        parts.extend(temps)
        return ', '.join(parts)

    if runtime:
        if cell_text and start is not None and end is not None and start <= REG_CELL01_MV <= end:
            runtime.append(cell_text)
        if temps:
            runtime.extend(temps)
        return ', '.join(runtime[:12])

    if cell_text:
        parts = [cell_text]
        parts.extend(temps)
        return ', '.join(parts)
    if temps:
        return ', '.join(temps)

    if start is not None:
        return 'regs 0x{:04X}..0x{:04X}'.format(start, end)
    return '{} register words'.format(len(registers))


def describe_registers_compact(registers):
    if not registers:
        return '0 regs'

    cell_text = summarize_cells(registers)
    if cell_text:
        parts = cell_text.split()
        count = next((part for part in parts if part.startswith('count=')), 'count=?')
        min_text = next((part for part in parts if part.startswith('min=')), '')
        max_text = next((part for part in parts if part.startswith('max=')), '')
        return 'cells {} {} {}'.format(count, min_text, max_text).strip()

    runtime = summarize_runtime(registers)
    if runtime:
        return ' '.join(runtime[:7])

    temps = summarize_temps(registers)
    if temps:
        return ' '.join(temps)

    if registers[0].get('addr') is not None:
        return 'regs 0x{:04X}..0x{:04X}'.format(registers[0]['addr'], registers[-1]['addr'])
    return '{} regs'.format(len(registers))


def describe_frame(frame):
    typ = frame.get('type')
    if typ == 'request':
        name = FUNCTION_NAMES.get(frame['func'], 'function')
        match = 'poll-block' if any(frame['start'] == start and frame['count'] == count for start, count in POLL_BLOCKS) else 'custom'
        return '{} start=0x{:04X} count={} {}'.format(name, frame['start'], frame['count'], match)
    if typ == 'response':
        regs = frame.get('registers', [])
        if frame.get('start') is not None and regs:
            return 'response regs 0x{:04X}..0x{:04X}: {}'.format(
                regs[0]['addr'], regs[-1]['addr'], describe_registers(regs))
        return 'response {} bytes'.format(frame.get('byte_count', 0))
    if typ == 'exception':
        return 'exception func=0x{:02X} code=0x{:02X}'.format(
            frame.get('func', 0), frame.get('exception_code', 0))
    return 'frame'


def describe_frame_variants(frame):
    full = describe_frame(frame)
    variants = [full]
    typ = frame.get('type')

    if typ == 'response':
        regs = frame.get('registers', [])
        if frame.get('start') is not None and regs:
            start = regs[0]['addr']
            end = regs[-1]['addr']
            compact = describe_registers_compact(regs)
            tentative = ' tentative' if not frame.get('crc_ok') else ''
            variants.append('0x{:04X}..0x{:04X}{} {}'.format(start, end, tentative, compact))
            variants.append('0x{:04X}..0x{:04X} {}'.format(start, end, 'CRC BAD' if not frame.get('crc_ok') else 'OK'))
    elif typ == 'request':
        variants.append('read 0x{:04X} count {}'.format(frame['start'], frame['count']))
    elif typ == 'exception':
        variants.append('exception 0x{:02X}'.format(frame.get('exception_code', 0)))

    variants.append('decoded')

    unique = []
    for text in variants:
        if text and text not in unique:
            unique.append(text)
    return unique


def frame_summary(frame, direction=''):
    prefix = (direction + ' ') if direction else ''
    crc_text = 'OK' if frame.get('crc_ok') else 'BAD'
    typ = frame.get('type')

    if typ == 'request':
        return '{}PACE Modbus req slave=0x{:02X} func=0x{:02X} start=0x{:04X} count={} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['start'], frame['count'], crc_text)
    if typ == 'response':
        regs = frame.get('registers', [])
        reg_text = ''
        if frame.get('start') is not None and regs:
            reg_text = ' regs=0x{:04X}..0x{:04X}'.format(regs[0]['addr'], regs[-1]['addr'])
        return '{}PACE Modbus rsp slave=0x{:02X} func=0x{:02X}{} bytes={} crc={}'.format(
            prefix, frame['slave'], frame['func'], reg_text, frame.get('byte_count', 0), crc_text)
    if typ == 'exception':
        return '{}PACE Modbus exception slave=0x{:02X} func=0x{:02X} code=0x{:02X} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['exception_code'], crc_text)
    return '{}PACE Modbus frame crc={}'.format(prefix, crc_text)
