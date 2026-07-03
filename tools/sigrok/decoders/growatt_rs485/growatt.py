##
## Growatt RS485 Modbus RTU frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FUNCTION_NAMES = {
    0x03: 'read holding registers',
    0x04: 'read input registers',
}

WARNING_FLAGS = (
    (0x0001, 'cell overvoltage'),
    (0x0002, 'cell undervoltage'),
    (0x0004, 'pack overvoltage'),
    (0x0008, 'pack undervoltage'),
    (0x0010, 'discharge overcurrent'),
    (0x0020, 'charge overcurrent'),
    (0x0040, 'discharge overtemperature'),
    (0x0080, 'discharge undertemperature'),
    (0x0100, 'charge overtemperature'),
    (0x0200, 'charge undertemperature'),
    (0x0400, 'MOS overtemperature'),
    (0x0800, 'ambient overtemperature'),
    (0x1000, 'ambient undertemperature'),
    (0x2000, 'system low voltage'),
)

PROTECTION_FLAGS = (
    (0x0001, 'discharge overcurrent'),
    (0x0002, 'discharge short circuit'),
    (0x0004, 'pack overvoltage'),
    (0x0008, 'pack undervoltage'),
    (0x0010, 'discharge overtemperature'),
    (0x0020, 'charge overtemperature'),
    (0x0040, 'discharge undertemperature'),
    (0x0080, 'charge undertemperature'),
    (0x0100, 'soft-start failure'),
    (0x0200, 'permanent fault'),
    (0x0400, 'cell voltage delta'),
    (0x0800, 'charge overcurrent'),
    (0x1000, 'MOS overtemperature'),
    (0x2000, 'ambient overtemperature'),
    (0x4000, 'ambient undertemperature'),
)

REGISTER_NAMES = {
    0x0013: 'status_flags',
    0x0014: 'protection_flags',
    0x0015: 'soc',
    0x0016: 'pack_voltage',
    0x0017: 'pack_current_abs',
    0x0018: 'pack_temperature',
    0x0019: 'charge_current_limit',
    0x001A: 'remaining_capacity',
    0x001B: 'full_capacity',
    0x001E: 'cycles',
    0x0020: 'soh',
    0x0021: 'charge_voltage_target',
    0x0022: 'warning_flags',
    0x0023: 'discharge_current_limit',
    0x0024: 'extended_error',
    0x0025: 'cell_max',
    0x0026: 'cell_min',
    0x0027: 'cell_max_idx',
    0x0028: 'cell_min_idx',
    0x0070: 'cell_header',
}

POLL_BLOCKS = (
    (0x0001, 0x000F),
    (0x0010, 0x0008),
    (0x0018, 0x0008),
    (0x0020, 0x000B),
    (0x0071, 0x0010),
)


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
    return ' '.join('{:02X}'.format(byte) for byte in data)


def be16(data, pos):
    return (data[pos] << 8) | data[pos + 1]


def be16s(data, pos):
    value = be16(data, pos)
    if value & 0x8000:
        value -= 0x10000
    return value


def flag_list(value, flags):
    names = [name for mask, name in flags if value & mask]
    return ', '.join(names) if names else 'none'


def growatt_run_mode(status):
    return {
        0: 'soft_starting',
        1: 'standby',
        2: 'charging',
        3: 'discharging',
    }.get(status & 0x0003, 'unknown')


def growatt_master_mode(status):
    return {
        0: 'standalone',
        1: 'parallel',
        2: 'parallel_ready',
    }.get((status >> 8) & 0x0003, 'reserved')


def growatt_sp_status(status):
    return {
        0: 'none',
        1: 'standby',
        2: 'charging',
        3: 'discharging',
    }.get((status >> 10) & 0x0003, 'unknown')


def register_name(addr):
    if 0x0071 <= addr <= 0x0080:
        return 'cell{:02d}'.format(addr - 0x0070)
    return REGISTER_NAMES.get(addr, 'reg_0x{:04X}'.format(addr))


def describe_register(addr, value):
    name = register_name(addr)

    if 0x0071 <= addr <= 0x0080:
        return '{}={:.3f}V'.format(name, value / 1000.0)

    if addr == 0x0013:
        return ('status=0x{:04X} mode={} err={} bal={} master={} sp={}').format(
            value,
            growatt_run_mode(value),
            'YES' if value & 0x0004 else 'NO',
            'ON' if value & 0x0008 else 'OFF',
            growatt_master_mode(value),
            growatt_sp_status(value))
    if addr == 0x0014:
        masked = value & 0x7fff
        return 'prot=0x{:04X} ({})'.format(masked, flag_list(masked, PROTECTION_FLAGS))
    if addr == 0x0015:
        return 'SOC={}%%'.format(value)
    if addr == 0x0016:
        return 'pack_v={:.2f}V'.format(value / 100.0)
    if addr == 0x0017:
        return '|pack_i|={:.2f}A'.format(value / 100.0)
    if addr == 0x0018:
        return 'temp={}C'.format(value)
    if addr == 0x0019:
        return 'chg_lim={:.2f}A'.format(value / 100.0)
    if addr == 0x001A:
        return 'remain={:.2f}Ah'.format(value / 100.0)
    if addr == 0x001B:
        return 'full={:.2f}Ah'.format(value / 100.0)
    if addr == 0x001E:
        return 'cycles={}'.format(value)
    if addr == 0x0020:
        return 'SOH={}%%'.format(value)
    if addr == 0x0021:
        return 'chg_v_target={:.2f}V'.format(value / 100.0)
    if addr == 0x0022:
        masked = value & 0x3fff
        return 'warn=0x{:04X} ({})'.format(masked, flag_list(masked, WARNING_FLAGS))
    if addr == 0x0023:
        return 'dis_lim={:.2f}A'.format(value / 100.0)
    if addr == 0x0024:
        return 'ext_err=0x{:04X}'.format(value)
    if addr == 0x0025:
        return 'cell_max={:.3f}V'.format(value / 1000.0)
    if addr == 0x0026:
        return 'cell_min={:.3f}V'.format(value / 1000.0)
    if addr == 0x0027:
        return 'cell_max_idx={}'.format(value)
    if addr == 0x0028:
        return 'cell_min_idx={}'.format(value)
    if addr == 0x0070:
        return 'cell_header=0x{:04X}'.format(value)

    return '{}=0x{:04X}'.format(name, value)


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
    for i in range(byte_count // 2):
        addr = (start + i) if start is not None else None
        regs.append({
            'addr': addr,
            'value': be16(raw, 3 + (i * 2)),
            'data_index': 3 + (i * 2),
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

    if len(raw) >= 5:
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
            if len(raw) >= expected_len:
                return True

    return len(raw) >= 8


def request_key(frame):
    return (frame.get('slave'), frame.get('func'))


def describe_registers(registers):
    known = []
    cells = []
    values_by_addr = {}

    for reg in registers:
        addr = reg.get('addr')
        value = reg.get('value')
        if addr is None:
            continue
        values_by_addr[addr] = value
        if 0x0071 <= addr <= 0x0080:
            cells.append((addr, value))
        elif addr in REGISTER_NAMES:
            known.append(describe_register(addr, value))

    if cells:
        valid_cells = [(addr, value) for addr, value in cells if 1000 <= value <= 6000]
        if valid_cells:
            min_cell = min(valid_cells, key=lambda item: item[1])
            max_cell = max(valid_cells, key=lambda item: item[1])
            return 'cells count={} min={:.3f}V#{} max={:.3f}V#{}'.format(
                len(valid_cells),
                min_cell[1] / 1000.0,
                min_cell[0] - 0x0070,
                max_cell[1] / 1000.0,
                max_cell[0] - 0x0070)
        return 'cells count={} (no valid 1.0..6.0V values)'.format(len(cells))

    if known:
        return ', '.join(known[:6])

    if registers and registers[0].get('addr') is not None:
        start = registers[0]['addr']
        end = registers[-1]['addr']
        return 'regs 0x{:04X}..0x{:04X}'.format(start, end)

    return '{} register words'.format(len(registers))


def describe_frame(frame):
    typ = frame.get('type')
    if typ == 'request':
        name = FUNCTION_NAMES.get(frame['func'], 'function')
        return '{} start=0x{:04X} count={}'.format(name, frame['start'], frame['count'])
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


def frame_summary(frame, direction=''):
    prefix = (direction + ' ') if direction else ''
    crc_text = 'OK' if frame.get('crc_ok') else 'BAD'
    typ = frame.get('type')

    if typ == 'request':
        return '{}Growatt RS485 req slave=0x{:02X} func=0x{:02X} start=0x{:04X} count={} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['start'], frame['count'], crc_text)
    if typ == 'response':
        regs = frame.get('registers', [])
        reg_text = ''
        if frame.get('start') is not None and regs:
            reg_text = ' regs=0x{:04X}..0x{:04X}'.format(regs[0]['addr'], regs[-1]['addr'])
        return '{}Growatt RS485 rsp slave=0x{:02X} func=0x{:02X}{} bytes={} crc={}'.format(
            prefix, frame['slave'], frame['func'], reg_text, frame.get('byte_count', 0), crc_text)
    if typ == 'exception':
        return '{}Growatt RS485 exception slave=0x{:02X} func=0x{:02X} code=0x{:02X} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['exception_code'], crc_text)
    return '{}Growatt RS485 frame crc={}'.format(prefix, crc_text)
