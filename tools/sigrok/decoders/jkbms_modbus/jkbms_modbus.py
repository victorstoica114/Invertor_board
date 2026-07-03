##
## JKBMS RS485 Modbus RTU frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FUNCTION_NAMES = {
    0x03: 'read holding registers',
    0x04: 'read input registers',
}

BASE_RUNTIME = 0x1200
MAX_CELLS = 32

REG_CELL0_MV = 0x1200
REG_CELL31_MV = 0x123E
REG_CELL_AVG_MV = 0x1244
REG_CELL_DIFF_MV = 0x1246
REG_MAX_MIN_CELL_IDX = 0x1248
REG_TEMP_MOS_DECIC = 0x128A
REG_PACK_VOLT_MV_U32 = 0x1290
REG_PACK_WATT_MW_U32 = 0x1294
REG_PACK_CURRENT_MA_I32 = 0x1298
REG_TEMP_BAT1_DECIC = 0x129C
REG_TEMP_BAT2_DECIC = 0x129E
REG_ALARM_STATUS_CANDIDATE_U32 = 0x12A0
REG_BALANCE_CURRENT_MA_I16 = 0x12A4
REG_BALANCE_SOC_U8X2 = 0x12A6
REG_REMAIN_MAH_I32 = 0x12A8
REG_FULL_MAH_U32 = 0x12AC
REG_CYCLES_U32 = 0x12B0
REG_SOH_PRECHARGE_U8X2 = 0x12B8

POLL_BLOCKS = (
    (0x1200, 0x0010),
    (0x1210, 0x0010),
    (0x1220, 0x0010),
    (0x1230, 0x0010),
    (0x1240, 0x0020),
    (0x1244, 0x0006),
    (0x128A, 0x0028),
    (0x12B8, 0x0002),
)

REGISTER_NAMES = {
    REG_CELL_AVG_MV: 'cell_avg_mv',
    REG_CELL_DIFF_MV: 'cell_diff_max_mv',
    REG_MAX_MIN_CELL_IDX: 'max_min_cell_idx',
    REG_TEMP_MOS_DECIC: 'temp_mos_deciC',
    REG_PACK_VOLT_MV_U32: 'pack_voltage_mV_hi',
    REG_PACK_VOLT_MV_U32 + 1: 'pack_voltage_mV_lo',
    REG_PACK_WATT_MW_U32: 'pack_power_mW_hi',
    REG_PACK_WATT_MW_U32 + 1: 'pack_power_mW_lo',
    REG_PACK_CURRENT_MA_I32: 'pack_current_mA_hi',
    REG_PACK_CURRENT_MA_I32 + 1: 'pack_current_mA_lo',
    REG_TEMP_BAT1_DECIC: 'temp_bat1_deciC',
    REG_TEMP_BAT2_DECIC: 'temp_bat2_deciC',
    REG_ALARM_STATUS_CANDIDATE_U32: 'alarm_status_candidate_hi',
    REG_ALARM_STATUS_CANDIDATE_U32 + 1: 'alarm_status_candidate_lo',
    REG_BALANCE_CURRENT_MA_I16: 'balance_current_mA',
    REG_BALANCE_SOC_U8X2: 'balance_soc_u8x2',
    REG_REMAIN_MAH_I32: 'remaining_mAh_hi',
    REG_REMAIN_MAH_I32 + 1: 'remaining_mAh_lo',
    REG_FULL_MAH_U32: 'full_mAh_hi',
    REG_FULL_MAH_U32 + 1: 'full_mAh_lo',
    REG_CYCLES_U32: 'cycles_hi',
    REG_CYCLES_U32 + 1: 'cycles_lo',
    REG_SOH_PRECHARGE_U8X2: 'soh_precharge_u8x2',
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


def be16s(data, pos):
    value = be16(data, pos)
    if value & 0x8000:
        value -= 0x10000
    return value


def bswap16(value):
    return ((value >> 8) | ((value & 0xFF) << 8)) & 0xFFFF


def u32_ab(a, b):
    return ((a & 0xFFFF) << 16) | (b & 0xFFFF)


def i32_from_u32(value):
    value &= 0xFFFFFFFF
    if value & 0x80000000:
        value -= 0x100000000
    return value


def normalize_cell_mv(raw):
    candidates = []
    for value in (raw, bswap16(raw), raw // 10, bswap16(raw) // 10):
        if 2000 <= value <= 5000:
            candidates.append(value)
    if not candidates:
        return None
    return min(candidates, key=lambda value: abs(value - 4000))


def normalize_delta_mv(raw):
    return raw // 10 if raw > 5000 else raw


def decode_pct_byte_pair(raw, prefer_high=False):
    hi = (raw >> 8) & 0xFF
    lo = raw & 0xFF
    hi_ok = hi <= 100
    lo_ok = lo <= 100
    if hi_ok and lo_ok:
        if hi == 0 and lo > 0:
            return lo
        if lo == 0 and hi > 0:
            return hi
        return hi if prefer_high else lo
    if hi_ok:
        return hi
    if lo_ok:
        return lo
    return None


def best_u32(values, reg, min_value=0, max_value=0xFFFFFFFF):
    if reg not in values or reg + 1 not in values:
        return None
    a = values[reg]
    b = values[reg + 1]
    candidates = (u32_ab(a, b), u32_ab(b, a))
    valid = [value for value in candidates if min_value <= value <= max_value]
    if not valid:
        return None
    return min(valid)


def best_i32(values, reg, abs_limit=0x7FFFFFFF):
    if reg not in values or reg + 1 not in values:
        return None
    a = values[reg]
    b = values[reg + 1]
    candidates = (i32_from_u32(u32_ab(a, b)), i32_from_u32(u32_ab(b, a)))
    valid = [value for value in candidates if -abs_limit <= value <= abs_limit]
    if not valid:
        return None
    return min(valid, key=lambda value: abs(value))


def signed16_from_word(value):
    value &= 0xFFFF
    if value & 0x8000:
        value -= 0x10000
    return value


def signed_deci_c(value):
    signed = signed16_from_word(value)
    if -1000 <= signed <= 1500:
        return signed / 10.0
    return None


def decode_soc_from_values(values):
    chosen = None
    for addr, prefer_high, allow_zero in (
        (REG_BALANCE_SOC_U8X2, False, False),
        (REG_BALANCE_SOC_U8X2 + 1, False, False),
        (REG_SOH_PRECHARGE_U8X2, True, True),
    ):
        if addr not in values:
            continue
        candidate = decode_pct_byte_pair(values[addr], prefer_high)
        if candidate is None:
            continue
        if candidate == 0 and not allow_zero:
            continue
        chosen = candidate
        if candidate > 0:
            return candidate
    return chosen


def decode_soh_precharge(values):
    if REG_SOH_PRECHARGE_U8X2 not in values:
        return None
    raw = values[REG_SOH_PRECHARGE_U8X2]
    hi = (raw >> 8) & 0xFF
    lo = raw & 0xFF
    if hi <= 100 and lo <= 1:
        return hi, lo
    if lo <= 100 and hi <= 1:
        return lo, hi
    soh = decode_pct_byte_pair(raw, True)
    if soh is not None:
        return soh, None
    return None


def describe_pack_power(values):
    return None


def summarize_known_runtime_values(values):
    parts = []

    voltage_mv = best_u32(values, REG_PACK_VOLT_MV_U32, 1000, 200000)
    if voltage_mv is not None:
        parts.append('pack_v={:.3f}V'.format(voltage_mv / 1000.0))

    current_ma = best_i32(values, REG_PACK_CURRENT_MA_I32, 1000000)
    if current_ma is not None:
        parts.append('pack_i={:+.3f}A'.format(current_ma / 1000.0))

    power_text = describe_pack_power(values)
    if power_text:
        parts.append(power_text)

    soc = decode_soc_from_values(values)
    if soc is not None:
        parts.append('SOC={}%'.format(soc))

    soh = decode_soh_precharge(values)
    if soh is not None:
        soh_pct, precharge = soh
        if precharge is None:
            parts.append('SOH={}%'.format(soh_pct))
        else:
            parts.append('SOH={}% precharge=0x{:02X}'.format(soh_pct, precharge))

    for addr, label in (
        (REG_TEMP_MOS_DECIC, 'MOS'),
        (REG_TEMP_BAT1_DECIC, 'T1'),
        (REG_TEMP_BAT2_DECIC, 'T2'),
    ):
        if addr in values:
            temp = signed_deci_c(values[addr])
            if temp is not None and temp != 0:
                parts.append('{}={:.1f}C'.format(label, temp))

    if (REG_ALARM_STATUS_CANDIDATE_U32 in values and
            REG_ALARM_STATUS_CANDIDATE_U32 + 1 in values):
        alarm = best_u32(values, REG_ALARM_STATUS_CANDIDATE_U32, 0, 0xFFFFFFFF)
        if alarm is not None:
            parts.append('alarm_candidate=0x{:08X}'.format(alarm))

    if REG_FULL_MAH_U32 in values and REG_FULL_MAH_U32 + 1 in values:
        full = best_u32(values, REG_FULL_MAH_U32, 0, 500000000)
        if full is not None:
            parts.append('full={:.2f}Ah'.format(full / 1000.0))

    if REG_CYCLES_U32 in values and REG_CYCLES_U32 + 1 in values:
        cycles = best_u32(values, REG_CYCLES_U32, 0, 1000000)
        if cycles is not None:
            parts.append('cycles={}'.format(cycles))

    return parts


def register_name(addr):
    if REG_CELL0_MV <= addr <= REG_CELL31_MV:
        off = addr - REG_CELL0_MV
        if off % 2 == 0:
            return 'cell{:02d}_mv'.format((off // 2) + 1)
        return 'cell_alt{:02d}_mv'.format(off + 1)
    return REGISTER_NAMES.get(addr, 'reg_0x{:04X}'.format(addr))


def describe_cell_register(addr, value):
    mv = normalize_cell_mv(value)
    off = addr - REG_CELL0_MV
    if off % 2 == 0:
        label = 'cell{:02d}'.format((off // 2) + 1)
    elif off + 1 > MAX_CELLS:
        label = 'cell_spare'
    else:
        label = 'cell_alt{:02d}'.format(off + 1)
    if mv is None:
        return '{} raw=0x{:04X}'.format(label, value)
    return '{}={:.3f}V raw=0x{:04X}'.format(label, mv / 1000.0, value)


def describe_register(addr, value, values_by_addr=None):
    if REG_CELL0_MV <= addr <= REG_CELL31_MV:
        return describe_cell_register(addr, value)

    if addr == REG_CELL_AVG_MV:
        mv = normalize_cell_mv(value)
        return 'cell_avg={}'.format('{:.3f}V'.format(mv / 1000.0) if mv else 'raw0x{:04X}'.format(value))
    if addr == REG_CELL_DIFF_MV:
        return 'cell_diff={}mV'.format(normalize_delta_mv(value))
    if addr == REG_MAX_MIN_CELL_IDX:
        max_idx = (value >> 8) & 0xFF
        min_idx = value & 0xFF
        if 1 <= max_idx <= MAX_CELLS and 1 <= min_idx <= MAX_CELLS:
            return 'cell_idx max#{} min#{}'.format(max_idx, min_idx)
        return 'cell_idx raw=0x{:04X}'.format(value)
    if addr in (REG_TEMP_MOS_DECIC, REG_TEMP_BAT1_DECIC, REG_TEMP_BAT2_DECIC):
        temp = signed_deci_c(value)
        return '{}={}'.format(
            register_name(addr),
            '{:.1f}C'.format(temp) if temp is not None else 'raw=0x{:04X}'.format(value))
    if addr == REG_BALANCE_CURRENT_MA_I16:
        signed = value - 0x10000 if value & 0x8000 else value
        if abs(signed) > 5000:
            return 'balance_current_candidate={:.3f}A implausible'.format(signed / 1000.0)
        return 'balance_current={:.3f}A'.format(signed / 1000.0)
    if addr == REG_BALANCE_SOC_U8X2:
        soc = decode_pct_byte_pair(value, False)
        return 'balance=0x{:02X} SOC={}%'.format((value >> 8) & 0xFF, soc if soc is not None else '-')
    if addr == REG_SOH_PRECHARGE_U8X2:
        soh = decode_pct_byte_pair(value, True)
        return 'SOH={}% precharge=0x{:02X}'.format(soh if soh is not None else '-', value & 0xFF)

    values = values_by_addr or {}
    if addr == REG_PACK_VOLT_MV_U32:
        combined = best_u32(values, addr, 1000, 200000)
        if combined is not None:
            return 'pack_v={:.3f}V'.format(combined / 1000.0)
        lo = values.get(addr + 1)
        return 'pack_v raw_hi=0x{:04X} raw_lo={}'.format(
            value, '0x{:04X}'.format(lo) if lo is not None else '-')
    if addr == REG_PACK_CURRENT_MA_I32:
        combined = best_i32(values, addr, 1000000)
        return 'pack_i={}'.format('{:+.3f}A'.format(combined / 1000.0) if combined is not None else 'hi=0x{:04X}'.format(value))
    if addr == REG_PACK_WATT_MW_U32:
        combined = best_i32(values, addr, 200000000)
        if combined is not None:
            return 'pack_p_candidate={:+.1f}W'.format(combined / 1000.0)
        return 'pack_p raw_hi=0x{:04X}'.format(value)
    if addr == REG_ALARM_STATUS_CANDIDATE_U32:
        combined = best_u32(values, addr, 0, 0xFFFFFFFF)
        return 'alarm_status_candidate=0x{:08X} tentative'.format(combined) if combined is not None else 'alarm_status_candidate_hi=0x{:04X}'.format(value)
    if addr == REG_REMAIN_MAH_I32:
        combined = best_i32(values, addr, 500000000)
        return 'remain={}'.format('{:.2f}Ah'.format(combined / 1000.0) if combined is not None else 'hi=0x{:04X}'.format(value))
    if addr == REG_FULL_MAH_U32:
        combined = best_u32(values, addr, 0, 500000000)
        return 'full={}'.format('{:.2f}Ah'.format(combined / 1000.0) if combined is not None else 'hi=0x{:04X}'.format(value))
    if addr == REG_CYCLES_U32:
        combined = best_u32(values, addr, 0, 1000000)
        return 'cycles={}'.format(combined if combined is not None else 'hi=0x{:04X}'.format(value))

    return '{}=0x{:04X}'.format(register_name(addr), value)


def describe_register_variants(addr, value, values_by_addr=None):
    full = '0x{:04X} {}'.format(addr, describe_register(addr, value, values_by_addr))
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


def describe_cell_registers(registers):
    candidate_regs = []
    for reg in registers:
        addr = reg.get('addr')
        if addr is None or not (REG_CELL0_MV <= addr <= REG_CELL31_MV):
            continue
        mv = normalize_cell_mv(reg.get('value', 0))
        if mv is None:
            continue
        candidate_regs.append((addr, mv))

    if not candidate_regs:
        return None

    offsets = [addr - REG_CELL0_MV for addr, _mv in candidate_regs]
    even_count = sum(1 for off in offsets if off % 2 == 0)
    odd_count = len(offsets) - even_count
    compact_mode = odd_count > 0 and even_count > 0 and (max(offsets) + 1) <= MAX_CELLS

    cells = []
    for addr, mv in candidate_regs:
        off = addr - REG_CELL0_MV
        if compact_mode:
            idx = off + 1
        elif off % 2 == 0:
            idx = (off // 2) + 1
        else:
            continue
        if 1 <= idx <= MAX_CELLS:
            cells.append((addr, idx, mv))

    if not cells:
        return None

    min_cell = min(cells, key=lambda item: item[2])
    max_cell = max(cells, key=lambda item: item[2])
    mode = 'compact' if compact_mode else 'stride2'
    cell_list = ' '.join('C{:02d}={:.3f}V'.format(idx, mv / 1000.0)
                         for _addr, idx, mv in cells)
    return 'cells {} count={} {} min={:.3f}V#{} max={:.3f}V#{}'.format(
        mode,
        len(cells),
        cell_list,
        min_cell[2] / 1000.0,
        min_cell[1],
        max_cell[2] / 1000.0,
        max_cell[1],
    )


def describe_registers(registers):
    if not registers:
        return '0 register words'

    values = {reg['addr']: reg['value'] for reg in registers if reg.get('addr') is not None}
    cell_text = describe_cell_registers(registers)
    if cell_text:
        return cell_text

    known = summarize_known_runtime_values(values)
    for addr in (REG_CELL_AVG_MV, REG_CELL_DIFF_MV, REG_MAX_MIN_CELL_IDX):
        if addr in values:
            known.append(describe_register(addr, values[addr], values))
    if known:
        return ', '.join(known[:7])

    if registers[0].get('addr') is not None:
        start = registers[0]['addr']
        end = registers[-1]['addr']
        return 'regs 0x{:04X}..0x{:04X}'.format(start, end)
    return '{} register words'.format(len(registers))


def describe_registers_compact(registers):
    if not registers:
        return '0 regs'

    values = {reg['addr']: reg['value'] for reg in registers if reg.get('addr') is not None}
    cell_text = describe_cell_registers(registers)
    if cell_text:
        parts = cell_text.split()
        min_text = next((part for part in parts if part.startswith('min=')), '')
        max_text = next((part for part in parts if part.startswith('max=')), '')
        mode = 'compact' if 'compact' in parts else 'stride2'
        count = next((part for part in parts if part.startswith('count=')), 'count=?')
        return 'cells {} {} {} {}'.format(mode, count, min_text, max_text).strip()

    known = summarize_known_runtime_values(values)
    compact = []
    for part in known:
        if part.startswith('full='):
            continue
        else:
            compact.append(part)
    for addr in (REG_CELL_AVG_MV, REG_CELL_DIFF_MV, REG_MAX_MIN_CELL_IDX):
        if addr in values:
            compact.append(describe_register(addr, values[addr], values))
    if compact:
        return ' '.join(compact[:6])

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
        return '{}JKBMS Modbus req slave=0x{:02X} func=0x{:02X} start=0x{:04X} count={} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['start'], frame['count'], crc_text)
    if typ == 'response':
        regs = frame.get('registers', [])
        reg_text = ''
        if frame.get('start') is not None and regs:
            reg_text = ' regs=0x{:04X}..0x{:04X}'.format(regs[0]['addr'], regs[-1]['addr'])
        return '{}JKBMS Modbus rsp slave=0x{:02X} func=0x{:02X}{} bytes={} crc={}'.format(
            prefix, frame['slave'], frame['func'], reg_text, frame.get('byte_count', 0), crc_text)
    if typ == 'exception':
        return '{}JKBMS Modbus exception slave=0x{:02X} func=0x{:02X} code=0x{:02X} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['exception_code'], crc_text)
    return '{}JKBMS Modbus frame crc={}'.format(prefix, crc_text)
