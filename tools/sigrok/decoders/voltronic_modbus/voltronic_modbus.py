##
## Voltronic-compatible RS485 Modbus frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FUNCTION_NAMES = {
    0x03: 'read holding registers',
    0x04: 'read input registers',
}

ORDER_CLASSIC = 'classic'
ORDER_FUNCTION_FIRST = 'function-first'

REG_PROTOCOL_TYPE = 0x0001
REG_PROTOCOL_VERSION = 0x0002

REG_CELL_COUNT = 0x0010
REG_CELL01 = 0x0011
REG_CELL_LAST = 0x0024
REG_TEMP_COUNT = 0x0025
REG_TEMP01_DECIK = 0x0026
REG_TEMP_LAST = 0x002F
REG_CHARGE_CURRENT_DA = 0x0030
REG_DISCHARGE_CURRENT_DA = 0x0031
REG_MODULE_VOLTAGE_DV = 0x0032
REG_SOC_PCT = 0x0033
REG_TOTAL_CAPACITY_MAH = 0x0034
REG_PARALLEL_PACKS = 0x0036
REG_CHARGE_ALARM = 0x0037
REG_DISCHARGE_ALARM = 0x0038
REG_CHARGE_PROTECT = 0x0039
REG_CHARGE_PROTECT2 = 0x003A
REG_DISCHARGE_PROTECT = 0x003B
REG_DISCHARGE_PROTECT2 = 0x003C
REG_BMS_STATE = 0x003D
REG_DESIGN_CAPACITY_MAH = 0x003E

REG_WARNING_CELL_COUNT = 0x0040
REG_CELL_STATE_PAIR01 = 0x0041
REG_CELL_STATE_PAIR_LAST = 0x004A
REG_WARNING_TEMP_COUNT = 0x0050
REG_TEMP_STATE_PAIR01 = 0x0051
REG_TEMP_STATE_PAIR_LAST = 0x0055

REG_MODULE_CHG_V_STATE = 0x0060
REG_MODULE_DIS_V_STATE = 0x0061
REG_CELL_CHG_V_STATE = 0x0062
REG_CELL_DIS_V_STATE = 0x0063
REG_MODULE_CHG_I_STATE = 0x0064
REG_MODULE_DIS_I_STATE = 0x0065
REG_MODULE_CHG_T_STATE = 0x0066
REG_MODULE_DIS_T_STATE = 0x0067
REG_CELL_CHG_T_STATE = 0x0068
REG_CELL_DIS_T_STATE = 0x0069

REG_CHARGE_V_LIMIT_DV = 0x0070
REG_DISCHARGE_V_LIMIT_DV = 0x0071
REG_CHARGE_I_LIMIT_DA = 0x0072
REG_DISCHARGE_I_LIMIT_DA = 0x0073
REG_CHG_DSG_STATUS = 0x0074
REG_RUNTIME_EMPTY_MIN = 0x0075
REG_REMAIN_CAPACITY_MAH = 0x0076

STATUS_FLAGS = (
    (0x0080, 'charge_enable'),
    (0x0040, 'discharge_enable'),
    (0x0020, 'charge_now'),
    (0x0010, 'charge_now2'),
    (0x0008, 'full_charge_req'),
)

JK_REG_CELL_COUNT = 0x106C
JK_REG_STATUS_START = JK_REG_CELL_COUNT
JK_REG_CHARGE_MOS = 0x106D
JK_REG_DISCHARGE_MOS = 0x106E
JK_REG_BALANCE_ACTIVE = 0x106F
JK_REG_RATED_CAPACITY_MAH = 0x1070

JK_REG_CELL01_MV = 0x1200
JK_REG_CELL_STEP = 0x0002
JK_MAX_CELLS = 20
JK_REG_CELL_AVG_MV = 0x1244
JK_REG_CELL_DIFF_MV = 0x1246
JK_REG_MAX_MIN_CELL_IDX = 0x1248

JK_REG_MOS_TEMP_DECIC = 0x128A
JK_REG_PACK_VOLTAGE_MV_U32 = 0x1290
JK_REG_PACK_VOLTAGE_CV = 0x1290
JK_REG_PACK_POWER_MW_U32 = 0x1294
JK_REG_PACK_CURRENT_MA_I32 = 0x1298
JK_REG_TEMP1_DECIC = 0x129C
JK_REG_TEMP2_DECIC = 0x129E
JK_REG_ALARM_U32 = 0x12A0
JK_REG_BALANCE_SOC_U8X2 = 0x12A6
JK_REG_REMAIN_MAH_I32 = 0x12A8
JK_REG_FULL_MAH_U32 = 0x12AC
JK_REG_CYCLES_U32 = 0x12B0
JK_REG_SOH_PRECHARGE_U8X2 = 0x12B8

POLL_BLOCKS = (
    (REG_PROTOCOL_TYPE, 1),
    (REG_CHARGE_CURRENT_DA, 1),
    (REG_DISCHARGE_CURRENT_DA, 1),
    (REG_MODULE_VOLTAGE_DV, 1),
    (REG_SOC_PCT, 1),
    (REG_TOTAL_CAPACITY_MAH, 1),
    (REG_TOTAL_CAPACITY_MAH + 1, 1),
    (REG_CHARGE_V_LIMIT_DV, 1),
    (REG_DISCHARGE_V_LIMIT_DV, 1),
    (REG_CHARGE_I_LIMIT_DA, 1),
    (REG_DISCHARGE_I_LIMIT_DA, 1),
    (REG_CHG_DSG_STATUS, 1),
    (REG_CELL_COUNT, 0x30),
    (REG_WARNING_CELL_COUNT, 0x2A),
    (REG_CHARGE_V_LIMIT_DV, 0x08),
    (JK_REG_STATUS_START, 5),
    (JK_REG_CELL01_MV, 16),
    (JK_REG_CELL01_MV + 0x10, 16),
    (JK_REG_CELL_AVG_MV, 6),
    (JK_REG_MOS_TEMP_DECIC, 40),
    (JK_REG_SOH_PRECHARGE_U8X2, 2),
)

REGISTER_NAMES = {
    REG_PROTOCOL_TYPE: 'protocol_type',
    REG_PROTOCOL_VERSION: 'protocol_version',
    REG_CELL_COUNT: 'cell_count',
    REG_TEMP_COUNT: 'temp_count',
    REG_CHARGE_CURRENT_DA: 'charge_current',
    REG_DISCHARGE_CURRENT_DA: 'discharge_current',
    REG_MODULE_VOLTAGE_DV: 'pack_voltage',
    REG_SOC_PCT: 'soc',
    REG_TOTAL_CAPACITY_MAH: 'total_capacity_hi',
    REG_TOTAL_CAPACITY_MAH + 1: 'total_capacity_lo',
    REG_PARALLEL_PACKS: 'parallel_packs',
    REG_CHARGE_ALARM: 'charge_alarm',
    REG_DISCHARGE_ALARM: 'discharge_alarm',
    REG_CHARGE_PROTECT: 'charge_protect',
    REG_CHARGE_PROTECT2: 'charge_protect2',
    REG_DISCHARGE_PROTECT: 'discharge_protect',
    REG_DISCHARGE_PROTECT2: 'discharge_protect2',
    REG_BMS_STATE: 'bms_state',
    REG_DESIGN_CAPACITY_MAH: 'design_capacity_hi',
    REG_DESIGN_CAPACITY_MAH + 1: 'design_capacity_lo',
    REG_WARNING_CELL_COUNT: 'warning_cell_count',
    REG_WARNING_TEMP_COUNT: 'warning_temp_count',
    REG_MODULE_CHG_V_STATE: 'module_chg_v_state',
    REG_MODULE_DIS_V_STATE: 'module_dis_v_state',
    REG_CELL_CHG_V_STATE: 'cell_chg_v_state',
    REG_CELL_DIS_V_STATE: 'cell_dis_v_state',
    REG_MODULE_CHG_I_STATE: 'module_chg_i_state',
    REG_MODULE_DIS_I_STATE: 'module_dis_i_state',
    REG_MODULE_CHG_T_STATE: 'module_chg_t_state',
    REG_MODULE_DIS_T_STATE: 'module_dis_t_state',
    REG_CELL_CHG_T_STATE: 'cell_chg_t_state',
    REG_CELL_DIS_T_STATE: 'cell_dis_t_state',
    REG_CHARGE_V_LIMIT_DV: 'charge_voltage_limit',
    REG_DISCHARGE_V_LIMIT_DV: 'discharge_voltage_limit',
    REG_CHARGE_I_LIMIT_DA: 'charge_current_limit',
    REG_DISCHARGE_I_LIMIT_DA: 'discharge_current_limit',
    REG_CHG_DSG_STATUS: 'charge_discharge_status',
    REG_RUNTIME_EMPTY_MIN: 'runtime_empty_min',
    REG_REMAIN_CAPACITY_MAH: 'remain_capacity_hi',
    REG_REMAIN_CAPACITY_MAH + 1: 'remain_capacity_lo',
    JK_REG_CELL_COUNT: 'jk_cell_count',
    JK_REG_CHARGE_MOS: 'jk_charge_mos',
    JK_REG_DISCHARGE_MOS: 'jk_discharge_mos',
    JK_REG_BALANCE_ACTIVE: 'jk_balance_active',
    JK_REG_RATED_CAPACITY_MAH: 'jk_rated_capacity',
    JK_REG_CELL_AVG_MV: 'jk_cell_avg',
    JK_REG_CELL_DIFF_MV: 'jk_cell_diff',
    JK_REG_MAX_MIN_CELL_IDX: 'jk_cell_idx_raw',
    JK_REG_MOS_TEMP_DECIC: 'jk_mos_temp',
    JK_REG_PACK_VOLTAGE_CV: 'jk_pack_voltage',
    JK_REG_PACK_VOLTAGE_CV + 2: 'jk_pack_current_candidate',
    JK_REG_PACK_POWER_MW_U32: 'jk_pack_power_hi',
    JK_REG_PACK_POWER_MW_U32 + 1: 'jk_pack_power_lo',
    JK_REG_PACK_CURRENT_MA_I32: 'jk_pack_current_hi',
    JK_REG_PACK_CURRENT_MA_I32 + 1: 'jk_pack_current_lo',
    JK_REG_TEMP1_DECIC: 'jk_temp1',
    JK_REG_TEMP2_DECIC: 'jk_temp2',
    JK_REG_ALARM_U32: 'jk_alarm_hi',
    JK_REG_ALARM_U32 + 1: 'jk_alarm_lo',
    JK_REG_BALANCE_SOC_U8X2: 'jk_balance_soc',
    JK_REG_REMAIN_MAH_I32: 'jk_remain_hi',
    JK_REG_REMAIN_MAH_I32 + 1: 'jk_remain_lo',
    JK_REG_FULL_MAH_U32: 'jk_full_hi',
    JK_REG_FULL_MAH_U32 + 1: 'jk_full_lo',
    JK_REG_CYCLES_U32: 'jk_cycles_hi',
    JK_REG_CYCLES_U32 + 1: 'jk_cycles_lo',
    JK_REG_SOH_PRECHARGE_U8X2: 'jk_soh_precharge',
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


def signed32(value):
    value &= 0xFFFFFFFF
    if value & 0x80000000:
        value -= 0x100000000
    return value


def u32_from_values(values, addr):
    if addr in values and (addr + 1) in values:
        return ((values[addr] & 0xFFFF) << 16) | (values[addr + 1] & 0xFFFF)
    return None


def i32_from_values(values, addr):
    raw = u32_from_values(values, addr)
    if raw is None:
        return None
    return signed32(raw)


def flag_list(value, flags):
    known_mask = 0
    names = []
    for mask, name in flags:
        known_mask |= mask
        if value & mask:
            names.append(name)
    unknown = value & ~known_mask
    if unknown:
        names.append('unknown=0x{:04X}'.format(unknown))
    return ', '.join(names) if names else 'none'


def bit_mask_text(value):
    if value == 0:
        return 'none'
    return ','.join('b{}'.format(i) for i in range(16) if value & (1 << i))


def normalize_cell_mv(value):
    if 1000 <= value <= 6000:
        return value
    if 100 <= value <= 600:
        return value * 10
    if 10 <= value <= 60:
        return value * 100
    return None


def public_temp_c(value):
    raw = signed16(value)
    if 2000 <= value <= 4500:
        return (value - 2731) / 10.0
    if -1000 <= raw <= 1500:
        return raw / 10.0
    return None


def decic_to_c(value):
    return signed16(value) / 10.0


def packed_pct(value):
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    if lo <= 100:
        return lo
    if hi <= 100:
        return hi
    if value <= 100:
        return value
    return None


def register_name(addr):
    if REG_CELL01 <= addr <= REG_CELL_LAST:
        return 'cell{:02d}'.format(addr - REG_CELL01 + 1)
    if REG_TEMP01_DECIK <= addr <= REG_TEMP_LAST:
        return 'temp{:02d}'.format(addr - REG_TEMP01_DECIK + 1)
    if REG_CELL_STATE_PAIR01 <= addr <= REG_CELL_STATE_PAIR_LAST:
        pair = addr - REG_CELL_STATE_PAIR01 + 1
        return 'cell_state_pair{:02d}'.format(pair)
    if REG_TEMP_STATE_PAIR01 <= addr <= REG_TEMP_STATE_PAIR_LAST:
        pair = addr - REG_TEMP_STATE_PAIR01 + 1
        return 'temp_state_pair{:02d}'.format(pair)
    if JK_REG_CELL01_MV <= addr < (JK_REG_CELL01_MV + (JK_MAX_CELLS * JK_REG_CELL_STEP)):
        offset = addr - JK_REG_CELL01_MV
        idx = (offset // JK_REG_CELL_STEP) + 1
        if offset % JK_REG_CELL_STEP == 0:
            return 'jk_cell{:02d}'.format(idx)
        return 'jk_cell{:02d}_gap'.format(idx)
    return REGISTER_NAMES.get(addr, 'reg_0x{:04X}'.format(addr))


def describe_register(addr, value):
    if REG_CELL01 <= addr <= REG_CELL_LAST:
        idx = addr - REG_CELL01 + 1
        mv = normalize_cell_mv(value)
        if mv is not None:
            return 'cell{:02d}={:.3f}V'.format(idx, mv / 1000.0)
        return 'cell{:02d}_raw=0x{:04X}'.format(idx, value)

    if REG_TEMP01_DECIK <= addr <= REG_TEMP_LAST:
        idx = addr - REG_TEMP01_DECIK + 1
        temp = public_temp_c(value)
        if temp is not None:
            return 'temp{:02d}={:.1f}C'.format(idx, temp)
        return 'temp{:02d}_raw=0x{:04X}'.format(idx, value)

    if JK_REG_CELL01_MV <= addr < (JK_REG_CELL01_MV + (JK_MAX_CELLS * JK_REG_CELL_STEP)):
        offset = addr - JK_REG_CELL01_MV
        idx = (offset // JK_REG_CELL_STEP) + 1
        if offset % JK_REG_CELL_STEP == 0:
            mv = normalize_cell_mv(value)
            if mv is not None:
                return 'jk_cell{:02d}={:.3f}V'.format(idx, mv / 1000.0)
            return 'jk_cell{:02d}_raw=0x{:04X}'.format(idx, value)
        return 'jk_cell{:02d}_gap=0x{:04X}'.format(idx, value)

    if addr == REG_PROTOCOL_TYPE:
        return 'protocol_type=0x{:04X}'.format(value)
    if addr == REG_PROTOCOL_VERSION:
        return 'protocol_version=0x{:04X}'.format(value)
    if addr == REG_CELL_COUNT:
        return 'cell_count={}'.format(value)
    if addr == REG_TEMP_COUNT:
        return 'temp_count={}'.format(value)
    if addr == REG_CHARGE_CURRENT_DA:
        return 'charge_i={:.1f}A'.format(signed16(value) / 10.0)
    if addr == REG_DISCHARGE_CURRENT_DA:
        return 'discharge_i={:.1f}A'.format(signed16(value) / 10.0)
    if addr == REG_MODULE_VOLTAGE_DV:
        return 'pack_v={:.1f}V'.format(value / 10.0)
    if addr == REG_SOC_PCT:
        return 'SOC={}%'.format(value)
    if addr in (REG_TOTAL_CAPACITY_MAH, REG_TOTAL_CAPACITY_MAH + 1,
                REG_DESIGN_CAPACITY_MAH, REG_DESIGN_CAPACITY_MAH + 1,
                REG_REMAIN_CAPACITY_MAH, REG_REMAIN_CAPACITY_MAH + 1):
        return '{}=0x{:04X}'.format(register_name(addr), value)
    if addr == REG_PARALLEL_PACKS:
        return 'parallel_packs={}'.format(value)
    if addr in (REG_CHARGE_ALARM, REG_DISCHARGE_ALARM, REG_CHARGE_PROTECT,
                REG_CHARGE_PROTECT2, REG_DISCHARGE_PROTECT, REG_DISCHARGE_PROTECT2,
                REG_BMS_STATE):
        return '{}=0x{:04X} ({})'.format(register_name(addr), value, bit_mask_text(value))
    if addr == REG_WARNING_CELL_COUNT:
        return 'warning_cell_count={}'.format(value)
    if REG_CELL_STATE_PAIR01 <= addr <= REG_CELL_STATE_PAIR_LAST:
        pair = addr - REG_CELL_STATE_PAIR01 + 1
        return 'cell_state_pair{:02d}=0x{:04X}'.format(pair, value)
    if addr == REG_WARNING_TEMP_COUNT:
        return 'warning_temp_count={}'.format(value)
    if REG_TEMP_STATE_PAIR01 <= addr <= REG_TEMP_STATE_PAIR_LAST:
        pair = addr - REG_TEMP_STATE_PAIR01 + 1
        return 'temp_state_pair{:02d}=0x{:04X}'.format(pair, value)
    if REG_MODULE_CHG_V_STATE <= addr <= REG_CELL_DIS_T_STATE:
        return '{}=0x{:04X} ({})'.format(register_name(addr), value, bit_mask_text(value))
    if addr == REG_CHARGE_V_LIMIT_DV:
        return 'chg_v_limit={:.1f}V'.format(value / 10.0)
    if addr == REG_DISCHARGE_V_LIMIT_DV:
        return 'dsg_v_limit={:.1f}V'.format(value / 10.0)
    if addr == REG_CHARGE_I_LIMIT_DA:
        return 'chg_i_limit={:.1f}A'.format(signed16(value) / 10.0)
    if addr == REG_DISCHARGE_I_LIMIT_DA:
        return 'dsg_i_limit={:.1f}A'.format(signed16(value) / 10.0)
    if addr == REG_CHG_DSG_STATUS:
        return 'status=0x{:04X} ({})'.format(value, flag_list(value, STATUS_FLAGS))
    if addr == REG_RUNTIME_EMPTY_MIN:
        return 'runtime_empty={}min'.format(value)

    if addr == JK_REG_CELL_COUNT:
        return 'jk_cell_count={}'.format(value)
    if addr == JK_REG_CHARGE_MOS:
        return 'jk_charge_mos={}'.format(value)
    if addr == JK_REG_DISCHARGE_MOS:
        return 'jk_discharge_mos={}'.format(value)
    if addr == JK_REG_BALANCE_ACTIVE:
        return 'jk_balance_active={}'.format(value)
    if addr == JK_REG_RATED_CAPACITY_MAH:
        return 'jk_rated_capacity={:.2f}Ah'.format(value / 1000.0)
    if addr == JK_REG_CELL_AVG_MV:
        mv = normalize_cell_mv(value)
        return 'cell_avg={:.3f}V'.format(mv / 1000.0) if mv else 'cell_avg_raw=0x{:04X}'.format(value)
    if addr == JK_REG_CELL_DIFF_MV:
        return 'cell_diff={}mV'.format(value)
    if addr == JK_REG_MAX_MIN_CELL_IDX:
        return 'cell_idx raw=0x{:04X}'.format(value)
    if addr == JK_REG_MOS_TEMP_DECIC:
        return 'MOS={:.1f}C'.format(decic_to_c(value))
    if addr == JK_REG_PACK_VOLTAGE_CV:
        if 1000 <= value <= 20000:
            return 'pack_v={:.2f}V'.format(value / 100.0)
        if 100 <= value <= 2000:
            return 'pack_v={:.1f}V'.format(value / 10.0)
        return 'pack_v_raw=0x{:04X}'.format(value)
    if addr == JK_REG_PACK_VOLTAGE_CV + 2:
        return 'pack_i_candidate={:+.2f}A'.format(signed16(value) / 100.0)
    if addr == JK_REG_TEMP1_DECIC:
        return 'T1={:.1f}C'.format(decic_to_c(value))
    if addr == JK_REG_TEMP2_DECIC:
        return 'T2={:.1f}C'.format(decic_to_c(value))
    if addr == JK_REG_BALANCE_SOC_U8X2:
        return 'balance=0x{:02X} SOC={}%'.format((value >> 8) & 0xFF, value & 0xFF)
    if addr == JK_REG_SOH_PRECHARGE_U8X2:
        return 'SOH={}% precharge=0x{:02X}'.format((value >> 8) & 0xFF, value & 0xFF)

    return '{}=0x{:04X}'.format(register_name(addr), value)


def describe_register_variants(addr, value):
    full = '0x{:04X} {}'.format(addr, describe_register(addr, value))
    name = register_name(addr)
    if name == 'reg_0x{:04X}'.format(addr):
        return [full, '0x{:04X}=0x{:04X}'.format(addr, value), '0x{:04X}'.format(addr)]
    return [full, '{}=0x{:04X}'.format(name, value), name, '0x{:04X}'.format(addr)]


def frame_identities(raw):
    raw = bytes(raw)
    ids = []
    if len(raw) < 2:
        return ids
    if (raw[1] & 0x7F) in FUNCTION_NAMES:
        ids.append((raw[0], raw[1] & 0x7F, ORDER_CLASSIC))
    if (raw[0] & 0x7F) in FUNCTION_NAMES and raw[1] != 0:
        ids.append((raw[1], raw[0] & 0x7F, ORDER_FUNCTION_FIRST))
    return ids


def id_for_order(raw, order):
    if order == ORDER_FUNCTION_FIRST:
        return raw[1], raw[0] & 0x7F
    return raw[0], raw[1] & 0x7F


def parse_request(raw, order, crc, expected_crc, crc_ok):
    slave, func = id_for_order(raw, order)
    return {
        'raw': bytes(raw),
        'type': 'request',
        'order': order,
        'format': 'request',
        'slave': slave,
        'func': func,
        'start': be16(raw, 2),
        'count': be16(raw, 4),
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def parse_response(raw, order, response_format, request, data_index, byte_count,
                   crc, expected_crc, crc_ok):
    slave, func = id_for_order(raw, order)
    regs = []
    start = request.get('start') if request else None
    for idx in range(byte_count // 2):
        addr = (start + idx) if start is not None else None
        regs.append({
            'addr': addr,
            'value': be16(raw, data_index + (idx * 2)),
            'data_index': data_index + (idx * 2),
        })
    return {
        'raw': bytes(raw),
        'type': 'response',
        'order': order,
        'format': response_format,
        'slave': slave,
        'func': func,
        'byte_count': byte_count,
        'start': start,
        'count': len(regs),
        'registers': regs,
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def parse_exception(raw, order, crc, expected_crc, crc_ok):
    slave, func = id_for_order(raw, order)
    exception_byte = raw[1] if order == ORDER_CLASSIC else raw[0]
    return {
        'raw': bytes(raw),
        'type': 'exception',
        'order': order,
        'format': 'exception',
        'slave': slave,
        'func': func,
        'exception_func': exception_byte,
        'exception_code': raw[2],
        'crc': crc,
        'expected_crc': expected_crc,
        'crc_ok': crc_ok,
    }


def _try_parse_response_with_order(raw, order, request, crc, expected_crc, crc_ok):
    if len(raw) >= 5:
        byte_count = raw[2]
        if byte_count >= 2 and byte_count % 2 == 0 and len(raw) == 3 + byte_count + 2:
            return parse_response(raw, order, 'standard', request, 3, byte_count,
                                  crc, expected_crc, crc_ok)

    if len(raw) >= 6:
        word_count = be16(raw, 2)
        if 0 < word_count <= 125 and len(raw) == 4 + (word_count * 2) + 2:
            return parse_response(raw, order, 'word-count', request, 4, word_count * 2,
                                  crc, expected_crc, crc_ok)

        wide_byte_count = be16(raw, 2)
        if (wide_byte_count >= 2 and wide_byte_count % 2 == 0 and
                wide_byte_count <= 250 and len(raw) == 4 + wide_byte_count + 2):
            return parse_response(raw, order, 'wide-byte-count', request, 4, wide_byte_count,
                                  crc, expected_crc, crc_ok)

    raise ValueError('not a response frame')


def _try_parse_request_with_order(raw, order, crc, expected_crc, crc_ok):
    if len(raw) == 8:
        start = be16(raw, 2)
        count = be16(raw, 4)
        if 0 < count <= 125:
            return parse_request(raw, order, crc, expected_crc, crc_ok)

    raise ValueError('not a request frame')


def _looks_like_known_poll_request(raw, order):
    if len(raw) != 8:
        return False
    slave, func = id_for_order(raw, order)
    if slave == 0 or func not in FUNCTION_NAMES:
        return False
    start = be16(raw, 2)
    count = be16(raw, 4)
    return (start, count) in POLL_BLOCKS


def _parse_with_order(raw, order, request, crc, expected_crc, crc_ok):
    slave, func = id_for_order(raw, order)
    if func not in FUNCTION_NAMES or slave == 0:
        raise ValueError('unsupported prefix')

    exception_byte = raw[1] if order == ORDER_CLASSIC else raw[0]
    if exception_byte & 0x80:
        if len(raw) != 5:
            raise ValueError('invalid exception frame length')
        return parse_exception(raw, order, crc, expected_crc, crc_ok)

    if _looks_like_known_poll_request(raw, order):
        try:
            return _try_parse_request_with_order(raw, order, crc, expected_crc, crc_ok)
        except ValueError:
            pass

    if request:
        try:
            return _try_parse_response_with_order(raw, order, request, crc, expected_crc, crc_ok)
        except ValueError:
            return _try_parse_request_with_order(raw, order, crc, expected_crc, crc_ok)

    try:
        return _try_parse_request_with_order(raw, order, crc, expected_crc, crc_ok)
    except ValueError:
        return _try_parse_response_with_order(raw, order, request, crc, expected_crc, crc_ok)

    raise ValueError('could not classify Modbus RTU frame')


def parse_frame(raw, request=None):
    raw = bytes(raw)
    if len(raw) < 5:
        raise ValueError('frame too short')

    crc = frame_crc(raw)
    expected_crc = modbus_crc16(raw[:-2])
    crc_ok = crc == expected_crc

    errors = []
    for _, _, order in frame_identities(raw):
        try:
            return _parse_with_order(raw, order, request, crc, expected_crc, crc_ok)
        except ValueError as exc:
            errors.append(str(exc))

    if errors:
        raise ValueError(errors[-1])
    raise ValueError('unsupported function/order')


def _crc_ok_first_n(raw, length):
    return len(raw) >= length and modbus_crc16(raw[:length - 2]) == frame_crc(raw[:length])


def frame_complete(raw):
    raw = bytes(raw)
    if len(raw) < 5:
        return False

    for _, func, order in frame_identities(raw):
        header_func = raw[1] if order == ORDER_CLASSIC else raw[0]
        if header_func & 0x80:
            return len(raw) >= 5

        if len(raw) >= 8 and _crc_ok_first_n(raw, 8):
            start = be16(raw, 2)
            count = be16(raw, 4)
            if func in FUNCTION_NAMES and 0 < count <= 125 and start <= 0xFFFF:
                return True

        if len(raw) >= 3:
            byte_count = raw[2]
            if byte_count >= 2 and byte_count % 2 == 0:
                expected_len = 3 + byte_count + 2
                if len(raw) < expected_len:
                    return False
                if len(raw) == expected_len:
                    return True

        if len(raw) >= 4:
            byte_count = be16(raw, 2)
            if byte_count >= 2 and byte_count % 2 == 0 and byte_count <= 250:
                expected_len = 4 + byte_count + 2
                if len(raw) == expected_len:
                    if _crc_ok_first_n(raw, expected_len):
                        return True
                if len(raw) < expected_len:
                    return False

            count = be16(raw, 2)
            if 0 < count <= 125:
                expected_len = 4 + (count * 2) + 2
                if len(raw) < expected_len:
                    return False
                if len(raw) == expected_len:
                    return True

    return len(raw) >= 8


def values_by_addr(registers):
    return {reg['addr']: reg['value'] for reg in registers if reg.get('addr') is not None}


def summarize_cells(registers):
    cells = []
    for reg in registers:
        addr = reg.get('addr')
        value = reg.get('value')
        if addr is None:
            continue
        if REG_CELL01 <= addr <= REG_CELL_LAST:
            idx = addr - REG_CELL01 + 1
        elif JK_REG_CELL01_MV <= addr < (JK_REG_CELL01_MV + (JK_MAX_CELLS * JK_REG_CELL_STEP)):
            offset = addr - JK_REG_CELL01_MV
            if offset % JK_REG_CELL_STEP != 0:
                continue
            idx = (offset // JK_REG_CELL_STEP) + 1
        else:
            continue
        mv = normalize_cell_mv(value)
        if mv is not None:
            cells.append((idx, mv))

    if not cells:
        return None

    cells.sort()
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


def summarize_public(registers):
    values = values_by_addr(registers)
    parts = []

    if REG_CHARGE_CURRENT_DA in values or REG_DISCHARGE_CURRENT_DA in values:
        charge = signed16(values.get(REG_CHARGE_CURRENT_DA, 0)) / 10.0
        discharge = signed16(values.get(REG_DISCHARGE_CURRENT_DA, 0)) / 10.0
        if REG_CHARGE_CURRENT_DA in values:
            parts.append('charge_i={:.1f}A'.format(charge))
        if REG_DISCHARGE_CURRENT_DA in values:
            parts.append('discharge_i={:.1f}A'.format(discharge))
        if REG_CHARGE_CURRENT_DA in values and REG_DISCHARGE_CURRENT_DA in values:
            parts.append('pack_i={:+.1f}A'.format(charge - discharge))

    for addr in (REG_MODULE_VOLTAGE_DV, REG_SOC_PCT, REG_PARALLEL_PACKS,
                 REG_CHARGE_ALARM, REG_DISCHARGE_ALARM, REG_CHARGE_PROTECT,
                 REG_CHARGE_PROTECT2, REG_DISCHARGE_PROTECT, REG_DISCHARGE_PROTECT2,
                 REG_BMS_STATE, REG_CHARGE_V_LIMIT_DV, REG_DISCHARGE_V_LIMIT_DV,
                 REG_CHARGE_I_LIMIT_DA, REG_DISCHARGE_I_LIMIT_DA, REG_CHG_DSG_STATUS,
                 REG_RUNTIME_EMPTY_MIN):
        if addr in values:
            parts.append(describe_register(addr, values[addr]))

    for label, addr in (('total_cap', REG_TOTAL_CAPACITY_MAH),
                        ('design_cap', REG_DESIGN_CAPACITY_MAH),
                        ('remain_cap', REG_REMAIN_CAPACITY_MAH)):
        u32 = u32_from_values(values, addr)
        if u32 is not None:
            parts.append('{}={:.2f}Ah'.format(label, u32 / 1000.0))

    return parts


def summarize_temps(registers):
    values = values_by_addr(registers)
    parts = []
    for addr in range(REG_TEMP01_DECIK, REG_TEMP_LAST + 1):
        if addr in values:
            parts.append(describe_register(addr, values[addr]))
    for addr in (JK_REG_MOS_TEMP_DECIC, JK_REG_TEMP1_DECIC, JK_REG_TEMP2_DECIC):
        if addr in values:
            parts.append(describe_register(addr, values[addr]))
    return parts


def summarize_jk(registers):
    values = values_by_addr(registers)
    parts = []

    for addr in (JK_REG_CELL_COUNT, JK_REG_CHARGE_MOS, JK_REG_DISCHARGE_MOS,
                 JK_REG_BALANCE_ACTIVE, JK_REG_RATED_CAPACITY_MAH,
                 JK_REG_CELL_AVG_MV, JK_REG_CELL_DIFF_MV,
                 JK_REG_MAX_MIN_CELL_IDX, JK_REG_MOS_TEMP_DECIC):
        if addr in values:
            parts.append(describe_register(addr, values[addr]))

    if JK_REG_PACK_VOLTAGE_CV in values:
        parts.append(describe_register(JK_REG_PACK_VOLTAGE_CV, values[JK_REG_PACK_VOLTAGE_CV]))

    pack_mv = u32_from_values(values, JK_REG_PACK_VOLTAGE_MV_U32)
    if pack_mv is not None and 1000 <= pack_mv <= 200000:
        parts.append('pack_v_u32={:.3f}V'.format(pack_mv / 1000.0))

    if (JK_REG_PACK_VOLTAGE_CV + 2) in values:
        parts.append(describe_register(JK_REG_PACK_VOLTAGE_CV + 2,
                                       values[JK_REG_PACK_VOLTAGE_CV + 2]))

    power_mw = u32_from_values(values, JK_REG_PACK_POWER_MW_U32)
    if power_mw is not None:
        parts.append('pack_p={:.1f}W'.format(power_mw / 1000.0))

    current_ma = i32_from_values(values, JK_REG_PACK_CURRENT_MA_I32)
    if current_ma is not None:
        parts.append('pack_i_i32={:+.3f}A'.format(current_ma / 1000.0))

    for addr in (JK_REG_TEMP1_DECIC, JK_REG_TEMP2_DECIC):
        if addr in values:
            parts.append(describe_register(addr, values[addr]))

    alarm = u32_from_values(values, JK_REG_ALARM_U32)
    if alarm is not None:
        parts.append('alarm=0x{:08X}'.format(alarm))

    if JK_REG_BALANCE_SOC_U8X2 in values:
        parts.append(describe_register(JK_REG_BALANCE_SOC_U8X2,
                                       values[JK_REG_BALANCE_SOC_U8X2]))

    remain = i32_from_values(values, JK_REG_REMAIN_MAH_I32)
    if remain is not None:
        parts.append('remain={:.2f}Ah'.format(remain / 1000.0))
    full = u32_from_values(values, JK_REG_FULL_MAH_U32)
    if full is not None:
        parts.append('full={:.2f}Ah'.format(full / 1000.0))
    cycles = u32_from_values(values, JK_REG_CYCLES_U32)
    if cycles is not None:
        parts.append('cycles={}'.format(cycles))
    if JK_REG_SOH_PRECHARGE_U8X2 in values:
        parts.append(describe_register(JK_REG_SOH_PRECHARGE_U8X2,
                                       values[JK_REG_SOH_PRECHARGE_U8X2]))

    return parts


def describe_registers(registers):
    if not registers:
        return '0 register words'

    parts = []
    cell_text = summarize_cells(registers)
    if cell_text:
        parts.append(cell_text)
    parts.extend(summarize_temps(registers))
    parts.extend(summarize_public(registers))
    parts.extend(summarize_jk(registers))

    if parts:
        return ', '.join(parts[:18])

    start = registers[0].get('addr')
    end = registers[-1].get('addr')
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

    parts = []
    parts.extend(summarize_public(registers))
    parts.extend(summarize_jk(registers))
    parts.extend(summarize_temps(registers))
    if parts:
        return ' '.join(parts[:7])

    if registers[0].get('addr') is not None:
        return 'regs 0x{:04X}..0x{:04X}'.format(registers[0]['addr'], registers[-1]['addr'])
    return '{} regs'.format(len(registers))


def describe_frame(frame):
    typ = frame.get('type')
    if typ == 'request':
        name = FUNCTION_NAMES.get(frame['func'], 'function')
        match = 'poll-block' if any(frame['start'] == start and frame['count'] == count
                                    for start, count in POLL_BLOCKS) else 'custom'
        return '{} start={} count={} {} {}'.format(
            name,
            request_target_text(frame['start'], frame['count']),
            frame['count'],
            frame.get('order', ORDER_CLASSIC),
            match)
    if typ == 'response':
        regs = frame.get('registers', [])
        if frame.get('start') is not None and regs:
            return 'response {} regs 0x{:04X}..0x{:04X}: {}'.format(
                frame.get('format', 'standard'),
                regs[0]['addr'],
                regs[-1]['addr'],
                describe_registers(regs))
        return 'response {} {} bytes'.format(frame.get('format', 'standard'),
                                             frame.get('byte_count', 0))
    if typ == 'exception':
        return 'exception func=0x{:02X} code=0x{:02X} {}'.format(
            frame.get('func', 0), frame.get('exception_code', 0),
            frame.get('order', ORDER_CLASSIC))
    return 'frame'


def request_target_text(start, count):
    if count == 1:
        return '0x{:04X} {}'.format(start, register_name(start))
    end = start + count - 1
    return '0x{:04X}..0x{:04X} {}..{}'.format(
        start, end, register_name(start), register_name(end))


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
            variants.append('0x{:04X}..0x{:04X} {}'.format(
                start, end, 'CRC BAD' if not frame.get('crc_ok') else 'OK'))
        else:
            variants.append('{} bytes'.format(frame.get('byte_count', 0)))
    elif typ == 'request':
        variants.append('read {} count {}'.format(
            request_target_text(frame['start'], frame['count']),
            frame['count']))
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
    order = frame.get('order', ORDER_CLASSIC)

    if typ == 'request':
        return '{}Voltronic Modbus req slave=0x{:02X} func=0x{:02X} start={} count={} order={} crc={}'.format(
            prefix, frame['slave'], frame['func'],
            request_target_text(frame['start'], frame['count']),
            frame['count'], order, crc_text)
    if typ == 'response':
        regs = frame.get('registers', [])
        reg_text = ''
        if frame.get('start') is not None and regs:
            reg_text = ' regs=0x{:04X}..0x{:04X}'.format(regs[0]['addr'], regs[-1]['addr'])
        return '{}Voltronic Modbus rsp slave=0x{:02X} func=0x{:02X}{} bytes={} fmt={} order={} crc={}'.format(
            prefix, frame['slave'], frame['func'], reg_text, frame.get('byte_count', 0),
            frame.get('format', 'standard'), order, crc_text)
    if typ == 'exception':
        return '{}Voltronic Modbus exception slave=0x{:02X} func=0x{:02X} code=0x{:02X} order={} crc={}'.format(
            prefix, frame['slave'], frame['func'], frame['exception_code'], order, crc_text)
    return '{}Voltronic Modbus frame crc={}'.format(prefix, crc_text)
