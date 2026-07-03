##
## Growatt CAN frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

FRAME_NAMES = {
    0x211: 'control event trigger',
    0x212: 'control query',
    0x311: 'status/limits',
    0x312: 'protections/alarms',
    0x313: 'pack V/I/SOC/SOH',
    0x314: 'capacity/delta/cycles',
    0x315: 'cell group 1',
    0x316: 'cell group 2',
    0x317: 'cell group 3',
    0x318: 'cell group 4',
    0x319: 'cell extremes/status',
    0x320: 'maker/software',
    0x321: 'upgrade info',
    0x322: 'temperature/SOC range',
    0x323: 'cell count/protection/warning',
    0x324: 'extension 1',
    0x325: 'extension 2',
}

PROT1_BITS = (
    'soft_start_fail',
    'module_uv_prot',
    'module_ov_prot',
    'cell_uv_prot',
    'cell_ov_prot',
    'scd_prot',
    'chg_oc_prot',
    'dis_oc_prot',
)

PROT2_BITS = (
    None,
    None,
    'delta_v_fail_prot',
    'system_error_prot',
    'utc_prot',
    'utd_prot',
    'otc_prot',
    'otd_prot',
)

ALM1_BITS = (
    None,
    'module_uv_alarm',
    'module_ov_alarm',
    'cell_uv_alarm',
    'cell_ov_alarm',
    None,
    'chg_oc_alarm',
    'dis_oc_alarm',
)

ALM2_BITS = (
    'int_comm_fail_alarm',
    'pack_turnoff_alarm',
    'delta_v_fail_alarm',
    None,
    'utc_warn',
    'utd_warn',
    'otc_warn',
    'otd_warn',
)

PROT3_BITS = (
    'olc_prot',
    'old_prot',
    'ext_com_fault',
    'pre_chg_fail',
    'hw_fault',
    'afe_com_fault',
    'cell_lost_fault',
    'pack_i_sample_fault',
)

PROT4_BITS = (
    'flt_sp_umain',
    'flt_sp_uload',
    'flt_eep_param',
    'flt_chbus_reverse',
    'flt_ovp',
    'flt_ocp',
    'flt_parallel',
    'flt_prll_udiff_over',
)

PROT5_BITS = (
    'flt_dis_ocp',
    'flt_ch_ilimit_norsp',
    'flt_di_ilimit_norsp',
    'flt_bus_open',
    None,
    None,
    None,
    None,
)

WARN3_BITS = (
    'olc_warn',
    'old_warn',
    'prll_i_inch_h2_oc_warn',
    'prll_i_indis_h2_oc_warn',
    None,
    None,
    None,
    None,
)


def be16(data, pos):
    return (data[pos] << 8) | data[pos + 1]


def be16s(data, pos):
    value = be16(data, pos)
    if value & 0x8000:
        value -= 0x10000
    return value


def le16(data, pos):
    return data[pos] | (data[pos + 1] << 8)


def le16s(data, pos):
    value = le16(data, pos)
    if value & 0x8000:
        value -= 0x10000
    return value


def u16(data, pos, little):
    return le16(data, pos) if little else be16(data, pos)


def i16(data, pos, little):
    return le16s(data, pos) if little else be16s(data, pos)


def format_data(data):
    return ' '.join('{:02X}'.format(value & 0xFF) for value in data)


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


def active_bits(value, names):
    parts = []
    for bit, name in enumerate(names):
        if value & (1 << bit):
            parts.append(name if name else 'bit{}'.format(bit))
    return '|'.join(parts) if parts else 'none'


def valid_pack_voltage_raw(raw):
    if 3000 <= raw <= 9000:
        return raw / 100.0
    if 300 <= raw <= 900:
        return raw / 10.0
    return None


def decode_voltage_word(data, pos):
    be_v = valid_pack_voltage_raw(be16(data, pos))
    le_v = valid_pack_voltage_raw(le16(data, pos))
    if be_v is not None and (le_v is None or be_v >= le_v):
        return be_v, False
    if le_v is not None:
        return le_v, True
    return None, False


def temp_deci_word(data, pos, little):
    return i16(data, pos, little) / 10.0


def cell_word(data, pos, pack_voltage_v=None):
    be = be16(data, pos)
    le = le16(data, pos)
    be_ok = 1500 <= be <= 5000
    le_ok = 1500 <= le <= 5000
    if not be_ok and not le_ok:
        return None
    if be_ok and le_ok and pack_voltage_v and pack_voltage_v > 10.0:
        avg = int((pack_voltage_v * 1000.0 / 16.0) + 0.5)
        be_diff = abs(be - avg)
        le_diff = abs(le - avg)
        return le if le_diff < be_diff else be
    return be if be_ok else le


def chem_str(code):
    return {
        0: 'LFP',
        1: 'Ternary',
        2: 'LTO',
        3: 'Reserved',
    }.get(code & 0x03, 'Reserved')


def mode_str(status):
    return {
        0: 'soft_start',
        1: 'standby',
        2: 'charging',
        3: 'discharging',
    }.get(status & 0x03, '?')


def op_mode_str(status):
    return {
        0: 'standalone',
        1: 'parallel',
        2: 'parallel_prep',
        3: 'reserved',
    }.get((status >> 8) & 0x03, 'reserved')


def describe_311(data):
    v0, little0 = decode_voltage_word(data, 0)
    v2, little2 = decode_voltage_word(data, 2)
    if v0 is not None:
        chg_i = i16(data, 2, little0) / 10.0
        dis_i = i16(data, 4, little0) / 10.0
        tail = be16(data, 6)
        return '0x311 limits chgV={:.1f}V chgI={:.1f}A disI={:.1f}A status_tail=0x{:04X}'.format(
            v0, chg_i, dis_i, tail)
    if v2 is not None:
        status = u16(data, 0, little2)
        chg_i = i16(data, 4, little2) / 10.0
        dis_i = i16(data, 6, little2) / 10.0
        return '0x311 status=0x{:04X} CV={:.1f}V IchgLim={:.1f}A IdisLim={:.1f}A mode={} op={}'.format(
            status, v2, chg_i, dis_i, mode_str(status), op_mode_str(status))
    return '0x311 status/limits raw={}'.format(format_data(data))


def describe_312(data):
    text = '0x312 prot1=0x{:02X} prot2=0x{:02X} alm1=0x{:02X} alm2=0x{:02X} pack={} pwrred=0x{:02X}{:02X}'.format(
        data[0], data[1], data[2], data[3], data[4], data[5], data[6])
    details = []
    for label, value, names in (
        ('prot1', data[0], PROT1_BITS),
        ('prot2', data[1], PROT2_BITS),
        ('alm1', data[2], ALM1_BITS),
        ('alm2', data[3], ALM2_BITS),
    ):
        bits = active_bits(value, names)
        if bits != 'none':
            details.append('{}={}'.format(label, bits))
    if details:
        text += ' ({})'.format(', '.join(details))
    return text


def describe_313(data):
    voltage, little = decode_voltage_word(data, 0)
    if voltage is None:
        voltage = be16s(data, 0) / 100.0
        little = False
    current = i16(data, 2, little) / 10.0
    temp = i16(data, 4, little) / 10.0
    soc = data[6]
    soh = data[7] & 0x7F
    life_warn = (data[7] >> 7) & 0x01
    return '0x313 pack V={:.2f}V I={:+.1f}A Tavg={:.1f}C SOC={}%% SOH={}%% lifeWarn={}'.format(
        voltage, current, temp, soc, soh, life_warn)


def describe_314(data):
    cycles_be = be16(data, 6)
    cycles_le = le16(data, 6)
    cycles = cycles_be if cycles_be <= 20000 else cycles_le
    return '0x314 RM={:.2f}Ah FCC={:.2f}Ah dV={}mV cycles={}'.format(
        be16(data, 0) / 100.0,
        be16(data, 2) / 100.0,
        be16(data, 4),
        cycles)


def describe_cell_group(can_id, data):
    base = ((can_id - 0x315) * 4) + 1
    cells = []
    for i in range(4):
        mv = cell_word(data, i * 2)
        if mv is None:
            cells.append('C{:02d}=raw0x{:04X}'.format(base + i, be16(data, i * 2)))
        else:
            cells.append('C{:02d}={:.3f}V'.format(base + i, mv / 1000.0))
    return '0x{:03X} cells {}'.format(can_id, ' '.join(cells))


def cell_candidate_score(max_mv, min_mv, pack_voltage_v):
    if not (1500 <= min_mv <= 5000 and 1500 <= max_mv <= 5000 and max_mv >= min_mv):
        return None
    score = max_mv - min_mv
    if pack_voltage_v and pack_voltage_v > 10.0:
        avg = int((pack_voltage_v * 1000.0 / 16.0) + 0.5)
        mid = (max_mv + min_mv) // 2
        score += abs(mid - avg) * 4
    return score


def decode_cell_extremes_319(data, pack_voltage_v=None):
    candidates = (
        (be16(data, 0), be16(data, 2), data[5], data[6]),
        (le16(data, 0), le16(data, 2), data[5], data[6]),
        (be16(data, 1), be16(data, 3), data[5], data[6]),
        (le16(data, 1), le16(data, 3), data[5], data[6]),
    )
    best = None
    best_score = None
    for max_mv, min_mv, max_idx, min_idx in candidates:
        score = cell_candidate_score(max_mv, min_mv, pack_voltage_v)
        if score is None:
            continue
        if best_score is None or score < best_score:
            best_score = score
            best = (max_mv, min_mv, max_idx, min_idx)
    return best


def describe_319(data):
    flags = data[0]
    candidate = decode_cell_extremes_319(data)
    if candidate is not None:
        max_mv, min_mv, max_idx, min_idx = candidate
        return '0x319 cell_max={:.3f}V#{} cell_min={:.3f}V#{} dV={}mV flags=0x{:02X} chem={} chgEn={} disEn={}'.format(
            max_mv / 1000.0,
            max_idx,
            min_mv / 1000.0,
            min_idx,
            max_mv - min_mv,
            flags,
            chem_str(flags),
            1 if (flags & 0x04) else 0,
            1 if (flags & 0x08) else 0)
    return '0x319 flags=0x{:02X} raw={}'.format(flags, format_data(data))


def ascii_pair(data):
    chars = []
    for value in data:
        chars.append(chr(value) if 32 <= value <= 126 else '.')
    return ''.join(chars).rstrip('. ')


def describe_320(data):
    return "0x320 maker='{}' hw=0x{:02X} swL=0x{:02X} swHext=0x{:02X} compat=0x{:02X} ext=0x{:02X}".format(
        ascii_pair(data[:2]), data[2], data[3], data[4], data[5], data[6])


def describe_321(data):
    if all(value == 0 for value in data):
        return '0x321 remote-upgrade unused (all zero)'
    return '0x321 updStatus=0x{:02X} progress={}%% progStatus=0x{:02X} raw={}'.format(
        data[0], data[1], data[2], format_data(data[3:]))


def describe_322(data):
    tmax = be16s(data, 0) / 10.0
    tmin = be16s(data, 2) / 10.0
    if not (-50.0 <= tmax <= 120.0 and -50.0 <= tmin <= 120.0):
        tmax = le16s(data, 0) / 10.0
        tmin = le16s(data, 2) / 10.0
    return '0x322 Tmax={:.1f}C#{} Tmin={:.1f}C#{} SOCmax={}%% SOCmin={}%%'.format(
        tmax, data[4], tmin, data[5], data[6], data[7])


def describe_323(data):
    text = '0x323 cellCount={} prot3=0x{:02X} prot4=0x{:02X} prot5=0x{:02X} warn3=0x{:02X}'.format(
        data[0], data[4], data[5], data[6], data[7])
    details = []
    for label, value, names in (
        ('prot3', data[4], PROT3_BITS),
        ('prot4', data[5], PROT4_BITS),
        ('prot5', data[6], PROT5_BITS),
        ('warn3', data[7], WARN3_BITS),
    ):
        bits = active_bits(value, names)
        if bits != 'none':
            details.append('{}={}'.format(label, bits))
    if details:
        text += ' ({})'.format(', '.join(details))
    return text


def describe_packet(can_packet):
    frame = can_data_bytes(can_packet)
    can_id = frame['id']
    data = frame['data']
    dlc = frame['dlc']

    if can_id == 0x311 and dlc >= 8:
        return describe_311(data)
    if can_id == 0x312 and dlc >= 8:
        return describe_312(data)
    if can_id == 0x313 and dlc >= 8:
        return describe_313(data)
    if can_id == 0x314 and dlc >= 8:
        return describe_314(data)
    if 0x315 <= can_id <= 0x318 and dlc >= 8:
        return describe_cell_group(can_id, data)
    if can_id == 0x319 and dlc >= 7:
        return describe_319(data)
    if can_id == 0x320 and dlc >= 8:
        return describe_320(data)
    if can_id == 0x321 and dlc >= 8:
        return describe_321(data)
    if can_id == 0x322 and dlc >= 8:
        return describe_322(data)
    if can_id == 0x323 and dlc >= 8:
        return describe_323(data)

    name = FRAME_NAMES.get(can_id, 'unknown')
    return '0x{:03X} {} raw={}'.format(can_id, name, format_data(data))


def frame_summary(can_packet):
    frame = can_data_bytes(can_packet)
    name = FRAME_NAMES.get(frame['id'], 'Growatt?')
    return 'Growatt CAN 0x{:03X} {} DLC={} [{}]'.format(
        frame['id'],
        name,
        frame['dlc'],
        format_data(frame['data']),
    )
