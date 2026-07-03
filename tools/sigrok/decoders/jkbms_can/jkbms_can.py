##
## JKBMS native CAN frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

DECODER_VERSION = 'v2026.07.03a'
JKBMS_CAN_MAX_CELLS = 25

SHORT_FRAME_NAMES = {
    0x02F0: 'battery status',
    0x04F0: 'cell voltage extremes',
    0x05F0: 'cell temperature summary',
    0x07F0: 'alarm severity map',
}

EXT_FRAME_NAMES = {
    0x18F128F0: 'capacity/cycles',
    0x18F228F0: 'extended temperatures',
    0x18F328F0: 'extended alarms raw',
    0x18F428F0: 'BMS info raw',
    0x18F528F0: 'BMS status raw',
    0x1806E5F0: 'charge limits',
}

for _idx in range(7):
    EXT_FRAME_NAMES[0x18E028F0 + (_idx * 0x00010000)] = 'cell voltages {}'.format(_idx + 1)

FRAME_NAMES = {}
FRAME_NAMES.update(SHORT_FRAME_NAMES)
FRAME_NAMES.update(EXT_FRAME_NAMES)

ALARM_NAMES = (
    'Cell overvoltage',
    'Cell undervoltage',
    'Pack overvoltage',
    'Pack undervoltage',
    'Cell voltage delta high',
    'Discharge overcurrent',
    'Charge overcurrent',
    'Temperature high',
    'Temperature low',
    'Temperature delta high',
    'SOC low',
    'Insulation low',
    'High-voltage interlock fault',
    'External communication failure',
    'Internal communication failure',
)


def le16(data, pos):
    return data[pos] | (data[pos + 1] << 8)


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


def command_id(can_id):
    return int(can_id) & 0xFFFFFFF0


def is_known_frame_id(can_id):
    return command_id(can_id) in FRAME_NAMES


def frame_suffix(can_id):
    return int(can_id) & 0x0F


def bms_id_text(can_id):
    suffix = frame_suffix(can_id)
    if suffix >= 4:
        return ' node={}'.format(suffix - 4)
    return ' suffix=0x{:X}'.format(suffix)


def current_a(raw):
    return (raw / 10.0) - 400.0


def temp_c(raw):
    return float(raw) - 50.0


def cell_voltage_text(cell_idx, mv):
    if mv == 0:
        return 'C{:02d}=0'.format(cell_idx)
    if mv > 7000:
        return 'C{:02d}=raw{}mV?'.format(cell_idx, mv)
    return 'C{:02d}={:.3f}V'.format(cell_idx, mv / 1000.0)


def alarm_level_text(level):
    return {
        0: 'none',
        1: 'L1',
        2: 'L2',
        3: 'L3',
    }.get(level, 'L?')


def active_alarm_levels(raw):
    parts = []
    for idx, name in enumerate(ALARM_NAMES):
        level = (raw >> (idx * 2)) & 0x03
        if level:
            parts.append('{}={}'.format(name, alarm_level_text(level)))
    return ', '.join(parts) if parts else 'none'


def ext_cell_frame_index(cmd):
    if 0x18E028F0 <= cmd <= 0x18E628F0 and ((cmd - 0x18E028F0) % 0x00010000) == 0:
        return (cmd - 0x18E028F0) // 0x00010000
    return None


def describe_extended_temps(data):
    if len(data) < 2:
        return 'temps raw={}'.format(format_data(data))

    mask = data[0]
    temps = []
    src = 1
    if mask & 0x01:
        for idx in range(5):
            if not (mask & (1 << idx)) or src >= len(data):
                break
            temps.append((idx + 1, temp_c(data[src])))
            src += 1
    elif len(data) >= 6:
        for idx in range(5):
            temps.append((idx + 1, temp_c(data[idx + 1])))

    if not temps:
        return 'temps mask=0x{:02X} raw={}'.format(mask, format_data(data))
    return 'temps mask=0x{:02X} {}'.format(
        mask,
        ' '.join('T{}={:.1f}C'.format(idx, value) for idx, value in temps),
    )


def describe_cell_voltage_frame(cmd, data):
    frame_idx = ext_cell_frame_index(cmd)
    if frame_idx is None:
        return None

    base_cell = frame_idx * 4 + 1
    cells = []
    for idx in range(4):
        off = idx * 2
        cell_idx = base_cell + idx
        if off + 1 >= len(data):
            break
        if cell_idx > JKBMS_CAN_MAX_CELLS:
            break
        mv = le16(data, off)
        cells.append(cell_voltage_text(cell_idx, mv))

    return '0x{:08X} cells {}'.format(cmd, ' '.join(cells) if cells else 'none')


def describe_packet(can_packet):
    frame = can_data_bytes(can_packet)
    can_id = frame['id']
    cmd = command_id(can_id)
    data = frame['data']
    dlc = frame['dlc']

    if cmd == 0x02F0 and dlc >= 5:
        voltage = le16(data, 0) / 10.0
        current = current_a(le16(data, 2))
        soc = data[4]
        text = '0x{:03X} pack V={:.1f}V I={:+.1f}A SOC={}%%'.format(cmd, voltage, current, soc)
        if dlc >= 8:
            text += ' discharge_time={}h'.format(le16(data, 6))
        return text

    if cmd == 0x04F0 and dlc >= 6:
        max_mv = le16(data, 0)
        min_mv = le16(data, 3)
        return '0x{:03X} cell_max={:.3f}V#{} cell_min={:.3f}V#{} dV={}mV'.format(
            cmd,
            max_mv / 1000.0,
            data[2],
            min_mv / 1000.0,
            data[5],
            max_mv - min_mv if max_mv >= min_mv else 0,
        )

    if cmd == 0x05F0 and dlc >= 5:
        return '0x{:03X} temp max={:.1f}C#{} min={:.1f}C#{} avg={:.1f}C'.format(
            cmd,
            temp_c(data[0]),
            data[1],
            temp_c(data[2]),
            data[3],
            temp_c(data[4]),
        )

    if cmd == 0x07F0 and dlc >= 4:
        raw = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
        return '0x{:03X} alarms raw=0x{:08X} {}'.format(cmd, raw, active_alarm_levels(raw))

    if cmd == 0x18F128F0 and dlc >= 8:
        return '0x{:08X} capacity remain={:.1f}Ah rated={:.1f}Ah reserved=0x{:04X} cycles={}'.format(
            cmd,
            le16(data, 0) / 10.0,
            le16(data, 2) / 10.0,
            le16(data, 4),
            le16(data, 6),
        )

    if cmd == 0x18F228F0:
        return '0x{:08X} {}'.format(cmd, describe_extended_temps(data))

    cell_text = describe_cell_voltage_frame(cmd, data)
    if cell_text is not None:
        return cell_text

    if cmd == 0x1806E5F0 and dlc >= 4:
        return '0x{:08X} charge info V={:.1f}V I={:.1f}A raw={}'.format(
            cmd,
            le16(data, 0) / 10.0,
            le16(data, 2) / 10.0,
            format_data(data),
        )

    name = FRAME_NAMES.get(cmd, 'unknown')
    return '0x{:X} {} raw={}'.format(cmd, name, format_data(data))


def frame_summary(can_packet):
    frame = can_data_bytes(can_packet)
    can_id = frame['id']
    cmd = command_id(can_id)
    name = FRAME_NAMES.get(cmd, 'JKBMS?')
    return 'JKBMS CAN 0x{:X} {}{} DLC={} [{}]'.format(
        can_id,
        name,
        bms_id_text(can_id),
        frame['dlc'],
        format_data(frame['data']),
    )
