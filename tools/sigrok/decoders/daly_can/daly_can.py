##
## Daly native CAN frame helpers.
##
## Kept dependency-free so the parser can be unit-tested without PulseView.
##

VERSION = 'v2026.07.04a'

DALY_PRIO = 0x18
DALY_DEFAULT_BMS_ID = 0x01
DALY_HOST_ID = 0x40

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


def be16(data, pos):
    return (data[pos] << 8) | data[pos + 1]


def be32(data, pos):
    return ((data[pos] << 24) | (data[pos + 1] << 16) |
            (data[pos + 2] << 8) | data[pos + 3])


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
    return (int(can_id) >> 16) & 0xFF


def priority(can_id):
    return (int(can_id) >> 24) & 0xFF


def source_id(can_id):
    return int(can_id) & 0xFF


def destination_id(can_id):
    return (int(can_id) >> 8) & 0xFF


def is_known_frame_id(can_id):
    return priority(can_id) == DALY_PRIO and command_id(can_id) in COMMAND_NAMES


def is_request_id(can_id):
    return (is_known_frame_id(can_id) and
            destination_id(can_id) == DALY_DEFAULT_BMS_ID and
            source_id(can_id) == DALY_HOST_ID)


def is_response_id(can_id):
    return (is_known_frame_id(can_id) and
            destination_id(can_id) == DALY_HOST_ID and
            source_id(can_id) == DALY_DEFAULT_BMS_ID)


def frame_role(can_id):
    if is_request_id(can_id):
        return 'req'
    if is_response_id(can_id):
        return 'rsp'
    return 'frame'


def all_zero(data):
    return all((value & 0xFF) == 0 for value in data)


def current_a_from_raw(raw):
    return (raw - 30000) / 10.0


def temp_c_from_offset(raw):
    return raw - 40


def on_off(value):
    return 'ON' if value else 'OFF'


def describe_payload(cmd, data):
    data = list(data or [])
    if len(data) < 8:
        return '0x{:02X} {} raw={}'.format(cmd, COMMAND_NAMES.get(cmd, 'unknown'), format_data(data))

    if cmd == 0x50:
        return '0x50 rated_capacity={:.3f}Ah raw={}'.format(
            be32(data, 0) / 1000.0,
            format_data(data),
        )

    if cmd == 0x5A:
        return '0x5A pack_voltage_extremes max={:.1f}V min={:.1f}V raw={}'.format(
            be16(data, 0) / 10.0,
            be16(data, 4) / 10.0,
            format_data(data),
        )

    if cmd == 0x5B:
        return '0x5B current_limits raw={}'.format(format_data(data))

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

    return '0x{:02X} {} raw={}'.format(cmd, COMMAND_NAMES.get(cmd, 'unknown'), format_data(data))


def describe_packet(can_packet):
    frame = can_data_bytes(can_packet)
    can_id = frame['id']
    cmd = command_id(can_id)
    data = frame['data']
    role = frame_role(can_id)

    if role == 'req' and all_zero(data):
        return '0x{:02X} request {}'.format(cmd, COMMAND_NAMES.get(cmd, 'unknown'))

    return '{} {}'.format(role, describe_payload(cmd, data))


def frame_summary(can_packet):
    frame = can_data_bytes(can_packet)
    can_id = frame['id']
    cmd = command_id(can_id)
    return 'Daly CAN {} {} id=0x{:08X} cmd=0x{:02X} {} dst=0x{:02X} src=0x{:02X} DLC={} [{}]'.format(
        VERSION,
        frame_role(can_id),
        can_id,
        cmd,
        COMMAND_NAMES.get(cmd, 'unknown'),
        destination_id(can_id),
        source_id(can_id),
        frame['dlc'],
        format_data(frame['data']),
    )
