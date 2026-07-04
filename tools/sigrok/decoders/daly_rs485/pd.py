##
## Daly RS485 native and Modbus decoder for PulseView/libsigrokdecode.
##
## Stack this decoder above the built-in UART decoder. It supports Daly native
## 13-byte frames and the Daly Modbus poller frames used by the firmware.
##

import sigrokdecode as srd

try:
    from .daly_rs485 import (DALY_FRAME_LEN, VERSION, describe_frame_variants,
                             describe_register_variants, frame_complete,
                             frame_summary, hex_bytes, parse_frame)
except Exception:
    from daly_rs485 import (DALY_FRAME_LEN, VERSION, describe_frame_variants,
                            describe_register_variants, frame_complete,
                            frame_summary, hex_bytes, parse_frame)


RX = 0
TX = 1


class Ann:
    FRAME, FIELD, REGISTER, DECODED, CHECKSUM, WARNING = range(6)


class Decoder(srd.Decoder):
    api_version = 3
    id = 'daly_rs485'
    name = 'Daly RS485 {}'.format(VERSION)
    longname = 'Daly native/Modbus RS485 {}'.format(VERSION)
    desc = 'Daly BMS native RS485 and Daly Modbus RTU frames.'
    license = 'gplv2+'
    inputs = ['uart']
    outputs = ['daly_rs485']
    tags = ['Embedded/industrial']

    options = (
        {'id': 'inter_frame_gap_us', 'desc': 'Inter-frame gap (us)', 'default': 5000},
    )

    annotations = (
        ('frame', 'Frame'),
        ('field', 'Field'),
        ('register', 'Register'),
        ('decoded', 'Decoded value'),
        ('checksum', 'Checksum'),
        ('warning', 'Warning'),
    )

    annotation_rows = (
        ('frames', 'Daly RS485 {}: Frames'.format(VERSION), (Ann.FRAME,)),
        ('fields', 'Daly RS485 {}: Fields'.format(VERSION), (Ann.FIELD, Ann.CHECKSUM)),
        ('registers', 'Daly RS485 {}: Registers'.format(VERSION), (Ann.REGISTER,)),
        ('decoded-values', 'Daly RS485 {}: Decoded'.format(VERSION), (Ann.DECODED,)),
        ('warnings', 'Daly RS485 {}: Warnings'.format(VERSION), (Ann.WARNING,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = [bytearray(), bytearray()]
        self.pos = [[], []]
        self.pending = {}
        self.last_request = None
        self.samplerate = None

    def metadata(self, key, value):
        if key == getattr(srd, 'SRD_CONF_SAMPLERATE', None):
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def put_ann(self, ss, es, ann, texts):
        self.put(ss, es, self.out_ann, [ann, texts])

    def put_idx(self, rxtx, start_idx, end_idx, ann, texts):
        if start_idx < 0 or end_idx >= len(self.pos[rxtx]) or start_idx > end_idx:
            return
        ss = self.pos[rxtx][start_idx][0]
        es = self.pos[rxtx][end_idx][1]
        self.put_ann(ss, es, ann, texts)

    def direction_name(self, rxtx):
        return 'RX' if rxtx == RX else 'TX'

    def reset_direction(self, rxtx):
        self.buf[rxtx] = bytearray()
        self.pos[rxtx] = []

    def remember_request(self, frame, rxtx):
        if frame.get('protocol') != 'modbus' or frame.get('type') != 'request':
            return
        req = {
            'slave': frame['slave'],
            'func': frame['func'],
            'start': frame['start'],
            'count': frame['count'],
        }
        self.last_request = req
        self.pending[(rxtx, frame['slave'], frame['func'])] = req
        self.pending[(frame['slave'], frame['func'])] = req
        self.pending[frame['slave']] = req

    def clear_pending_response(self, frame, rxtx):
        if frame.get('protocol') != 'modbus' or frame.get('type') != 'response':
            return
        slave = frame.get('slave')
        func = frame.get('func')
        other = TX if rxtx == RX else RX
        self.pending.pop((rxtx, slave, func), None)
        self.pending.pop((other, slave, func), None)
        self.pending.pop((slave, func), None)
        self.pending.pop(slave, None)

    def pending_for(self, slave, func, rxtx):
        other = TX if rxtx == RX else RX
        req = (self.pending.get((rxtx, slave, func)) or
               self.pending.get((other, slave, func)) or
               self.pending.get((slave, func)) or
               self.pending.get(slave))
        if req is None and self.last_request and self.last_request.get('func') == func:
            req = self.last_request
        return req

    def frame_request_hint(self, raw, rxtx):
        if not raw or raw[0] == 0xA5 or len(raw) < 2:
            return None
        slave = raw[0]
        func = raw[1] & 0x7f
        return self.pending_for(slave, func, rxtx)

    def maybe_flush_on_gap(self, rxtx, ss):
        if not self.buf[rxtx] or not self.samplerate:
            return
        options = getattr(self, 'options', {}) or {}
        gap_us = int(options.get('inter_frame_gap_us', 5000))
        if gap_us <= 0:
            return
        last_es = self.pos[rxtx][-1][1]
        gap_samples = int((gap_us * self.samplerate) / 1000000)
        if gap_samples > 0 and (ss - last_es) > gap_samples:
            if len(self.buf[rxtx]) < 5:
                self.reset_direction(rxtx)
                return
            start = self.pos[rxtx][0][0]
            self.put_ann(start, last_es, Ann.WARNING,
                         ['Incomplete Daly RS485 frame before idle gap: {}'.format(hex_bytes(self.buf[rxtx])),
                          'Incomplete Daly'])
            self.reset_direction(rxtx)

    def annotate_native_fields(self, rxtx, frame):
        self.put_idx(rxtx, 0, 0, Ann.FIELD, ['Start 0xA5', 'Start'])
        self.put_idx(rxtx, 1, 1, Ann.FIELD, ['Addr 0x{:02X}'.format(frame['addr']), 'Addr'])
        self.put_idx(rxtx, 2, 2, Ann.FIELD,
                     ['Cmd 0x{:02X} {}'.format(frame['cmd'], frame['cmd_name']), 'Cmd'])
        self.put_idx(rxtx, 3, 3, Ann.FIELD,
                     ['Payload length {}'.format(frame['payload_len']), 'Len'])
        self.put_idx(rxtx, 4, 11, Ann.FIELD,
                     ['Data {}'.format(hex_bytes(frame['data'])), 'Data'])
        chk_text = 'CHK 0x{:02X} {}'.format(
            frame['checksum'], 'OK' if frame['checksum_ok'] else 'BAD')
        self.put_idx(rxtx, 12, 12, Ann.CHECKSUM, [chk_text, 'CHK'])
        if not frame.get('payload_len_ok'):
            self.put_idx(rxtx, 3, 3, Ann.WARNING,
                         ['Expected payload length 8', 'Bad length'])
        if not frame.get('checksum_ok'):
            self.put_idx(rxtx, 12, 12, Ann.WARNING,
                         ['Checksum expected 0x{:02X}'.format(frame['expected_checksum']), 'CHK BAD'])

    def annotate_modbus_fields(self, rxtx, frame):
        raw = frame['raw']
        self.put_idx(rxtx, 0, 0, Ann.FIELD,
                     ['Slave 0x{:02X}'.format(frame['slave']), 'Slave'])
        self.put_idx(rxtx, 1, 1, Ann.FIELD,
                     ['Func 0x{:02X}'.format(raw[1]), 'Func'])

        if frame['type'] == 'request':
            self.put_idx(rxtx, 2, 3, Ann.FIELD,
                         ['Start 0x{:04X}'.format(frame['start']), 'Start'])
            self.put_idx(rxtx, 4, 5, Ann.FIELD,
                         ['Count {}'.format(frame['count']), 'Count'])
        elif frame['type'] == 'response':
            self.put_idx(rxtx, 2, 2, Ann.FIELD,
                         ['Byte count {}'.format(frame['byte_count']), 'Bytes'])
            if frame['byte_count'] > 0:
                self.put_idx(rxtx, 3, len(raw) - 3, Ann.FIELD,
                             ['Data {}'.format(hex_bytes(raw[3:-2])), 'Data'])
        elif frame['type'] == 'exception':
            self.put_idx(rxtx, 2, 2, Ann.FIELD,
                         ['Exception 0x{:02X}'.format(frame['exception_code']), 'Exception'])

        crc_text = 'CRC 0x{:04X} {}'.format(
            frame['crc'], 'OK' if frame['crc_ok'] else 'BAD')
        self.put_idx(rxtx, len(raw) - 2, len(raw) - 1, Ann.CHECKSUM, [crc_text, 'CRC'])
        if not frame['crc_ok']:
            self.put_idx(rxtx, len(raw) - 2, len(raw) - 1, Ann.WARNING,
                         ['CRC expected 0x{:04X}'.format(frame['expected_crc']), 'CRC BAD'])

    def annotate_fields(self, rxtx, frame):
        if frame.get('protocol') == 'native':
            self.annotate_native_fields(rxtx, frame)
            return
        self.annotate_modbus_fields(rxtx, frame)

    def annotate_registers(self, rxtx, frame):
        if frame.get('protocol') != 'modbus' or frame.get('type') != 'response':
            return
        for reg in frame.get('registers', []):
            idx = reg['data_index']
            self.put_idx(rxtx, idx, idx + 1, Ann.REGISTER,
                         describe_register_variants(reg.get('addr'), reg.get('value')))

    def finish_frame(self, rxtx, es):
        if not self.buf[rxtx]:
            return

        ss = self.pos[rxtx][0][0]
        raw = bytes(self.buf[rxtx])
        request = self.frame_request_hint(raw, rxtx)

        try:
            frame = parse_frame(raw, request)
        except Exception as exc:
            self.put_ann(ss, es, Ann.WARNING,
                         ['Invalid Daly RS485 frame: {} [{}]'.format(exc, hex_bytes(raw)),
                          'Invalid Daly', 'Invalid'])
            self.reset_direction(rxtx)
            return

        self.put_ann(ss, es, Ann.FRAME,
                     [frame_summary(frame, self.direction_name(rxtx))])
        self.annotate_fields(rxtx, frame)
        self.annotate_registers(rxtx, frame)
        self.put_ann(ss, es, Ann.DECODED, describe_frame_variants(frame))

        if frame.get('protocol') == 'modbus' and frame.get('type') == 'response' and not frame.get('crc_ok'):
            self.put_ann(ss, es, Ann.WARNING,
                         ['CRC BAD; decoded payload is tentative',
                          'tentative decode',
                          'CRC BAD'])

        if frame.get('protocol') == 'modbus' and frame.get('type') == 'request':
            self.remember_request(frame, rxtx)
        elif frame.get('protocol') == 'modbus' and frame.get('type') == 'response':
            self.clear_pending_response(frame, rxtx)

        self.reset_direction(rxtx)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data
        if ptype != 'DATA':
            return

        value = pdata[0]
        if value < 0 or value > 0xff:
            return

        if rxtx not in (RX, TX):
            rxtx = RX

        self.maybe_flush_on_gap(rxtx, ss)
        self.buf[rxtx].append(value)
        self.pos[rxtx].append((ss, es))

        if frame_complete(self.buf[rxtx]):
            self.finish_frame(rxtx, es)
            return

        max_len = max(320, DALY_FRAME_LEN)
        if len(self.buf[rxtx]) > max_len:
            self.put_ann(self.pos[rxtx][0][0], es, Ann.WARNING,
                         ['Daly RS485 frame too long, dropping', 'Too long'])
            self.reset_direction(rxtx)
