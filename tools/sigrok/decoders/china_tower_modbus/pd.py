##
## China Tower / JK 008 RS485 Modbus RTU decoder for PulseView/libsigrokdecode.
##
## Stack this decoder above the built-in UART decoder. The JK app profile
## 008 traffic observed by this firmware uses 9600 8N1.
##

import sigrokdecode as srd

try:
    from .china_tower_modbus import (describe_frame_variants, describe_register_variants,
                                     frame_complete, frame_summary, hex_bytes, parse_frame)
except Exception:
    from china_tower_modbus import (describe_frame_variants, describe_register_variants,
                                    frame_complete, frame_summary, hex_bytes, parse_frame)


RX = 0
TX = 1
DECODER_VERSION = 'v2026.07.03a'


class Ann:
    FRAME, FIELD, REGISTER, DECODED, CHECKSUM, WARNING = range(6)


class Decoder(srd.Decoder):
    api_version = 3
    id = 'china_tower_modbus'
    name = 'China Tower Modbus {}'.format(DECODER_VERSION)
    longname = 'China Tower / JK 008 RS485 Modbus {}'.format(DECODER_VERSION)
    desc = 'China Tower shared battery cabinet RS485 Modbus frames used by JK UART profile 008.'
    license = 'gplv2+'
    inputs = ['uart']
    outputs = ['china_tower_modbus']
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
        ('frames', 'Frames', (Ann.FRAME,)),
        ('fields', 'Fields', (Ann.FIELD, Ann.CHECKSUM)),
        ('registers', 'Registers', (Ann.REGISTER,)),
        ('decoded-values', 'Decoded', (Ann.DECODED,)),
        ('warnings', 'Warnings', (Ann.WARNING,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = [bytearray(), bytearray()]
        self.pos = [[], []]
        self.pending = {}
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

    def pending_for(self, slave, func, rxtx):
        other = TX if rxtx == RX else RX
        return (self.pending.get((rxtx, slave, func)) or
                self.pending.get((other, slave, func)) or
                self.pending.get((slave, func)) or
                self.pending.get(slave))

    def remember_request(self, frame, rxtx):
        if frame.get('type') != 'request':
            return
        req = {
            'slave': frame['slave'],
            'func': frame['func'],
            'start': frame['start'],
            'count': frame['count'],
        }
        self.pending[(rxtx, frame['slave'], frame['func'])] = req
        self.pending[(frame['slave'], frame['func'])] = req
        self.pending[frame['slave']] = req

    def clear_pending_response(self, frame, rxtx):
        if frame.get('type') != 'response':
            return
        slave = frame.get('slave')
        func = frame.get('func')
        if slave is None or func is None:
            return
        other = TX if rxtx == RX else RX
        self.pending.pop((rxtx, slave, func), None)
        self.pending.pop((other, slave, func), None)
        self.pending.pop((slave, func), None)
        self.pending.pop(slave, None)

    def frame_request_hint(self, raw, rxtx):
        if len(raw) < 2:
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
                         ['Incomplete China Tower/Modbus frame before idle gap: {}'.format(hex_bytes(self.buf[rxtx])),
                          'Incomplete Modbus'])
            self.reset_direction(rxtx)

    def annotate_fields(self, rxtx, frame):
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

    def annotate_registers(self, rxtx, frame):
        if frame.get('type') != 'response':
            return
        for reg in frame.get('registers', []):
            idx = reg['data_index']
            addr = reg.get('addr')
            value = reg.get('value')
            if addr is None:
                texts = ['word=0x{:04X}'.format(value), 'word']
            else:
                texts = describe_register_variants(addr, value)
            self.put_idx(rxtx, idx, idx + 1, Ann.REGISTER, texts)

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
                         ['Invalid China Tower/Modbus frame: {} [{}]'.format(exc, hex_bytes(raw)),
                          'Invalid China Tower', 'Invalid'])
            self.reset_direction(rxtx)
            return

        self.put_ann(ss, es, Ann.FRAME,
                     [frame_summary(frame, self.direction_name(rxtx))])
        self.annotate_fields(rxtx, frame)
        self.annotate_registers(rxtx, frame)
        self.put_ann(ss, es, Ann.DECODED, describe_frame_variants(frame))
        if frame.get('type') == 'response' and not frame.get('crc_ok'):
            self.put_ann(ss, es, Ann.WARNING,
                         ['CRC BAD; decoded payload is tentative',
                          'tentative decode',
                          'CRC BAD'])

        if frame.get('type') == 'request':
            self.remember_request(frame, rxtx)
        elif frame.get('type') == 'response':
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

        if len(self.buf[rxtx]) > 260:
            self.put_ann(self.pos[rxtx][0][0], es, Ann.WARNING,
                         ['China Tower/Modbus frame too long, dropping', 'Too long'])
            self.reset_direction(rxtx)
