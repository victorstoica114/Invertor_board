##
## Pylon RS485 protocol decoder for PulseView/libsigrokdecode.
##
## Stack this decoder above the built-in UART decoder. The UART decoder should
## normally be configured as 9600 8N1, LSB-first.
##

import sigrokdecode as srd

try:
    from .pylon import (REQUEST_NAMES, describe_info, frame_summary, infer_response_request,
                        is_request, parse_frame)
except Exception:
    from pylon import (REQUEST_NAMES, describe_info, frame_summary, infer_response_request,
                       is_request, parse_frame)


RX = 0
TX = 1


class Ann:
    FRAME, FIELD, PAYLOAD, DECODED, CHECKSUM, WARNING = range(6)


class Decoder(srd.Decoder):
    api_version = 3
    id = 'pylon_rs485'
    name = 'Pylon RS485'
    longname = 'Pylon-compatible RS485 ASCII'
    desc = 'Pylon-compatible BMS/inverter ASCII frames over UART/RS485.'
    license = 'gplv2+'
    inputs = ['uart']
    outputs = ['pylon_rs485']
    tags = ['Embedded/industrial']

    annotations = (
        ('frame', 'Frame'),
        ('field', 'Field'),
        ('payload', 'Payload'),
        ('decoded', 'Decoded value'),
        ('checksum', 'Checksum'),
        ('warning', 'Warning'),
    )

    annotation_rows = (
        ('frames', 'Frames', (Ann.FRAME,)),
        ('fields', 'Fields', (Ann.FIELD, Ann.CHECKSUM)),
        ('payloads', 'Payload', (Ann.PAYLOAD,)),
        ('decoded-values', 'Decoded', (Ann.DECODED,)),
        ('warnings', 'Warnings', (Ann.WARNING,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = [bytearray(), bytearray()]
        self.pos = [[], []]
        self.pending = {}

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

    def reset_direction(self, rxtx):
        self.buf[rxtx] = bytearray()
        self.pos[rxtx] = []

    def direction_name(self, rxtx):
        return 'RX' if rxtx == RX else 'TX'

    def pending_for_frame(self, frame, rxtx):
        by_addr = self.pending.get(frame['addr'])
        if by_addr is not None:
            return by_addr
        other = TX if rxtx == RX else RX
        return self.pending.get((other, frame['addr']))

    def remember_request(self, frame, rxtx):
        if is_request(frame):
            self.pending[frame['addr']] = frame['code']
            self.pending[(rxtx, frame['addr'])] = frame['code']

    def clear_pending_response(self, frame):
        if frame.get('cid1') == 0x46 and frame.get('code') == 0x00:
            self.pending.pop(frame['addr'], None)
            self.pending.pop((RX, frame['addr']), None)
            self.pending.pop((TX, frame['addr']), None)

    def annotate_fields(self, rxtx, frame, pending_cid2):
        raw = frame['raw']
        info_start = 13
        info_end = info_start + len(frame['info_ascii']) - 1
        chk_start = len(raw) - 5
        chk_end = len(raw) - 2

        self.put_idx(rxtx, 1, 2, Ann.FIELD, ['VER 0x{:02X}'.format(frame['ver']), 'VER'])
        self.put_idx(rxtx, 3, 4, Ann.FIELD, ['ADDR 0x{:02X}'.format(frame['addr']), 'ADDR'])
        self.put_idx(rxtx, 5, 6, Ann.FIELD, ['CID1 0x{:02X}'.format(frame['cid1']), 'CID1'])
        self.put_idx(rxtx, 7, 8, Ann.FIELD, ['CID2/RTN 0x{:02X}'.format(frame['code']), 'CID2'])
        self.put_idx(rxtx, 9, 12, Ann.FIELD,
                     ['LEN 0x{:04X} ({})'.format(frame['length_field'], frame['info_len']), 'LEN'])

        if frame['info_ascii']:
            self.put_idx(rxtx, info_start, info_end, Ann.PAYLOAD,
                         [frame['info_ascii'], 'INFO'])
            self.put_idx(rxtx, info_start, info_end, Ann.DECODED,
                         [describe_info(frame, pending_cid2), 'decoded'])

        chk_text = 'CHK 0x{:04X} {}'.format(
            frame['checksum'], 'OK' if frame['checksum_ok'] else 'BAD')
        self.put_idx(rxtx, chk_start, chk_end, Ann.CHECKSUM, [chk_text, 'CHK'])

        if not frame['length_ok']:
            self.put_idx(rxtx, 9, 12, Ann.WARNING,
                         ['Length check failed', 'LEN BAD'])
        if not frame['checksum_ok']:
            self.put_idx(rxtx, chk_start, chk_end, Ann.WARNING,
                         ['Checksum expected 0x{:04X}'.format(frame['expected_checksum']),
                          'CHK BAD'])

    def finish_frame(self, rxtx, es):
        if not self.buf[rxtx]:
            return

        ss = self.pos[rxtx][0][0]
        raw = bytes(self.buf[rxtx])

        try:
            frame = parse_frame(raw)
        except Exception as exc:
            self.put_ann(ss, es, Ann.WARNING,
                         ['Invalid Pylon frame: {}'.format(exc), 'Invalid Pylon', 'Invalid'])
            self.reset_direction(rxtx)
            return

        pending_cid2 = self.pending_for_frame(frame, rxtx)
        summary = frame_summary(frame, self.direction_name(rxtx), pending_cid2)
        self.put_ann(ss, es, Ann.FRAME, [summary])
        self.annotate_fields(rxtx, frame, pending_cid2)

        if is_request(frame):
            self.remember_request(frame, rxtx)
        elif frame.get('cid1') == 0x46 and frame.get('code') == 0x00:
            inferred = infer_response_request(frame, pending_cid2)
            if inferred in REQUEST_NAMES:
                self.put_ann(ss, es, Ann.DECODED,
                             ['Response to 0x{:02X} {}'.format(inferred, REQUEST_NAMES[inferred]),
                              'rsp 0x{:02X}'.format(inferred)])
            self.clear_pending_response(frame)

        self.reset_direction(rxtx)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data
        if ptype != 'DATA':
            return

        value = pdata[0]
        if value < 0 or value > 0xff:
            return

        if value == ord('~'):
            self.buf[rxtx] = bytearray([value])
            self.pos[rxtx] = [(ss, es)]
            return

        if not self.buf[rxtx]:
            return

        self.buf[rxtx].append(value)
        self.pos[rxtx].append((ss, es))

        if value == ord('\r'):
            self.finish_frame(rxtx, es)
            return

        if len(self.buf[rxtx]) > 240:
            self.put_ann(self.pos[rxtx][0][0], es, Ann.WARNING,
                         ['Pylon frame too long, dropping', 'Too long'])
            self.reset_direction(rxtx)
