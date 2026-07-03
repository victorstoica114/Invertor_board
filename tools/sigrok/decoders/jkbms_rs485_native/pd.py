##
## JKBMS native RS485 protocol decoder for PulseView/libsigrokdecode.
##
## Stack this decoder above the built-in UART decoder. The UART decoder should
## normally be configured as 9600 8N1, LSB-first.
##

import sigrokdecode as srd

try:
    from .jkbms_native import (END_ID, START, describe_entry, describe_frame, expected_frame_len,
                               frame_complete, frame_summary, hex_bytes, parse_frame)
except Exception:
    from jkbms_native import (END_ID, START, describe_entry, describe_frame, expected_frame_len,
                              frame_complete, frame_summary, hex_bytes, parse_frame)


RX = 0
TX = 1


class Ann:
    FRAME, FIELD, ENTRY, DECODED, CHECKSUM, WARNING = range(6)


class Decoder(srd.Decoder):
    api_version = 3
    id = 'jkbms_rs485_native'
    name = 'JKBMS RS485 Native'
    longname = 'JKBMS native RS485 binary'
    desc = 'JK BMS native binary frames over UART/RS485.'
    license = 'gplv2+'
    inputs = ['uart']
    outputs = ['jkbms_rs485_native']
    tags = ['Embedded/industrial']

    options = (
        {'id': 'inter_frame_gap_us', 'desc': 'Inter-frame gap (us)', 'default': 5000},
    )

    annotations = (
        ('frame', 'Frame'),
        ('field', 'Field'),
        ('entry', 'Data entry'),
        ('decoded', 'Decoded value'),
        ('checksum', 'Checksum'),
        ('warning', 'Warning'),
    )

    annotation_rows = (
        ('frames', 'Frames', (Ann.FRAME,)),
        ('fields', 'Fields', (Ann.FIELD, Ann.CHECKSUM)),
        ('entries', 'Entries', (Ann.ENTRY,)),
        ('decoded-values', 'Decoded', (Ann.DECODED,)),
        ('warnings', 'Warnings', (Ann.WARNING,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.buf = [bytearray(), bytearray()]
        self.pos = [[], []]
        self.samplerate = None

    def metadata(self, key, value):
        if key == getattr(srd, 'SRD_CONF_SAMPLERATE', None):
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def put_ann(self, ss, es, ann, texts):
        self.put(ss, es, self.out_ann, [ann, texts])

    def put_idx(self, rxtx, start_idx, end_idx, ann, texts):
        if start_idx is None or end_idx is None:
            return
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
            start = self.pos[rxtx][0][0]
            self.put_ann(start, last_es, Ann.WARNING,
                         ['Incomplete JKBMS native frame before idle gap: {}'.format(hex_bytes(self.buf[rxtx])),
                          'Incomplete JK'])
            self.reset_direction(rxtx)

    def sync_start(self, rxtx):
        while self.buf[rxtx] and self.buf[rxtx][0] != START[0]:
            self.buf[rxtx].pop(0)
            self.pos[rxtx].pop(0)
        if len(self.buf[rxtx]) >= 2 and self.buf[rxtx][1] != START[1]:
            if self.buf[rxtx][-1] == START[0]:
                last_pos = self.pos[rxtx][-1]
                self.buf[rxtx] = bytearray([START[0]])
                self.pos[rxtx] = [last_pos]
            else:
                self.reset_direction(rxtx)

    def annotate_fields(self, rxtx, frame):
        raw = frame['raw']
        self.put_idx(rxtx, 0, 1, Ann.FIELD, ['Start 4E 57', 'Start'])
        self.put_idx(rxtx, 2, 3, Ann.FIELD,
                     ['Length {} (declared 0x{:04X})'.format(frame['length'], frame['declared_length']), 'Len'])
        self.put_idx(rxtx, 4, 7, Ann.FIELD, ['Terminal {}'.format(hex_bytes(frame['terminal'])), 'Terminal'])
        self.put_idx(rxtx, 8, 8, Ann.FIELD, ['Command 0x{:02X}'.format(frame['command']), 'Cmd'])
        self.put_idx(rxtx, 9, 9, Ann.FIELD, ['Source 0x{:02X}'.format(frame['source']), 'Src'])
        self.put_idx(rxtx, 10, 10, Ann.FIELD, ['Frame type 0x{:02X}'.format(frame['frame_type']), 'Type'])

        for entry in frame.get('entries', []):
            entry_id = entry.get('id')
            if entry_id == END_ID:
                self.put_idx(rxtx, entry['start_index'], entry['end_index'], Ann.FIELD, ['End 0x68', 'End'])
                continue
            self.put_idx(rxtx, entry['start_index'], entry['start_index'], Ann.FIELD,
                         ['ID 0x{:02X} {}'.format(entry_id, entry.get('name', '')), 'ID'])
            if entry.get('length_index') is not None:
                self.put_idx(rxtx, entry['length_index'], entry['length_index'], Ann.FIELD,
                             ['Entry length {}'.format(len(entry.get('data', b''))), 'Entry len'])
            self.put_idx(rxtx, entry.get('data_start_index'), entry.get('data_end_index'), Ann.ENTRY,
                         [describe_entry(entry)])

        crc_text = 'Checksum 0x{:04X} {}'.format(
            frame['checksum'], 'OK' if frame['checksum_ok'] else 'BAD')
        self.put_idx(rxtx, len(raw) - 4, len(raw) - 1, Ann.CHECKSUM, [crc_text, 'Checksum'])
        if not frame['checksum_ok']:
            self.put_idx(rxtx, len(raw) - 4, len(raw) - 1, Ann.WARNING,
                         ['Checksum expected 0x{:04X}'.format(frame['expected_checksum']), 'Checksum BAD'])

    def finish_frame(self, rxtx, es):
        if not self.buf[rxtx]:
            return

        ss = self.pos[rxtx][0][0]
        length = expected_frame_len(self.buf[rxtx])
        raw = bytes(self.buf[rxtx] if length is None else self.buf[rxtx][:length])

        try:
            frame = parse_frame(raw)
        except Exception as exc:
            self.put_ann(ss, es, Ann.WARNING,
                         ['Invalid JKBMS native frame: {} [{}]'.format(exc, hex_bytes(raw)),
                          'Invalid JKBMS', 'Invalid'])
            self.reset_direction(rxtx)
            return

        self.put_ann(ss, es, Ann.FRAME,
                     [frame_summary(frame, self.direction_name(rxtx))])
        self.annotate_fields(rxtx, frame)
        self.put_ann(ss, es, Ann.DECODED, [describe_frame(frame), 'decoded'])

        del self.buf[rxtx][:frame['length']]
        del self.pos[rxtx][:frame['length']]

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
        self.sync_start(rxtx)

        if frame_complete(self.buf[rxtx]):
            self.finish_frame(rxtx, es)
            return

        if len(self.buf[rxtx]) > 520:
            self.put_ann(self.pos[rxtx][0][0], es, Ann.WARNING,
                         ['JKBMS native frame too long, dropping', 'Too long'])
            self.reset_direction(rxtx)
