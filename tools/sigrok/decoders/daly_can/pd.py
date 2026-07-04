##
## Daly native CAN protocol decoder for PulseView/libsigrokdecode.
##
## This decoder is standalone on a logic CAN RX signal. It reuses the built-in
## CAN decoder and adds Daly-specific annotations. It can also derive CAN RX
## from digitized CANH/CANL signals when a logic analyzer is attached directly
## to the bus wires.
##

import sigrokdecode as srd

try:
    from can.pd import Decoder as CanDecoder, SamplerateError
except Exception:
    CanDecoder = None
    SamplerateError = Exception

try:
    from .daly_can import VERSION, describe_packet, format_data, frame_summary, is_known_frame_id
except Exception:
    from daly_can import VERSION, describe_packet, format_data, frame_summary, is_known_frame_id


class ChannelError(Exception):
    pass


class Ann:
    DALY_FRAME = len(CanDecoder.annotations) if CanDecoder is not None else 0
    DALY_PAYLOAD = DALY_FRAME + 1
    DALY_DECODED = DALY_FRAME + 2
    DALY_WARNING = DALY_FRAME + 3


if CanDecoder is not None:
    BaseDecoder = CanDecoder
else:
    BaseDecoder = srd.Decoder


class Decoder(BaseDecoder):
    api_version = 3
    id = 'daly_can'
    name = 'Daly CAN {}'.format(VERSION)
    longname = 'Daly native CAN {}'.format(VERSION)
    desc = 'Daly BMS native CAN frames over Classic CAN.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['daly_can']
    tags = ['Embedded/industrial']

    channels = (
        {'id': 'can_rx_or_h', 'name': 'CAN RX/H',
         'desc': 'CAN RX logic signal, CANL direct, or CANH for inverted/diff modes'},
    )
    optional_channels = (
        {'id': 'can_l', 'name': 'CANL', 'desc': 'Optional CANL for CANH/CANL digital diff mode'},
    )
    options = (
        {'id': 'nominal_bitrate', 'desc': 'Nominal bitrate (bits/s)', 'default': 250000},
        {'id': 'fast_bitrate', 'desc': 'Fast bitrate (bits/s)', 'default': 250000},
        {'id': 'sample_point', 'desc': 'Sample point (%)', 'default': 70.0},
        {'id': 'input_mode', 'desc': 'Input mode', 'default': 'rx/canl-direct',
         'values': ('rx/canl-direct', 'canh-inverted', 'canh-canl-diff')},
    )

    if CanDecoder is not None:
        annotations = CanDecoder.annotations + (
            ('daly-frame', 'Daly frame'),
            ('daly-payload', 'Daly payload'),
            ('daly-decoded', 'Daly decoded value'),
            ('daly-warning', 'Daly warning'),
        )
        annotation_rows = CanDecoder.annotation_rows + (
            ('daly-frames', 'Daly Frames', (Ann.DALY_FRAME,)),
            ('daly-payloads', 'Daly Payload', (Ann.DALY_PAYLOAD,)),
            ('daly-decoded-values', 'Daly Decoded', (Ann.DALY_DECODED,)),
            ('daly-warnings', 'Daly Warnings', (Ann.DALY_WARNING,)),
        )
    else:
        annotations = (
            ('daly-warning', 'Daly warning'),
        )
        annotation_rows = (
            ('daly-warnings', 'Daly Warnings', (Ann.DALY_WARNING,)),
        )

    def start(self):
        if CanDecoder is None:
            self.out_ann = self.register(srd.OUTPUT_ANN)
            return
        self.have_can_l = self.has_channel(1) if hasattr(self, 'has_channel') else False
        super().start()

    def put_daly_ann(self, ss, es, ann, texts):
        self.put(ss, es, self.out_ann, [ann, texts])

    def derive_can_rx(self, can_h_or_rx, can_l=None):
        mode = self.options.get('input_mode', 'rx/canl-direct')
        can_h_or_rx = 1 if can_h_or_rx else 0
        if can_l is not None:
            can_l = 1 if can_l else 0

        if mode == 'canh-inverted':
            return 0 if can_h_or_rx else 1

        if mode == 'canh-canl-diff':
            if can_l is None:
                raise ChannelError('CANL channel is required for CANH/CANL digital diff mode.')
            return 0 if can_h_or_rx and not can_l else 1

        return can_h_or_rx

    def _edge_wait_conditions(self):
        if self.options.get('input_mode') == 'canh-canl-diff':
            return [{0: 'e'}, {1: 'e'}]
        return [{0: 'e'}]

    def _sample_pins(self, pins):
        can_h_or_rx = pins[0]
        can_l = pins[1] if len(pins) > 1 else None
        return self.derive_can_rx(can_h_or_rx, can_l)

    def _decode_derived_lines(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        if self.options.get('input_mode') == 'canh-canl-diff' and not self.have_can_l:
            raise ChannelError('CANL channel is required for CANH/CANL digital diff mode.')

        last_can_rx = 1
        edge_conditions = self._edge_wait_conditions()

        while True:
            if self.state == 'IDLE':
                pins = self.wait(edge_conditions)
                can_rx = self._sample_pins(pins)
                if can_rx == 0:
                    self.sof = self.samplenum
                    self.dom_edge_seen(force=True)
                    self.state = 'GET BITS'
                last_can_rx = can_rx

            elif self.state == 'GET BITS':
                pos = self.get_sample_point(self.curbit)
                skip = pos - self.samplenum
                if skip < 1:
                    skip = 1
                pins = self.wait([{'skip': skip}] + edge_conditions)
                can_rx = self._sample_pins(pins)

                edge_seen = any(self.matched[1:])
                if edge_seen and last_can_rx == 1 and can_rx == 0:
                    self.dom_edge_seen()
                if edge_seen:
                    last_can_rx = can_rx

                if self.matched[0]:
                    self.handle_bit(can_rx)

    def decode(self):
        if CanDecoder is None:
            return

        mode = self.options.get('input_mode', 'rx/canl-direct')
        if mode == 'rx/canl-direct':
            super().decode()
            return

        self._decode_derived_lines()

    def putpy(self, data):
        if CanDecoder is not None:
            super().putpy(data)

        if not isinstance(data, tuple) or len(data) != 5:
            return

        frame_type, can_id, rtr_type, dlc, payload = data
        can_id = int(can_id)
        payload = list(payload or [])
        if len(payload) > int(dlc):
            payload = payload[:int(dlc)]

        if not is_known_frame_id(can_id):
            return

        if rtr_type not in (None, 'Data', 'data', 0):
            self.put_daly_ann(self.ss_packet, self.es_packet, Ann.DALY_WARNING,
                              ['Remote frame 0x{:08X} ignored'.format(can_id), 'RTR ignored'])
            return

        packet = (frame_type, can_id, rtr_type, dlc, payload)
        self.put_daly_ann(self.ss_packet, self.es_packet, Ann.DALY_FRAME,
                          [frame_summary(packet)])
        self.put_daly_ann(self.ss_packet, self.es_packet, Ann.DALY_PAYLOAD,
                          [format_data(payload), 'DATA'])
        self.put_daly_ann(self.ss_packet, self.es_packet, Ann.DALY_DECODED,
                          [describe_packet(packet), 'decoded'])
