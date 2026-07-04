##
## SMA Sunny Island compatible CAN protocol decoder for PulseView/libsigrokdecode.
##
## This decoder is standalone on a logic CAN RX signal. It reuses the built-in
## CAN decoder and adds SMA-specific annotations for decoded frames. It can
## also derive CAN RX from digitized CANH/CANL signals when a logic analyzer is
## attached directly to the bus wires.
##

import sigrokdecode as srd

try:
    from can.pd import Decoder as CanDecoder, SamplerateError
except Exception:
    CanDecoder = None
    SamplerateError = Exception

try:
    from .sma_can import FRAME_NAMES, VERSION, describe_packet, format_data, frame_summary
except Exception:
    from sma_can import FRAME_NAMES, VERSION, describe_packet, format_data, frame_summary


class ChannelError(Exception):
    pass


class Ann:
    SMA_FRAME = len(CanDecoder.annotations) if CanDecoder is not None else 0
    SMA_PAYLOAD = SMA_FRAME + 1
    SMA_DECODED = SMA_FRAME + 2
    SMA_WARNING = SMA_FRAME + 3


if CanDecoder is not None:
    BaseDecoder = CanDecoder
else:
    BaseDecoder = srd.Decoder


class Decoder(BaseDecoder):
    api_version = 3
    id = 'sma_can'
    name = 'SMA CAN {}'.format(VERSION)
    longname = 'SMA Sunny Island compatible CAN {}'.format(VERSION)
    desc = 'SMA Sunny Island compatible low-voltage BMS frames over Classic CAN.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['sma_can']
    tags = ['Embedded/industrial']

    channels = (
        {'id': 'can_rx_or_h', 'name': 'CAN RX/H',
         'desc': 'CAN RX logic signal, CANL direct, or CANH for inverted/diff modes'},
    )
    optional_channels = (
        {'id': 'can_l', 'name': 'CANL', 'desc': 'Optional CANL for CANH/CANL digital diff mode'},
    )
    options = (
        {'id': 'nominal_bitrate', 'desc': 'Nominal bitrate (bits/s)', 'default': 500000},
        {'id': 'fast_bitrate', 'desc': 'Fast bitrate (bits/s)', 'default': 500000},
        {'id': 'sample_point', 'desc': 'Sample point (%)', 'default': 70.0},
        {'id': 'input_mode', 'desc': 'Input mode', 'default': 'rx/canl-direct',
         'values': ('rx/canl-direct', 'canh-inverted', 'canh-canl-diff')},
    )

    if CanDecoder is not None:
        annotations = CanDecoder.annotations + (
            ('sma-frame', 'SMA frame'),
            ('sma-payload', 'SMA payload'),
            ('sma-decoded', 'SMA decoded value'),
            ('sma-warning', 'SMA warning'),
        )
        annotation_rows = CanDecoder.annotation_rows + (
            ('sma-frames', 'SMA Frames', (Ann.SMA_FRAME,)),
            ('sma-payloads', 'SMA Payload', (Ann.SMA_PAYLOAD,)),
            ('sma-decoded-values', 'SMA Decoded', (Ann.SMA_DECODED,)),
            ('sma-warnings', 'SMA Warnings', (Ann.SMA_WARNING,)),
        )
    else:
        annotations = (
            ('sma-warning', 'SMA warning'),
        )
        annotation_rows = (
            ('sma-warnings', 'SMA Warnings', (Ann.SMA_WARNING,)),
        )

    def start(self):
        if CanDecoder is None:
            self.out_ann = self.register(srd.OUTPUT_ANN)
            return
        self.have_can_l = self.has_channel(1) if hasattr(self, 'has_channel') else False
        super().start()

    def put_sma_ann(self, ss, es, ann, texts):
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

        if can_id not in FRAME_NAMES:
            return

        if rtr_type not in (None, 'Data', 'data', 0):
            self.put_sma_ann(self.ss_packet, self.es_packet, Ann.SMA_WARNING,
                             ['Remote frame 0x{:03X} ignored'.format(can_id), 'RTR ignored'])
            return

        packet = (frame_type, can_id, rtr_type, dlc, payload)
        self.put_sma_ann(self.ss_packet, self.es_packet, Ann.SMA_FRAME,
                         [frame_summary(packet)])
        self.put_sma_ann(self.ss_packet, self.es_packet, Ann.SMA_PAYLOAD,
                         [format_data(payload), 'DATA'])
        self.put_sma_ann(self.ss_packet, self.es_packet, Ann.SMA_DECODED,
                         [describe_packet(packet), 'decoded'])
