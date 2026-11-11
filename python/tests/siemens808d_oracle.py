"""Literal native4.90 emitted-subset oracle; imports no post/dialect/decoder code.

Data contracts only. Manual pages/provenance: docs/siemens808d-post.md. This is
independent local interpretation, not Siemens firmware or machine qualification.
"""

import math
import re
from camlib.motion import Motion, Line, Arc, Helix, ToolChange


class Oracle:
    def __init__(
        self,
        *,
        tools,
        initial_position,
        tool_change,
        coolant_codes,
        stop_window='fine',
        work_offset=1,
        spindle_position_deg=None,
    ):
        reserved = {0, 1, 2, 3, 4, 5, 6, 17, 19, 30, 40, 41, 42, 43, 44, 45, 70}
        if any(not re.fullmatch(r'M[0-9]{1,2}', code) or int(code[1:]) in reserved for code in coolant_codes.values()):
            raise ValueError('oracle: invalid/reserved OEM coolant declaration')
        if len(tools) > 64 or any(not 1 <= t.number <= 32000 or not 1 <= t.length_register <= 9 for t in tools):
            raise ValueError('oracle: tool/edge resource range')
        self.tools = {(t.number, t.length_register): t for t in tools}
        self.position = initial_position
        self.tool_change = tool_change
        self.coolant_codes = coolant_codes
        self.stop_window = stop_window
        self.work_offset = work_offset
        self.ready = set()
        self.units = None
        self.offset = None
        self.selected = None
        self.active = None
        self.edge = None
        self.rpm = 0
        self.coolant = 'off'
        self.ended = False
        self.count = 0
        # CYCLE84: the declared oriented-stop angle, the M function of the
        # running spindle (3/4) and the taps made (hand, start, bottom, feed).
        self.spindle_position_deg = spindle_position_deg
        self.turning = None
        self.taps = []

    def read(self, line):
        if self.ended:
            raise ValueError('oracle: code after end')
        if len(line) > 1024 or not line.isascii() or any(ord(c) < 32 and c != '\t' or ord(c) == 127 for c in line):
            raise ValueError('oracle: invalid block')
        text = line.partition(';')[0].strip().upper()
        text = re.sub(r'^N[0-9]{1,8}\s*', '', text)
        if not text:
            return []
        if text == 'G290':
            self.ready.add('native')
            return []
        if 'native' not in self.ready:
            raise ValueError('oracle: language not selected')
        if text.startswith('CYCLE84'):
            return self.cycle84(text)
        resets = {
            'G90': 'absolute',
            'G17': 'plane',
            'G94': 'feed',
            'G40': 'radius',
            'COMPOF': 'compression',
            'MCALL': 'cycle',
            'TRANS': 'frame',
            'G60': 'path',
            'RNDM=0': 'rounding',
            'CFTCP': 'circle_feed',
        }
        if text in resets:
            self.ready.add(resets[text])
            return []
        if text in ('G700', 'G710'):
            self.units = 25.4 if text == 'G700' else 1.0
            return []
        if text in ('G601', 'G602'):
            if text != ('G601' if self.stop_window == 'fine' else 'G602'):
                raise ValueError('oracle: stop criterion')
            self.ready.add('window')
            return []
        if text in ('G54', 'G55', 'G56', 'G57', 'G58', 'G59'):
            self.offset = int(text[1:]) - 53
            if self.offset != self.work_offset:
                raise ValueError('oracle: unexpected work offset')
            return []
        if text in self.coolant_codes.values():
            mode = next(k for k, v in self.coolant_codes.items() if v == text)
            if mode != 'off' and self.coolant != 'off':
                raise ValueError('oracle: coolant not cleared')
            self.coolant = mode
            self.ready.add('coolant')
            return []
        if text == 'M5':
            self.rpm = 0
            self.turning = None
            self.ready.add('spindle')
            return []
        if text == 'D0':
            self.edge = None
            self.ready.add('length')
            return []
        if text in ('M0', 'M1'):
            return []
        if text == 'M2':
            if self.rpm or self.coolant != 'off' or self.edge is not None:
                raise ValueError('oracle: shutdown not explicit')
            self.ended = True
            return []
        turn = text.endswith(' TURN=0')
        if turn:
            text = text[:-7]
        pieces = re.findall(r'([A-Z])([+-]?(?:\d+(?:\.\d*)?|\.\d+))', text.replace(' ', ''))
        if ''.join(a + b for a, b in pieces) != text.replace(' ', ''):
            raise ValueError('oracle: unsupported syntax')
        if len({a for a, b in pieces}) != len(pieces):
            raise ValueError('oracle: duplicate word')
        values = {a: float(b) for a, b in pieces}
        if any(not math.isfinite(v) or abs(v) > 1e9 for v in values.values()):
            raise ValueError('oracle: invalid number')
        if set(values) <= {'M', 'S'} and values.get('M') in (3, 4) and 'S' in values:
            if self.active is None or self.edge is None or not 0 < values['S'] <= 99999.999:
                raise ValueError('oracle: spindle state')
            self.rpm = values['S']
            self.turning = int(values['M'])
            return []
        if 'T' in values:
            if self.rpm or values['T'] != int(values['T']) or not set(values) <= {'T', 'D'}:
                raise ValueError('oracle: tool select')
            self.selected = int(values.pop('T'))
            if self.tool_change == 'direct_t':
                self.active = self.selected
                self.edge = 1
        if 'M' in values:
            if values.pop('M') != 6 or self.tool_change != 'm6' or self.selected is None or self.rpm:
                raise ValueError('oracle: M6 policy')
            self.active = self.selected
            self.edge = 1
        if 'D' in values:
            number = values.pop('D')
            if number != int(number) or values:
                raise ValueError('oracle: D edge')
            setting = self.tools.get((self.active, int(number)))
            if setting is None:
                raise ValueError('oracle: edge absent from physical tool')
            self.edge = int(number)
            self.position = tuple(setting.position_after_mm)
            return [ToolChange(setting.tool, setting.number, setting.length_register, self.position)]
        if not values:
            return []
        required = {
            'native',
            'absolute',
            'plane',
            'feed',
            'radius',
            'compression',
            'cycle',
            'frame',
            'path',
            'rounding',
            'window',
            'coolant',
            'spindle',
            'length',
            'circle_feed',
        }
        if (
            not required <= self.ready
            or self.units is None
            or self.offset is None
            or self.active is None
            or self.edge is None
        ):
            raise ValueError('oracle: motion before initialized')
        code = values.pop('G', None)
        if code not in (0, 1, 2, 3):
            raise ValueError('oracle: unsupported interpolation')
        setting = self.tools.get((self.active, self.edge))
        if setting is None:
            raise ValueError('oracle: unknown active geometry')
        end = tuple(
            values.pop(axis) * self.units if axis in values else self.position[i] for i, axis in enumerate('XYZ')
        )
        if code == 0:
            feed = None
        else:
            if self.rpm <= 0 or values.get('F', 0) <= 0:
                raise ValueError('oracle: no cutting spindle/feed')
            feed = values.pop('F') * self.units
        if any(abs(v) > 99999.999 for v in end) or feed is not None and feed > 99999.999:
            raise ValueError('oracle: reference work-coordinate/feed range')
        curve = Line(self.position, end)
        if code in (2, 3):
            center = (
                self.position[0] + values.pop('I') * self.units,
                self.position[1] + values.pop('J') * self.units,
                self.position[2],
            )
            x, y = self.position[0] - center[0], self.position[1] - center[1]
            a, b = end[0] - center[0], end[1] - center[1]
            if math.hypot(x, y) != math.hypot(a, b):
                raise ValueError('oracle: unmodeled circle correction')
            angle = math.atan2(x * b - y * a, x * a + y * b)
            sweep = angle % math.tau if code == 3 else -((-angle) % math.tau)
            if sweep == 0:
                sweep = math.tau if code == 3 else -math.tau
            helical = end[2] != self.position[2]
            if turn != helical:
                raise ValueError('oracle: explicit TURN=0 helix form required')
            curve = (Helix if helical else Arc)(self.position, end, center, sweep)
        elif turn:
            raise ValueError('oracle: TURN on straight motion')
        if values:
            raise ValueError('oracle: unused words')
        self.position = end
        self.count += 1
        return [Motion(self.count, 'observed', setting.tool, curve, feed, 'rapid' if code == 0 else 'cut')]

    def cycle84(self, text):
        """CYCLE84 as the 07/2018 manual gives it (pp.166-169), written out
        move by move. An omitted parameter is taken as zero (p.154, p.168)."""
        call = re.fullmatch(r'CYCLE84\((.*)\)', text)
        if call is None or '(' in call[1] or ')' in call[1]:
            raise ValueError('oracle: cycle call syntax')
        given = [a.strip() for a in call[1].split(',')]
        if len(given) > 18:
            raise ValueError('oracle: alarm 12340, too many parameters')
        try:
            numbers = [float(a) if a else 0.0 for a in given + [''] * (18 - len(given))]
        except ValueError:
            raise ValueError('oracle: cycle parameter is not a number') from None
        rtp, rfp, sdis, dp, dpr, dtb, sdac, mpit, pit, poss, sst, sst1, axn, sys1, sys2, vari, dam, vrt = numbers
        if (
            not {'native', 'absolute', 'plane', 'feed', 'radius', 'frame', 'window', 'length'} <= self.ready
            or self.units != 1.0
            or self.offset is None
            or self.active is None
            or self.edge is None
        ):
            raise ValueError('oracle: cycle before initialized (mm, G17, tool edge, work offset)')
        setting = self.tools.get((self.active, self.edge))
        if setting is None or self.turning is None or self.rpm <= 0:
            raise ValueError('oracle: tapping without tool geometry or a running spindle')
        if self.spindle_position_deg is None or poss != self.spindle_position_deg:
            raise ValueError('oracle: POSS is not the declared oriented stop')
        if axn != 3 or sys1 or sys2 or vari or dtb or mpit:
            raise ValueError('oracle: outside the one-pass Z subset')
        if dpr or sdis:
            raise ValueError('oracle: relative depth/safety clearance not in the subset')
        x, y, z = self.position
        # p.167: G0 to RFP + SDIS; the subset has the tool already there.
        if rfp + sdis != z or rtp != z:
            raise ValueError('oracle: the cycle would traverse before or after tapping')
        if not dp < z or not 0.001 <= abs(pit) <= 2000:
            raise ValueError('oracle: depth or lead')
        # p.168: positive lead right-hand (as M3), negative left-hand (as M4).
        hand = 'right' if pit > 0 else 'left'
        if sdac not in (3, 4):
            raise ValueError('oracle: SDAC leaves the spindle stopped or undefined')
        down = abs(pit) * sst
        up = abs(pit) * (sst1 or sst)  # p.168: SST1 = 0 retracts at SST
        if sst <= 0 or sst != self.rpm:
            raise ValueError('oracle: tapping speed is not the programmed spindle speed')
        top, bottom = (x, y, z), (x, y, dp)
        self.taps.append((hand, self.turning, top, bottom, down, up))
        # p.167: afterwards the earlier speed with the direction SDAC.
        self.turning = int(sdac)
        moves = []
        for start, end, feed in ((top, bottom, down), (bottom, top, up)):
            self.count += 1
            moves.append(Motion(self.count, 'observed', setting.tool, Line(start, end), feed, 'tap'))
        return moves
