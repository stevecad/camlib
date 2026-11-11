"""Literal ISO milling interpreter, independent of formatter/dialect/decoder.

Interprets actual words, including MD10884/10886 integer scaling. Only motion
value types are shared. This is a local manual oracle, not controller firmware.
"""

import math
import re
from camlib.motion import Arc, Helix, Line, Motion


class Oracle:
    def __init__(
        self,
        *,
        tools,
        initial_position,
        coolant_codes,
        tool_change='m6',
        units='mm',
        calculator=False,
        increment='B',
        work_offset=1,
        safety_clearance=None,
    ):
        self.tools = {t.number: t for t in tools}
        self.position = tuple(initial_position)
        self.coolant_codes = {int(v[1:]): k for k, v in coolant_codes.items()}
        self.tool_change = tool_change
        self.expected_units = units
        self.calculator = calculator
        self.increment = increment
        self.work_offset = work_offset
        self.preparation = 0
        self.flags = set()
        self.units = None
        self.selected = self.active = self.length = self.offset = None
        self.pending = False
        self.rpm = 0
        self.coolant = 'off'
        self.ended = False
        self.count = 0
        # G84/G74: GUD _ZSFR[0] (mm), the running M3/M4, the modal cycle and
        # the taps made (hand, M function, start, bottom, feed).
        self.safety_clearance = safety_clearance
        self.turning = None
        self.cycle = False
        self.taps = []

    def read(self, line):
        if self.ended or not line.isascii() or len(line) > 1024:
            raise ValueError('oracle: physical block/end')
        text = line.split(';')[0].strip().upper()
        if self.preparation == 6:
            text = re.sub(r'\([^()]*\)', '', text).strip()
        text = re.sub(r'^N[0-9]{1,8}(?=\s|[A-Z]|$)', '', text).strip()
        if not text:
            return []
        if self.preparation < 6:
            expected = ('G290', 'COMPOF', 'RNDM=0', 'CFTCP', 'G601', 'G291')[self.preparation]
            if text != expected:
                raise ValueError('oracle: language/compressor/center-feed preparation')
            self.preparation += 1
            return []
        if text == 'G291':
            return []
        text = text.replace(' ', '').replace('\t', '')
        if self.cycle and text != 'G80':
            raise ValueError('oracle: block inside a modal tapping cycle')
        tapping = re.fullmatch(r'G98G(84|74)(.+)', text)
        if tapping is not None:
            return self.tap(tapping[1] == '84', tapping[2])
        parts = re.findall(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))', text)
        if ''.join(w + v for w, v in parts) != text or len({w for w, v in parts}) != len(parts):
            raise ValueError('oracle: unsupported words')
        v = {w: float(x) for w, x in parts}
        if any(not math.isfinite(x) or abs(x) > 1e9 for x in v.values()):
            raise ValueError('oracle: numeric range')
        # Manual p11 milling table: integer F is 1 mm/min or .01 inch/min;
        # geometric addresses use IS-B/C quanta, unless calculator notation.
        quantum = (0.001 if self.expected_units == 'mm' else 0.0001) / (10 if self.increment == 'C' else 1)
        if not self.calculator:
            for w, x in parts:
                if '.' not in x:
                    if w in 'XYZIJK':
                        v[w] *= quantum
                    elif w == 'F':
                        v[w] *= 1 if self.expected_units == 'mm' else 0.01
        g, m = v.get('G'), v.get('M')
        if g is not None and m is not None:
            raise ValueError('oracle: mixed transitions')
        if g in (20, 21):
            if set(v) != {'G'} or g != (21 if self.expected_units == 'mm' else 20):
                raise ValueError('oracle: unit change')
            self.units = 1 if g == 21 else 25.4
            return []
        setters = {
            90: 'absolute',
            17: 'plane',
            94: 'feed',
            40: 'radius',
            49: 'length',
            80: 'cycles',
            61: 'stop',
            50: 'scale',
            69: 'rotation',
        }
        if g in setters:
            if set(v) != {'G'}:
                raise ValueError('oracle: modal block')
            self.flags.add(setters[g])
            if g == 49:
                self.length = None
            if g == 80:
                self.cycle = False
            return []
        if g in (50.1, 92.1):
            if set(v) != {'G', 'X', 'Y', 'Z'} or any(v[a] != 0 for a in 'XYZ'):
                raise ValueError('oracle: explicit all-axis reset')
            self.flags.add('mirror' if g == 50.1 else 'frame')
            return []
        if g in range(54, 60):
            if set(v) != {'G'} or g - 53 != self.work_offset:
                raise ValueError('oracle: stored work frame')
            self.offset = int(g - 53)
            return []
        if m in self.coolant_codes:
            if set(v) != {'M'}:
                raise ValueError('oracle: coolant block')
            mode = self.coolant_codes[m]
            if mode != 'off' and self.coolant != 'off':
                raise ValueError('oracle: coolant exclusivity')
            self.coolant = mode
            if mode == 'off':
                self.flags.add('coolant')
            return []
        if m == 5:
            if set(v) != {'M'}:
                raise ValueError('oracle: spindle off')
            self.rpm = 0
            self.turning = None
            self.flags.add('spindle')
            return []
        if m in (3, 4):
            if set(v) != {'M', 'S'} or not self.active or self.pending or self.length is None or v['S'] <= 0:
                raise ValueError('oracle: spindle on')
            self.rpm = v['S']
            self.turning = int(m)
            return []
        if set(v) == {'T'}:
            if v['T'] not in self.tools or self.rpm or self.coolant != 'off':
                raise ValueError('oracle: tool selection')
            self.selected = int(v['T'])
            if self.tool_change == 'direct_t':
                self.activate()
            return []
        if m in (0, 1, 6):
            if set(v) != {'M'} or self.rpm or self.coolant != 'off':
                raise ValueError('oracle: stopped source required')
            if m == 6:
                if self.tool_change != 'm6' or self.selected is None:
                    raise ValueError('oracle: M6')
                self.activate()
            return []
        if g == 43:
            if set(v) != {'G', 'H'} or self.active is None or not self.pending or v['H'] != self.active.length_register:
                raise ValueError('oracle: TC_DPH mapping')
            self.length = int(v['H'])
            self.pending = False
            return []
        if m == 30:
            if set(v) != {'M'} or self.rpm or self.coolant != 'off' or self.length is not None or self.pending:
                raise ValueError('oracle: shutdown')
            self.ended = True
            return []
        needed = {
            'absolute',
            'plane',
            'feed',
            'radius',
            'length',
            'cycles',
            'stop',
            'scale',
            'rotation',
            'mirror',
            'frame',
            'coolant',
            'spindle',
        }
        if not needed <= self.flags or self.units is None or self.offset is None or self.length is None or self.pending:
            raise ValueError('oracle: incomplete motion initialization')
        if g not in (1, 2, 3) or not set(v) <= {'G', 'X', 'Y', 'Z', 'I', 'J', 'F'} or v.get('F', 0) <= 0:
            raise ValueError('oracle: motion syntax')
        start = self.position
        end = tuple(v[a] * self.units if a in v else start[i] for i, a in enumerate('XYZ'))
        if g == 1:
            if 'I' in v or 'J' in v:
                raise ValueError('oracle: line centers')
            curve = Line(start, end)
        else:
            if not {'I', 'J'} <= v.keys():
                raise ValueError('oracle: circle offsets')
            center = (start[0] + v['I'] * self.units, start[1] + v['J'] * self.units, start[2])
            x, y = start[0] - center[0], start[1] - center[1]
            ex, ey = end[0] - center[0], end[1] - center[1]
            if math.hypot(x, y) == 0 or math.hypot(x, y) != math.hypot(ex, ey):
                raise ValueError('oracle: exact radius')
            angle = math.atan2(x * ey - y * ex, x * ex + y * ey)
            sweep = angle % math.tau if g == 3 else -((-angle) % math.tau)
            if sweep == 0:
                sweep = math.tau if g == 3 else -math.tau
            curve = (Arc if start[2] == end[2] else Helix)(start, end, center, sweep)
        self.position = end
        if g == 1 and start == end:
            return []
        self.count += 1
        return [Motion(self.count, 'observed', self.active.tool, curve, v['F'] * self.units)]

    def activate(self):
        self.active = self.tools[self.selected]
        self.position = tuple(self.active.position_after_mm)
        self.pending = True
        self.length = None

    def tap(self, right, words):
        """G98 G84/G74 X Y Z R F as the 02/2016 ISO manual gives it (pp.128,
        143-146): to R plus the safety clearance, tap to Z at F, reverse back
        and return to the initial plane."""
        parts = re.findall(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))', words)
        if ''.join(w + x for w, x in parts) != words or sorted(w for w, x in parts) != ['F', 'R', 'X', 'Y', 'Z']:
            raise ValueError('oracle: tapping cycle words')
        quantum = 0.001 / (10 if self.increment == 'C' else 1)
        v = {}
        for w, x in parts:
            v[w] = float(x)
            # p.11: an undotted geometric word counts input increments.
            if '.' not in x and not self.calculator and w != 'F':
                v[w] *= quantum
        needed = {'absolute', 'plane', 'feed', 'radius', 'length', 'cycles', 'scale', 'rotation', 'mirror', 'frame'}
        if not needed <= self.flags or self.units != 1 or self.offset is None or self.length is None or self.pending:
            raise ValueError('oracle: tapping cycle before initialized (mm, G17, G43, work offset)')
        if self.safety_clearance is None or self.rpm <= 0 or self.turning is None:
            raise ValueError('oracle: tapping without a declared safety clearance or a running spindle')
        x, y, z = self.position
        if (v['X'], v['Y']) != (x, y):
            raise ValueError('oracle: the cycle would position in XY')
        # p.144: approach of R shifted by the safety clearance; G98 returns to
        # the initial plane. The subset has the tool at both already.
        if abs(v['R'] + self.safety_clearance - z) > 1e-9:
            raise ValueError('oracle: the cycle would traverse before tapping')
        if not v['Z'] < v['R'] or v['F'] <= 0:
            raise ValueError('oracle: depth or feed')
        top, bottom = (x, y, z), (x, y, v['Z'])
        self.taps.append(('right' if right else 'left', self.turning, top, bottom, v['F'], v['F'] / self.rpm))
        self.cycle = True
        moves = []
        for start, end in ((top, bottom), (bottom, top)):
            self.count += 1
            moves.append(Motion(self.count, 'observed', self.active.tool, Line(start, end), v['F'], 'tap'))
        return moves
