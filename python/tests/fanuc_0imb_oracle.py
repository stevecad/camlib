"""Literal 0i-MB edition-02 oracle; imports only motion data, no post/decoder.

Physical spindle Z and actual H values independently model compensation blocks.
"""

import math
import re
from camlib.motion import Line, Motion


class Oracle:
    def __init__(
        self,
        *,
        tools,
        initial_position,
        output_units='mm',
        program_number=1000,
        work_offset=1,
        calculator_decimal=False,
        optional_stop_enabled=False,
        m_codes=None,
        tool_change_gauge_z=None,
    ):
        self.tools = {t.number: t for t in tools}
        # None: M06 leaves each tool at its measured table pose. A number: it
        # leaves the spindle gauge line at this work Z, X and Y unmoved.
        self.tool_change_gauge_z = tool_change_gauge_z
        self.position = tuple(initial_position)
        self.units = 1 if output_units == 'mm' else 25.4
        self.precision = 3 if output_units == 'mm' else 4
        self.calculator_decimal = calculator_decimal
        self.program_number = program_number
        self.work_offset = work_offset
        self.optional_stop_enabled = optional_stop_enabled
        m_codes = m_codes or dict(
            tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8'
        )
        self.m_codes = {int(code[1:]): effect for effect, code in m_codes.items()}
        self.opened = self.numbered = self.end_seen = self.ended = False
        self.flags = set()
        self.active = self.selected = self.length = None
        self.offset = None
        self.rpm = 0
        self.coolant = 'off'
        self.count = 0
        self.transitions = []

    def read(self, line):
        if self.ended:
            raise ValueError('oracle: output after closing percent')
        body = line.strip()
        if body == '%':
            if not self.opened:
                self.opened = True
            elif self.end_seen:
                self.ended = True
            else:
                raise ValueError('oracle: early percent')
            return []
        if not self.opened or self.end_seen:
            raise ValueError('oracle: framing')
        if body.startswith('O'):
            if body != 'O' + str(self.program_number) or self.numbered:
                raise ValueError('oracle: O number')
            self.numbered = True
            return []
        if not self.numbered:
            raise ValueError('oracle: missing O')
        compact = ''.join(body.split())
        parts = re.findall(r'([A-Z])([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))', compact)
        if ''.join(a + b for a, b in parts) != compact:
            raise ValueError('oracle: syntax')
        gs = [float(v) for a, v in parts if a == 'G']
        words = {a: float(v) for a, v in parts if a != 'G'}
        if len(words) != len(parts) - len(gs):
            raise ValueError('oracle: duplicate address')
        for a, text in parts:
            if a in 'XYZF' and '.' not in text and not self.calculator_decimal:
                words[a] *= 10 ** (-self.precision)
        if any(not math.isfinite(v) for v in words.values()):
            raise ValueError('oracle: finite values')
        if gs in ([1, 43], [1, 49]):
            if self.active is None or self.rpm or self.coolant != 'off':
                raise ValueError('oracle: compensation setup')
            if words.get('F', 0) <= 0 or self.offset is None:
                raise ValueError('oracle: compensation feed/frame')
            before_spindle = self.position[2] + self.active.gauge_length_mm
            if gs[-1] == 43:
                if (
                    set(words) != {'H', 'Z', 'F'}
                    or words['H'] != self.active.length_register
                    or self.length is not None
                ):
                    raise ValueError('oracle: H activation')
                offset = self.active.gauge_length_mm
            else:
                if set(words) != {'Z', 'F'} or self.length is None:
                    raise ValueError('oracle: G49')
                offset = 0
            after_spindle = words['Z'] * self.units + offset
            if abs(before_spindle - after_spindle) > 1e-9:
                raise ValueError('oracle: unmodeled compensation movement')
            self.length = int(words['H']) if gs[-1] == 43 else None
            self.transitions.append((gs[-1], before_spindle, after_spindle))
            return []
        if len(gs) > 1:
            raise ValueError('oracle: combined G codes')
        g = gs[0] if gs else None
        if g in (20, 21):
            if words or g != (21 if self.units == 1 else 20):
                raise ValueError('oracle: unit change')
            self.flags.add('units')
            return []
        if g in (90, 17, 94, 40, 49, 80, 61):
            if words or g == 49 and self.length is not None:
                raise ValueError('oracle: state/implicit length motion')
            self.flags.add(g)
            return []
        if g == 52:
            if set(words) != {'X', 'Y', 'Z'} or any(words.values()):
                raise ValueError('oracle: local shift')
            self.flags.add('frame')
            return []
        if g in (54, 55, 56, 57, 58, 59):
            if words or g - 53 != self.work_offset:
                raise ValueError('oracle: wrong WCS')
            self.offset = int(g - 53)
            return []
        if set(words) == {'T'} and g is None:
            if words['T'] not in self.tools:
                raise ValueError('oracle: unknown T')
            self.selected = int(words['T'])
            return []
        if 'M' in words and g is None:
            code = words['M']
            effect = self.m_codes.get(code)
            if effect in ('spindle_cw', 'spindle_ccw'):
                if set(words) != {'M', 'S'} or self.length is None or words['S'] <= 0 or words['S'] != int(words['S']):
                    raise ValueError('oracle: spindle state')
                self.rpm = words['S']
                return []
            if set(words) != {'M'}:
                raise ValueError('oracle: M words')
            if effect == 'spindle_off':
                self.rpm = 0
                self.flags.add('spindle')
            elif effect == 'coolant_off':
                self.coolant = 'off'
                self.flags.add('coolant')
            elif effect == 'flood':
                self.coolant = 'flood'
            elif effect == 'tool_change':
                if self.rpm or self.coolant != 'off' or self.selected is None or self.length is not None:
                    raise ValueError('oracle: ATC state')
                self.active = self.tools[self.selected]
                if self.tool_change_gauge_z is None:
                    self.position = tuple(self.active.position_after_mm)
                else:
                    spindle_z = self.tool_change_gauge_z
                    self.position = (*self.position[:2], spindle_z - self.active.gauge_length_mm)
            elif code in (0, 1):
                if self.rpm or self.coolant != 'off' or code == 1 and not self.optional_stop_enabled:
                    raise ValueError('oracle: stop state')
            elif code == 30:
                if self.rpm or self.coolant != 'off' or self.length is not None:
                    raise ValueError('oracle: end state')
                self.end_seen = True
            else:
                raise ValueError('oracle: unknown M code')
            return []
        required = {'units', 'frame', 'spindle', 'coolant', 90, 17, 94, 40, 49, 80, 61}
        if g != 1 or not required <= self.flags or self.active is None or self.length is None or self.offset is None:
            raise ValueError('oracle: motion state')
        if set(words) != {'X', 'Y', 'Z', 'F'} or words['F'] <= 0:
            raise ValueError('oracle: G1 words')
        start = self.position
        self.position = tuple(words[a] * self.units for a in 'XYZ')
        if self.position == start:
            return []
        self.count += 1
        return [Motion(self.count, 'observed', self.active.tool, Line(start, self.position), words['F'] * self.units)]
