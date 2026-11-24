# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Torque statistics per movement, for tuning the drive crash protection.

The effort of /joint_states is the drive torque actual value (0x6077), in per
mille of the rated torque. Here everything is in **% of the rated torque**
(per mille / 10), the unit of the crash protection threshold P1-57 in
delta*.yaml, so the numbers compare directly with it.

Per axis: max |torque| (and when), mean |torque|, p99 (exceeded in 1 % of the
samples), and the time above the threshold (total and longest stretch, to
compare with P1-58 in ms). Pure Python: no ROS, so it is unit-tested.
"""

import datetime
import math


def percent_of_rated(effort_permille: float) -> float:
    """0x6077 (per mille of the rated torque) -> % of the rated torque."""
    return effort_permille / 10.0


CRASH_THRESHOLD_INDEX = 0x2139  # Delta P1-57, motor crash protection [% rated]


def crash_threshold_pct(drive_config_path: str) -> float | None:
    """P1-57 written at startup by a drive config (delta*.yaml), or None."""
    import yaml
    try:
        with open(drive_config_path, encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)
        for entry in config.get('sdo') or []:
            if int(entry.get('index', -1)) == CRASH_THRESHOLD_INDEX:
                return float(entry['value'])
    except (OSError, ValueError, TypeError, AttributeError, KeyError, yaml.YAMLError):
        return None
    return None


def _clock_text(t: float) -> str:
    return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S')


class AxisStats:
    """|torque| samples of one axis in one movement."""

    def __init__(self):
        self.values: list[float] = []
        self.max_value = 0.0
        self.max_signed = 0.0
        self.max_t = 0.0
        self.over_s = 0.0
        self.longest_over_s = 0.0
        self._run_start: float | None = None
        self._last_t: float | None = None

    def add(self, t: float, torque_pct: float, threshold_pct: float) -> None:
        magnitude = abs(torque_pct)
        if self._last_t is not None and self._run_start is not None:
            self.over_s += max(0.0, t - self._last_t)
        if magnitude > threshold_pct:
            if self._run_start is None:
                self._run_start = t
            self.longest_over_s = max(self.longest_over_s, t - self._run_start)
        else:
            self._run_start = None
        self._last_t = t
        # Strictly greater: the time of the maximum is when it is first reached.
        if not self.values or magnitude > self.max_value:
            self.max_value, self.max_signed, self.max_t = magnitude, torque_pct, t
        self.values.append(magnitude)

    def merge(self, other: 'AxisStats') -> None:
        if other.values and (not self.values or other.max_value > self.max_value):
            self.max_value, self.max_signed, self.max_t = (
                other.max_value, other.max_signed, other.max_t)
        self.values.extend(other.values)
        self.over_s += other.over_s
        self.longest_over_s = max(self.longest_over_s, other.longest_over_s)

    def mean(self) -> float:
        return sum(self.values) / len(self.values) if self.values else math.nan

    def percentile(self, q: float) -> float:
        if not self.values:
            return math.nan
        ordered = sorted(self.values)
        return ordered[min(len(ordered) - 1, int(math.ceil(q * len(ordered))) - 1)]


class MovementStats:
    """One trajectory (a repetition, a return to start) or a group of them."""

    def __init__(self, label: str, start_t: float):
        self.label = label
        self.start_t = start_t
        self.end_t = start_t
        self.result = ''
        self.axes: dict[str, AxisStats] = {}

    def add_sample(self, t, names, efforts_permille, threshold_pct) -> None:
        for name, effort in zip(names, efforts_permille):
            if effort is None or math.isnan(effort):
                continue
            self.axes.setdefault(name, AxisStats()).add(
                t, percent_of_rated(effort), threshold_pct)
        self.end_t = max(self.end_t, t)

    def merge(self, other: 'MovementStats') -> None:
        for name, axis in other.axes.items():
            self.axes.setdefault(name, AxisStats()).merge(axis)
        self.start_t = min(self.start_t, other.start_t)
        self.end_t = max(self.end_t, other.end_t)

    def samples(self) -> int:
        return max((len(a.values) for a in self.axes.values()), default=0)

    def report(self, threshold_pct: float) -> str:
        header = (f'{self.label}: {_clock_text(self.start_t)}-{_clock_text(self.end_t)} '
                  f'({self.end_t - self.start_t:.1f} s)')
        if self.result:
            header += f', {self.result}'
        lines = [header]
        if not self.axes:
            lines.append('  (no torque samples)')
        for name in sorted(self.axes):
            a = self.axes[name]
            lines.append(
                f'  {name}: max {a.max_value:5.1f}% ({a.max_signed:+.1f}% at '
                f'+{max(0.0, a.max_t - self.start_t):.1f} s)  mean {a.mean():5.1f}%  '
                f'p99 {a.percentile(0.99):5.1f}%  over {threshold_pct:g}%: '
                f'{a.over_s * 1000:.0f} ms (longest {a.longest_over_s * 1000:.0f} ms)')
        return '\n'.join(lines)


FILE_HEADER = (
    '# Torque per movement, in % of the rated torque (drive 0x6077 / 10), the unit of\n'
    '# the crash protection threshold P1-57 in delta*.yaml. max = largest |torque|\n'
    '# (signed value, seconds from the start of the movement); mean = mean |torque|;\n'
    '# p99 = exceeded in 1% of the samples; over N%: time above the threshold, total\n'
    '# and longest stretch (compare the longest stretch with P1-58, in ms).\n'
    '# "repetition" = one trajectory of an exercise, "return to start" = go-to-start,\n'
    '# "EXERCISE" = consecutive repetitions, "START TOTAL" = all the movements.\n'
)
