# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Torque statistics in % of the rated torque, as P1-57 in delta*.yaml."""

import math
import os

import pytest

from tecnobody_workbench_utils.torque_stats import MovementStats, percent_of_rated


def test_effort_per_mille_becomes_percent_of_rated():
    assert percent_of_rated(1000) == 100.0
    assert percent_of_rated(-150) == -15.0


def _movement(efforts_permille, dt=0.02, threshold_pct=100.0):
    m = MovementStats('repetition 1', 0.0)
    for i, e in enumerate(efforts_permille):
        m.add_sample(i * dt, ['joint_x'], [e], threshold_pct)
    return m


def test_max_mean_p99_in_percent():
    m = _movement([100] * 98 + [-500, 200])   # 10% normally, one -50% peak
    a = m.axes['joint_x']
    assert a.max_value == 50.0 and a.max_signed == -50.0
    assert a.max_t == pytest.approx(98 * 0.02)
    assert a.mean() == pytest.approx((98 * 10 + 50 + 20) / 100)
    assert a.percentile(0.99) == 20.0          # the single 50% peak is above p99


def test_time_over_threshold_and_longest_stretch():
    # 120% for 6 samples (100 ms at 50 Hz), back to 10%, then 120% for 3 samples
    efforts = [100] * 5 + [1200] * 6 + [100] * 5 + [1200] * 3 + [100] * 5
    a = _movement(efforts).axes['joint_x']
    assert a.longest_over_s == pytest.approx(5 * 0.02)
    assert a.over_s == pytest.approx((6 + 3) * 0.02)


def test_nan_efforts_are_skipped_and_merge_keeps_the_largest():
    m1 = _movement([100, math.nan, 300])
    m2 = MovementStats('repetition 2', 10.0)
    m2.add_sample(10.0, ['joint_x'], [-400], 100.0)
    m1.merge(m2)
    a = m1.axes['joint_x']
    assert len(a.values) == 3
    assert a.max_signed == -40.0
    assert m1.end_t == 10.0


def test_report_shows_percent_and_threshold():
    text = _movement([100, 250]).report(100.0)
    assert 'max  25.0%' in text and 'over 100%' in text


def test_time_of_the_maximum_is_when_it_is_first_reached():
    a = _movement([100, 300, 300, 300]).axes['joint_x']
    assert a.max_t == pytest.approx(0.02)


def test_crash_threshold_is_read_from_the_drive_config(tmp_path):
    from tecnobody_workbench_utils.torque_stats import crash_threshold_pct
    config = tmp_path / 'delta.yaml'
    config.write_text('sdo:\n'
                      '  - {index: 0x6098, sub_index: 0, type: int8, value: 35}\n'
                      '  - {index: 0x2139, sub_index: 0, type: int16, value: 120}\n')
    assert crash_threshold_pct(str(config)) == 120.0
    assert crash_threshold_pct(str(tmp_path / 'missing.yaml')) is None
    real = os.path.join(os.path.dirname(__file__), '..', '..',
                        'tecnobody_workbench', 'config', 'delta400x.yaml')
    assert crash_threshold_pct(real) == 100.0
