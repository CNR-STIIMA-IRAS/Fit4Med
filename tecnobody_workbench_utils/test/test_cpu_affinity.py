# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Tests of the housekeeping CPU set read from sysfs."""

import pytest

from tecnobody_workbench_utils.cpu_affinity import housekeeping_cpus, parse_cpu_list


def _fake_sysfs(tmp_path, online, isolated):
    (tmp_path / 'online').write_text(online + '\n')
    (tmp_path / 'isolated').write_text(isolated + '\n')
    return str(tmp_path)


@pytest.mark.parametrize('text, cpus', [
    ('', set()),
    ('3', {3}),
    ('2-3,6-7', {2, 3, 6, 7}),
    (' 0-1, 4 ,5 ', {0, 1, 4, 5}),
])
def test_parse_cpu_list(text, cpus):
    """Kernel CPU lists are parsed into sets."""
    assert parse_cpu_list(text) == cpus


@pytest.mark.parametrize('text', ['3-2', 'a', '-1', '1-'])
def test_parse_cpu_list_rejects_bad_input(text):
    """Malformed lists raise ValueError."""
    with pytest.raises(ValueError):
        parse_cpu_list(text)


def test_housekeeping_excludes_isolated_cpus(tmp_path):
    """Isolated CPUs are not housekeeping CPUs."""
    # Robot PC today: isolcpus=2-3,6-7.
    assert housekeeping_cpus(_fake_sysfs(tmp_path, '0-7', '2-3,6-7')) == {0, 1, 4, 5}


def test_housekeeping_follows_kernel_command_line(tmp_path):
    """The set follows the isolcpus of the running kernel."""
    # Option B: only the two RT CPUs isolated.
    assert housekeeping_cpus(_fake_sysfs(tmp_path, '0-7', '2-3')) == {0, 1, 4, 5, 6, 7}


def test_housekeeping_without_isolation(tmp_path):
    """Without isolcpus every online CPU is a housekeeping CPU."""
    # The kernel writes an empty line when nothing is isolated.
    assert housekeeping_cpus(_fake_sysfs(tmp_path, '0-3', '')) == {0, 1, 2, 3}


def test_housekeeping_raises_when_every_cpu_is_isolated(tmp_path):
    """No housekeeping CPU left is an error."""
    with pytest.raises(RuntimeError):
        housekeeping_cpus(_fake_sysfs(tmp_path, '0-1', '0-1'))
