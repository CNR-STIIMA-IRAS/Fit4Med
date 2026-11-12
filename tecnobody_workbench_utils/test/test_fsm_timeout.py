# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Timeouts of the pending FSM transitions: by time or by number of steps."""

from enum import Enum, auto

import pytest

from tecnobody_workbench_utils import fsm as fsm_module
from tecnobody_workbench_utils.fsm import (
    StateMachine, TransitionStatus, TransitionTimeout,
)


class S(Enum):
    """States of the test machine."""

    IDLE = auto()
    RUNNING = auto()
    ERROR = auto()


class E(Enum):
    """Events of the test machine."""

    START = auto()


class FakeClock:
    """Stand-in for time.monotonic, moved by hand."""

    def __init__(self):
        """Start at t = 1000 s."""
        self.now = 1000.0

    def __call__(self):
        """Return the current fake time."""
        return self.now


@pytest.fixture
def clock(monkeypatch):
    """Replace time.monotonic inside the fsm module."""
    fake = FakeClock()
    monkeypatch.setattr(fsm_module.time, 'monotonic', fake)
    return fake


def _machine(**limits):
    machine = StateMachine(S.IDLE)
    machine.add_transition(
        E.START, S.IDLE, S.RUNNING,
        success_check=lambda: False,
        failure_destination=S.ERROR,
        **limits,
    )
    machine.trigger(E.START)
    return machine


def test_time_timeout_ignores_the_number_of_steps(clock):
    """Many fast steps do not time out a transition before its time."""
    machine = _machine(timeout_sec=2.0)
    for _ in range(10000):
        assert machine.step() == TransitionStatus.PENDING
    clock.now += 1.9
    assert machine.step() == TransitionStatus.PENDING


def test_time_timeout_fires_after_the_time(clock):
    """A slow step rate still times out after timeout_sec."""
    machine = _machine(timeout_sec=2.0)
    clock.now += 2.0
    with pytest.raises(TransitionTimeout, match='after 2 s'):
        machine.step()
    assert machine.state == S.ERROR
    assert machine.pending is None


def test_step_timeout_is_unchanged_without_timeout_sec(clock):
    """Without timeout_sec, max_steps still decides, whatever the time."""
    machine = _machine(max_steps=3)
    clock.now += 1e6
    assert machine.step() == TransitionStatus.PENDING
    assert machine.step() == TransitionStatus.PENDING
    with pytest.raises(TransitionTimeout, match='after 3 steps'):
        machine.step()


def test_timeout_sec_must_be_positive():
    """A zero or negative timeout is a configuration error."""
    with pytest.raises(ValueError):
        _machine(timeout_sec=0.0)
