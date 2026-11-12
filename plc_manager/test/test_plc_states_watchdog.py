# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Watchdog on /PLC_controller/plc_states (PLCControllerInterface)."""

import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import plc_manager.plc_manager as pm
from plc_manager.plc_types import Event, State

import pytest

PERIOD = pm.PLC_STATES_WATCHDOG_PERIOD_SEC
TIMEOUT = 0.2


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
    """Replace time.monotonic inside plc_manager."""
    fake = FakeClock()
    monkeypatch.setattr(pm.time, 'monotonic', fake)
    return fake


def _node(state=State.RUNNING):
    return SimpleNamespace(
        bringup_done=True,
        _plc_states_timeout_sec=TIMEOUT,
        _last_plc_states_monotonic=None,
        _last_watchdog_monotonic=None,
        _plc_states_lost=False,
        lock=threading.Lock(),
        plc_commands=MagicMock(),
        fsm=SimpleNamespace(pending=None, state=state, trigger=MagicMock()),
        get_logger=MagicMock(),
        _announce_stop_to_gui=MagicMock(),
    )


def _run(node, clock, seconds, message_every=None):
    """Run the watchdog every PERIOD for `seconds`, with optional messages."""
    elapsed = 0.0
    while elapsed < seconds - 1e-9:
        clock.now += PERIOD
        elapsed += PERIOD
        if message_every is not None:
            node._last_plc_states_monotonic = clock.now
        pm.PLCControllerInterface._plc_states_watchdog(node)


def test_heartbeat_keeps_the_watchdog_quiet(clock):
    """Messages every 20-50 ms never trip it."""
    node = _node()
    _run(node, clock, 5.0, message_every=PERIOD)
    node.plc_commands.raise_sw_estop.assert_not_called()
    node.fsm.trigger.assert_not_called()


def test_silence_raises_sw_estop_and_fails_once(clock):
    """Silence longer than the timeout: SW ESTOP and FAIL, only once."""
    node = _node()
    _run(node, clock, 0.1, message_every=PERIOD)
    _run(node, clock, 2.0)
    node.plc_commands.raise_sw_estop.assert_called_once()
    node.plc_commands.set_automatic_mode.assert_called_once()
    node.fsm.trigger.assert_called_once()
    assert node.fsm.trigger.call_args.args[0] == Event.FAIL
    assert node._plc_states_lost


def test_trips_within_the_timeout_plus_one_period(clock):
    """Detection happens between TIMEOUT and TIMEOUT + PERIOD."""
    node = _node()
    _run(node, clock, 0.1, message_every=PERIOD)
    _run(node, clock, TIMEOUT)
    node.plc_commands.raise_sw_estop.assert_not_called()
    _run(node, clock, PERIOD)
    node.plc_commands.raise_sw_estop.assert_called_once()


def test_no_fail_event_when_not_running(clock):
    """In IDLE the SW ESTOP is raised, but no FAIL transition exists."""
    node = _node(state=State.IDLE)
    _run(node, clock, 0.1, message_every=PERIOD)
    _run(node, clock, 1.0)
    node.plc_commands.raise_sw_estop.assert_called_once()
    node.fsm.trigger.assert_not_called()


def test_a_late_watchdog_run_does_not_trip(clock):
    """A stalled node (timer late) is not taken for a silent PLC."""
    node = _node()
    _run(node, clock, 0.1, message_every=PERIOD)
    clock.now += 1.0  # the whole node stalled for 1 s
    pm.PLCControllerInterface._plc_states_watchdog(node)
    node.plc_commands.raise_sw_estop.assert_not_called()
    node._last_plc_states_monotonic = clock.now  # queued messages arrive
    _run(node, clock, 1.0, message_every=PERIOD)
    node.plc_commands.raise_sw_estop.assert_not_called()


def test_rearms_after_messages_come_back(clock):
    """After a loss and a recovery, a new loss trips again."""
    node = _node(state=State.IDLE)
    _run(node, clock, 0.1, message_every=PERIOD)
    _run(node, clock, 1.0)
    _run(node, clock, 0.5, message_every=PERIOD)
    assert not node._plc_states_lost
    _run(node, clock, 1.0)
    assert node.plc_commands.raise_sw_estop.call_count == 2


def test_disabled_with_zero_timeout(clock):
    """plc_states_timeout_sec = 0 (the default) switches the watchdog off."""
    node = _node()
    node._plc_states_timeout_sec = 0.0
    _run(node, clock, 0.1, message_every=PERIOD)
    _run(node, clock, 5.0)
    node.plc_commands.raise_sw_estop.assert_not_called()
    node.fsm.trigger.assert_not_called()


def test_quiet_before_bringup(clock):
    """Before the bring-up nothing is supervised."""
    node = _node()
    node.bringup_done = False
    _run(node, clock, 5.0)
    node.plc_commands.raise_sw_estop.assert_not_called()
