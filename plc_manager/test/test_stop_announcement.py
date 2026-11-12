# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""STOP/FAIL is announced to the GUI before rosbridge is stopped."""

import json
from types import SimpleNamespace
from unittest.mock import MagicMock

import plc_manager.environment_manager as em
from plc_manager.gui_status import GuiStatusPublisher
from plc_manager.plc_types import Event, State

from tecnobody_workbench_utils.fsm import StateMachine


def _payload(fsm, announced_event=None):
    outputs = SimpleNamespace(interface_names=[], values=[])
    raw = GuiStatusPublisher(MagicMock(), MagicMock()).fsm_status_payload(
        fsm, {}, outputs, [], [], announced_event=announced_event,
    )
    return json.loads(raw)


def _fsm():
    fsm = StateMachine(State.RUNNING)
    fsm.add_transition(
        Event.STOP, State.RUNNING, State.ESTOP, success_check=lambda: False,
    )
    return fsm


def test_announced_stop_is_sent_as_pending():
    """The GUI sees pending STOP before the transition actions run."""
    pending = _payload(_fsm(), announced_event=Event.STOP)['pending']
    assert pending == {
        'event': 'STOP', 'source': 'RUNNING', 'target': 'ESTOP', 'steps': 0,
    }


def test_no_announcement_without_a_transition():
    """An event with no transition from the current state is not announced."""
    assert _payload(_fsm(), announced_event=Event.FAIL)['pending'] is None


def test_real_pending_wins_over_the_announcement():
    """Once triggered, the real pending transition is reported."""
    fsm = _fsm()
    fsm.trigger(Event.STOP)
    fsm.pending.steps = 7
    assert _payload(fsm, announced_event=Event.STOP)['pending']['steps'] == 7


def test_no_announcement_by_default():
    """Without announced_event the payload is unchanged."""
    assert _payload(_fsm())['pending'] is None


def test_kill_env_stops_rosbridge_after_the_platform(monkeypatch):
    """Platform first, then a pause, then rosbridge."""
    calls = []
    monkeypatch.setattr(
        em, 'stop_launch_environment', lambda pattern, *a: calls.append(pattern))
    monkeypatch.setattr(em, 'find_matching_pids', lambda pattern: [1234])
    monkeypatch.setattr(em.time, 'sleep', lambda s: calls.append(f'sleep {s}'))
    env = em.EnvironmentManager.__new__(em.EnvironmentManager)
    env.platform_controller_readiness_monitor = MagicMock()
    env.kill_env()
    assert calls == [
        'run_platform_control.launch.py',
        f'sleep {em.ROSBRIDGE_STOP_DELAY_SEC}',
        em.ROSBRIDGE_LAUNCH_PATTERN,
    ]


def test_no_pause_when_rosbridge_is_not_running(monkeypatch):
    """Without a rosbridge launcher there is nothing to wait for."""
    sleeps = []
    monkeypatch.setattr(em, 'stop_launch_environment', lambda *a: None)
    monkeypatch.setattr(em, 'find_matching_pids', lambda pattern: [])
    monkeypatch.setattr(em.time, 'sleep', sleeps.append)
    env = em.EnvironmentManager.__new__(em.EnvironmentManager)
    env.platform_controller_readiness_monitor = MagicMock()
    env.kill_env()
    assert sleeps == []
