# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""ROS-independent regression tests for the bus-fault safety path."""
import ast
import json
import socket
import sys
from datetime import datetime
import os
from pathlib import Path
import threading
import time
import types
import unittest
from unittest.mock import Mock
from typing import *
from dataclasses import dataclass
from enum import Enum, auto

ROOT = Path(__file__).resolve().parents[2]


def load(relative, **extra):
    tree = ast.parse((ROOT / relative).read_text(encoding='utf-8'))
    tree.body = [item for item in tree.body if not isinstance(item, (ast.Import, ast.ImportFrom))]
    scope = dict(globals(), **extra)
    scope["__name__"] = "safety_test_module"
    exec(compile(tree, relative, 'exec'), scope)
    return scope


types_scope = load('plc_manager/plc_manager/plc_types.py')
State, Event, Check = (types_scope[name] for name in ('State', 'Event', 'EthercatCheckState'))
Monitor = load('plc_manager/plc_manager/ethercat_monitor.py',
               EthercatCheckState=Check, GetSlaveStates=Mock())['EthercatMonitor']
Manager = load('plc_manager/plc_manager/plc_manager.py', Node=object,
               State=State, Event=Event, EStopState=types_scope['EStopState'],
               EthercatCheckState=Check, Subscription=object, PlcStates=object)['PLCControllerInterface']
colors = types.SimpleNamespace(**{name: '' for name in (
    'OKBLUE', 'ENDC', 'OKGREEN', 'FAIL', 'WARNING', 'BOLD')})
fsm_scope = load('tecnobody_workbench_utils/tecnobody_workbench_utils/fsm.py',
                 Logger=object, bcolors=colors)
build = load('plc_manager/plc_manager/fsm_config.py', State=State, Event=Event,
             StateMachine=fsm_scope['StateMachine'])['build_plc_fsm']

Udp = load('rehab_gui/rehab_gui/UdpCommunicationManager.py',
           QObject=object, QThread=object, pyqtSignal=lambda *a: Mock(),
           pyqtSlot=lambda *a: (lambda f: f))['UdpCommunicationManager']


class EthercatSafetyTests(unittest.TestCase):
    def test_gui_retains_fault_in_display_after_controller_shutdown(self):
        udp = Udp.__new__(Udp)
        udp.number_of_ec_slaves = 0
        payload = {'ethercat': {'latched_fault': 'axis offline', 'slaves': []}}
        udp._cache_plc_status_payload(payload, 'ESTOP', None)
        self.assertIn('axis offline', udp.getUdpDisplayMessage())
        self.assertIn('Supervised restart', udp.getUdpDisplayMessage())

    def test_failed_fresh_check_blocks_restart_and_retains_reason(self):
        c = self.controller(State.ESTOP)
        c._ethercat_fault_reason = 'axis offline'
        c._estop_rearm_delay_elapsed = lambda: True
        c._ros_gui_disconnected = lambda: True
        c.ethercat.startup_status_ok.return_value = False
        c.fsm = build(c)
        c.fsm.state = State.ESTOP
        with self.assertRaises(fsm_scope['GuardFailed']):
            c.fsm.trigger(Event.START)
        self.assertEqual(c._ethercat_fault_reason, 'axis offline')
        c.environment.bringup_env.assert_not_called()

    def monitor(self):
        m = Monitor.__new__(Monitor)
        m._cache_lock = threading.Lock()
        m._last_update_monotonic = time.monotonic()
        m._state_stale_after = 5.0
        m._last_error = None
        m._poll_errors = []
        m._bus_fault_reason = None
        m._expected_slave_names = {'a', 'b'}
        m._slave_names = ['a', 'b']
        m._slave_states = ['OP', 'OP']
        m.check_state = Check.READY
        return m

    @staticmethod
    def drive_states(*status_words):
        return types.SimpleNamespace(
            dof_names=[f'joint_{i}' for i in range(len(status_words))],
            status_words=list(status_words))

    def test_driver_bus_fault_status_word_is_fault(self):
        m = self.monitor()
        m._on_drive_states(self.drive_states(0x0008, 0x0008, 0x0008))
        self.assertIn('bus fault', m.runtime_fault(time.monotonic()))
        self.assertIn('joint_2', m.runtime_fault(time.monotonic()))

    def test_real_drive_fault_is_not_a_bus_fault(self):
        m = self.monitor()
        m._on_drive_states(self.drive_states(0x0618, 0x0637, 0x0637))
        self.assertIsNone(m.runtime_fault(time.monotonic()))

    def test_bus_fault_clears_with_driver_report(self):
        m = self.monitor()
        m._on_drive_states(self.drive_states(0x0008, 0x0008))
        m._on_drive_states(self.drive_states(0x0650, 0x0650))
        self.assertIsNone(m.runtime_fault(time.monotonic()))

    def test_non_op_is_fault(self):
        m = self.monitor()
        m._slave_states[1] = 'SAFEOP'
        m._refresh_check_state_locked()
        self.assertIn('b => SAFEOP', m.runtime_fault(time.monotonic()))

    def test_missing_slave_is_fault_even_if_remaining_are_op(self):
        m = self.monitor()
        m._slave_names, m._slave_states = ['a'], ['OP']
        m._refresh_check_state_locked()
        self.assertIn('b => MISSING', m.runtime_fault(time.monotonic()))

    def test_partial_service_failure_is_fault(self):
        m = self.monitor()
        m._poll_errors = ['master2: call failed']
        m._refresh_check_state_locked()
        self.assertIsNotNone(m.runtime_fault(time.monotonic()))

    def test_stale_telemetry_is_fault(self):
        m = self.monitor()
        m._last_update_monotonic -= 6
        self.assertIn('stale', m.runtime_fault(time.monotonic()))

    def test_initial_wait_is_bounded(self):
        m = self.monitor()
        m._last_update_monotonic = None
        self.assertIsNone(m.runtime_fault(time.monotonic()))
        self.assertIsNotNone(m.runtime_fault(time.monotonic() - 6))

    def test_pending_poll_does_not_clear_last_health_result(self):
        m = self.monitor()
        m.node = Mock()
        m.node.get_parameter.return_value.value = ['master']
        m._fsm_state_getter = lambda: State.RUNNING
        m._pending_requests = {}
        client = Mock()
        client.service_is_ready.return_value = True
        m._slave_state_clients = {'master': client}
        m.poll_slave_states()
        self.assertEqual(m.check_state, Check.READY)
        self.assertIsNone(m.runtime_fault(time.monotonic()))

    def test_missing_configured_service_is_fault(self):
        m = self.monitor()
        m.node = Mock()
        m._fsm_state_getter = lambda: State.RUNNING
        m._pending_requests = {}
        good, missing = Mock(), Mock()
        good.service_is_ready.return_value = True
        missing.service_is_ready.return_value = False
        m._slave_state_clients = {'master0': good, 'master1': missing}
        m.poll_slave_states()
        self.assertIsNotNone(m.runtime_fault(time.monotonic()))
        good.call_async.assert_not_called()

    def controller(self, state):
        c = Manager.__new__(Manager)
        c.environment = Mock()
        c.environment.check_env_running_stopped.return_value = True
        c.environment.check_env_running_recovery_stopped.return_value = True
        c.plc_commands = Mock()
        c.get_logger = Mock(return_value=Mock())
        c.lock = threading.Lock()
        c.bringup_done = True
        c._ethercat_fault_reason = None
        c._ethercat_stop_pending = False
        c.estop_cached = types_scope['EStopState'].OK
        c._ethercat_monitoring_since = None
        c.ethercat = Mock()
        c.ethercat.runtime_fault.return_value = 'axis offline'
        c._publish_gui_status_if_due = Mock()
        c.fsm = build(c)
        c.fsm.state = state
        return c

    def test_watchdog_stops_without_plc_messages_and_holds_latch(self):
        for state, target in ((State.RUNNING, State.ESTOP),
                              (State.RUNNING_RECOVERY, State.ESTOP_RECOVERY)):
            c = self.controller(state)
            calls = Mock()
            calls.attach_mock(c.plc_commands.close_brake, 'brake')
            calls.attach_mock(c.plc_commands.raise_sw_estop, 'estop')
            c._check_ethercat_safety()
            self.assertEqual(c.fsm.state, target)
            self.assertEqual([x[0] for x in calls.mock_calls[:2]], ['brake', 'estop'])
            c.plc_commands.clear_sw_estop.assert_not_called()
            self.assertEqual(c._ethercat_fault_reason, 'axis offline')
            self.assertEqual(c.ethercat.startup_check_state, Check.FAILED)
            self.assertTrue(c.ethercat.startup_check_requested)
            c._publish_gui_status_if_due.assert_called()

    def test_software_stop_held_until_plc_observes_emergency(self):
        c = self.controller(State.RUNNING)
        c._check_ethercat_safety()
        self.assertTrue(c._ethercat_stop_pending)
        c._check_ethercat_safety()
        self.assertTrue(c._ethercat_stop_pending)
        c.estop_cached = types_scope['EStopState'].EMERGENCY
        c._check_ethercat_safety()
        self.assertFalse(c._ethercat_stop_pending)
        self.assertEqual(c._ethercat_fault_reason, 'axis offline')
        self.assertEqual(c.fsm.state, State.ESTOP)

    def test_pending_stop_progresses_without_plc_feedback(self):
        c = self.controller(State.RUNNING)
        c.environment.check_env_running_stopped.return_value = False
        c._check_ethercat_safety()
        self.assertEqual(c.fsm.pending.event, Event.ETHERCAT_FAULT)
        c.environment.check_env_running_stopped.return_value = True
        c._check_ethercat_safety()
        self.assertIsNone(c.fsm.pending)
        self.assertEqual(c.fsm.state, State.ESTOP)
        c.plc_commands.clear_sw_estop.assert_not_called()



if __name__ == '__main__':
    unittest.main()


