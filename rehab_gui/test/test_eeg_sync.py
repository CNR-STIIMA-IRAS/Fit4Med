# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Exercise production EEG methods with ROS, Qt and the clock replaced by fakes.

AST loading avoids importing the GUI and starting ROS on development machines.
The methods themselves are compiled unchanged from the production source.
"""

import ast
from pathlib import Path
import threading
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1]
EEG = 'PLC_node/eeg_sync'


class PlcMessage:
    def __init__(self, names=(), values=()):
        self.interface_names = list(names)
        self.values = list(values)


def load_methods(path, class_name, names, namespace):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == class_name)
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef)
               and node.name in names]
    assert len(methods) == len(names)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
    return {name: namespace[name] for name in names}


def load_publisher():
    path = ROOT.parent / 'plc_manager/plc_manager/plc_commands.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    nodes = [node for node in tree.body if isinstance(node, (ast.Assign, ast.ClassDef))]
    namespace = {'PlcController': PlcMessage, 'threading': threading, 'Any': object}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['PlcCommandPublisher']


class EegSyncTest(unittest.TestCase):
    def setUp(self):
        self.messages = []
        self.publisher = load_publisher()(
            SimpleNamespace(publish=self.record), Mock(),
        )
        self.now = 0.0
        sync_methods = load_methods(
            ROOT / 'rehab_gui/sync_ros_events.py', 'SyncRosManager',
            ['send_eeg_sync'], {'roslibpy': SimpleNamespace(Message=dict)},
        )
        self.gui_messages = []

        def relay(message):
            self.gui_messages.append(message)
            self.publisher.receive_gui_eeg_sync(
                PlcMessage(message['interface_names'], message['values']))

        self.sync = SimpleNamespace(eeg_sync_publisher=SimpleNamespace(publish=relay))
        self.sync.send_eeg_sync = MethodType(sync_methods['send_eeg_sync'], self.sync)
        methods = load_methods(
            ROOT / 'rehab_gui/RosCommunicationManager.py', 'RosCommunicationManager',
            ['eegSync', 'cancelEegSync', '_retryEegSync'],
            {'time': SimpleNamespace(monotonic=lambda: self.now)},
        )
        self.gui = SimpleNamespace(
            ROS=self.sync, rOk=lambda: True, _eeg_sync_timer=Mock(),
            _eeg_sync_retries=0,
        )
        for name, method in methods.items():
            setattr(self.gui, name, MethodType(method, self.gui))

    def record(self, message):
        self.messages.append(dict(zip(message.interface_names, message.values)))

    def retry(self, elapsed):
        self.now = elapsed
        self.gui._retryEegSync()

    def test_startup_does_not_reset_unknown_eeg(self):
        self.publisher.publish_bringup_commands()
        self.assertTrue(self.messages)
        self.assertTrue(all(EEG not in message for message in self.messages))
        self.assertNotIn(EEG, self.publisher.plc_outputs.interface_names)
        self.assertEqual(len(self.messages[-1]), 9)

    def test_snapshots_keep_gui_value_and_other_outputs(self):
        self.publisher.power_force_sensors()
        prior_snapshot = self.publisher.plc_outputs
        self.gui.eegSync(41)
        self.publisher.close_brake()
        self.publisher.clear_sw_estop()
        self.assertEqual(self.messages[-1][EEG], 42)
        self.assertEqual(self.messages[-1]['PLC_node/force_sensors_pwr'], 1)
        self.assertEqual(len(self.messages[-1]), 10)
        self.assertNotIn(EEG, prior_snapshot.interface_names)
        self.assertEqual(self.messages[1], {EEG: 42})  # Forward EEG only.

    def test_three_identical_publications_then_stop(self):
        self.gui.eegSync(4)
        self.retry(0.02)
        self.retry(0.04)
        self.retry(0.06)
        self.assertEqual(self.messages, [{EEG: 5}] * 3)
        self.assertEqual(self.gui._eeg_sync_retries, 0)
        self.gui._eeg_sync_timer.stop.assert_called()

    def test_wraparound_and_zero_are_preserved(self):
        for count, value in [(254, 255), (255, 0), (256, 1)]:
            self.gui.eegSync(count)
            self.publisher.set_automatic_mode()
            self.assertEqual(self.messages[-1][EEG], value)

    def test_new_movement_replaces_pending_retries(self):
        self.gui.eegSync(4)
        self.gui.eegSync(5)
        self.retry(0.02)
        self.retry(0.04)
        self.assertEqual(self.messages, [{EEG: 5}] + [{EEG: 6}] * 3)

    def test_cancel_never_sends_a_reset(self):
        self.gui.eegSync(4)
        self.gui.cancelEegSync()
        self.retry(0.02)
        self.assertEqual(self.messages, [{EEG: 5}])

    def test_disconnected_retry_is_discarded(self):
        self.gui.eegSync(4)
        self.gui.rOk = lambda: False
        self.retry(0.02)
        self.gui.rOk = lambda: True
        self.retry(0.04)
        self.assertEqual(self.messages, [{EEG: 5}])

    def test_disconnected_start_does_not_schedule(self):
        self.gui.rOk = lambda: False
        self.gui.eegSync(4)
        self.assertEqual(self.messages, [])
        self.gui._eeg_sync_timer.start.assert_not_called()

    def test_reconnected_client_does_not_receive_old_retry(self):
        self.gui.eegSync(4)
        self.gui.ROS = Mock()
        self.retry(0.02)
        self.gui.ROS.send_eeg_sync.assert_not_called()
        self.assertEqual(self.messages, [{EEG: 5}])

    def test_late_callback_is_discarded(self):
        self.gui.eegSync(4)
        self.retry(0.101)
        self.assertEqual(self.messages, [{EEG: 5}])

    def test_local_eeg_changes_and_malformed_gui_commands_are_rejected(self):
        with self.assertRaises(ValueError):
            self.publisher._publish_command(EEG, 0)
        for message in [PlcMessage([], []), PlcMessage([EEG], []),
                        PlcMessage(['PLC_node/estop'], [0]),
                        PlcMessage([EEG, 'PLC_node/estop'], [1, 0]),
                        PlcMessage([EEG], [256])]:
            self.publisher.receive_gui_eeg_sync(message)
        self.assertEqual(self.messages, [])

    def test_snapshot_and_gui_publications_are_serialized(self):
        self.gui.eegSync(4)
        entered = threading.Event()
        release = threading.Event()
        updating = threading.Event()

        def blocked_publish(message):
            if len(message.interface_names) > 1:
                entered.set()
                if not release.wait(2):
                    return
            self.record(message)

        self.publisher.command_publisher.publish = blocked_publish
        snapshot = threading.Thread(target=self.publisher.close_brake)

        def receive_new_value():
            updating.set()
            self.publisher.receive_gui_eeg_sync(PlcMessage([EEG], [6]))

        update = threading.Thread(target=receive_new_value)
        snapshot.start()
        try:
            self.assertTrue(entered.wait(2))
            update.start()
            self.assertTrue(updating.wait(2))
        finally:
            release.set()
            snapshot.join(2)
            if update.ident is not None:
                update.join(2)
        self.assertFalse(snapshot.is_alive())
        self.assertFalse(update.is_alive())
        self.assertEqual([message[EEG] for message in self.messages], [5, 5, 6])


if __name__ == '__main__':
    unittest.main()
