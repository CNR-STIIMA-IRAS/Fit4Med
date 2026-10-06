# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""The Z recovery must always end with the end-stroke sensor active again.

estop_bypass=1 disables the Z end-stroke safety sensor in the safety PLC.
Every way out of the recovery (target reached, jog timeout, safety reclosure,
SIGINT/SIGTERM from plc_manager or launch) must publish estop_bypass=0.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tecnobody_msgs.msg import PlcController

import tecnobody_workbench_utils.auto_z_recovery_node as azr

PLC_TOPIC = '/PLC_controller/plc_commands'
RESTORED = {'PLC_node/estop_bypass': 0, 'PLC_node/z_recovery': 1}


class FakePublisher:
    """Capture the PLC commands as {interface: value} dicts."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        """Store one published command."""
        self.messages.append(dict(zip(msg.interface_names, msg.values)))


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(azr, 'RESTORE_SPACING_S', 0.0)
    rclpy.init()
    recovery = azr.AutoZRecoveryNode()
    recovery._plc_pub = FakePublisher()
    yield recovery
    recovery.destroy_node()
    rclpy.shutdown()


def _ago(node, seconds):
    return node.get_clock().now() - Duration(seconds=seconds)


def test_bypass_and_z_recovery_are_sent_in_one_message(node):
    """PLC_controller keeps only the last message per cycle: one message."""
    node._state = azr._S.INIT_RECOVERY
    node._loop()
    assert node._plc_pub.messages[0] == {
        'PLC_node/z_recovery': 0, 'PLC_node/estop_bypass': 1}


def test_restore_is_repeated_and_sent_once(node):
    """The restore is sent RESTORE_REPEATS times, and only the first call sends it."""
    node._restore_plc_safety()
    node._restore_plc_safety()
    assert node._plc_pub.messages == [RESTORED] * azr.RESTORE_REPEATS


@pytest.mark.parametrize('reason', ['timeout', 'reclosure', 'target'])
def test_every_end_of_the_jog_restores_the_end_stroke_sensor(node, reason):
    """Jog timeout and safety reclosure too, not only the target distance."""
    node._state = azr._S.JOGGING
    node._z_start = 0.0
    node._z_current = 0.0
    node._jog_started_at = node.get_clock().now()
    if reason == 'timeout':
        node._jog_started_at = _ago(node, azr.JOG_TIMEOUT_S + 1.0)
    elif reason == 'target':
        node._z_current = azr.RECOVERY_DISTANCE_M
    else:
        node._jog_started_at = _ago(node, azr.JOG_EMERGENCY_ARM_S + 1.0)
        node._consume_jog_state_poll = lambda: type(
            'States', (), {'fault_present': True, 'drives_on': False, 'drive_states': []})()

    node._loop()  # JOGGING -> STOPPING_JOG
    assert node._state == azr._S.STOPPING_JOG
    node._stop_start = _ago(node, azr.STOP_TIMEOUT_S + 1.0)  # no stop acknowledgements
    node._loop()  # STOPPING_JOG -> DONE

    assert node._state == azr._S.DONE
    assert node._plc_pub.messages[-1] == RESTORED
    node.destroy_timer(node._shutdown_timer)


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_signal_restores_the_end_stroke_sensor(sig):
    """plc_manager stops the recovery env with SIGINT; launch escalates to SIGTERM."""
    env = dict(os.environ)
    src = str(Path(__file__).resolve().parents[1])
    env['PYTHONPATH'] = src + os.pathsep + env.get('PYTHONPATH', '')
    proc = subprocess.Popen(
        [sys.executable, '-c',
         'from tecnobody_workbench_utils.auto_z_recovery_node import main; main()'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    rclpy.init()
    received = []
    try:
        listener = Node('test_plc_listener')
        listener.create_subscription(
            PlcController, PLC_TOPIC,
            lambda msg: received.append(dict(zip(msg.interface_names, msg.values))),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        deadline = time.monotonic() + 20.0
        while listener.count_publishers(PLC_TOPIC) == 0 and time.monotonic() < deadline:
            rclpy.spin_once(listener, timeout_sec=0.1)
        assert listener.count_publishers(PLC_TOPIC) > 0, 'node did not come up'
        time.sleep(1.0)  # let the subscription match on the node side too
        proc.send_signal(sig)
        deadline = time.monotonic() + 10.0
        while RESTORED not in received and time.monotonic() < deadline:
            rclpy.spin_once(listener, timeout_sec=0.1)
        listener.destroy_node()
    finally:
        rclpy.shutdown()
        if proc.poll() is None:
            proc.send_signal(sig)
        output, _ = proc.communicate(timeout=15)
    assert RESTORED in received, output
    assert proc.returncode == 0, output
    assert 'Traceback' not in output, output
