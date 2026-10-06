# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""The brake released by start_motors must close whenever the drives lose torque.

The Z axis falls by gravity without torque, and the brake is a PLC output that
the drives do not control. ethercat_checker closes it when the drives are no
longer all enabled or report a fault, when the drive states stop arriving, and
when the node exits.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from ethercat_controller_msgs.msg import Cia402DriveStates
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tecnobody_msgs.msg import PlcController

import tecnobody_workbench_utils.eth_checker as ec

PLC_TOPIC = '/PLC_controller/plc_commands'
JOINTS = ['joint_x', 'joint_y', 'joint_z']
BRAKE_OPEN = {'PLC_node/brake_disable': 1}
BRAKE_CLOSED = {'PLC_node/brake_disable': 0}


class FakePublisher:
    """Capture the PLC commands as {interface: value} dicts."""

    def __init__(self):
        """Start with no message."""
        self.messages = []

    def publish(self, msg):
        """Store one published command."""
        self.messages.append(dict(zip(msg.interface_names, msg.values)))


def drive_states(drives_on=True, fault_present=False):
    """Build a drive-state message for the three axes."""
    msg = Cia402DriveStates()
    msg.dof_names = JOINTS
    state = 'STATE_OPERATION_ENABLED' if drives_on else 'STATE_SWITCH_ON_DISABLED'
    msg.drive_states = [state] * 3
    msg.modes_of_operation = ['MODE_CYCLIC_SYNC_POSITION'] * 3
    msg.status_words = [0] * 3
    msg.drives_on = drives_on
    msg.fault_present = fault_present
    return msg


@pytest.fixture
def checker(monkeypatch):
    """Return an ethercat_checker node whose PLC publisher is captured."""
    monkeypatch.setattr(ec, 'BRAKE_COMMAND_SPACING_S', 0.0)
    rclpy.init()
    node = ec.EthercatCheckerNode()
    node.plc_command_publisher = FakePublisher()
    node.drive_states_callback(drive_states())
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_brake_command_is_sent_a_few_times_not_flooded(checker):
    """3 messages, not a 1 s busy loop overriding plc_manager."""
    started = time.monotonic()
    checker._release_brake()
    assert time.monotonic() - started < 0.5
    assert checker.plc_command_publisher.messages == [BRAKE_OPEN] * ec.BRAKE_COMMAND_REPEATS


@pytest.mark.parametrize('states', [
    drive_states(drives_on=False),
    drive_states(drives_on=True, fault_present=True),
], ids=['drives_off', 'fault'])
def test_drives_losing_torque_close_the_released_brake(checker, states):
    """Fault on any axis disables every drive: the brake must close."""
    checker._release_brake()
    checker.drive_states_callback(states)
    assert checker.plc_command_publisher.messages[-1] == BRAKE_CLOSED
    assert not checker._brake_released

    count = len(checker.plc_command_publisher.messages)
    checker.drive_states_callback(states)  # closed once, not at every message
    assert len(checker.plc_command_publisher.messages) == count


def test_brake_not_released_by_the_node_is_left_alone(checker):
    """Drives off in homing or before start_motors: no automatic command."""
    checker.drive_states_callback(drive_states(drives_on=False))
    checker.check_states()
    assert checker.plc_command_publisher.messages == []


def test_missing_drive_states_close_the_released_brake(checker):
    """ros2_control_node crashed or stalled: no states, close the brake."""
    checker._release_brake()
    checker.check_states()
    assert checker.plc_command_publisher.messages[-1] == BRAKE_OPEN  # still fresh

    checker._last_drive_states_monotonic = time.monotonic() - 2 * ec.DRIVE_STATES_TIMEOUT_S
    checker.check_states()
    assert checker.plc_command_publisher.messages[-1] == BRAKE_CLOSED


def test_stop_motors_always_closes_the_brake(checker):
    """stop_motors closes the brake even if this node did not release it."""
    checker.try_turn_off = lambda: True
    checker.drive_states_callback(drive_states(drives_on=False))
    response = checker.stop_motors_callback(Trigger.Request(), Trigger.Response())
    assert response.success
    assert checker.plc_command_publisher.messages[0] == BRAKE_CLOSED


class FakeStateController(Node):
    """try_turn_on service and drive states at 100 Hz, like state_controller."""

    def __init__(self):
        """Advertise the service and start publishing the states."""
        super().__init__('test_fake_state_controller')
        self.drives_on = False
        self.create_service(Trigger, '/state_controller/try_turn_on', self._turn_on)
        self.create_service(Trigger, '/state_controller/try_turn_off', self._turn_off)
        self._states_pub = self.create_publisher(
            Cia402DriveStates, '/state_controller/drive_states', 10)
        self.create_timer(0.01, lambda: self._states_pub.publish(drive_states(self.drives_on)))
        self.plc = []
        self.create_subscription(
            PlcController, PLC_TOPIC,
            lambda msg: self.plc.append(dict(zip(msg.interface_names, msg.values))),
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))

    def _turn_on(self, request, response):
        self.drives_on = True
        response.success = True
        return response

    def _turn_off(self, request, response):
        self.drives_on = False
        response.success = True
        return response


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_node_exit_closes_the_released_brake(sig):
    """Launch shutdown (SIGINT, then SIGTERM) with the motors on: brake closed."""
    env = dict(os.environ)
    src = str(Path(__file__).resolve().parents[1])
    env['PYTHONPATH'] = src + os.pathsep + env.get('PYTHONPATH', '')
    proc = subprocess.Popen(
        [sys.executable, '-c',
         'from tecnobody_workbench_utils.eth_checker import main; main()'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=2)
    fake = FakeStateController()
    executor.add_node(fake)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        client = fake.create_client(Trigger, '/ethercat_checker/start_motors')
        assert client.wait_for_service(timeout_sec=20.0), 'node did not come up'
        time.sleep(1.0)  # PLC subscription matched on the node side too
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert future.done() and future.result().success
        assert BRAKE_OPEN in fake.plc

        proc.send_signal(sig)  # drive states keep arriving: only the exit closes it
        deadline = time.monotonic() + 10.0
        while fake.plc[-1] != BRAKE_CLOSED and time.monotonic() < deadline:
            time.sleep(0.05)
    finally:
        if proc.poll() is None:
            proc.send_signal(sig)
        output, _ = proc.communicate(timeout=15)
        executor.shutdown()
        fake.destroy_node()
        rclpy.shutdown()
    assert fake.plc[-1] == BRAKE_CLOSED, output
    assert 'Brake closed: ethercat_checker node exiting' in output, output
    assert proc.returncode == 0, output
    assert 'Traceback' not in output, output
