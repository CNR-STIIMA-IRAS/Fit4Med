# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""GuiNotifier: GUI result notifications with real rclpy nodes, in one process."""

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tecnobody_msgs.srv import TrajectoryResult

from tecnobody_workbench_utils.gui_trajectory_manager import GuiNotifier

SERVICE = '/test_gui/exercise_suspended'


@pytest.fixture
def ros():
    rclpy.init()
    sender = Node('test_notifier_sender')
    gui = Node('test_notifier_gui')
    executor = SingleThreadedExecutor()
    executor.add_node(sender)
    executor.add_node(gui)
    logs = []
    yield sender, gui, executor, logs
    executor.shutdown()
    sender.destroy_node()
    gui.destroy_node()
    rclpy.shutdown()


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def capture_logs(notifier, logs):
    """Record the notifier's log messages (it uses the node only for logging after init)."""
    class RecordingLogger:
        def warning(self, msg):
            logs.append(('warning', msg))

        def error(self, msg):
            logs.append(('error', msg))

        def debug(self, msg):
            pass
    notifier._node = type('NodeLogs', (), {'get_logger': staticmethod(RecordingLogger)})()


def gui_service(gui, received, accepted=True):
    def handle(request, response):
        received.append(request)
        response.accepted = accepted
        return response
    return gui.create_service(TrajectoryResult, SERVICE, handle)


def test_waits_for_a_late_gui_and_delivers_once(ros):
    sender, gui, executor, logs = ros
    notifier = GuiNotifier(sender, SERVICE, wait_timeout_s=5.0)
    capture_logs(notifier, logs)
    notifier.notify(False, 6, -4, 'Repetition 3: path tolerance violated', 'exercise')
    spin_for(executor, 1.0)  # the GUI is not there yet: nothing lost, nothing sent
    received = []
    gui_service(gui, received)
    spin_for(executor, 1.5)
    assert len(received) == 1
    request = received[0]
    assert (request.success, request.action_status, request.error_code) == (False, 6, -4)
    assert request.message == 'Repetition 3: path tolerance violated'
    assert request.movement_kind == 'exercise'
    spin_for(executor, 0.5)
    assert len(received) == 1  # never resent
    assert logs == []


def test_keeps_the_order_of_queued_notifications(ros):
    sender, gui, executor, logs = ros
    notifier = GuiNotifier(sender, SERVICE, wait_timeout_s=5.0)
    for i in range(3):
        notifier.notify(True, 4, 0, f'Exercise repetition {i} completed', 'exercise')
    received = []
    gui_service(gui, received)
    spin_for(executor, 1.5)
    assert [r.message for r in received] == [f'Exercise repetition {i} completed' for i in range(3)]


def test_refusal_by_the_gui_is_logged(ros):
    sender, gui, executor, logs = ros
    received = []
    gui_service(gui, received, accepted=False)
    notifier = GuiNotifier(sender, SERVICE)
    capture_logs(notifier, logs)
    spin_for(executor, 0.5)
    notifier.notify(True, 4, 0, 'Exercise repetition 0 completed', 'exercise')
    spin_for(executor, 1.0)
    assert len(received) == 1
    assert any(level == 'warning' and 'did NOT accept' in msg for level, msg in logs)


def test_notification_is_dropped_with_an_error_when_the_gui_never_comes(ros):
    sender, gui, executor, logs = ros
    notifier = GuiNotifier(sender, SERVICE, wait_timeout_s=0.5)
    capture_logs(notifier, logs)
    notifier.notify(False, 6, -4, 'lost one', 'exercise')
    spin_for(executor, 1.5)
    assert any(level == 'error' and 'notification LOST' in msg and 'lost one' in msg
               for level, msg in logs)
    received = []
    gui_service(gui, received)
    spin_for(executor, 1.0)
    assert received == []  # dropped, not delivered late


# --- What on_exercise_goal_done tells the GUI -------------------------------

def _goal_future(status, error_code=0, error_string=''):
    from types import SimpleNamespace
    result = SimpleNamespace(status=status, result=SimpleNamespace(
        error_code=error_code, error_string=error_string))
    return SimpleNamespace(result=lambda: result)


def _manager_stub():
    from types import SimpleNamespace
    from unittest.mock import Mock
    return SimpleNamespace(_goal_handle=object(), exercise_cnt=2, get_logger=Mock,
                           _cancel_exercise_status_timer=Mock(), _advance_exercise=Mock(),
                           exercise_suspended_notifier=Mock())


def test_aborted_repetition_is_reported_with_the_controller_error():
    from action_msgs.msg import GoalStatus

    from tecnobody_workbench_utils.gui_trajectory_manager import FollowJointTrajectoryActionManager
    stub = _manager_stub()
    FollowJointTrajectoryActionManager.on_exercise_goal_done(
        stub, _goal_future(GoalStatus.STATUS_ABORTED, -4, 'path tolerance violated on joint_y'))
    stub._cancel_exercise_status_timer.assert_called_once()  # no more time-based progress
    assert stub._goal_handle is None
    stub.exercise_suspended_notifier.notify.assert_called_once_with(
        False, GoalStatus.STATUS_ABORTED, -4, 'Repetition 2: path tolerance violated on joint_y', 'exercise')
    stub._advance_exercise.assert_not_called()


def test_repetition_without_a_result_is_a_suspension_too():
    from types import SimpleNamespace

    from tecnobody_workbench_utils.gui_trajectory_manager import (
        FollowJointTrajectoryActionManager, NO_RESULT_ERROR_CODE)
    stub = _manager_stub()

    def broken():
        raise RuntimeError('action server gone')
    FollowJointTrajectoryActionManager.on_exercise_goal_done(stub, SimpleNamespace(result=broken))
    args = stub.exercise_suspended_notifier.notify.call_args[0]
    assert args[0] is False and args[2] == NO_RESULT_ERROR_CODE and 'action server gone' in args[3]


def test_succeeded_and_cancelled_repetitions_are_not_suspensions():
    from action_msgs.msg import GoalStatus

    from tecnobody_workbench_utils.gui_trajectory_manager import FollowJointTrajectoryActionManager
    for status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED):
        stub = _manager_stub()
        FollowJointTrajectoryActionManager.on_exercise_goal_done(stub, _goal_future(status))
        stub.exercise_suspended_notifier.notify.assert_not_called()


# --- Trigger services (/rehab_gui/movement_stopped) --------------------------

def test_trigger_notification_waits_for_the_gui_and_checks_success(ros):
    from std_srvs.srv import Trigger
    sender, gui, executor, logs = ros
    notifier = GuiNotifier(sender, '/test_gui/movement_stopped', Trigger, wait_timeout_s=5.0)
    capture_logs(notifier, logs)
    notifier.send(Trigger.Request())
    spin_for(executor, 0.8)  # GUI not there yet: kept, not dropped
    received = []

    def handle(request, response):
        received.append(request)
        response.success = False  # e.g. the GUI could not use it
        return response
    gui.create_service(Trigger, '/test_gui/movement_stopped', handle)
    spin_for(executor, 1.5)
    assert len(received) == 1
    assert any(level == 'warning' and 'did NOT accept' in msg and 'Trigger' in msg
               for level, msg in logs)


# --- Cost at rest: the node's main loop runs ~one callback every 50 ms ------

def test_timer_runs_only_while_something_is_pending(ros):
    sender, gui, executor, logs = ros
    notifier = GuiNotifier(sender, SERVICE, wait_timeout_s=5.0)
    assert notifier._timer.is_canceled()  # idle: no polling
    notifier.notify(True, 4, 0, 'waiting for the GUI', 'exercise')
    assert not notifier._timer.is_canceled()  # GUI absent: polls for it
    received = []
    gui_service(gui, received)
    spin_for(executor, 1.5)
    assert len(received) == 1
    assert notifier._timer.is_canceled()  # delivered and answered: idle again
    notifier.notify(True, 4, 0, 'GUI already there', 'exercise')
    spin_for(executor, 0.5)
    assert len(received) == 2
    assert notifier._timer.is_canceled()
