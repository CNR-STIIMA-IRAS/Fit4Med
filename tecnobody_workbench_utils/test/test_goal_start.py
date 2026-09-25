# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Movements that never start must be reported to the GUI.

The real manager node runs against a fake FollowJointTrajectory controller in
the same process: absent, rejecting, accepting, or answering too late.
"""

import itertools
import threading
import time
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from tecnobody_msgs.msg import CartesianPoint
from tecnobody_msgs.srv import SetExercise, SetTrajectory

import tecnobody_workbench_utils.gui_trajectory_manager as gtm

# One controller name per test: DDS may still list the previous test's
# (destroyed) action server for a while, which would fake its presence.
_CONTROLLER_IDS = itertools.count()


class FakeController:
    """FollowJointTrajectory server for a controller name, with a configurable answer."""

    def __init__(self, node, controller, accept=True, answer_delay_s=0.0, delays_by_target=None):
        self.accept = accept
        self.answer_delay_s = answer_delay_s
        self.delays_by_target = delays_by_target or {}  # target x -> answer delay
        self.cancel_requested = threading.Event()
        self.cancelled_targets = []
        self.succeeded_targets = []
        self.server = ActionServer(
            node, FollowJointTrajectory, f'/{controller}/follow_joint_trajectory',
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, callback_group=ReentrantCallbackGroup())

    @staticmethod
    def target(goal):
        return round(goal.trajectory.points[-1].positions[0], 3)

    def on_goal(self, goal):
        time.sleep(self.delays_by_target.get(self.target(goal), self.answer_delay_s))
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def on_cancel(self, goal_handle):
        self.cancelled_targets.append(self.target(goal_handle.request))
        self.cancel_requested.set()
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            time.sleep(0.05)
        self.succeeded_targets.append(self.target(goal_handle.request))
        goal_handle.succeed()
        return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.SUCCESSFUL)


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setattr(gtm, 'GOAL_ACCEPTANCE_TIMEOUT_S', 1.0)
    rclpy.init()
    controller = f'test_jtc_{next(_CONTROLLER_IDS)}'
    manager = gtm.FollowJointTrajectoryActionManager(controller_name=controller)
    manager.trajectory_finished_notifier = Mock()
    manager.exercise_finished_notifier = Mock()
    manager.exercise_suspended_notifier = Mock()
    manager.movement_stopped_notifier = Mock()
    controller_node = Node('test_fake_controller')
    # As on the robot: the manager alone on a single-threaded executor (see
    # main()); the fake controller on its own, like a separate process.
    manager_executor = SingleThreadedExecutor()
    manager_executor.add_node(manager)
    controller_executor = MultiThreadedExecutor(num_threads=4)
    controller_executor.add_node(controller_node)
    for executor in (manager_executor, controller_executor):
        threading.Thread(target=executor.spin, daemon=True).start()
    yield manager, controller_node, controller
    manager_executor.shutdown()
    controller_executor.shutdown()
    manager.destroy_node()
    controller_node.destroy_node()
    rclpy.shutdown()


def trajectory_request(target_x=0.05):
    request = SetTrajectory.Request()
    request.cartesian_positions = [CartesianPoint(point=[0.0, 0.0, 0.0], time_from_start=0.0),
                                   CartesianPoint(point=[target_x, 0.0, 0.0], time_from_start=2.0)]
    request.override = 100.0
    return request


def exercise_request():
    request = SetExercise.Request()
    request.cartesian_positions = [CartesianPoint(point=[0.0, 0.0, 0.0], time_from_start=0.0),
                                   CartesianPoint(point=[0.05, 0.0, 0.0], time_from_start=2.0)]
    request.repetition_durations = [2.0]
    request.repetition_ovrs = [100.0]
    return request


def wait_until(condition, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return False


def wait_for_server(manager):
    assert wait_until(manager.follow_joint_trajectory_action_client.server_is_ready)


def test_no_controller_refuses_every_movement(ros):
    manager, _, _ = ros
    assert manager.set_trajectory(trajectory_request(), SetTrajectory.Response()).success is False
    assert manager.set_rehab_exercise(exercise_request(), SetExercise.Response()).success is False
    assert manager.set_eeg_exercise(exercise_request(), SetExercise.Response()).success is False
    assert manager.set_go_to_start_trajectory(trajectory_request(), SetTrajectory.Response()).success is False


def test_rejected_trajectory_is_reported(ros):
    manager, controller_node, name = ros
    FakeController(controller_node, name, accept=False)
    wait_for_server(manager)
    assert manager.set_trajectory(trajectory_request(), SetTrajectory.Response()).success is True
    notify = manager.trajectory_finished_notifier.notify
    assert wait_until(lambda: notify.called)
    notify.assert_called_once_with(False, GoalStatus.STATUS_UNKNOWN,
                                   FollowJointTrajectory.Result.INVALID_GOAL,
                                   'goal rejected by the controller', 'ptp')


def test_rejected_exercise_is_a_suspension(ros):
    manager, controller_node, name = ros
    FakeController(controller_node, name, accept=False)
    wait_for_server(manager)
    assert manager.set_rehab_exercise(exercise_request(), SetExercise.Response()).success is True
    notify = manager.exercise_suspended_notifier.notify
    assert wait_until(lambda: notify.called)
    args = notify.call_args[0]
    assert args[0] is False and args[2] == FollowJointTrajectory.Result.INVALID_GOAL
    assert args[4] == 'exercise' and 'rejected' in args[3]


def test_accepted_trajectory_reports_success(ros):
    manager, controller_node, name = ros
    FakeController(controller_node, name, accept=True)
    wait_for_server(manager)
    manager.set_trajectory(trajectory_request(), SetTrajectory.Response())
    notify = manager.trajectory_finished_notifier.notify
    assert wait_until(lambda: notify.called, timeout_s=8.0)
    assert notify.call_args[0][:3] == (True, GoalStatus.STATUS_SUCCEEDED, 0)


def test_unanswered_goal_is_reported_and_a_late_acceptance_cancelled(ros):
    manager, controller_node, name = ros
    controller = FakeController(controller_node, name, accept=True, answer_delay_s=2.0)
    wait_for_server(manager)
    manager.set_trajectory(trajectory_request(), SetTrajectory.Response())
    notify = manager.trajectory_finished_notifier.notify
    assert wait_until(lambda: notify.called, timeout_s=3.0)  # after the 1 s acceptance timeout
    args = notify.call_args[0]
    assert args[0] is False and args[2] == gtm.NO_RESULT_ERROR_CODE and 'did not answer' in args[3]
    # The acceptance arrives at 2 s: the goal must be cancelled, not executed.
    assert controller.cancel_requested.wait(timeout=5.0)


def test_controller_appearing_shortly_after_the_request_is_not_refused(ros):
    """Right after a controller switch its action server may take a moment to appear."""
    manager, controller_node, name = ros
    appear = threading.Timer(0.4, lambda: FakeController(controller_node, name, accept=True))
    appear.start()
    started = time.monotonic()
    assert manager.set_trajectory(trajectory_request(), SetTrajectory.Response()).success is True
    assert time.monotonic() - started >= 0.4  # really waited for it
    appear.join()


def test_late_acceptance_of_a_superseded_goal_is_cancelled(ros):
    """Timeout, then a retry: the first goal's late acceptance must not run it."""
    manager, controller_node, name = ros
    controller = FakeController(controller_node, name, accept=True, delays_by_target={0.05: 2.0})
    wait_for_server(manager)
    manager.set_trajectory(trajectory_request(0.05), SetTrajectory.Response())
    notify = manager.trajectory_finished_notifier.notify
    assert wait_until(lambda: notify.called, timeout_s=3.0)  # first goal: not answered in 1 s
    manager.set_trajectory(trajectory_request(0.10), SetTrajectory.Response())  # the retry
    assert wait_until(lambda: 0.05 in controller.cancelled_targets, timeout_s=5.0)
    assert wait_until(lambda: 0.10 in controller.succeeded_targets, timeout_s=8.0)
    assert 0.05 not in controller.succeeded_targets
    assert wait_until(lambda: notify.call_count == 2)  # timeout of the first, result of the retry
    assert notify.call_args[0][0] is True


def test_trajectory_stopped_by_the_gui_reports_only_movement_stopped(ros):
    manager, controller_node, name = ros
    controller = FakeController(controller_node, name, accept=True)
    wait_for_server(manager)
    manager.set_trajectory(trajectory_request(), SetTrajectory.Response())
    assert wait_until(lambda: manager._goal_handle is not None and manager._goal_handle.accepted)
    from std_srvs.srv import Trigger
    assert manager.stop(Trigger.Request(), Trigger.Response()).success is True
    assert controller.cancel_requested.wait(timeout=3.0)
    assert wait_until(lambda: manager.movement_stopped_notifier.send.called)
    time.sleep(1.0)  # the cancelled goal's result has arrived by now
    manager.trajectory_finished_notifier.notify.assert_not_called()


def test_node_main_answers_and_stops_cleanly(tmp_path):
    """The real main(): single-threaded spin() serves a request, Ctrl-C ends it."""
    import signal
    import subprocess
    import sys

    controller = f'test_main_jtc_{next(_CONTROLLER_IDS)}'
    node = subprocess.Popen(
        [sys.executable, '-c',
         'import sys; from tecnobody_workbench_utils.gui_trajectory_manager import main; '
         f'sys.argv = ["fct_manager_node", "{controller}"]; main()'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    try:
        client_node = Node('test_main_client')
        client = client_node.create_client(SetTrajectory, '/tecnobody_workbench_utils/set_trajectory')
        assert client.wait_for_service(timeout_sec=20.0), 'node did not come up'
        future = client.call_async(trajectory_request())
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=10.0)
        # No controller running: refused after the short wait, not left hanging.
        assert future.done() and future.result().success is False
        client_node.destroy_node()
    finally:
        rclpy.shutdown()
        node.send_signal(signal.SIGINT)
        output, _ = node.communicate(timeout=15)
    assert node.returncode == 0, output
    assert 'Traceback' not in output, output
