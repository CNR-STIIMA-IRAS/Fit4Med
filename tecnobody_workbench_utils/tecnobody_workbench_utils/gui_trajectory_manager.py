# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Rehabilitation exercise trajectory manager for GUI-controlled robot motion execution.

This module implements a bridge between the rehabilitation GUI and the ROS 2 control 
system, managing the execution of multi-repetition sets of a given trajectory with dynamic
speed scaling. The node handles trajectory planning, repetition management, and
real-time exercise progress tracking.

Core Responsibilities:
    1. Trajectory Planning: Receives waypoints from GUI, interpolates them
       using cubic splines, and resamples at fixed 10 Hz frequency for controller compatibility
    2. Exercise Execution: Manages multi-repetition exercise loops with individual
       speed override factors for each repetition (e.g., 80%, 100%, 120% of nominal speed)
    3. Progress Tracking: Monitors exercise progression at 10 Hz and reports completion
       percentage back to GUI for visual feedback and speed adjustment
    4. Pause/Resume: Detects speed scaling factor drops (< 0.01) as pause signals,
       accounting for pause duration in progress calculations
    5. Action Synchronization: Shields GUI from asynchronous action client complexity
       by managing all FollowJointTrajectory action handshakes internally

Service Interface:
    - /tecnobody_workbench_utils/set_trajectory: Single-repetition trajectory execution
    - /tecnobody_workbench_utils/set_exercise: Multi-repetition exercise with per-rep scaling
    - /tecnobody_workbench_utils/stop_movement: Emergency stop and cleanup

Callback Interface (to GUI):
    - /rehab_gui/trajectory_finished: Signals end of single-rep trajectory
    - /rehab_gui/exercise_finished: Signals end of current repetition (loops N times)
    - /rehab_gui/exercise_progress: Periodic status (10 Hz) with progress % for display

Architecture:
    Two execution modes:
    - TRAJECTORY mode (N=1): Single-shot trajectory from set_trajectory service
    - EXERCISE mode (N>1): Multi-rep loop from set_exercise with per-rep speed factors
    
    Each repetition follows identical execution flow but with different speed override
    applied via setSpeedOverride() before goal submission.

Performance:
    - Trajectory resampling: ~100 ms (cubic spline interpolation)
    - Progress reporting: 10 Hz (100 ms timer interval)
    - Pause detection: 2 ms (real-time speed factor subscription)
    - Joint trajectory controller interface: ~10 Hz (100 ms trajectory points)

Attributes:
    controller_name (str): Name of joint_trajectory_controller to target
    goal_fjt (FollowJointTrajectory.Goal): Current trajectory goal being executed
    exercise_in_execution (bool): Flag indicating exercise is active
    exercise_cnt (int): Current repetition number (0-indexed) in multi-rep exercise
    _total_time_s (float): Duration of complete exercise in seconds
    _dt (float): Trajectory resampling period (default: 0.1 s = 10 Hz)
    _is_paused (bool): Current pause state detected from speed scaling factor
    _paused_duration (float): Accumulated pause time [s]
    repetition_ovrs (List[float]): Speed override factors for each repetition [%]
    exercise_status_timer (Timer): 10 Hz progress monitoring timer
    follow_joint_trajectory_action_client (ActionClient): Connection to controller
"""

import sys
import threading
import time
import random
from typing import Callable, List, Optional
from copy import deepcopy

# Mathematics libraries
import numpy as np
from numpy.typing import NDArray
import math
from scipy.interpolate import CubicSpline

# ROS 2 core
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.service import Service
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# ROS 2 control and message types
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import SpeedScalingFactor, JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tecnobody_msgs.srv import SetExercise, SetTrajectory, MovementProgress, TrajectoryResult
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Development tooling
from rich.traceback import install
install(show_locals=True)


DEFAULT_EEG_DELAY_MS = 4000

# control_msgs/FollowJointTrajectory result code used when the action gives none.
NO_RESULT_ERROR_CODE = 999
# Time the controller has to accept or reject a goal before it counts as rejected.
GOAL_ACCEPTANCE_TIMEOUT_S = 5.0
# Time a movement request waits for the controller's action server: right after
# a controller switch its discovery can take a few hundred ms.
CONTROLLER_WAIT_S = 1.0


class GuiNotifier:
    """Deliver notifications to one GUI service (TrajectoryResult by default).

    The GUI advertises its services through rosbridge, so they can be missing
    for a while (GUI starting, websocket reconnecting). notify() never blocks
    the calling callback: a notification waits up to wait_timeout_s for the
    service, then it is sent exactly once. Resending a request whose reply got
    lost could make the GUI count a repetition twice. The GUI's `accepted`
    answer is checked (`success` for std_srvs/Trigger services): refusals and
    missing replies are logged.
    """

    def __init__(self, node: Node, service_name: str, srv_type=TrajectoryResult,
                 wait_timeout_s: float = 10.0, reply_timeout_s: float = 5.0,
                 poll_period_s: float = 0.2) -> None:
        self._node = node
        self._service_name = service_name
        self._wait_timeout_s = wait_timeout_s
        self._reply_timeout_s = reply_timeout_s
        self._client = node.create_client(srv_type, service_name)
        self._lock = threading.RLock()  # _on_reply may run inside _flush's call_async
        self._waiting = []   # [(queued_at, request)], oldest first
        self._in_flight = []  # [(sent_at, future, request, warned)]
        # Runs only while something waits for the GUI or for its reply: the
        # node's main loop executes about one callback every 50 ms, an always
        # running timer per notifier would take a large share of it.
        self._timer = node.create_timer(poll_period_s, self._flush)
        self._timer.cancel()

    def notify(self, success: bool, action_status: int, error_code: int,
               message: str, movement_kind: str) -> None:
        request = TrajectoryResult.Request()
        request.success = bool(success)
        request.action_status = int(action_status)
        request.error_code = int(error_code)
        request.message = str(message)
        request.movement_kind = str(movement_kind)
        self.send(request)

    def send(self, request) -> None:
        """Queue any request of this service's type (e.g. Trigger.Request())."""
        with self._lock:
            self._waiting.append((time.monotonic(), request))
        self._flush()

    def _flush(self) -> None:
        now = time.monotonic()
        with self._lock:
            self._check_replies(now)
            if self._waiting and not self._client.service_is_ready():
                expired = [r for t, r in self._waiting if now - t > self._wait_timeout_s]
                self._waiting = [(t, r) for t, r in self._waiting if now - t <= self._wait_timeout_s]
                for request in expired:
                    self._node.get_logger().error(
                        f'{self._service_name} not available for {self._wait_timeout_s:.0f} s: '
                        f'notification LOST ({self._describe(request)})')
            elif self._waiting:
                to_send, self._waiting = [r for _, r in self._waiting], []
                for request in to_send:
                    future = self._client.call_async(request)
                    self._in_flight.append([now, future, request, False])
                    future.add_done_callback(lambda f, r=request: self._on_reply(f, r))
            self._update_timer()

    def _update_timer(self) -> None:
        """Poll only while something is pending (called with the lock held)."""
        pending = bool(self._waiting) or any(not entry[1].done() for entry in self._in_flight)
        if pending and self._timer.is_canceled():
            self._timer.reset()
        elif not pending and not self._timer.is_canceled():
            self._timer.cancel()

    def _check_replies(self, now: float) -> None:
        still_open = []
        for entry in self._in_flight:
            sent_at, future, request, warned = entry
            if future.done():
                continue
            if not warned and now - sent_at > self._reply_timeout_s:
                entry[3] = True
                self._node.get_logger().warning(
                    f'{self._service_name}: no reply after {self._reply_timeout_s:.0f} s '
                    f'({self._describe(request)})')
            still_open.append(entry)
        self._in_flight = still_open

    def _on_reply(self, future, request) -> None:
        with self._lock:
            self._in_flight = [entry for entry in self._in_flight if entry[1] is not future]
            self._update_timer()
        try:
            response = future.result()
        except Exception as exc:
            self._node.get_logger().error(f'{self._service_name} call failed: {exc!r} ({self._describe(request)})')
            return
        accepted = getattr(response, 'accepted', getattr(response, 'success', False))
        if response is None or not accepted:
            self._node.get_logger().warning(f'{self._service_name}: GUI did NOT accept ({self._describe(request)})')
        else:
            self._node.get_logger().debug(f'{self._service_name}: accepted ({self._describe(request)})')

    @staticmethod
    def _describe(request) -> str:
        if not hasattr(request, 'movement_kind'):
            return type(request).__name__
        return (f'kind={request.movement_kind} success={request.success} '
                f'status={request.action_status} code={request.error_code} msg="{request.message}"')


def goal_result_fields(future) -> tuple:
    """(action_status, error_code, error_string) of a FollowJointTrajectory result future."""
    try:
        result = future.result()
        return int(result.status), int(result.result.error_code), str(result.result.error_string)
    except Exception as exc:
        return int(GoalStatus.STATUS_UNKNOWN), NO_RESULT_ERROR_CODE, f'no result: {exc!r}'


class FollowJointTrajectoryActionManager(Node):
    """ROS 2 node managing multi-repetition rehabilitation exercises with dynamic speed control.
    
    This node implements a stateful exercise manager that bridges the rehabilitation GUI
    and the ros2_control FollowJointTrajectory action interface. It provides transparent
    handling of:
    
    1. Trajectory Interpolation: Cubic spline interpolation with fixed resampling frequency
    2. Multi-Repetition Loops: Automatic looping of exercises with per-repetition speed factors
    3. Pause Detection: Real-time pause/resume via SpeedScalingFactor subscription (< 0.01 = paused)
    4. Progress Tracking: 10 Hz progress reporting with elapsed time percentage
    5. Action Management: Asynchronous action client handling, goal state monitoring
    
    The node operates in two modes:
    
    TRAJECTORY mode (set_trajectory service):
        - Single execution of provided trajectory
        - No repetition loop
        - Signals completion via /rehab_gui/trajectory_finished callback
        - Use case: ad-hoc single movements
    
    EXERCISE mode (set_exercise service):
        - Multi-repetition execution with different speed override per repetition
        - Receives N speed factors and automatically loops N times
        - Each repetition uses different speed scaling applied via setSpeedOverride()
        - Signals repetition completion via /rehab_gui/exercise_finished
        - Signals overall exercise completion when all repetitions exhausted
        - Use case: rehabilitation workout (e.g., 5 reps @ 80%, 5 reps @ 100%, 5 reps @ 120%)
    
    Progress Monitoring:
        - 10 Hz timer (100 ms) checks exercise_status via check_exercise_status()
        - Calculates elapsed time percentage: (elapsed_time / total_time) * 100%
        - Detects and accounts for pauses (speed_factor < 0.01)
        - Calls /rehab_gui/exercise_progress service with progress_percentage
        - GUI uses progress % for real-time progress bar display
    
    Pause/Resume Mechanism:
        - GUI lowers speed_scaling_factor below 0.01 to signal pause
        - desampled_joint_trajectory_controller_scaling_callback() detects this
        - During pause: check_exercise_status() reports pause warning but doesn't increment time
        - Resume: speed_factor goes back above 0.01, paused_duration is subtracted from elapsed time
    
    Attributes:
        controller_name (str): Target joint_trajectory_controller name
        goal_fjt (FollowJointTrajectory.Goal): Current trajectory goal
        exercise_in_execution (bool): Exercise active flag
        exercise_cnt (int): Current repetition index (0-based)
        repetition_ovrs (List[float]): Speed override factor for each repetition [%]
        _total_time_s (float): Duration of complete trajectory [s]
        _dt (float): Resampling frequency period (0.1 s = 10 Hz)
        _is_paused (bool): Current pause state from speed_scaling_factor
        _paused_duration (float): Accumulated pause time [s]
        _joint_names (List[str]): Controlled joints ['joint_x', 'joint_y', 'joint_z']
        speed_scaling_factor (float): Current speed multiplier (1.0 = 100%)
        follow_joint_trajectory_action_client (ActionClient): Action interface to controller
    """

    def __init__(self, controller_name: str):
        """Initialize the trajectory manager node with controller interface.
        
        Creates ROS 2 node infrastructure for managing rehabilitation exercises:
        - Sets up service servers for trajectory and exercise requests from GUI
        - Creates subscription to speed scaling factors for pause detection
        - Initializes action client for FollowJointTrajectory controller
        
        Args:
            controller_name (str): Name of ros2_control joint_trajectory_controller
                (e.g., "joint_trajectory_controller"). Will be used to form action
                namespace: "/{controller_name}/follow_joint_trajectory"
        
        Side Effects:
            - Creates node "fct_manager_node" in ROS 2 graph
            - Initializes empty trajectory goal
            - Registers CPU affinity to core 6 (deterministic timing)
        """
        super().__init__("fct_manager_node")
        
        self.controller_name = controller_name
        self.declare_parameter('eeg_delay_ms', DEFAULT_EEG_DELAY_MS)
        self.eeg_delay_ms = self._get_eeg_delay_ms()
        self.eeg_delay_s = self.eeg_delay_ms / 1000.0
        self.number_of_repetition : int = 0
        self.cancel_from_gui : bool = False
        self.additional_speed_override : float = 1.0
        self.goal_fjt : List[FollowJointTrajectory.Goal] = None #type: ignore
        self.exercise_in_execution : bool = False
        self.exercise_cnt : int = 0
        self._total_time_s : List[float] = list()
        self.speed_scaling_factor : List[float] = list()
        self._last_time_from_start_percentage : float = 0.0
        self._progress_report_period_s: float = 0.1
        self._last_reported_progress_pct: Optional[int] = None
        
        self._dt : float = 0.1
        self._init_time_s : float = 0
        self._is_paused : bool = False
        self._pause_start_time : float = 0.0
        self._paused_duration = 0.0
        self._joint_names = ['joint_x', 'joint_y', 'joint_z']
        self.absolute_positions = None #type: ignore

        # ========== Status Tracking ==========
        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self._goal_acceptance_pending : bool = False
        self._goal_handle = None
        self._init_time = self.get_clock().now().nanoseconds
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_group = MutuallyExclusiveCallbackGroup()
        self.exercise_progress_client : Client = None #type: ignore
        self.exercise_status_timer : Timer = None #type: ignore
        self._trajectory_kind : str = 'ptp'  # movement_kind reported for single trajectories
        self._acceptance_expired : bool = False  # the controller did not answer the last goal in time
        self._acceptance_timer : Timer = None #type: ignore
        self._stopped_by_gui : bool = False  # the current goal was cancelled by a GUI stop

        ############################# Trajectory Services ##########################
        self.set_trajectory_server : Service = None #type: ignore
        self.stop_movement_server : Service = None #type: ignore
        self._init_sevices()
        
        self.movement_status_publisher : Publisher = None #type: ignore
        self.desampled_joint_trajectory_controller_scaling_publisher : Subscription = None #type: ignore
        self._init_publishers_subscribers()

        # ========== Action Client ==========
        self.follow_joint_trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self.controller_name}/follow_joint_trajectory"
        )
        # Dedicated client for go_to_start_controller, which shares command interfaces
        # with joint_trajectory_controller and is only active when the GUI switches to it.
        self.go_to_start_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/go_to_start_controller/follow_joint_trajectory"
        )
        self.clear(0)

    def _get_eeg_delay_ms(self) -> int:
        raw_delay_ms = self.get_parameter('eeg_delay_ms').value
        try:
            eeg_delay_ms = int(raw_delay_ms) #type: ignore[arg-type]
        except (TypeError, ValueError):
            self.get_logger().warning(
                f"Invalid eeg_delay_ms parameter {raw_delay_ms!r}; using {DEFAULT_EEG_DELAY_MS} ms."
            )
            return DEFAULT_EEG_DELAY_MS

        if eeg_delay_ms < 0:
            self.get_logger().warning(
                f"Invalid negative eeg_delay_ms parameter {eeg_delay_ms}; using {DEFAULT_EEG_DELAY_MS} ms."
            )
            return DEFAULT_EEG_DELAY_MS

        return eeg_delay_ms

    def _init_sevices(self) -> None:
        """Initialize service servers for trajectory and exercise commands.
        
        Creates three service endpoints:
        1. set_exercise: Multi-repetition exercise with per-rep speed factors
        2. set_trajectory: Single-shot trajectory execution
        3. stop_movement: Emergency stop and cleanup
        
        These services are called asynchronously by the rehabilitation GUI to
        command robot motion. The node processes requests and returns completion
        status to the caller.
        """
        self.set_trajectory_server = self.create_service(
            SetExercise,
            "/tecnobody_workbench_utils/set_rehab_exercise",
            self.set_rehab_exercise
        )
        self.set_trajectory_server = self.create_service(
            SetExercise,
            "/tecnobody_workbench_utils/set_eeg_exercise",
            self.set_eeg_exercise
        )
        self.set_trajectory_server = self.create_service(
            SetTrajectory,
            "/tecnobody_workbench_utils/set_trajectory",
            self.set_trajectory
        )
        self.set_go_to_start_trajectory_server = self.create_service(
            SetTrajectory,
            "/tecnobody_workbench_utils/set_go_to_start_trajectory",
            self.set_go_to_start_trajectory
        )
        self.stop_movement_server = self.create_service(
            Trigger,
            "/tecnobody_workbench_utils/stop_movement",
            self.stop
        )
        self.exercise_progress_client = self.create_client(
            MovementProgress, 
            "/rehab_gui/exercise_progress"
        )
        # Results for the GUI (tecnobody_msgs/TrajectoryResult, answered with `accepted`).
        self.trajectory_finished_notifier = GuiNotifier(self, "/rehab_gui/trajectory_finished")
        self.exercise_finished_notifier = GuiNotifier(self, "/rehab_gui/exercise_finished")
        self.exercise_suspended_notifier = GuiNotifier(self, "/rehab_gui/exercise_suspended")
        # Answer to a stop request: nothing to report but the fact (Trigger).
        self.movement_stopped_notifier = GuiNotifier(self, "/rehab_gui/movement_stopped", Trigger)


    def _init_publishers_subscribers(self) -> None:
        """Initialize publishers and subscribers for movement status and speed control.
        
        Creates communication channels:
        1. Publisher: movement status updates (currently unused, reserved for future)
        2. Subscriber: speed scaling factor for pause/resume detection at max rate
        
        The speed scaling subscriber uses TRANSIENT_LOCAL durability to handle
        late subscribers and ensure no pause commands are lost.
        """
        self.movement_status_publisher: Publisher = self.create_publisher(
            String,
            '/tecnobody_workbench_utils/movement_status',
            10
        )

        qos_profile1 = QoSProfile(depth=10)
        qos_profile1.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.desampled_joint_trajectory_controller_scaling_publisher : Subscription = self.create_subscription(
            SpeedScalingFactor,
            f'/tecnobody_workbench_utils/speed_scaling_input', 
            self.desampled_joint_trajectory_controller_scaling_callback, 
            qos_profile1
        )

        # qos_profile2 = QoSProfile(depth=10)
        # qos_profile2.durability = DurabilityPolicy.VOLATILE
        # self.joint_states_subscriber : Subscription = self.create_subscription(
        #     JointTrajectoryControllerState,
        #     f'/{self.controller_name}/controller_state',
        #     self.joint_states_cbk,
        #     qos_profile2,
        #     callback_group=self.subscriber_group
        # )


    def desampled_joint_trajectory_controller_scaling_callback(self, msg: SpeedScalingFactor) -> None:
        """Detect pause/resume transitions from speed scaling factor.
        
        This callback monitors the speed scaling factor in real-time to implement
        pause/resume functionality. The GUI signals pause by lowering the factor
        below 0.01 (1%), which is below any practical exercise speed.
        
        Pause Transition (factor < 0.01):
            - Records pause start time for duration accounting
            - Sets _is_paused flag to prevent progress time increment
            - Controller continues outputting last trajectory point (freeze)
        
        Resume Transition (factor >= 0.01):
            - Calculates pause duration: current_time - pause_start_time
            - This duration is subtracted from elapsed time in check_exercise_status()
            - Clears _is_paused flag to resume time accumulation
        
        Args:
            msg (SpeedScalingFactor): Speed scaling message with factor in [0, ∞]
                Typical values: 0.8 (80%), 1.0 (100%), 1.2 (120%)
                Pause signal: factor < 0.01
        """
        is_paused : bool = self.goal_fjt is not None and msg.factor < 0.01
        
        # ========== Detect pause transition ==========
        if is_paused and not self._is_paused:
            self._pause_start_time = time.time()
            self._is_paused = is_paused
        
        # ========== Detect resume transition ==========
        elif not is_paused and self._is_paused:
            self._paused_duration = time.time() - self._pause_start_time
            self._is_paused = is_paused


    def clear(self,size) -> None:
        """Reset trajectory and exercise state to initial conditions.
        
        Called before each new trajectory or exercise to ensure clean state.
        Clears current goal, resets counters, and initializes empty trajectory arrays.
        
        This prevents leftover state from previous movements from affecting
        new requests (e.g., exercise_cnt persisting across exercise calls).
        """
        self.goal_fjt : List[FollowJointTrajectory.Goal] = [FollowJointTrajectory.Goal()]*size #type: ignore
        self.exercise_in_execution = False
        self.exercise_cnt = 0
        self._total_time_s : List[float] = [0.0]*size
        self.speed_scaling_factor : List[float] = [1.0]*size
        self.additional_speed_override : float = 1.0
        if size != 0:
            self.cancel_from_gui = False

    def set_trajectory(
        self,
        request: SetTrajectory.Request,
        response: SetTrajectory.Response
    ) -> SetTrajectory.Response:
        """Execute single-shot trajectory from GUI request (TRAJECTORY mode).
        
        Implements the set_trajectory service handler for one-time trajectory execution.
        The trajectory is user-defined and executed once at the requested speed override.
        
        Processing Steps:
            1. Clear previous state
            2. Extract cartesian waypoints and timestamps from request
            3. Interpolate using cubic splines and resample at 10 Hz
            4. Build FollowJointTrajectory goal with resampled points
            5. Apply speed override scaling to velocity/acceleration
            6. Submit goal to action client with on_trajectory_goal_accepted callback
            7. Return success status immediately (non-blocking)
        
        Args:
            request (SetTrajectory.Request): Contains:
                - cartesian_positions: List of (position, time_from_start) pairs
                - override: Speed override factor [%] (e.g., 100)
            response (SetTrajectory.Response): Response object to populate
        
        Returns:
            SetTrajectory.Response: success=True if goal submitted successfully
        
        Note:
            - Execution is asynchronous; completion callback is on_trajectory_goal_done()
            - GUI is freed immediately and receives completion via callback
            - No repetition loop (unlike set_exercise)
        """
        if self._refuse_without_controller(self.follow_joint_trajectory_action_client, self.controller_name, response):
            return response
        self.clear(size=1)

        # ========== Extract trajectory waypoints and times ==========
        _P = [r.point for r in request.cartesian_positions]
        _t = [r.time_from_start for r in request.cartesian_positions]

        self.goal_fjt[0].trajectory.joint_names = self._joint_names
        
        # ========== Interpolate and resample trajectory ==========
        # N=1: single execution (no repetition)
        self.get_logger().info(f'Before resampling, time: {self.get_clock().now().nanoseconds - self._init_time}')
        t, p, v, a = self.resample_with_speed_override(P=_P, t=_t, dt=self._dt, total_time=_t[-1], speed_ovr=int(request.override))

        # ========== Build FollowJointTrajectory goal ==========
        for i, tau in enumerate(t):
            self.addPoint(
                self.goal_fjt[0], # trajectory index
                p[i],
                v[i],
                a[i],
                Duration(sec=int(tau), nanosec=int((tau - int(tau)) * 1e9))
            )

        self.get_logger().info(f"Subsampled at {1.0/self._dt:.1f} Hz -> {len(t)} total points.")
        self.get_logger().info(f'Goal has {len(self.goal_fjt[0].trajectory.points)} points.')
        self._total_time_s[0] = t[-1]

        # ========== Apply speed override and submit goal ==========
        self.get_logger().info(f'Set Trajectory -> sending the new FJT Goal')
        self._trajectory_kind = 'ptp'
        response.success = self.sendFollowJointTrajectoryGoal(self.on_trajectory_goal_accepted)
        self.get_logger().info(f"Trajectory sento to FCT with result: {response.success}")
        response.success = True
        return response

    def set_go_to_start_trajectory(
        self,
        request: SetTrajectory.Request,
        response: SetTrajectory.Response
    ) -> SetTrajectory.Response:
        """Execute a single-shot trajectory to the go_to_start_controller action server.

        Identical interpolation pipeline to set_trajectory, but routes the goal to
        /go_to_start_controller/follow_joint_trajectory instead of the default controller.
        Called when the GUI "Go To Start" button is pressed and go_to_start_controller
        is the active controller (joint_trajectory_controller is inactive).
        """
        if self._refuse_without_controller(self.go_to_start_action_client, 'go_to_start_controller', response):
            return response
        self.clear(size=1)

        _P = [r.point for r in request.cartesian_positions]
        _t = [r.time_from_start for r in request.cartesian_positions]

        self.goal_fjt[0].trajectory.joint_names = self._joint_names
        t, p, v, a = self.resample_with_speed_override(
            P=_P, t=_t, dt=self._dt, total_time=_t[-1], speed_ovr=int(request.override)
        )
        for i, tau in enumerate(t):
            self.addPoint(
                self.goal_fjt[0],
                p[i], v[i], a[i],
                Duration(sec=int(tau), nanosec=int((tau - int(tau)) * 1e9))
            )
        self._total_time_s[0] = t[-1]

        self.get_logger().info('Set Go-To-Start Trajectory -> sending FJT goal to go_to_start_controller')
        self._trajectory_kind = 'go_to_start'
        self.sendFollowJointTrajectoryGoal(self.on_trajectory_goal_accepted, self.go_to_start_action_client)
        response.success = True
        return response

    def set_rehab_exercise(
        self,
        request: SetExercise.Request,
        response: SetExercise.Response
    ) -> SetExercise.Response:
        """Execute multi-repetition exercise from GUI request (EXERCISE mode).
        
        Implements the set_exercise service handler for repetitive rehabilitation exercises.
        The trajectory is repeated multiple times with different speed override factors
        provided by the GUI (e.g., reps at 80%, 100%, 120%).
        
        Processing Steps:
            1. Clear previous state
            2. Extract cartesian waypoints (single repetition) and normalize times
            3. Calculate number of repetitions N = floor(duration / single_rep_time)
            4. Interpolate single repetition, resample at 10 Hz, repeat N times
            5. Build FollowJointTrajectory goal from repeated resampled trajectory
            6. Apply speed override for first repetition (repetition_ovrs[0])
            7. Submit goal to action client with on_exercise_goal_accepted callback
            8. Start 10 Hz progress monitoring timer
            9. Return success immediately (non-blocking)
        
        The node then manages the repetition loop internally:
            - on_exercise_goal_done() callback checks if more reps remain
            - If yes: increment exercise_cnt, apply next speed override, resubmit goal
            - If no: signal exercise completion to GUI
        
        Args:
            request (SetExercise.Request): Contains:
                - cartesian_positions: Single-rep waypoints (position, time_from_start)
                - repetition_durations: Duration for current exercise [s]
                  (must be >= single_rep_duration * num_reps)
                - repetition_ovrs: Speed override for each repetition [%]
                  (length = number of reps to execute)
            response (SetExercise.Response): Response object to populate
        
        Returns:
            SetExercise.Response: success=True if first goal submitted successfully
        
        State Tracking:
            - exercise_cnt: Incremented after each repetition completion
            - repetition_ovrs: Persisted to check for more reps in on_exercise_goal_done()
            - _total_time_s: Duration of single repetition (used for progress %)
            - exercise_status_timer: 10 Hz timer for progress updates
        
        Note:
            - GUI can modify speed_scaling_factor during exercise via pause/resume
            - Progress is reported via /rehab_gui/exercise_progress service
            - GUI can request stop via /tecnobody_workbench_utils/stop_movement
        """
        if self._refuse_without_controller(self.follow_joint_trajectory_action_client, self.controller_name, response):
            return response
        self.number_of_repetition = len(request.repetition_ovrs)
        self.clear(self.number_of_repetition)

        # ========== Extract single-repetition trajectory waypoints ==========
        _P = [r.point for r in request.cartesian_positions]
        _t = [r.time_from_start - request.cartesian_positions[0].time_from_start for r in request.cartesian_positions] #type: ignore
        
        for trj_idx in range(self.number_of_repetition):

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self._joint_names
            goal.trajectory.points = []

            speed_scaling_pct = int(request.repetition_ovrs[trj_idx])
            self.speed_scaling_factor[trj_idx] = speed_scaling_pct/100

            # ========== Interpolate single rep, scale and repeat N times ==========
            t,p,v,a = self.resample_with_speed_override(P=_P,t=_t, dt=self._dt, total_time=request.repetition_durations[trj_idx], speed_ovr=speed_scaling_pct)

            self._total_time_s[trj_idx] = t[-1]
            for i, tau in enumerate(t):
                self.addPoint(goal, p[i], v[i], a[i], Duration(sec=int(tau), nanosec=int((tau - int(tau)) * 1e9)))
            self.goal_fjt[trj_idx] = deepcopy(goal)
        
        # ========== Initialize progress tracking ==========
        self.initProgressData(0)
        
        # ========== Submit first repetition goal ==========
        self.get_logger().info('Set Exercise -> sending the first FJT Goal')
        response.success = self.sendFollowJointTrajectoryGoal(self.on_exercise_goal_accepted) 
        self.get_logger().info(f"Trajectory sento to FCT with result: {response.success}") 
        response.success = True 
        return response
    
    def set_eeg_exercise(
            self,
            request: SetExercise.Request,
            response: SetExercise.Response
        ) -> SetExercise.Response:
        """Execute multi-repetition exercise from GUI request (EXERCISE mode).
        
        Implements the set_exercise service handler for repetitive rehabilitation exercises.
        The trajectory is repeated multiple times with different speed override factors
        provided by the GUI (e.g., reps at 80%, 100%, 120%).
        
        Processing Steps:
            1. Clear previous state
            2. Extract cartesian waypoints (single repetition) and normalize times
            3. Calculate number of repetitions N = floor(duration / single_rep_time)
            4. Interpolate single repetition, resample at 10 Hz, repeat N times
            5. Build FollowJointTrajectory goal from repeated resampled trajectory
            6. Apply speed override for first repetition (repetition_ovrs[0])
            7. Submit goal to action client with on_exercise_goal_accepted callback
            8. Start 50 Hz progress monitoring timer
            9. Return success immediately (non-blocking)
        
        The node then manages the repetition loop internally:
            - on_exercise_goal_done() callback checks if more reps remain
            - If yes: increment exercise_cnt, apply next speed override, resubmit goal
            - If no: signal exercise completion to GUI
        
        Args:
            request (SetExercise.Request): Contains:
                - cartesian_positions: Single-rep waypoints (position, time_from_start)
                - repetition_durations: Duration for current exercise [s]
                (must be >= single_rep_duration * num_reps)
                - repetition_ovrs: Speed override for each repetition [%]
                (length = number of reps to execute)
            response (SetExercise.Response): Response object to populate
        
        Returns:
            SetExercise.Response: success=True if first goal submitted successfully
        
        State Tracking:
            - exercise_cnt: Incremented after each repetition completion
            - repetition_ovrs: Persisted to check for more reps in on_exercise_goal_done()
            - _total_time_s: Duration of single repetition (used for progress %)
            - exercise_status_timer: 50 Hz timer for progress updates
        
        Note:
            - GUI can modify speed_scaling_factor during exercise via pause/resume
            - Progress is reported via /rehab_gui/exercise_progress service
            - GUI can request stop via /tecnobody_workbench_utils/stop_movement
        """
        if self._refuse_without_controller(self.follow_joint_trajectory_action_client, self.controller_name, response):
            return response
        self.number_of_repetition = len(request.repetition_ovrs)
        self.clear(self.number_of_repetition)

        # ========== Extract single-repetition trajectory waypoints ==========
        _P = [r.point for r in request.cartesian_positions]
        _t = [r.time_from_start - request.cartesian_positions[0].time_from_start for r in request.cartesian_positions] #type: ignore

        for trj_idx in range(self.number_of_repetition):

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self._joint_names
            goal.trajectory.points = []

            speed_scaling_pct = int(request.repetition_ovrs[trj_idx])
            self.speed_scaling_factor[trj_idx] = speed_scaling_pct/100

            # ========== Interpolate single rep, scale and repeat N times ==========
            # Draw a fresh pause duration for every individual pause inside this
            # phase, instead of once per phase: a single value reused for every
            # gap made every pause within a phase identical, defeating the point
            # of jittering it for the EEG paradigm.
            def _next_pause_duration(_trj_idx=trj_idx) -> float:
                value = self.eeg_delay_s + random.uniform(0.1, 3.9)
                self.get_logger().info(f"Pause duration for repetition {_trj_idx}: {value:.2f} seconds")
                return value
            t,p,v,a = self.resample_with_speed_override(P=_P,t=_t, dt=self._dt, total_time=request.repetition_durations[trj_idx], pause_duration_fn=_next_pause_duration, speed_ovr=speed_scaling_pct)

            self._total_time_s[trj_idx] = t[-1]
            for i, tau in enumerate(t):
                self.addPoint(goal, p[i], v[i], a[i], Duration(sec=int(tau), nanosec=int((tau - int(tau)) * 1e9)))
            self.goal_fjt[trj_idx] = deepcopy(goal)

        # ========== Initialize progress tracking ==========
        self.initProgressData(0)

        # ========== Submit first repetition goal ==========
        self.get_logger().info('Set EEG Exercise -> sending the first FJT Goal')
        response.success = self.sendFollowJointTrajectoryGoal(self.on_exercise_goal_accepted)
        self.get_logger().info(f"Trajectory sento to FCT with result: {response.success}")
        response.success = True
        return response

    def resample_with_speed_override(
            self,
            P: List[List[float]],
            t: List[float],
            dt: float,
            total_time: float,
            speed_ovr: int = 100,
            pause_duration_fn: Optional[Callable[[], float]] = None
        ) -> tuple[NDArray, NDArray, NDArray, NDArray]:

        # Convert inputs to numpy arrays for efficient operations
        _t = np.array(t)
        _P = np.array(P)

        # Ensures zero velocity at trajectory start and end (safe transitions)
        splines = [
            CubicSpline(_t, _P[:, i], bc_type='clamped')
            for i in range(_P.shape[1])
        ]

        # ========== Resample at uniform dt frequency ==========
        # Generate uniform time grid from start to end with dt interval
        tk = np.linspace(_t[0], _t[-1], int(_t[-1] / dt))
        
        # ========== Compute trajectory derivatives ==========
        # Position: spline evaluation at resampled times
        positions = np.stack([s(tk) for s in splines], axis=1)
        # Velocity: first derivative of spline (d/dt)
        velocity = np.stack([s.derivative(1)(tk) for s in splines], axis=1)
        # Acceleration: second derivative of spline (d²/dt²)
        acceleration = np.stack([s.derivative(2)(tk) for s in splines], axis=1)
        
        # ========== Scale computed trajectory ==========
        speed_factor = speed_ovr / 100.0
        vel_scaled = velocity * speed_factor
        acc_scaled = acceleration * speed_factor * speed_factor
        t_scaled = np.array([_tk / speed_factor for _tk in tk])

        pos_scaled_N = np.array(positions)
        vel_scaled_N = np.array(vel_scaled)
        acc_scaled_N = np.array(acc_scaled)
        t_scaled_N = np.array(t_scaled)

        single_rep_time = t_scaled[-1]

        # ========== Concatenate trajectory repetitions ==========
        if pause_duration_fn is None:
            # No pause: repetition count can be precomputed, as before.
            _N = math.floor(total_time / single_rep_time)
            for _ in range(_N - 1):
                t_scaled_N = np.concatenate((t_scaled_N, float(t_scaled_N[-1] + dt) + t_scaled))
                pos_scaled_N = np.concatenate((pos_scaled_N, positions))
                vel_scaled_N = np.concatenate((vel_scaled_N, vel_scaled))
                acc_scaled_N = np.concatenate((acc_scaled_N, acc_scaled))
        else:
            # Each gap gets its own freshly-drawn pause duration, so the
            # repetition count can't be precomputed up front (unlike the
            # no-pause branch above) -- keep adding (pause + repetition)
            # blocks while there is still room in the requested total_time
            # for one more repetition. The pause itself is accounted for
            # only after it is drawn, so the very last pause can overshoot
            # total_time slightly; that is preferable to guessing the
            # repetition count from an average pause duration up front.
            while t_scaled_N[-1] + single_rep_time <= total_time:
                pause_duration = pause_duration_fn()
                pause_points = int(pause_duration / dt) if pause_duration > 0 else 0
                if pause_points > 0:
                    last_time = t_scaled_N[-1]
                    last_pos = pos_scaled_N[-1]
                    for i in range(pause_points):
                        pause_time = last_time + (i + 1) * dt
                        t_scaled_N = np.append(t_scaled_N, pause_time)
                        pos_scaled_N = np.vstack((pos_scaled_N, last_pos))
                        vel_scaled_N = np.vstack((vel_scaled_N, np.zeros_like(last_pos)))
                        acc_scaled_N = np.vstack((acc_scaled_N, np.zeros_like(last_pos)))

                t_scaled_N = np.concatenate((t_scaled_N, float(t_scaled_N[-1] + dt) + t_scaled))
                pos_scaled_N = np.concatenate((pos_scaled_N, positions))
                vel_scaled_N = np.concatenate((vel_scaled_N, vel_scaled))
                acc_scaled_N = np.concatenate((acc_scaled_N, acc_scaled))

        return t_scaled_N, pos_scaled_N, vel_scaled_N, acc_scaled_N

    def addPoint(
        self,
        goal_element : FollowJointTrajectory.Goal,
        positions: List[float],
        velocities: List[float],
        accelerations: List[float],
        time: Duration
    ) -> None:
        """Add a single trajectory point to the current FollowJointTrajectory goal.
        
        Helper method to construct trajectory points from position, velocity,
        acceleration, and time components. Called iteratively during trajectory
        construction to build the complete goal point list.
        
        Args:
            positions (List[float]): Joint positions [m] for 3-DOF system
            velocities (List[float]): Joint velocities [m/s]
            accelerations (List[float]): Joint accelerations [m/s²]
            time (Duration): Time from trajectory start to this point
        
        Returns:
            None. Appends JointTrajectoryPoint to self.goal_fjt.trajectory.points
        """
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.effort = [] * len(positions)
        point.time_from_start = time
        goal_element.trajectory.points.append(point)  # type: ignore

    def initProgressData(self, trajectory_index: int) -> None:
        """Initialize progress tracking state for exercise execution.
        
        Called at the start of each repetition (or trajectory) to reset timing
        and progress counters. Ensures each repetition starts with clean state.
        
        Initializes:
            - _init_time_s: Current time (reference point for elapsed time)
            - _paused_duration: Resets pause accumulation to zero
            - _pause_start_time: Clears previous pause start point
            - trajectory.header.stamp: Current ROS time (for controller synchronization)
        
        Returns:
            None. Updates internal state variables.
        """
        self._init_time_s = time.time()
        self._paused_duration = 0.0
        self._last_actual_time_pct = 0.0
        self._last_reported_progress_pct = None
        self._pause_start_time = 0.0
        self.goal_fjt[trajectory_index].trajectory.header.stamp = self.get_clock().now().to_msg()


    def _refuse_without_controller(self, action_client: ActionClient, controller: str, response) -> bool:
        """Answer success=False when the controller's action server is not there.

        The goal would otherwise never be answered and the GUI, told success,
        would wait forever with the motors on.
        """
        # Blocks this callback only when the controller is really missing
        # (wait_for_server polls the graph, it does not need the executor).
        if action_client.server_is_ready() or action_client.wait_for_server(timeout_sec=CONTROLLER_WAIT_S):
            return False
        self.get_logger().error(
            f'Movement refused: {controller}/follow_joint_trajectory not available after '
            f'{CONTROLLER_WAIT_S:.1f} s (controller not active?)')
        response.success = False
        return True

    def _report_goal_not_started(self, on_goal_accepted, error_code: int, reason: str) -> None:
        """Tell the GUI that the goal sent with on_goal_accepted will not run."""
        self._goal_handle = None
        if on_goal_accepted == self.on_exercise_goal_accepted:
            self.get_logger().error(f'Repetition {self.exercise_cnt}: {reason}. Exercise suspended.')
            self.exercise_suspended_notifier.notify(
                False, GoalStatus.STATUS_UNKNOWN, error_code,
                f'Repetition {self.exercise_cnt}: {reason}', 'exercise')
        else:
            self.get_logger().error(f'Trajectory ({self._trajectory_kind}): {reason}.')
            self.trajectory_finished_notifier.notify(
                False, GoalStatus.STATUS_UNKNOWN, error_code, reason, self._trajectory_kind)

    def _watch_goal_acceptance(self, send_goal_future, on_goal_accepted) -> None:
        """Report the goal as not started if the controller does not answer in time."""
        if self._acceptance_timer is not None:
            self.destroy_timer(self._acceptance_timer)
        self._acceptance_expired = False

        def check() -> None:
            self._acceptance_timer.cancel()
            if send_goal_future.done() or not self._goal_acceptance_pending:
                return
            self._acceptance_expired = True  # a late acceptance is cancelled at once
            self._goal_acceptance_pending = False
            self._report_goal_not_started(
                on_goal_accepted, NO_RESULT_ERROR_CODE,
                f'the controller did not answer the goal within {GOAL_ACCEPTANCE_TIMEOUT_S:.0f} s')
        self._acceptance_timer = self.create_timer(GOAL_ACCEPTANCE_TIMEOUT_S, check)

    def _discard_stale_goal(self, future, goal_handle) -> bool:
        """True (and cancel it if accepted) for an answer that must not run.

        Either the answer to an older goal (a new one was sent meanwhile, e.g.
        the operator retried after a timeout), or to the current goal after it
        was already reported to the GUI as not started.
        """
        stale = future is not self._send_goal_future
        if not stale and not self._acceptance_expired:
            return False
        if goal_handle.accepted:
            self.get_logger().warn(
                f'Goal accepted {"for a superseded request" if stale else "after the acceptance timeout"}: '
                'cancelling it.')
            goal_handle.cancel_goal_async()
        return True

    def _handle_late_or_rejected_goal(self, on_goal_accepted) -> bool:
        """Common part of the acceptance callbacks; True when the goal must not run."""
        if not self._goal_handle.accepted:
            if self.cancel_from_gui:
                self.get_logger().info('Goal rejected after a stop request.')
                self._goal_handle = None
                self._notify_movement_stopped()
                self.clear(0)
            else:
                self._report_goal_not_started(
                    on_goal_accepted, FollowJointTrajectory.Result.INVALID_GOAL,
                    'goal rejected by the controller')
            return True
        return False

    def sendFollowJointTrajectoryGoal(self, on_goal_accepted, action_client: ActionClient = None) -> bool:  # type: ignore
        """Submit trajectory goal to FollowJointTrajectory action client.
        
        Asynchronous submission of the current goal to the ros2_control
        joint_trajectory_controller. Returns immediately without waiting for
        execution completion.
        
        The callback on_goal_accepted will be invoked when the server responds
        with either acceptance or rejection. This allows the GUI to be freed
        from blocking action client calls.
        
        Args:
            on_goal_accepted (Callable): Callback function to invoke when
                server responds with goal acceptance/rejection. Should have
                signature: on_goal_accepted(future) where future.result()
                contains the goal handle.
        
        Returns:
            bool: Always returns True (submission is non-blocking)
        
        Note:
            - Callback must check goal_handle.accepted to detect rejections
            - Two different callbacks used:
              * on_trajectory_goal_accepted(): for set_trajectory requests
              * on_exercise_goal_accepted(): for set_exercise requests
        """
        action_client = action_client or self.follow_joint_trajectory_action_client
        self._stopped_by_gui = False
        self.get_logger().info(f'Sending goal number {self.exercise_cnt} to the FJT Controller')
        self._send_goal_future = action_client.send_goal_async(self.goal_fjt[self.exercise_cnt])
        self._goal_acceptance_pending = True
        self._watch_goal_acceptance(self._send_goal_future, on_goal_accepted)
        self._send_goal_future.add_done_callback(on_goal_accepted)
        return True

    def on_trajectory_goal_accepted(self, future) -> None:  # type: ignore
        """Callback for trajectory goal acceptance/rejection (TRAJECTORY mode).
        
        Invoked when the FollowJointTrajectory action server responds to
        our goal submission. For single-shot trajectories (set_trajectory),
        we simply wait for completion and signal the GUI via on_trajectory_goal_done.
        
        Goal Lifecycle:
            1. Goal submitted via send_goal_async() in sendFollowJointTrajectoryGoal()
            2. Server responds with goal handle (accepted/rejected)
            3. This callback invoked with future containing goal_handle
            4. If accepted: request result asynchronously via get_result_async()
            5. When result available: on_trajectory_goal_done() callback invoked
            6. on_trajectory_goal_done() signals GUI via /rehab_gui/trajectory_finished
        
        Args:
            future: Future object containing result from send_goal_async()
                Result is a goal_handle with .accepted property
        
        Returns:
            None. Sets up result callback if accepted.
        
        Error Handling:
            - Logs rejection if goal not accepted by server
            - Does not retry or escalate; caller must handle
        """
        goal_handle = future.result()
        if self._discard_stale_goal(future, goal_handle):
            return
        self._goal_acceptance_pending = False
        self._goal_handle = goal_handle
        if self._handle_late_or_rejected_goal(self.on_trajectory_goal_accepted):
            return
        self.get_logger().info('Trajectory Goal accepted!!')
        if self.cancel_from_gui:
            self.get_logger().warn('Trajectory goal accepted after stop request; cancelling immediately.')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_cancelled)
            return
        self._get_goal_result_future = self._goal_handle.get_result_async()
        self._get_goal_result_future.add_done_callback(self.on_trajectory_goal_done)
        self._goal_status = GoalStatus.STATUS_UNKNOWN

    def on_trajectory_goal_done(self, future) -> None:  # type: ignore
        """Callback for trajectory execution completion (TRAJECTORY mode).
        
        Invoked when the trajectory execution completes (successfully or with error).
        For single-shot trajectories, signals completion back to the GUI.
        
        Actions:
            1. Create client for /rehab_gui/trajectory_finished service
            2. Wait for service availability (5 second timeout)
            3. Call service to notify GUI that trajectory completed
            4. Clear goal handle to prevent memory leaks
        
        Args:
            future: Future object containing result from get_result_async()
        
        Returns:
            None. Makes async service call to GUI; result ignored.
        
        Exception Handling:
            - Catches all exceptions and logs them (non-fatal)
            - Service unavailable logged as info (GUI may not be running)
            - Continues gracefully without state corruption
        """
        status, error_code, error_string = goal_result_fields(future)
        self._goal_handle = None
        if status == GoalStatus.STATUS_CANCELED and self._stopped_by_gui:
            # The GUI asked for it and gets movement_stopped (on_cancelled). A
            # trajectory_finished here would leave its "completed" flag set,
            # ending the next movement as soon as the motors are on.
            self.get_logger().info(f'Trajectory ({self._trajectory_kind}) cancelled by the GUI.')
            return
        success = (status == GoalStatus.STATUS_SUCCEEDED
                   and error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        self.get_logger().info(
            f'Trajectory ({self._trajectory_kind}) DONE: status={status} code={error_code} '
            f'"{error_string}", notifying GUI')
        # Sent whatever the outcome: the GUI switches the motors off on it.
        self.trajectory_finished_notifier.notify(
            success, status, error_code,
            error_string or ('Trajectory completed' if success else 'Trajectory failed'),
            self._trajectory_kind)

    def on_exercise_goal_accepted(self, future) -> None:  # type: ignore
        """Callback for exercise goal acceptance/rejection (EXERCISE mode).
        
        Invoked when the FollowJointTrajectory action server responds to goal
        submission for exercise repetitions. Unlike set_trajectory, exercise mode
        must also monitor progress during execution for GUI progress bar updates.
        
        Actions:
            1. Check if goal accepted by server
            2. If accepted: request result asynchronously
            3. Start 10 Hz progress monitoring timer (check_exercise_status)
            4. Set up result callback for completion handling
        
        The progress timer runs at 10 Hz (100 ms interval) and continuously
        computes elapsed time percentage for the current repetition, accounting
        for pause/resume events.
        
        Args:
            future: Future object containing goal_handle from send_goal_async()
        
        Returns:
            None. Sets up result and progress callbacks if accepted.
        """
        goal_handle = future.result()
        if self._discard_stale_goal(future, goal_handle):
            return
        self._goal_acceptance_pending = False
        self._goal_handle = goal_handle
        if self._handle_late_or_rejected_goal(self.on_exercise_goal_accepted):
            return
        self.get_logger().info(f'Exercise Goal accepted!!')
        if self.cancel_from_gui:
            self.get_logger().warn('Exercise goal accepted after stop request; cancelling immediately.')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_cancelled)
            return
        self._get_goal_result_future = self._goal_handle.get_result_async()
        self._get_goal_result_future.add_done_callback(self.on_exercise_goal_done)
        self._goal_status = GoalStatus.STATUS_UNKNOWN

        self._jtc_feedback_triggered = False

        # Start progress monitoring. The GUI refreshes at 10 Hz, so faster
        # rosbridge service calls only create redundant load.
        self.exercise_status_timer = self.create_timer(self._progress_report_period_s, self.check_exercise_status, callback_group=self.timer_group)

    def _cancel_exercise_status_timer(self) -> None:
        if self.exercise_status_timer is not None:
            self.exercise_status_timer.cancel()
            self.exercise_status_timer = None #type: ignore

    def _advance_exercise(self) -> None:
        """Advance to the next repetition or signal exercise completion."""
        if self._goal_handle is None:
            return

        # Notify GUI (was dropped when the GUI did not answer within 0.5 s)
        self.exercise_finished_notifier.notify(
            True, GoalStatus.STATUS_SUCCEEDED, FollowJointTrajectory.Result.SUCCESSFUL,
            f'Exercise repetition {self.exercise_cnt} completed', 'exercise')

        # Cancel old timer
        self._cancel_exercise_status_timer()

        self._goal_handle = None

        if self.exercise_cnt + 1 < self.number_of_repetition and not self.cancel_from_gui:
            self.exercise_cnt += 1
            self.initProgressData(self.exercise_cnt)
            self.sendFollowJointTrajectoryGoal(self.on_exercise_goal_accepted)
            self.get_logger().info(
                f'Starting repetition {self.exercise_cnt}/{self.number_of_repetition}'
            )
        else:
            self.get_logger().info('All repetitions completed!')

    def on_exercise_goal_done(self, future):
        if self._goal_handle is None:
            return
        status, error_code, error_string = goal_result_fields(future)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Repetition {self.exercise_cnt} completed successfully.')
            self._advance_exercise()
        elif status == GoalStatus.STATUS_CANCELED:
            # Stop requested by the GUI: on_cancelled notifies movement_stopped.
            self.get_logger().info(f'Repetition {self.exercise_cnt} was cancelled.')
        else:
            # ABORTED (e.g. tracking error: PATH/GOAL_TOLERANCE_VIOLATED) or no
            # usable result: the exercise is suspended, the GUI is told why.
            self.get_logger().error(
                f'Repetition {self.exercise_cnt} ended with status {status}, '
                f'code {error_code}: "{error_string}". Exercise suspended.')
            self._cancel_exercise_status_timer()  # stop time-based progress of a dead goal
            self._goal_handle = None
            self.exercise_suspended_notifier.notify(
                False, status, error_code,
                f'Repetition {self.exercise_cnt}: {error_string}' if error_string
                else f'Repetition {self.exercise_cnt} failed', 'exercise')


    def check_exercise_status(self) -> None:
        """Monitor exercise progress and report percentage to GUI (10 Hz timer).
        
        This timer callback executes at 10 Hz during exercise execution and
        computes real-time progress for display on the GUI progress bar.
        
        Progress Calculation:
            1. Calculate elapsed time since repetition start
            2. Subtract accumulated pause durations
            3. Account for speed scaling: elapsed_time_scaled = elapsed_time / speed_factor
            4. Calculate percentage: (elapsed_time_scaled / total_duration) * 100%
            5. Clamp to [0%, 100%]
            6. Report via /rehab_gui/exercise_progress service
        
        Pause Accounting:
            - During pause (speed_factor < 0.01): elapsed time does not accumulate
            - When resume detected: pause duration subtracted from elapsed time
            - Progress bar appears frozen during pause, continues on resume
        
        Time Formula (accounting for pause):
            effective_time = (current_time - start_time) - pause_duration
            total_scaled_time = total_time / speed_scaling_factor
            progress_percent = (effective_time / total_scaled_time) * 100%
        
        Example:
            - Repetition duration: 10 seconds @ 100% speed
            - User pauses after 5 seconds
            - Check_exercise_status shows 50% progress, stays frozen
            - After 3 seconds of pause, user resumes
            - Progress jumps to 50%, continues incrementing
            - Final time in controller: ~8 seconds (5 + 3 = total_time - pause)
        
        Args:
            None. Triggered by 100 ms timer at 10 Hz.
        
        Returns:
            None. Makes asynchronous service call to GUI with progress percentage.
        
        Side Effects:
            - Updates _last_time_from_start_percentage for next iteration
            - Logs warning if exercise paused (speed_factor < 0.01)
        """
        # ========== Safety check: goal must be active ==========
        if self._goal_handle is None:
            return

        if self.cancel_from_gui:
            return

        if (
            self.exercise_cnt >= len(self._total_time_s)
            or self.exercise_cnt >= len(self.speed_scaling_factor)
        ):
            self.get_logger().warning(
                'Skipping exercise status update because progress state is inconsistent: '
                f'exercise_cnt={self.exercise_cnt}, '
                f'total_time_len={len(self._total_time_s)}, '
                f'speed_scaling_len={len(self.speed_scaling_factor)}'
            )
            self._cancel_exercise_status_timer()
            return
        
        self._goal_status = self._goal_handle.status #type: ignore
        
        # ========== Initialize with previous percentage (for paused state) ==========
        actual_time_from_start_percentage = self._last_time_from_start_percentage
        
        # ========== Calculate progress if not paused ==========
        if not self._is_paused and self._total_time_s[self.exercise_cnt] > 0 and self.speed_scaling_factor[self.exercise_cnt] > 0:
            actual_time = time.time()
            # Elapsed time minus pause duration
            effective_time = actual_time - self._init_time_s - self._paused_duration
            # Progress percentage
            actual_time_from_start_percentage = (effective_time / self._total_time_s[self.exercise_cnt]) * 100
            # Clamp to [0%, 100%]
            actual_time_from_start_percentage = min(actual_time_from_start_percentage, 100.0)

            # Cache for next iteration (during pause)
            self._last_time_from_start_percentage = actual_time_from_start_percentage
        else:
            self.get_logger().warning(
                f'The movement is paused at {self._last_time_from_start_percentage:.2f}%',
                throttle_duration_sec=5.0
            )

        # ========== Report progress to GUI ==========
        progress_pct = int(actual_time_from_start_percentage)
        should_report_progress = progress_pct != self._last_reported_progress_pct

        if should_report_progress and self.exercise_progress_client.wait_for_service(timeout_sec=0.0):
            req = MovementProgress.Request()
            req.progress = actual_time_from_start_percentage
            future = self.exercise_progress_client.call_async(req)
            future.add_done_callback(self.on_progress_response)
            self._last_reported_progress_pct = progress_pct
        
        self._last_actual_time_pct = progress_pct

    def on_progress_response(self, future) -> None:  # type: ignore
        #if len(self.repetition_ovrs) == len(future.result().repetition_ovrs.tolist()):
            #self.repetition_ovrs = future.result().repetition_ovrs.tolist()
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'exercise_progress service call failed: {exc!r}')
            return
        if response is not None:
            self.additional_speed_override = response.additional_speed_override
        
    def _notify_movement_stopped(self) -> None:
        # Non-blocking (it used to wait up to 5 s inside the callback, holding
        # the node's default callback group, and then drop the notification).
        self.movement_stopped_notifier.send(Trigger.Request())

    def on_cancelled(self, future) -> None:  # type: ignore
        """Callback for cancelled exercise goals (emergency stop).
        
        Invoked when the user requests exercise stop via /stop_movement service.
        Performs cleanup and notifies GUI that movement was cancelled by user.
        
        Args:
            future: Future object containing cancellation result
        
        Returns:
            None. Performs cleanup and signals GUI.
        """
        self.get_logger().warn('The Goal has been succesfully cancelled. Notify to the remote GUI')
        try:
            self._notify_movement_stopped()
            if future.result() is not None:
                self.get_logger().info(f'Goal cancelled: {future.result()}')
            else:
                self.get_logger().info('Goal cancelled without result')
        finally:
            self.clear(0)
            self._goal_handle = None

    def stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:  # type: ignore
        """Stop current exercise and cancel running goal (emergency stop).
        
        Service handler for /stop_movement requests from GUI. Cancels the active
        goal, or records the stop request until a pending goal handle arrives.
        
        Actions:
            1. Latch cancellation so repetition advancement stops
            2. Cancel progress monitoring timer
            3. Cancel active goal if present (calls controller to stop trajectory)
            4. Let on_cancelled clear state after the controller acknowledges cancel
        
        Args:
            request: Empty Trigger request
            response: Trigger response to populate
        
        Returns:
            Trigger.Response: success=True if goal cancelled, False otherwise
        """
        self.cancel_from_gui = True
        # Unlike cancel_from_gui (reset by on_cancelled's clear(), possibly before
        # the goal result arrives), this lasts until the next goal is sent.
        self._stopped_by_gui = True
        
        # Cancel progress timer
        self._cancel_exercise_status_timer()
        
        # Cancel active goal
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            self.get_logger().warn('Received a Goal Cancel Request')
            cancel_future = self._goal_handle.cancel_goal_async()

            cancel_future.add_done_callback(self.on_cancelled)
            response.success = True
        elif self._goal_acceptance_pending:
            self.get_logger().info('Stop requested while goal handle is pending; it will be cancelled on acceptance.')
            response.success = True
        else:
            self.get_logger().info('Goal handle has not been created yet')
            response.success = False
        
        return response


def main(args=None):
    """Entry point for the trajectory manager node.
    
    Initializes ROS 2 system, creates the FollowJointTrajectoryActionManager node,
    and spins it with a single-threaded executor (callbacks run one at a time).
    
    Configuration:
        - CPU affinity: select the core to have deterministic timing for real-time control
        - Executor: SingleThreadedExecutor, spin() (each callback as soon as ready)
        - Controller: specified via command-line argument (default: "joint_trajectory_controller")
    
    Args:
        args (list, optional): Command-line arguments (default: None)
            Expected: [script_name, controller_name, ...]
            If not provided: defaults to "joint_trajectory_controller"
    
    Returns:
        None. Blocks until KeyboardInterrupt or shutdown signal.
    """
    rclpy.init(args=args)

    # Set CPU affinity to core 5 for deterministic scheduling
    import os
    os.sched_setaffinity(0, {5})

    # Get controller name from command line or use default
    input_controller = sys.argv[1] if len(sys.argv) > 1 else "joint_trajectory_controller"

    # Create node instance
    fjtam = FollowJointTrajectoryActionManager(controller_name=input_controller)

    # One thread, callbacks strictly one after the other. The process is pinned
    # to one core and Python runs one thread at a time anyway: more threads
    # gave no parallelism, only callbacks interleaving (e.g. the progress timer
    # reading the lists clear() rebuilds). No callback blocks, so nothing waits
    # for another. spin() runs each callback as soon as it is ready: the former
    # spin_once() + sleep(0.05) loop ran at most ~20 callbacks/s, dropping most
    # speed scaling messages (25 Hz). Idle, spin() uses no CPU (it waits in rcl).
    executor = SingleThreadedExecutor()
    executor.add_node(fjtam)

    try:
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            try:
                fjtam.get_logger().info('Keyboard interrupt, shutting down.\n')
            except Exception:
                print('[fct_manager_node] Keyboard interrupt, shutting down.')
        else:
            print('[fct_manager_node] Keyboard interrupt, shutting down.')

    # Cleanup
    try:
        rclpy.try_shutdown()
    except Exception:
        pass
    try:
        executor.shutdown()
    except Exception:
        pass
    # fjtam.destroy_node()


if __name__ == '__main__':
    main()
