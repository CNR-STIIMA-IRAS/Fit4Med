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
    3. Progress Tracking: Monitors exercise progression at 50 Hz and reports completion
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
    - /rehab_gui/exercise_progress: Periodic status (50 Hz) with progress % for display

Architecture:
    Two execution modes:
    - TRAJECTORY mode (N=1): Single-shot trajectory from set_trajectory service
    - EXERCISE mode (N>1): Multi-rep loop from set_exercise with per-rep speed factors
    
    Each repetition follows identical execution flow but with different speed override
    applied via setSpeedOverride() before goal submission.

Performance:
    - Trajectory resampling: ~100 ms (cubic spline interpolation)
    - Progress reporting: 50 Hz (20 ms timer interval)
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
    exercise_status_timer (Timer): 50 Hz progress monitoring timer
    follow_joint_trajectory_action_client (ActionClient): Connection to controller
"""

import sys
import time
import random
from typing import List
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
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# ROS 2 control and message types
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import SpeedScalingFactor, JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tecnobody_msgs.srv import SetExercise, SetTrajectory, MovementProgress
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Development tooling
from rich.traceback import install
install(show_locals=True)


class FollowJointTrajectoryActionManager(Node):
    """ROS 2 node managing multi-repetition rehabilitation exercises with dynamic speed control.
    
    This node implements a stateful exercise manager that bridges the rehabilitation GUI
    and the ros2_control FollowJointTrajectory action interface. It provides transparent
    handling of:
    
    1. Trajectory Interpolation: Cubic spline interpolation with fixed resampling frequency
    2. Multi-Repetition Loops: Automatic looping of exercises with per-repetition speed factors
    3. Pause Detection: Real-time pause/resume via SpeedScalingFactor subscription (< 0.01 = paused)
    4. Progress Tracking: 50 Hz progress reporting with elapsed time percentage
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
        - 50 Hz timer (20 ms) checks exercise_status via check_exercise_status()
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
        - Configures multi-threaded execution for concurrent callback handling
        
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
        self.number_of_repetition : int = 0
        self.cancel_from_gui : bool = False
        self.additional_speed_override : float = 1.0
        self.goal_fjt : List[FollowJointTrajectory.Goal] = None #type: ignore
        self.exercise_in_execution : bool = False
        self.exercise_cnt : int = 0
        self._total_time_s : List[float] = list()
        self.speed_scaling_factor : List[float] = list()
        self._last_time_from_start_percentage : float = 0.0
        
        self._dt : float = 0.1
        self._init_time_s : float = 0
        self._is_paused : bool = False
        self._pause_start_time : float = 0.0
        self._paused_duration = 0.0
        self._joint_names = ['joint_x', 'joint_y', 'joint_z']
        self.absolute_positions = None #type: ignore

        # ========== Status Tracking ==========
        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self._init_time = self.get_clock().now().nanoseconds
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_group = MutuallyExclusiveCallbackGroup()
        self.exercise_progress_client : Client = None #type: ignore
        self.exercise_status_timer : Timer = None #type: ignore

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
        self.clear(0)

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
        self.stop_movement_server = self.create_service(
            Trigger,
            "/tecnobody_workbench_utils/stop_movement",
            self.stop
        )
        self.exercise_progress_client = self.create_client(
            MovementProgress, 
            "/rehab_gui/exercise_progress"
        )


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
        response.success = self.sendFollowJointTrajectoryGoal(self.on_trajectory_goal_accepted)
        self.get_logger().info(f"Trajectory sento to FCT with result: {response.success}") 
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
                # Set pause duration
                pause_duration = 6.0 + random.uniform(0.1, 3.9)
                self.get_logger().info(f"Pause duration for repetition {trj_idx}: {pause_duration:.2f} seconds")
                t,p,v,a = self.resample_with_speed_override(P=_P,t=_t, dt=self._dt, total_time=request.repetition_durations[trj_idx], pause_duration=pause_duration)

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

    def resample_with_speed_override(
            self,
            P: List[List[float]],
            t: List[float],
            dt: float,
            total_time: float,
            speed_ovr: int = 100,
            pause_duration: float = 0.0
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

        _N = math.floor(total_time / t_scaled[-1])

        # ========== Concatenate trajectory N times ==========
        # For each additional repetition, append time-shifted copies
        pause_points = int(pause_duration / dt) if pause_duration > 0 else 0
        
        for _ in range(_N - 1):
            # ========== Add pause if requested ==========
            if pause_points > 0:
                last_time = t_scaled_N[-1]
                last_pos = pos_scaled_N[-1]
                
                for i in range(pause_points):
                    pause_time = last_time + (i + 1) * dt
                    t_scaled_N = np.append(t_scaled_N, pause_time)
                    pos_scaled_N = np.vstack((pos_scaled_N, last_pos))
                    vel_scaled_N = np.vstack((vel_scaled_N, np.zeros_like(last_pos)))
                    acc_scaled_N = np.vstack((acc_scaled_N, np.zeros_like(last_pos)))
            
            # ========== Offset time to continue from last point ==========
            t_scaled_N = np.concatenate((t_scaled_N, float(t_scaled_N[-1] + dt) + t_scaled))
            # Append position waypoints unchanged
            pos_scaled_N = np.concatenate((pos_scaled_N, positions))
            # Append velocity and acceleration with scaling
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
        self._pause_start_time = 0.0
        self.goal_fjt[trajectory_index].trajectory.header.stamp = self.get_clock().now().to_msg()


    def sendFollowJointTrajectoryGoal(self, on_goal_accepted) -> bool:  # type: ignore
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
        self.get_logger().info(f'Sending goal number {self.exercise_cnt} to the FJT Controller')
        self._send_goal_future = self.follow_joint_trajectory_action_client.send_goal_async(self.goal_fjt[self.exercise_cnt])
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
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Trajectory Goal rejected!!')
            return
        self.get_logger().info('Trajectory Goal accepted!!')
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
        try:
            self.get_logger().info('Trajectory execution DONE, notifying GUI...')
            client: Client = self.create_client(Trigger, '/rehab_gui/trajectory_finished')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Trajectory DONE server is not available.')

            req = Trigger.Request()
            client.call_async(req)
            self.get_logger().info('Trajectory DONE sent to GUI')
            self._goal_handle = None
        except Exception as e:
            self.get_logger().info(f'Exception in on_trajectory_goal_done: {e}')

    def on_exercise_goal_accepted(self, future) -> None:  # type: ignore
        """Callback for exercise goal acceptance/rejection (EXERCISE mode).
        
        Invoked when the FollowJointTrajectory action server responds to goal
        submission for exercise repetitions. Unlike set_trajectory, exercise mode
        must also monitor progress during execution for GUI progress bar updates.
        
        Actions:
            1. Check if goal accepted by server
            2. If accepted: request result asynchronously
            3. Start 50 Hz progress monitoring timer (check_exercise_status)
            4. Set up result callback for completion handling
        
        The progress timer runs at 50 Hz (20 ms interval) and continuously
        computes elapsed time percentage for the current repetition, accounting
        for pause/resume events.
        
        Args:
            future: Future object containing goal_handle from send_goal_async()
        
        Returns:
            None. Sets up result and progress callbacks if accepted.
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Exercise Goal rejected!!')
            return
        self.get_logger().info(f'Exercise Goal accepted!!')
        self._get_goal_result_future = self._goal_handle.get_result_async()
        self._get_goal_result_future.add_done_callback(self.on_exercise_goal_done)
        self._goal_status = GoalStatus.STATUS_UNKNOWN

        self._jtc_feedback_triggered = False

        # Start progress monitoring
        self.exercise_status_timer = self.create_timer(0.02, self.check_exercise_status, callback_group=self.timer_group)

    def _advance_exercise(self) -> None:
        """Advance to the next repetition or signal exercise completion."""
        if self._goal_handle is None:
            return

        # Notify GUI
        client = self.create_client(Trigger, '/rehab_gui/exercise_finished')
        if client.wait_for_service(timeout_sec=0.5):
            client.call_async(Trigger.Request())

        # Cancel old timer
        if self.exercise_status_timer is not None:
            self.exercise_status_timer.cancel()

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
        if self._goal_handle is not None:
            self._advance_exercise()

    def check_exercise_status(self) -> None:
        """Monitor exercise progress and report percentage to GUI (50 Hz timer).
        
        This timer callback executes at 50 Hz during exercise execution and
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
            None. Triggered by 20 ms timer at 50 Hz.
        
        Returns:
            None. Makes asynchronous service call to GUI with progress percentage.
        
        Side Effects:
            - Updates _last_time_from_start_percentage for next iteration
            - Logs warning if exercise paused (speed_factor < 0.01)
        """
        # ========== Safety check: goal must be active ==========
        if self._goal_handle is None:
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
                f'The movement is paused at {self._last_time_from_start_percentage:.2f}%'
            )

        # ========== Report progress to GUI ==========
        if self.exercise_progress_client.wait_for_service(timeout_sec=.5):
            req = MovementProgress.Request()
            req.progress = actual_time_from_start_percentage
            future = self.exercise_progress_client.call_async(req)
            future.add_done_callback(self.on_progress_response)
        
        self._last_actual_time_pct = int(actual_time_from_start_percentage)

    def on_progress_response(self, future) -> None:  # type: ignore
        #if len(self.repetition_ovrs) == len(future.result().repetition_ovrs.tolist()):
            #self.repetition_ovrs = future.result().repetition_ovrs.tolist()
        self.additional_speed_override = future.result().additional_speed_override
        
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
        client : Client = self.create_client(Trigger, '/rehab_gui/movement_stopped')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Movement Stopped Server is not available.')
        else:        
            req = Trigger.Request()
            client.call_async(req)

            if future.result() is not None:
                self.get_logger().info(f'Goal cancelled: {future.result()}')
            else:
                self.get_logger().info('Goal cancelled without result')
        
        self.clear(0)
        self._goal_handle = None

    def stop(self, request, response) -> Trigger.Response:  # type: ignore
        """Stop current exercise and cancel running goal (emergency stop).
        
        Service handler for /stop_movement requests from GUI. Cancels the active
        goal, clears state, and signals GUI that movement stopped.
        
        Actions:
            1. Clear repetition_ovrs to break repetition loop
            2. Cancel progress monitoring timer
            3. Cancel active goal if present (calls controller to stop trajectory)
            4. Invoke on_cancelled callback for cleanup
        
        Args:
            request: Empty Trigger request
            response: Trigger response to populate
        
        Returns:
            Trigger.Response: success=True if goal cancelled, False otherwise
        """
        self.cancel_from_gui = True
        self._total_time_s = []
        
        # Cancel progress timer
        if self.exercise_status_timer is not None:
            self.exercise_status_timer.cancel()
        
        # Cancel active goal
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            self.get_logger().warn('Received a Goal Cancel Request')
            cancel_future = self._goal_handle.cancel_goal_async()

            cancel_future.add_done_callback(self.on_cancelled)
            response.success = True
        else:
            self.get_logger().info('Goal handle has not been created yet')
            response.success = False
        
        return response


def main(args=None):
    """Entry point for the trajectory manager node.
    
    Initializes ROS 2 system, creates the FollowJointTrajectoryActionManager node,
    and runs the multi-threaded executor with 4 threads for concurrent callback handling.
    
    Configuration:
        - CPU affinity: select the core to have deterministic timing for real-time control
        - Executor: MultiThreadedExecutor with 4 threads
        - Spin interval: 100 ms (polling frequency)
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

    # Create multi-threaded executor (4 threads for concurrent callbacks)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fjtam)
    
    try:
        while rclpy.ok():
            executor.spin_once()
            time.sleep(0.05)

    except KeyboardInterrupt:
        fjtam.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    # Cleanup
    executor.shutdown()
    fjtam.destroy_node()


if __name__ == '__main__':
    main()
