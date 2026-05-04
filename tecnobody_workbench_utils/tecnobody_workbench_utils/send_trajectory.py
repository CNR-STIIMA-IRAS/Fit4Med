# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""
Simplified trajectory loader that sends multi-repetition joint trajectories directly
to the joint_trajectory_controller via the FollowJointTrajectory action.

This node reads a simple YAML file containing:
  - target_position: target joint positions [joint_x, joint_y, joint_z] [m]
  - nominal_duration: time to reach target [seconds]
  - num_waypoints: number of intermediate waypoints

The node then:
  1. Generates intermediate waypoints via linear interpolation
  2. Sends the trajectory directly to:
       /joint_trajectory_controller/follow_joint_trajectory
  3. Repeats for each repetition, scaling time by (100 / speed_factor)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SimpleTrajectoryLoader(Node):
    """
    Load trajectory from YAML and send as joint trajectory to joint_trajectory_controller.

    YAML Format (trajectory_params.yaml):
        target_position: [0.3, 0.3, 0.0]    # Target joint positions [joint_x, joint_y, joint_z] [m]
        nominal_duration: 5.0                # Time to reach target [s]
        num_waypoints: 10                    # Number of intermediate points

    Python Configuration (beginning of file):
        NUM_REPETITIONS = 3                  # Number of exercise repetitions
        SPEED_FACTORS = [80, 100, 120]      # Speed override per repetition [%] (scales time)
    """

    # ========== Configuration Parameters ==========
    NUM_REPETITIONS = 3                     # Number of exercise repetitions
    SPEED_FACTORS = [50.0, 70.0, 100.0]   # Speed override per repetition [%]
    START_POSITION = [0.0, 0.0, 0.0]       # Starting joint positions [m]
    CIRCULAR = False                        # If True, use circular trajectory; if False, use linear trajectory

    # Joint names must match the controller configuration
    JOINT_NAMES = ['joint_x', 'joint_y', 'joint_z']

    # Action server name
    ACTION_SERVER = '/joint_trajectory_controller/follow_joint_trajectory'

    def __init__(self):
        super().__init__('simple_trajectory_loader')

        self._action_client = ActionClient(self, FollowJointTrajectory, self.ACTION_SERVER)

        self.get_logger().info("Trajectory loader initialized")
        self.get_logger().info(f"  Action server:  {self.ACTION_SERVER}")
        self.get_logger().info(f"  Joint names:    {self.JOINT_NAMES}")
        self.get_logger().info(f"  Repetitions:    {self.NUM_REPETITIONS}")
        self.get_logger().info(f"  Speed factors:  {self.SPEED_FACTORS}")
        self.get_logger().info(f"  Trajectory mode: {'CIRCULAR' if self.CIRCULAR else 'LINEAR'}")

    def load_trajectory_params(self) -> dict:
        """
        Load trajectory parameters from YAML.

        Expected YAML structure:
            target_position: [joint_x, joint_y, joint_z]   # meters
            nominal_duration: float                          # seconds
            num_waypoints: int                               # number of intermediate points

        Returns:
            dict: Loaded parameters
        """
        param_file_path = os.path.join(
            get_package_share_directory('tecnobody_workbench_utils'),
            'config',
            'trajectory_params.yaml'
        )

        self.get_logger().info(f"Loading trajectory from: {param_file_path}")
        with open(param_file_path, 'r') as file:
            params = yaml.safe_load(file)

        # Validate required fields
        required = ['target_position', 'nominal_duration']
        for field in required:
            if field not in params:
                self.get_logger().error(f"Missing required field: {field}")
                return None

        # Set defaults
        params.setdefault('num_waypoints', 10)

        self.get_logger().info(f"Trajectory loaded:")
        self.get_logger().info(f"  Target: {params['target_position']}")
        self.get_logger().info(f"  Duration: {params['nominal_duration']} s")
        self.get_logger().info(f"  Waypoints: {params['num_waypoints']}")

        return params

    @staticmethod
    def _smoothstep(alpha: float) -> float:
        """Cubic S-curve easing: maps [0,1]->[0,1] with zero velocity at both ends."""
        return alpha * alpha * (3.0 - 2.0 * alpha)

    def generate_waypoints(self, target: list, duration: float, num_waypoints: int) -> tuple:
        """
        Generate waypoints from start to target and back, with S-curve easing.

        Time is distributed linearly; positions follow a smoothstep profile so
        velocity is zero at the start, target, and return endpoints — giving a
        smooth acceleration and deceleration instead of abrupt motion.

        Args:
            target (list): Target position [x, y, z]
            duration (float): Total duration for one leg (start→target) [s]
            num_waypoints (int): Number of waypoints per leg

        Returns:
            tuple: (cartesian_positions, times)
                - cartesian_positions: List of [x, y, z] waypoints
                - times: List of time_from_start values [s]
        """
        cartesian_positions = []
        times = []

        # Andata (start → target)
        for i in range(num_waypoints):
            alpha = i / (num_waypoints - 1)
            s = self._smoothstep(alpha)  # smooth acceleration/deceleration

            position_a = [
                self.START_POSITION[j] + s * (target[j] - self.START_POSITION[j])
                for j in range(3)
            ]
            cartesian_positions.append(position_a)
            times.append(alpha * duration)

        # Ritorno (target → start)
        for i in range(1, num_waypoints):
            alpha = i / (num_waypoints - 1)
            s = self._smoothstep(alpha)  # smooth acceleration/deceleration

            position_r = [
                (1.0 - s) * target[j] + s * self.START_POSITION[j]
                for j in range(3)
            ]
            cartesian_positions.append(position_r)
            times.append(duration + alpha * duration)

        self.get_logger().debug(f"Generated {len(cartesian_positions)} waypoints")
        return cartesian_positions, times

    def generate_waypoints_circular(self, center: list, duration: float, num_waypoints: int) -> tuple:
        """
        Generate circular waypoints around a center point, returning to START_POSITION.
        
        The radius is determined by the distance from START_POSITION to center.
        Waypoints are generated in a circular motion around the center, with the
        trajectory starting from START_POSITION and returning to it.
        
        Args:
            center (list): Center of the circular trajectory [x, y, z]
            duration (float): Total duration to complete one circle [s]
            num_waypoints (int): Number of intermediate waypoints
        
        Returns:
            tuple: (cartesian_positions, times)
                - cartesian_positions: List of [x, y, z] waypoints on circle
                - times: List of time_from_start values [s]
        """
        cartesian_positions = []
        times = []
        
        # Calculate radius as distance from START_POSITION to center
        dx = center[0] - self.START_POSITION[0]
        dy = center[1] - self.START_POSITION[1]
        dz = center[2] - self.START_POSITION[2]
        radius = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Calculate initial angle from center to START_POSITION (not from START to center)
        if radius > 1e-6:
            initial_angle = math.atan2(self.START_POSITION[1] - center[1], self.START_POSITION[0] - center[0])
        else:
            self.get_logger().warn("Radius is too small for circular trajectory!")
            initial_angle = 0.0
        
        self.get_logger().info(f"Circular trajectory: center={center}, radius={radius:.4f}, initial_angle={initial_angle:.4f}")
        
        # Generate waypoints from 0 to 2π (one complete circle)
        for i in range(num_waypoints):
            alpha = (i / (num_waypoints - 1)) if num_waypoints > 1 else 0
            # S-curve easing on angular progress: slow start/end, fast middle
            s = self._smoothstep(alpha)
            angle_progress = s * 2 * math.pi
            current_angle = initial_angle + angle_progress

            # Position on circle in XY plane (Z remains at center[2])
            x = center[0] + radius * math.cos(current_angle)
            y = center[1] + radius * math.sin(current_angle)
            z = center[2]  # Keep Z constant at center

            position = [x, y, z]
            time_val = alpha * duration
            
            cartesian_positions.append(position)
            times.append(time_val)
        
        self.get_logger().debug(f"Generated {len(cartesian_positions)} circular waypoints")
        return cartesian_positions, times

    def send_trajectory(self, positions: list, times: list) -> bool:
        """
        Build a JointTrajectory and send it to the joint_trajectory_controller
        via the FollowJointTrajectory action.

        Args:
            positions (list): List of [joint_x, joint_y, joint_z] waypoints
            times (list): List of time_from_start values [s] (already speed-scaled)

        Returns:
            bool: True if the trajectory was executed successfully
        """
        # Build trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.JOINT_NAMES

        n = len(positions)
        for idx, (pos, t) in enumerate(zip(positions, times)):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in pos]
            # Only enforce velocity=0 at the boundary points (first and last).
            # Intermediate points get no velocity so the JTC computes natural
            # velocities via finite differences → smooth, trackable profile.
            if idx == 0 or idx == n - 1:
                point.velocities = [0.0] * len(self.JOINT_NAMES)
            secs = int(t)
            nanosecs = int(round((t - secs) * 1e9))
            point.time_from_start = Duration(sec=secs, nanosec=nanosecs)
            trajectory.points.append(point)

        # Build action goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Wait for action server
        self.get_logger().info(f"Waiting for action server: {self.ACTION_SERVER}")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False

        self.get_logger().info(f"Sending trajectory ({len(trajectory.points)} points) ...")
        send_goal_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by controller!")
            return False

        self.get_logger().info("Goal accepted, waiting for result ...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory executed successfully!")
            return True
        else:
            self.get_logger().error(
                f"Trajectory failed (code {result.error_code}): {result.error_string}"
            )
            return False


def main(args=None):
    """
    Main entry point: load trajectory from YAML and send directly to
    joint_trajectory_controller via FollowJointTrajectory action.
    Each repetition scales the time axis by (100 / speed_factor).
    """
    rclpy.init(args=args)

    # Create loader node
    loader = SimpleTrajectoryLoader()

    # Load trajectory parameters from YAML
    params = loader.load_trajectory_params()
    if params is None:
        loader.get_logger().error("Failed to load trajectory parameters")
        rclpy.shutdown()
        return

    # Generate waypoints (choose between linear or circular based on CIRCULAR flag)
    if loader.CIRCULAR:
        # Circular trajectory: target_position is the center of the circle
        positions, times = loader.generate_waypoints_circular(
            center=params['target_position'],
            duration=params['nominal_duration'],
            num_waypoints=params.get('num_waypoints', 10)
        )
    else:
        # Linear trajectory: target_position is the target endpoint
        positions, times = loader.generate_waypoints(
            target=params['target_position'],
            duration=params['nominal_duration'],
            num_waypoints=params.get('num_waypoints', 10)
        )

    # Execute repetitions, each with its own speed factor (scales the time axis)
    for rep_idx, speed_factor in enumerate(loader.SPEED_FACTORS):
        loader.get_logger().info(
            f"Repetition {rep_idx + 1}/{len(loader.SPEED_FACTORS)} "
            f"at speed factor {speed_factor:.1f}%"
        )
        time_scale = 100.0 / speed_factor
        scaled_times = [t * time_scale for t in times]

        success = loader.send_trajectory(positions, scaled_times)
        if not success:
            loader.get_logger().error(f"Repetition {rep_idx + 1} failed, aborting.")
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
