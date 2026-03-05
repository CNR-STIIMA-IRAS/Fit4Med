"""
Simplified trajectory loader that sends multi-repetition exercises to gui_trajectory_manager.

This node reads a simple YAML file containing:
  - target_position: final cartesian position [x, y, z]
  - nominal_duration: time to reach target [seconds]
  - num_repetitions: number of exercise repetitions
  - speed_factors: array of speed overrides per repetition [%]

The node then:
  1. Generates intermediate waypoints via linear interpolation
  2. Calls gui_trajectory_manager.set_exercise() service
  3. gui_trajectory_manager handles all complexity:
     - Cubic spline interpolation
     - Speed override scaling (velocity/acceleration modification + time dilation)
     - Multi-repetition orchestration
     - Progress monitoring
     - Pause/resume handling
"""

import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tecnobody_msgs.srv import SetExercise
from tecnobody_msgs.msg import CartesianPoint
import math

class SimpleTrajectoryLoader(Node):
    """
    Load trajectory from YAML and submit as multi-rep exercise to gui_trajectory_manager.
    
    YAML Format (trajectory_params.yaml):
        target_position: [0.5, 0.2, 0.1]    # Final cartesian position [m]
        nominal_duration: 5.0                # Time to reach target [s]
        num_waypoints: 10                    # Number of intermediate points
    
    Python Configuration (beginning of file):
        NUM_REPETITIONS = 3                  # Number of exercise repetitions
        SPEED_FACTORS = [80, 100, 120]      # Speed override per repetition [%]
    """
    
    # ========== Configuration Parameters ==========
    NUM_REPETITIONS = 3                     # Number of exercise repetitions
    SPEED_FACTORS = [50.0, 70.0, 100.0]   # Speed override per repetition [%]
    START_POSITION = [0.0, 0.0, 0.0]       # Starting position (assumed)
    CIRCULAR = True                        # If True, use circular trajectory; if False, use linear trajectory
    
    def __init__(self):
        super().__init__('simple_trajectory_loader')
        
        self.get_logger().info(f"Trajectory loader initialized")
        self.get_logger().info(f"  Repetitions: {self.NUM_REPETITIONS}")
        self.get_logger().info(f"  Speed factors: {self.SPEED_FACTORS}")
        self.get_logger().info(f"  Trajectory mode: {'CIRCULAR' if self.CIRCULAR else 'LINEAR'}")

    def load_trajectory_params(self) -> dict:
        """
        Load trajectory parameters from YAML.
        
        Expected YAML structure:
            target_position: [x, y, z]      # meters
            nominal_duration: float          # seconds
            num_waypoints: int               # number of intermediate points
        
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

    def generate_waypoints(self, target: list, duration: float, num_waypoints: int) -> tuple:
        """
        Generate linearly interpolated waypoints from start to target.
        
        Uses linear interpolation for simplicity. gui_trajectory_manager will then
        apply cubic spline interpolation for smooth motion.
        
        Args:
            target (list): Target position [x, y, z]
            duration (float): Total duration to reach target [s]
            num_waypoints (int): Number of intermediate waypoints
        
        Returns:
            tuple: (cartesian_positions, times)
                - cartesian_positions: List of [x, y, z] waypoints
                - times: List of time_from_start values [s]
        """
        cartesian_positions = []
        times = []
        
        # Andata
        for i in range(num_waypoints):
            # Normalized progress (0 to 1)
            alpha = i / (num_waypoints - 1)
            
            # Linear interpolation from START to target
            position_a = [
                self.START_POSITION[j] + alpha * (target[j] - self.START_POSITION[j])
                for j in range(3)
            ]
            
            # Time at this waypoint (0 to duration)
            time_val_a = alpha * duration
            
            cartesian_positions.append(position_a)
            times.append(time_val_a)
        
        # Ritorno
        for i in range(1, num_waypoints):
            # Normalized progress (0 to 1)
            alpha = i / (num_waypoints - 1)
            
            # Linear interpolation from target to START
            position_r = [
                (1 - alpha) * target[j] + alpha * self.START_POSITION[j]
                for j in range(3)
            ]
            
            # Time at this waypoint (0 to duration)
            time_val_r = duration + alpha * duration
            
            cartesian_positions.append(position_r)
            times.append(time_val_r)

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
            # Angle progress from 0 to 2π
            angle_progress = (i / (num_waypoints - 1)) * 2 * math.pi if num_waypoints > 1 else 0
            current_angle = initial_angle + angle_progress
            
            # Position on circle in XY plane (Z remains at center[2])
            x = center[0] + radius * math.cos(current_angle)
            y = center[1] + radius * math.sin(current_angle)
            z = center[2]  # Keep Z constant at center
            
            position = [x, y, z]
            time_val = (i / (num_waypoints - 1)) * duration if num_waypoints > 1 else 0
            
            cartesian_positions.append(position)
            times.append(time_val)
        
        self.get_logger().debug(f"Generated {len(cartesian_positions)} circular waypoints")
        return cartesian_positions, times

    def create_cartesian_trajectory_points(self, positions: list, times: list):
        """
        Convert positions and times into CartesianPoint messages.
        
        Args:
            positions (list): List of [x, y, z] cartesian positions
            times (list): List of time_from_start values
        
        Returns:
            list: CartesianPoint objects
        """
        points = []
        for pos, t in zip(positions, times):
            point = CartesianPoint()
            point.point = pos  # [x, y, z]
            point.time_from_start = t
            points.append(point)
        
        return points

    def send_exercise(self, cartesian_points: list) -> bool:
        """
        Send exercise request to gui_trajectory_manager via SetExercise service.
        
        The gui_trajectory_manager will:
          1. Interpolate waypoints with cubic splines
          2. Resample @ 10 Hz for controller compatibility
          3. Apply speed override scaling for each repetition
          4. Execute N repetitions with different speeds
          5. Monitor progress and handle pause/resume
        
        Args:
            cartesian_points (list): CartesianPoint objects
        
        Returns:
            bool: True if exercise started successfully
        """
        # Create SetExercise service client
        client = self.create_client(SetExercise, '/tecnobody_workbench_utils/set_exercise')
        
        # Wait for service
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_exercise service not available!")
            return False
        
        # Build request
        request = SetExercise.Request()
        request.cartesian_positions = cartesian_points
        
        # Repetition durations: Each repetition has the same duration (nominal_duration)
        # gui_trajectory_manager will scale timing internally based on speed_factors
        nominal_duration = cartesian_points[-1].time_from_start
        request.repetition_durations = [nominal_duration] * len(self.SPEED_FACTORS)
        
        # Speed factors [%] - will be applied by gui_trajectory_manager
        request.repetition_ovrs = self.SPEED_FACTORS
        
        self.get_logger().info(f"Sending exercise to gui_trajectory_manager:")
        self.get_logger().info(f"  Waypoints: {len(cartesian_points)}")
        self.get_logger().info(f"  Repetitions: {len(request.repetition_durations)}")
        self.get_logger().info(f"  Speed factors: {request.repetition_ovrs}")
        
        # Send request
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info("Exercise sent successfully!")
            return True
        else:
            self.get_logger().error("Failed to send exercise")
            return False


def main(args=None):
    """
    Main entry point: load trajectory from YAML and send to gui_trajectory_manager.
    """
    rclpy.init(args=args)

    # Create loader node
    loader = SimpleTrajectoryLoader()

    # Load trajectory parameters from YAML
    params = loader.load_trajectory_params()
    if params is None:
        loader.get_logger().error("Failed to load trajectory parameters")
        return

    # Generate waypoints (choose between linear or circular based on CIRCULAR flag)
    if loader.CIRCULAR:
        # Circular trajectory: target_position is the center of the circle
        cartesian_positions, times = loader.generate_waypoints_circular(
            center=params['target_position'],
            duration=params['nominal_duration'],
            num_waypoints=params.get('num_waypoints', 10)
        )
    else:
        # Linear trajectory: target_position is the target endpoint
        cartesian_positions, times = loader.generate_waypoints(
            target=params['target_position'],
            duration=params['nominal_duration'],
            num_waypoints=params.get('num_waypoints', 10)
        )

    # Convert to CartesianPoint messages
    cartesian_points = loader.create_cartesian_trajectory_points(cartesian_positions, times)

    # Send exercise to gui_trajectory_manager
    success = loader.send_exercise(cartesian_points)

    if success:
        loader.get_logger().info("Exercise execution started. Monitoring progress...")
        # Keep node alive while exercise runs (optional)
        # In a real scenario, you might want to subscribe to exercise_progress
        # or wait for exercise_finished callback
    else:
        loader.get_logger().error("Failed to send exercise")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
