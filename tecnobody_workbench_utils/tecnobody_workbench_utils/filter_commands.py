# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""
Command Filtering Node for Smooth Velocity and Speed Override Transitions.

This module provides a ROS 2 node that applies zero-derivative low-pass filtering
to command signals destined for robot controllers. It handles two types of filtered
commands:

1. Forward Velocity Commands: Smooth velocity references sent to the forward velocity
   controller for jogging operations on individual axes.

2. Speed Override Factor: Smooth speed scaling factors sent to the joint trajectory
   controller to modulate the execution speed of pre-planned trajectories.

The filtering ensures smooth transitions during both start and stop operations,
preventing abrupt changes that could cause jerky motion or controller instability.

The node supports:
- Soft movement start/stop with configurable time constants
- Individual axis jogging with smooth ramping
- Speed override with smooth transitions
- Configurable sampling rates and desam pling for different subscribers

Typical usage:
    ros2 run tecnobody_workbennch_utils filter_commands.py
    (or include the node in a launch file)
    
Key features:
- Zero-derivative low-pass filtering for smooth derivatives
- Desam pling support for lower-bandwidth subscribers
- Soft start/stop capabilities to prevent step changes
- Separate handling of jog and speed override commands
"""

import os
# Set CPU affinity to core 6 for deterministic timing
os.sched_setaffinity(0, {6})

import sys
from typing import List
from copy import deepcopy

# ROS 2 imports
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rclpy.task import Future
from control_msgs.msg import SpeedScalingFactor
from tecnobody_msgs.srv import SoftMovement
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

# Enhanced error reporting
from rich.traceback import install
install(show_locals=True)


class ZeroDerivativeLowPass:
    """
    Zero-derivative low-pass filter for smooth signal filtering.
    
    Implements a second-order low-pass filter with zero derivative initialization.
    This ensures smooth transient responses without sudden changes in velocity.
    
    The filter uses the bilinear transformation of a continuous-time Butterworth
    low-pass filter to discrete-time coefficients.
    
    Attributes:
        tau (float): Time constant (seconds) - controls filter cutoff frequency
        dt (float): Sampling period (seconds)
        a1, a2 (float): Feedback coefficients for the filter
        b0, b1, b2 (float): Feedforward coefficients for the filter
        x1, x2 (float): Previous input samples
        y1, y2 (float): Previous output samples
        initialized (bool): Flag indicating if filter has been initialized
    """
    
    def __init__(self, tau: float, dt: float):
        """
        Initialize the zero-derivative low-pass filter.
        
        Computes filter coefficients using bilinear transformation of a 
        continuous Butterworth filter with time constant tau.
        
        Args:
            tau (float): Time constant (seconds). Larger values = more filtering
            dt (float): Sampling period (seconds). Inverse of sampling frequency
        """
        # tau = time constant (seconds)
        # dt  = sampling period (seconds)
        w = 1 / tau  # Natural frequency
        k = 2 / dt   # Bilinear transformation parameter

        # Compute filter coefficients using bilinear transformation
        a0 = k*k + 2*w*k + w*w
        self.a1 = (2*w*w - 2*k*k) / a0
        self.a2 = (k*k - 2*w*k + w*w) / a0
        self.b0 = w*w / a0
        self.b1 = 2*w*w / a0
        self.b2 = w*w / a0

        # State variables for difference equation
        self.y1 = 0  # Previous output sample
        self.y2 = 0  # Two samples ago output
        self.x1 = 0  # Previous input sample
        self.x2 = 0  # Two samples ago input
        self.initialized = False


    def update(self, x: float) -> float:
        """
        Apply filter to new input sample and return filtered output.
        
        Implements the second-order difference equation:
        y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        
        On first call, initializes the filter state with zero-derivative condition:
        y = x and y' = 0 to ensure smooth start without transients.
        
        Args:
            x (float): Current input sample
            
        Returns:
            float: Filtered output sample
        """
        if not self.initialized:
            # Force initial condition: y = x, y' = 0 (zero derivative)
            # This ensures smooth startup without introducing step transients
            self.y1 = x
            self.y2 = x
            self.x1 = x
            self.x2 = x
            self.initialized = True
            return x

        # Apply difference equation of the second-order filter
        y = (self.b0*x + self.b1*self.x1 + self.b2*self.x2
             - self.a1*self.y1 - self.a2*self.y2)

        # Shift state variables for next iteration
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y

        return y


#####################################################
# Main Filter and Node Classes
#####################################################

class FilterMath():
    """
    Multi-dimensional zero-derivative low-pass filter container.
    
    Manages an array of independent low-pass filters for filtering multi-axis
    command signals. Supports dynamic filter reconfiguration for adapting to
    changing time constants.
    
    Attributes:
        sample_rate (float): Sampling frequency in Hz
        time_constant (float): Filter time constant in seconds
        low_pass_filter (list): Array of ZeroDerivativeLowPass filters
        dimension (int): Number of axes/signals being filtered
        last_received_command (list): Previous command values for each axis
    """
    
    def __init__(self, dimension: int, default_rate: float = 250.0, default_time_constant: float = 1.0):
        """
        Initialize the multi-dimensional filter.
        
        Args:
            dimension (int): Number of signals to filter (e.g., 3 for XYZ axes)
            default_rate (float): Sampling rate in Hz (default: 250 Hz)
            default_time_constant (float): Filter time constant in seconds (default: 1.0 s)
        """

        self.sample_rate : float = default_rate
        self.time_constant : float = default_time_constant

	# Create independent filter for each dimension
        self.low_pass_filter : List[ZeroDerivativeLowPass] = list() #type:ignore
        for i in range(dimension):
            self.low_pass_filter.append(ZeroDerivativeLowPass(tau=self.time_constant, dt=1.0/self.sample_rate))
        self.dimension : int = dimension
        self.last_received_command : List[float] = [0]* self.dimension

    def create_filter(self, sample_rate: float, time_constant: float):
        """
        Recreate filters with new sample rate and time constant.
        
        Used to dynamically adjust filter response when receiving soft movement
        commands with different time constants. Destroys old filters and creates
        new ones with updated parameters.
        
        Args:
            sample_rate (float): New sampling rate in Hz
            time_constant (float): New time constant in seconds
        """
        # Clean up old filters
        if self.low_pass_filter is not None:
            for filter in self.low_pass_filter[:]:
                del filter
            del self.low_pass_filter
        
        # Create new filters with updated parameters
        self.low_pass_filter = list() #type: ignore
        for i in range(self.dimension):
            self.low_pass_filter.append(ZeroDerivativeLowPass(tau=self.time_constant, dt=1.0/self.sample_rate))

        self.sample_rate = sample_rate
        self.time_constant = time_constant


    def update(self, command: List[float]) -> List[float]:
        """
        Apply filtering to multi-dimensional command vector.
        
        Applies each filter to its corresponding command component independently.
        
        Args:
            command (list): Command values for each axis
            
        Returns:
            list: Filtered command values for each axis
        """
        filtered_command : List[float] = [0.0]*self.dimension
        for i in range(self.dimension):
            filtered_command[i] = self.low_pass_filter[i].update(command[i])
        return filtered_command

    

class FilterCommands(Node):
    """
    ROS 2 Node for filtering velocity and speed override commands.
    
    This node receives command references (velocity commands for jogging or
    speed override factors for trajectory modulation) and applies zero-derivative
    low-pass filtering to ensure smooth transitions. It manages two separate
    filtering pipelines:
    
    1. Forward Velocity Filtering: For smooth jog operations on individual axes
    2. Speed Override Filtering: For smooth speed scaling of pre-programmed motions
    
    The node supports soft start/stop operations where the filter time constant
    can be specified per command to control the ramp rate.
    
    Publishers:
        - /{forward_velocity_controller_name}/commands: Filtered velocity commands
        - /{joint_trajectory_controller_name}/speed_scaling_input: Filtered speed override
        - /tecnobody_workbench_utils/speed_scaling_input: Desam pled speed override
    
    Services:
        - /tecnobody_workbench_utils/soft_movement_start: Start soft filtering
        - /tecnobody_workbench_utils/soft_movement_stop: Stop soft filtering
        - /tecnobody_workbench_utils/reset_speed_ovr: Reset speed override to 1.0
    
    Attributes:
        sample_rate (int): Update frequency in Hz
        desampled_rate (int): Reduced rate for desam pled publisher
        fwd_commands (FilterMath): Filter container for velocity commands
        ovr_command (FilterMath): Filter container for speed override
        running_behaviour (str): Current operating mode ('idle', 'jog', 'speed_ovr')
    """

    def __init__(self):
        """
        Initialize the FilterCommands node.
        
        Sets up parameters, publishers, services, and filtering objects.
        Declares ROS 2 parameters for configuration and creates all necessary
        publishers and service servers.
        """
        super().__init__("filter_commands_node") # type: ignore

        # Declare ROS 2 parameters with default values
        self.declare_parameter('default_sample_rate', 250)
        self.declare_parameter('desampled_rate', 25)
        self.declare_parameter('forward_velocity_controller_name', 'forward_velocity_controller')
        self.declare_parameter('joint_trajectory_controller_name', 'joint_trajectory_controller')
        self.declare_parameter('joint_names', ['joint_x', 'joint_y', 'joint_z'])

        # Retrieve parameters from ROS 2 parameter server
        self.sample_rate : int = self.get_parameter('default_sample_rate').get_parameter_value().integer_value
        self.desampled_rate : int = self.get_parameter('desampled_rate').get_parameter_value().integer_value
        self.forward_velocity_controller_name : str = self.get_parameter('forward_velocity_controller_name').get_parameter_value().string_value
        self.joint_trajectory_controller_name : str = self.get_parameter('joint_trajectory_controller_name').get_parameter_value().string_value
        self.joint_names : List[str] = self.get_parameter('joint_names').get_parameter_value().string_array_value #type: ignore

        ##### Publishers #####
        # Publisher for filtered forward velocity commands to the velocity controller
        self.forward_velocity_controller_commands_publisher : Publisher = self.create_publisher(
            Float64MultiArray,
            f'/{self.forward_velocity_controller_name}/commands',
            10,
        )
        
        # Publisher for filtered speed scaling factor to trajectory controller
        # Uses TRANSIENT_LOCAL durability to preserve last message for late subscribers
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.joint_trajectory_controller_scaling_publisher : Publisher = self.create_publisher(
            SpeedScalingFactor,
            f'/{self.joint_trajectory_controller_name}/speed_scaling_input', 
            qos_profile
        )

        # Desam pled version of speed scaling for subscribers with lower bandwidth requirements
        self.desampled_joint_trajectory_controller_scaling_publisher : Publisher = self.create_publisher(
            SpeedScalingFactor,
            f'/tecnobody_workbench_utils/speed_scaling_input', 
            qos_profile
        )

        ##### Services #####
        # Service to start soft movement with configurable time constant
        self.soft_movement_start_server : Service = self.create_service(
            SoftMovement,
            "/tecnobody_workbench_utils/soft_movement_start",
            self.soft_command_start
        )

        # Service to stop soft movement and return commands to neutral
        self.soft_movement_stop_server_trajectory_server : Service  = self.create_service(
            Trigger,
            "/tecnobody_workbench_utils/soft_movement_stop",
            self.soft_movement_stop
        )

        # Service to reset speed override factor to 1.0 (nominal speed)
        self.soft_movement_stop_server_trajectory_server : Service  = self.create_service(
            Trigger,
            "/tecnobody_workbench_utils/reset_speed_ovr",
            self.reset_speed_ovr
        )

        ##### Filter Initialization #####
        self.update_rate : int = 250
        self.fwd_commands : FilterMath = FilterMath(dimension=len(self.joint_names), default_rate=self.update_rate, default_time_constant=0.2)
        
        self.fwd_filtered_commands : Float64MultiArray = Float64MultiArray()
        self.fwd_filtered_commands.data = [0]* len(self.joint_names)
        self.fwd_last_commands : List[float] = [0]* len(self.joint_names)

        self.ovr_command : FilterMath = FilterMath(dimension=1, default_rate=self.update_rate, default_time_constant=0.2)
        self.ovr_filtered_command : SpeedScalingFactor = SpeedScalingFactor()
        self.ovr_filtered_command.factor = 1.0
        self.ovr_last_command : float = 1.0

        ##### State and Timing #####
        # Current operating mode: 'idle', 'jog', or 'speed_ovr'
        self.running_behaviour : str = 'idle'

        self.ready : bool = False        
        self.get_parameter_future : Future = None #type: ignore

        # Desam pling configuration
        self.desampling_window = int(self.sample_rate/self.desampled_rate)
        self.desampling_cnt = 0
        
        # Timer for periodic command publishing at sample_rate frequency
        self.publisher_timer : Timer = self.create_timer( 1.0 / self.sample_rate, self.publish_command)

        # Log configuration parameters
        self.get_logger().info(f'default_sample_rate: {self.sample_rate}')
        self.get_logger().info(f'desampled_rate: {self.desampled_rate}')
        self.get_logger().info(f'forward_velocity_controller_name: {self.forward_velocity_controller_name}')
        self.get_logger().info(f'joint_trajectory_controller_name: {self.joint_trajectory_controller_name}')
        self.get_logger().info(f'joint_names: {self.joint_names}')


    ########################################################################
    # Service Callbacks
    ########################################################################

    def reset_speed_ovr(self, request, response):
        """
        Service callback to reset speed override factor to nominal (1.0).
        
        Immediately sets the speed override factor to 1.0, which corresponds to
        nominal trajectory execution speed (no scaling).
        
        Args:
            request: Empty trigger request
            response (Trigger.Response): Response with success flag and message
            
        Returns:
            Trigger.Response: Response indicating reset was successful
        """
        self.ovr_last_command=1.0
        response.success = True  # You may want to set this based on your logic
        response.message = "Speed override reset."
        return response


    ########################################################################
    # Main Command Publishing Loop
    ########################################################################

    def publish_command(self):
        """
        Periodic callback to publish filtered commands at sample_rate frequency.
        
        This method is called at the configured sample rate (default 250 Hz) and:
        1. Updates and publishes filtered speed override factor to trajectory controller
        2. Publishes desam pled speed override at reduced rate for lower-bandwidth subscribers
        3. Updates and publishes filtered forward velocity commands
        
        The desam pling mechanism reduces the published rate by a configurable factor
        while maintaining the high-frequency filter update for smooth response.
        """
        # Increment desam pling counter and wrap around
        self.desampling_cnt = self.desampling_cnt + 1 if self.desampling_cnt < self.desampling_window else 0

        # Update and publish speed override factor
        ovr_filtered_data = self.ovr_command.update([self.ovr_last_command])
        self.ovr_filtered_command.factor = ovr_filtered_data[0]
        
        # Always publish to trajectory controller at full rate
        self.joint_trajectory_controller_scaling_publisher.publish(self.ovr_filtered_command)
        
        # Publish desam pled version to low-bandwidth subscribers
        if self.desampling_cnt == self.desampling_window:
            self.desampled_joint_trajectory_controller_scaling_publisher.publish(self.ovr_filtered_command)

        # Update and publish forward velocity commands
        fwd_filtered_data = self.fwd_commands.update(self.fwd_last_commands)
        self.fwd_filtered_commands.data = deepcopy(fwd_filtered_data)
        self.forward_velocity_controller_commands_publisher.publish(self.fwd_filtered_commands)


    ########################################################################
    # Soft Movement Start/Stop
    ########################################################################

    def soft_command_start(self, request: SoftMovement.Request, response: SoftMovement.Response) -> SoftMovement.Response:
        """
        Service callback to start a soft-filtered command sequence.
        
        Supports two types of soft movements:
        1. 'speed_ovr': Ramp speed override factor smoothly to specified value
        2. 'jog': Ramp velocity command on single axis smoothly to specified value
        
        Each command type specifies its own time constant to control ramp duration.
        Prevents multiple concurrent movements and validates parameters.
        
        Args:
            request (SoftMovement.Request): Request containing:
                - target: 'speed_ovr' or 'jog'
                - amplitude: Target value for command
                - time_constant: Ramp duration in seconds
                - jog_joint_idx: For jog commands, axis index to ramp
            response (SoftMovement.Response): Response with success flag and message
            
        Returns:
            SoftMovement.Response: Response indicating success or error condition
        """
        # Check if another movement is already in progress
        if self.running_behaviour != 'idle':
            if self.running_behaviour != request.data.target:
                response.success = False
                response.message = f"Error: cannot process '{request.data.target}' while processing '{self.running_behaviour}'."
                return response
            else: 
                response.success = True
                response.message = f"Warning: already processing '{self.running_behaviour}' request. Skip"
                return response
            
        time_constant = request.data.time_constant
        self.running_behaviour = request.data.target
        
        ##### Speed Override Ramp #####
        if self.running_behaviour == 'speed_ovr':
            self.get_logger().info(f"************************ Soft Speed Override Start called ************************")

            # Verify that trajectory controller is subscribed
            if self.joint_trajectory_controller_scaling_publisher.get_subscription_count()==0:
                response.success = False
                response.message = f"Error: no subscribers to {self.joint_trajectory_controller_scaling_publisher.topic_name}"
                return response
            
            # Recreate filter with new time constant if different from current
            if time_constant != self.ovr_command.time_constant:
                self.ovr_command.create_filter(sample_rate=self.sample_rate, time_constant=time_constant)
            
            # Set target speed override value (will ramp smoothly due to filtering)
            self.ovr_last_command = request.data.amplitude
            
            response.success = True
            response.message = "Soft Command Started successfully."
        
        ##### Jog Ramp #####
        elif self.running_behaviour == 'jog':
            self.get_logger().info(f"************************ Soft Jog Start called ************************")

            # Verify that velocity controller is subscribed
            if self.forward_velocity_controller_commands_publisher.get_subscription_count()==0:
                response.success = False
                response.message = f"Error: no subscribers to {self.forward_velocity_controller_commands_publisher.topic_name}"
                return response
            
            # Validate jog axis index
            if request.data.jog_joint_idx < 0 or request.data.jog_joint_idx > len(self.joint_names):
                response.success = False
                response.message = f"Error: joint index {request.data.jog_joint_idx} out of range [0, {len(self.joint_names)-1}]"
                return response
            
            # Recreate filter with new time constant if different from current
            if time_constant != self.fwd_commands.time_constant:
                self.fwd_commands.create_filter(sample_rate=self.sample_rate, time_constant=time_constant)
            
            # Initialize all velocity commands to zero
            self.fwd_last_commands = [0.0] * self.fwd_commands.dimension
            # Set target velocity on specified axis (will ramp smoothly due to filtering)
            self.fwd_last_commands[request.data.jog_joint_idx] = request.data.amplitude
            
            response.success = True
            response.message = "Soft Command Started successfully."
        
        return response
        

    def soft_movement_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service callback to stop soft movement and ramp commands back to neutral.
        
        Depending on the current operating mode:
        - 'speed_ovr': Ramps speed override back to 1.0 (nominal speed)
        - 'jog': Ramps all velocity commands back to 0.0 (no motion)
        - 'idle': Returns warning that no movement is active
        
        The filters ensure smooth ramping back to neutral over their configured
        time constant period.
        
        Args:
            request (Trigger.Request): Empty trigger request
            response (Trigger.Response): Response with success flag and message
            
        Returns:
            Trigger.Response: Response indicating stop was processed
        """
        self.get_logger().info(f"************************ Soft Movement Stop called ************************")
        
        if self.running_behaviour == 'idle':
            response.message = "Warning: NO active process to stop. Skip"
        elif self.running_behaviour == 'speed_ovr':
            # Ramp speed override back to 1.0 (nominal)
            self.ovr_last_command = 1.0
            response.message = "Override set to 1"
        else:  # 'jog'
            # Ramp all velocity commands back to 0.0
            self.fwd_last_commands = [0.0] * self.fwd_commands.dimension
            response.message = "Command set to 0"
            
        # Return to idle state
        self.running_behaviour = 'idle'
        response.success = True
        return response


def main():
    """
    Main entry point for the filter commands node.
    
    Initializes ROS 2, creates the FilterCommands node, and runs the main
    spin loop. Handles keyboard interrupt for graceful shutdown.
    
    Args:
        None
    """
    rclpy.init(args=sys.argv)

    # Create and run the filter commands node
    fc = FilterCommands()
    try:
        rclpy.spin(fc)
    except KeyboardInterrupt:
        fc.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    # Cleanup
    fc.destroy_node()


if __name__ == '__main__':
    main()
