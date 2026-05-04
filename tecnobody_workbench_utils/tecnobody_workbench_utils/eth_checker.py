# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""
EtherCAT Checker Node for Drive State Monitoring and Control.

This module provides a ROS 2 node that monitors the state of EtherCAT-based
Cia402 drives and provides services for motor control, fault handling, and
state queries. It acts as an intermediate layer between high-level position or 
velocity controllers and the low-level state controller managing the EtherCAT hardware.

The node performs the following operations:
1. Subscribes to drive state updates from the state controller
2. Provides services for querying drive states
3. Handles motor startup and shutdown with timeout protection
4. Automatic fault detection and optional auto-reset
5. PLC command publishing for brake control

Typical usage:
    ros2 run tecnobody_workbench_utils ethercat_checker_nodes
    (or include the node in a launch file)
    
Services provided:
    - /ethercat_checker/enable_error_checking: Enable/disable auto fault reset
    - /ethercat_checker/start_motors: Turn on motors with brake control
    - /ethercat_checker/stop_motors: Turn off motors with brake control
    - /ethercat_checker/get_drive_states: Query current drive states (Can Over Ethercat)
    - /ethercat_checker/get_slave_states: Query current ethercat states (PREOP/SAFEOP/OP)
    - /ethercat_checker/request_shutdown: Gracefully shutdown the node
"""

from typing import List
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ethercat_controller_msgs.msg import Cia402DriveStates
from ethercat_controller_msgs.srv import GetModesOfOperation, GetDriveStates
from tecnobody_msgs.srv import GetSlaveStates
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tecnobody_msgs.msg import PlcController
from .boot_hw import get_ethercat_slaves_status

# Global flag for coordinating node shutdown across threads
_shutdown_request = False


class DriverStates:
    """
    Container class for storing the current state of all EtherCAT drives.
    
    Maintains state information for multiple degrees of freedom (DOFs) including
    their operational state, mode of operation, and status words as per CiA402 standard.
    
    Attributes:
        states (dict): Current state of each DOF (e.g., STATE_OPERATION_ENABLED)
        mode_of_operations (dict): Current operation mode of each DOF
        status_words (dict): CiA402 status words providing detailed drive status
        fault_present (bool): Flag indicating if any drive has a fault condition
        drives_on (bool): Flag indicating if drives are powered on
    """
    
    def __init__(self, joint_names):
        """
        Initialize the DriverStates container.
        
        Args:
            joint_names (list): List of DOF names to track (e.g., ['joint_x', 'joint_y', 'joint_z'])
        """
        self.states = {dof: 'STATE_UNDEFINED' for dof in joint_names}
        self.mode_of_operations = {dof: 'MODE_NO_MODE' for dof in joint_names}
        self.status_words = {dof: 0 for dof in joint_names}
        self.fault_present = False
        self.drives_on = False


class EthercatCheckerNode(Node):
    """
    ROS 2 Node for monitoring and controlling EtherCAT drives.
    
    This node acts as a bridge between high-level controllers and the EtherCAT
    state controller. It provides services for motor control, state querying,
    and fault handling while monitoring drive states in real-time.
    
    The node uses multiple callback groups to prevent blocking between
    concurrent service calls and state monitoring.
    
    Attributes:
        feedback (DriverStates): Current state of all monitored drives
        error_auto_reset_enabled (bool): Flag to enable automatic fault recovery
        dof_names (list): Names of degrees of freedom being monitored
    """
    
    def __init__(self):
        """
        Initialize the EtherCAT Checker Node.
        
        Sets up subscriptions, services, clients, and timers for monitoring
        and controlling EtherCAT drives. Uses multiple callback groups to
        prevent blocking between concurrent operations.
        """
        super().__init__('ethercat_checker_node')
        self.get_logger().info('ethercat checker node started')

        # Define callback groups for thread safety and non-blocking operation
        driver_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()
        
        # Subscription to drive status updates from state controller
        self.subscription = self.create_subscription(
            Cia402DriveStates, 
            '/state_controller/drive_states',
            self.drive_states_callback,
            qos_profile=10,
            callback_group=driver_group
        )

	# Publisher for PLC commands (brake control, etc.)
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)  # Adjust as necessary
        self.plc_command_publisher = self.create_publisher(
            PlcController, 
            '/PLC_controller/plc_commands', 
            qos
        )

        # Service Servers
        # Enable/disable automatic error checking and fault recovery
        self.enable_error_check_srv = self.create_service(
            SetBool,
            '/ethercat_checker/enable_error_checking',
            self.enable_error_checking_callback,
            callback_group=service_group
        )

        # Service to start motors with brake control integration
        self.start_motors_service = self.create_service(
            Trigger,
            "/ethercat_checker/start_motors", 
            self.start_motors_callback,
            callback_group=service_group
        )

        # Service to stop motors with brake control integration
        self.stop_motors_service = self.create_service(
            Trigger,
            "/ethercat_checker/stop_motors", 
            self.stop_motors_callback,
            callback_group=service_group
        )

        # Service to query current drive states
        self.get_drives_status_srv = self.create_service(
            GetDriveStates,
            '/ethercat_checker/get_drive_states', 
            self.get_drive_states_callback,
            callback_group=service_group
        )
        
        # Service to query current drive states
        self.get_slaves_status_srv = self.create_service(
            GetSlaveStates,
            '/ethercat_checker/get_slave_states', 
            self.get_slave_states_callback,
            callback_group=service_group
        )

        # Service to request graceful node shutdown
        self.shutdown_service = self.create_service(
            Trigger,
            "/ethercat_checker/request_shutdown", 
            self.shutdown_node_callback,
            callback_group=driver_group
        )

        # Clients for communicating with state controller
        self.motors_service_group = MutuallyExclusiveCallbackGroup()
        self.try_turn_on_client = self.create_client(
            Trigger, 
            '/state_controller/try_turn_on', 
            callback_group=self.motors_service_group
        )
        self.try_turn_off_client = self.create_client(
            Trigger, 
            '/state_controller/try_turn_off', 
            callback_group=self.motors_service_group
        )
        self.reset_faults_client = self.create_client(
            Trigger, 
            '/state_controller/reset_fault', 
            callback_group=self.motors_service_group
        )        

        # Timer for periodic state checking and fault detection
        # 0.004 seconds = 250 Hz to match controller_manager update rate
        self.check_states_timer = self.create_timer(
            0.004, 
            self.check_states,
            callback_group=timer_group
        )

        # Node configuration
        self.error_auto_reset_enabled = False
        self.dof_names = ['joint_x', 'joint_y', 'joint_z']

        # Current drive feedback container
        self.feedback : DriverStates = DriverStates(self.dof_names)

    ########################################################################
    # Service Callbacks
    ########################################################################
    
    def enable_error_checking_callback(self, request, response):
        """
        Handle requests to enable/disable automatic error checking.
        
        When enabled, the node will automatically attempt to reset faults
        detected in the EtherCAT drives.
        
        Args:
            request (SetBool.Request): Request with data field (bool) for enable/disable
            response (SetBool.Response): Response with success and message fields
            
        Returns:
            SetBool.Response: Response indicating success
        """
        self.error_auto_reset_enabled = request.data
        response.success = True
        response.message = f"Error checking {'enabled' if request.data else 'disabled'}."
        return response


    def drive_states_callback(self, msg : Cia402DriveStates):
        """
        Callback for drive state updates from the state controller.
        
        Updates the feedback container with current state information for all
        monitored DOFs. This is called at ~250 Hz to maintain synchronized state.
        
        Args:
            msg (Cia402DriveStates): Message containing current drive states,
                                     modes, and status words for all DOFs
        """
        for dof in self.dof_names:
            try:
                index = msg.dof_names.index(dof) #type: ignore
                self.feedback.states[dof] = msg.drive_states[index] #type: ignore
                self.feedback.mode_of_operations[dof] = msg.modes_of_operation[index] #type: ignore
                self.feedback.status_words[dof] = msg.status_words[index] #type: ignore
                self.feedback.fault_present = msg.fault_present
                self.feedback.drives_on = msg.drives_on
            except ValueError:
                self.get_logger().error(f'DoF {dof} not found in received message')
    

    def get_drive_states_callback(self, request: GetDriveStates.Request, response: GetDriveStates.Response):
        """
        Service callback to query current drive states.
        
        Returns a snapshot of the current state of all monitored drives including
        their operational state, mode, and status words.
        
        Args:
            request (GetDriveStates.Request): Empty request
            response (GetDriveStates.Response): Response populated with current state
            
        Returns:
            GetDriveStates.Response: Response containing current drive states
        """
        response = GetDriveStates.Response()
        response.states : Cia402DriveStates = Cia402DriveStates() #type: ignore
        response.states.dof_names = self.dof_names
        response.states.drive_states = [state for state in self.feedback.states.values()]
        response.states.modes_of_operation = [mode for mode in self.feedback.mode_of_operations.values()]
        response.states.status_words = [status_word for status_word in self.feedback.status_words.values()]
        response.states.fault_present = self.feedback.fault_present
        response.states.drives_on = self.feedback.drives_on
        return response
    
    def get_slave_states_callback(self, request: GetSlaveStates.Request, response: GetSlaveStates.Response):
        """
        Service callback to query current ethercat slave states.
        
        Args:
            request (GetDriveStates.Request): Empty request
            response (GetDriveStates.Response): Response populated with current state and slave names
            
        Returns:
            GetDriveStates.Response: Response containing current drive states
        """
        ethercat_states : List[dict] = get_ethercat_slaves_status(self._logger)
        response = GetSlaveStates.Response()
        response.slave_names =  [info['name'] for info in ethercat_states] #type: ignore
        response.slave_states = [info['state'] for info in ethercat_states] #type: ignore
        return response


    def start_motors_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service callback to start motors with timeout protection and brake control.
        
        This callback performs the following steps:
        1. Calls try_turn_on to enable the drives
        2. Waits for drives to reach operational state
        3. Disables brake via PLC commands
        4. Returns success if completed within timeout, failure otherwise
        
        Args:
            request (Trigger.Request): Empty trigger request
            response (Trigger.Response): Response with success flag and message
            
        Returns:
            Trigger.Response: Response indicating success or timeout failure
        """
        self.try_turn_on()
        timeout_s : float = 5.0
        start_time = time.time()
        response.success = False
        
        while not response.success:
            if self.feedback.drives_on:
                # Disable brake only if not in homing mode (homing disables brake)
                if not any([self.feedback.mode_of_operations[dof] == 'MODE_HOMING' for dof in self.dof_names]):
                    self.publish_plc_command(['PLC_node/brake_disable'], [1])
                response.success = True
    
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Timeout protection
            if elapsed > timeout_s:
                response.success = False
                response.message = f'Motors failed to turn on within the timeout (Mode: {self.feedback.mode_of_operations}))'
                break

            time.sleep(0.1)
        return response
    

    def stop_motors_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Service callback to stop motors with timeout protection and brake control.
        
        This callback performs the following steps:
        1. Engages brake via PLC commands
        2. Calls try_turn_off to disable the drives
        3. Waits for drives to reach disabled state
        4. Returns success if completed within timeout, failure otherwise
        
        Args:
            request (Trigger.Request): Empty trigger request
            response (Trigger.Response): Response with success flag and message
            
        Returns:
            Trigger.Response: Response indicating success or timeout failure
        """
        self.publish_plc_command(['PLC_node/brake_disable'], [0])
        time.sleep(0.2)  # Allow brake to engage before turning off drives
        self.try_turn_off()
        timeout_s : float = 5.0
        start_time = time.time()
        response.success = False
        
        while not response.success:
            if not self.feedback.drives_on:
                response.success = True
    
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Timeout protection
            if elapsed > timeout_s:
                response.success = False
                response.message = f'Motors failed to turn off within the timeout (Mode: {self.feedback.mode_of_operations}))'
                break

            time.sleep(0.1)
        return response


    def reset_fault_callback(self, future):
        """
        Callback for asynchronous fault reset service response.
        
        Handles the result of a fault reset request. If the reset fails,
        attempts to turn off the drives to prevent further issues.
        
        Args:
            future: Async future object containing the service result
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Fault reset successfully')
            else:
                self.get_logger().error('Failed to reset fault - Try to switch off the drives')
                self.try_turn_off()
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
        self.fault_reset_in_execution = False
    

    def shutdown_node_callback(self, request, response):
        """
        Service callback to request graceful node shutdown.
        
        Sets the global shutdown flag which is monitored by the main loop.
        Allows the node to complete current operations before terminating.
        
        Args:
            request: Empty trigger request
            response (Trigger.Response): Response with success flag
            
        Returns:
            Trigger.Response: Response indicating shutdown was requested
        """
        global _shutdown_request
        _shutdown_request = True
        response.success = True
        return response
    

    ########################################################################
    # Communication Methods
    ########################################################################
    
    def publish_plc_command(self, name, value):
        """
        Publish a command to the PLC controller for hardware control.
        
        Sends commands to the PLC controller (typically for brake control).
        The message is published repeatedly for 1 second to ensure reliable delivery.
        
        Args:
            name (list): List of interface names to control (e.g., ['PLC_node/brake_disable'])
            value (list): List of command values corresponding to each interface
        """
        command_msg = PlcController()
        command_msg.interface_names = name
        command_msg.values = value

        # Publish repeatedly for 1 second to ensure delivery
        timeout = time.time() + 1
        while time.time() < timeout:
            self.plc_command_publisher.publish(command_msg)
        

    def check_states(self):
        """
        Periodic check for fault conditions and automatic recovery.
        
        Called at 250 Hz by the check_states_timer. If a fault is detected
        and auto-reset is enabled, attempts to reset the fault condition.
        
        This method runs in a separate callback group to avoid blocking
        other node operations.
        """
        if self.feedback.fault_present and self.error_auto_reset_enabled:
            self.get_logger().error(
                "Detected FAULT state - Trying to resetting faults...", 
                throttle_duration_sec=1
            )
            self.reset_fault()


    def reset_fault(self):
        """
        Asynchronously request fault reset from the state controller.
        
        Returns:
            bool: True if service call was initiated successfully, False otherwise
        """
        if self.reset_faults_client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            _ = self.reset_faults_client.call_async(request)
            return True
        
        self.get_logger().error('Service /state_controller/reset_fault not available')
        return False


    def try_turn_on(self) -> bool:
        """
        Asynchronously request to turn on drives via the state controller.
        
        Returns:
            bool: True if service call was initiated successfully, False otherwise
        """
        if self.try_turn_on_client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            _ = self.try_turn_on_client.call_async(request)
            return True
        
        self.get_logger().error('Service /state_controller/try_turn_on not available')
        return False
            

    def try_turn_off(self):
        """
        Asynchronously request to turn off drives via the state controller.
        
        Returns:
            bool: True if service call was initiated successfully, False otherwise
        """
        self.get_logger().info('Turning off drives...')
        if self.try_turn_off_client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            _ = self.try_turn_off_client.call_async(request)
            return True
        
        self.get_logger().error('Service /state_controller/try_turn_off not available')
        return False


    def get_op_mode_number(self, mode):
        """
        Convert CiA402 operation mode string to numeric value.
        
        Args:
            mode (str): Mode string (e.g., 'MODE_HOMING', 'MODE_CYCLIC_SYNC_POSITION')
            
        Returns:
            int: Numeric mode value (0-10), or 0 if mode not found
        """
        op_mode_dict={
            'MODE_NO_MODE': 0,
            'MODE_PROFILED_POSITION': 1,
            'MODE_PROFILED_VELOCITY': 3,
            'MODE_PROFILED_TORQUE': 4,
            'MODE_HOMING': 6,
            'MODE_INTERPOLATED_POSITION': 7,
            'MODE_CYCLIC_SYNC_POSITION': 8,
            'MODE_CYCLIC_SYNC_VELOCITY': 9,
            'MODE_CYCLIC_SYNC_TORQUE': 10
        }
        return op_mode_dict.get(mode, 0)


def main(args=None):
    """
    Main entry point for the EtherCAT Checker Node.
    
    Initializes ROS 2, sets CPU affinity, creates the node, and runs the
    main event loop. The loop continues until a shutdown request is received
    via the shutdown service or a keyboard interrupt occurs.
    
    Args:
        args: Command-line arguments passed to rclpy.init()
    """
    rclpy.init(args=args)

    # Set CPU affinity to core 6 for deterministic timing
    import os
    os.sched_setaffinity(0, {6})

    node = EthercatCheckerNode()

    # Use multi-threaded executor with 4 threads for concurrent callback handling
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        # Main loop continues until shutdown is requested
        while not _shutdown_request:
            executor.spin_once()
    except KeyboardInterrupt:
        import signal as _signal
        _signal.signal(_signal.SIGINT, _signal.SIG_IGN)
        try:
            node.get_logger().info('Keyboard interrupt, shutting down.\n')
        except Exception:
            print('[ethercat_checker_node] Keyboard interrupt, shutting down.')
    finally:
        # Cleanup: shutdown executor and destroy node
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
