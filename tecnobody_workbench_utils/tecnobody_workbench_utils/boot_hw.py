# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""
Boot Hardware Node for EtherCAT-based Robot Homing and Mode Switching.

This module provides a ROS 2 node that handles the initialization and homing of 
EtherCAT-based drives (Cia402 protocol). It manages drive state transitions, mode 
switching, and fault recovery through service calls to the state controller.

The node performs the following operations in sequence:
1. Checks EtherCAT slaves status
2. Turns on drives
3. Switches drives to homing mode
4. Performs homing procedure
5. Switches to target cyclic mode
6. Handles faults and timeouts

Typical usage:
    ros2 run tecnobody_workbench_utils boot_hw [target_mode]
    (or include the node in a launch file)
    
where target_mode is one of the MODE_* constants defined in "MODE_OF_OPERATION_MAP" 
(default: MODE_CYCLIC_SYNC_POSITION)
"""

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ethercat_controller_msgs.srv import SwitchDriveModeOfOperation
from ethercat_controller_msgs.msg import Cia402DriveStates
import time
import subprocess
import sys

# Mapping of mode string names to their corresponding numeric values (CiA402 standard)
MODE_OF_OPERATION_MAP={
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


def get_op_mode_number(mode_string):
    """
    Convert mode string to numeric mode value.
    
    Args:
        mode_string (str): String representation of the mode (e.g., 'MODE_HOMING')
        
    Returns:
        int: Numeric mode value, or 0 (MODE_NO_MODE) if mode_string not found
    """
    return MODE_OF_OPERATION_MAP.get(mode_string, 0)


def get_op_mode_string(mode_number):
    """
    Convert numeric mode value to mode string.
    
    Args:
        mode_number (int): Numeric mode value (0-10)
        
    Returns:
        str: String representation of the mode, or 'MODE_NO_MODE' if not found
    """
    if mode_number in MODE_OF_OPERATION_MAP.values():
        return list(MODE_OF_OPERATION_MAP.keys())[list(MODE_OF_OPERATION_MAP.values()).index(mode_number)]
    return MODE_OF_OPERATION_MAP['MODE_NO_MODE']

def get_ethercat_slaves_status(logger) -> List[dict]:
    """
    Query the EtherCAT master for the status of slave devices.
    
    Parses the output of the 'ethercat slaves' command to extract slave names
    and their current state (INIT, PRE-OP, SAFE-OP, OP, etc.).
    
    Returns:
        dict: Dictionary mapping slave indices to {'name': str, 'state': str}
        Empty dict if 'ethercat' command is not available
        
    Raises:
        No exceptions are raised; errors are logged instead
    """
    try:
        result = subprocess.run(['ethercat', 'slaves'],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True,
                                check=True)
        lines = result.stdout.strip().splitlines()
        
        # Extract lines for Master1 section
        lines_master1 = []
        section_master1_found = False
        for line in lines:
            section_master1_found = section_master1_found or 'Master1' in line
            if section_master1_found and not 'Master1' in line:
                lines_master1.append(line)

        # Parse slave information
        eth_states : List[dict] = [dict()] * len(lines_master1)
        for iSlave in range(len(lines_master1)):
            parts = lines_master1[iSlave].split()
            if len(parts) > 2:
                slave_name, state = f"{parts[4]}-{parts[1]}", parts[2]
                eth_states[iSlave] = {'name': slave_name, 'state': state}
            else:
                logger.warn(f"Formato non valido per la linea dello slave {iSlave}: {lines_master1[iSlave]}")
        return eth_states

    except FileNotFoundError:
        logger.error("'ethercat' command not found. Verify that EtherCAT master is active.")
        return []

class HomingNode(Node):
    """
    ROS 2 Node for managing EtherCAT drive homing and mode switching.
    
    This node coordinates the boot sequence of EtherCAT-based Cia402 drives,
    including state transitions, fault handling, and mode switching to achieve
    a target cyclic operation mode.
    
    Attributes:
        target_op_mode (str): Target operation mode after homing completion
        ethercat_ready (bool): Flag indicating if all EtherCAT slaves are in OP state
        dof_names (list): Names of degrees of freedom (e.g., joint_x, joint_y, joint_z)
        state (dict): Current state of each DOF
        mode (dict): Current operation mode of each DOF
        status_words (dict): CiA402 status words for each DOF
        homing_process_started (bool): Flag indicating if homing has been initiated
        homing_process_attained (bool): Flag indicating if homing has completed successfully
    """

    def __init__(self, target_op_mode):
        """
        Initialize the HomingNode.
        
        Args:
            target_op_mode (str): Target cyclic mode to switch to after homing
        """
        super().__init__('boot_hw_node')
        self.get_logger().info('Booting Cia402 drives and performing homing...')
        
        # EtherCAT status tracking
        self.ethercat_ready = False
        self.ethercat_timer = self.create_timer(5.0, self.check_ethercat_slaves)
        
        # Define callback group for topic subscriptions
        topic_group = MutuallyExclusiveCallbackGroup()
        # Subscription to drive status updates
        self.subscription = self.create_subscription(Cia402DriveStates, '/state_controller/drive_states', self.handle_drive_status, qos_profile=10, callback_group=topic_group)
        self.dof_names = ['joint_x', 'joint_y', 'joint_z']
        self.state = {dof: 'STATE_UNDEFINED' for dof in self.dof_names}
        self.mode = {dof: 'MODE_NO_MODE' for dof in self.dof_names}
        self.status_words = {dof: 0 for dof in self.dof_names}
        self.slave_indices = [0, 1, 2]  # Positions of drives in the EtherCAT slaves list
        
        # Homing and mode switching state tracking
        self.cyclic_modes = {'MODE_CYCLIC_SYNC_POSITION', 'MODE_CYCLIC_SYNC_VELOCITY', 'MODE_CYCLIC_SYNC_TORQUE'}
        self.homing_process_started = False
        self.homing_process_starting_time = None
        self.homing_process_attained = False
        self.target_cyclic_mode = target_op_mode
        
        # Execution flags to prevent concurrent service calls
        self.try_turn_off_in_execution = False
        self.try_turn_on_in_execution = False
        self.reset_faults_in_execution = False
        self.switch_mode_in_execution = {dof: False for dof in self.dof_names}
        self.set_target_moo_in_execution = False
        

    def check_ethercat_slaves(self):
        """
        Periodically check if all EtherCAT slaves are in operational (OP) state.
        
        When all slaves reach OP state, this method cancels the timer to stop
        periodic checks and sets the ethercat_ready flag.
        """
        eth_states = get_ethercat_slaves_status(self._logger)
        if eth_states and all(info['state'] == 'OP' for info in eth_states):
            self.get_logger().info('All EtherCAT slaves are in OP state.')
            self.ethercat_ready = True
            self.ethercat_timer.cancel()  # Stop the timer
        else:
            self.get_logger().info(f'Not all EtherCAT slaves in OP state. Current states: {eth_states}')


    def handle_drive_status(self, msg):
        """
        Callback handler for drive status updates from the state controller.
        
        Updates the internal state, mode, and status word dictionaries based on
        the received Cia402DriveStates message. Only processes updates after
        EtherCAT slaves are confirmed operational.
        
        Args:
            msg (Cia402DriveStates): Message containing drive states for all DOFs
        """
        if not self.ethercat_ready:
            return
        else:
            for dof in self.dof_names:
                try:
                    index = msg.dof_names.index(dof)
                    self.state[dof] = msg.drive_states[index]
                    self.mode[dof] = msg.modes_of_operation[index]
                    self.status_words[dof] = msg.status_words[index]
                    self.get_logger().info(
                        f'DOF {dof} - State: {self.state[dof]}, Mode: {self.mode[dof]}, Status Word: {self.status_words[dof]:>019_b}', 
                        throttle_duration_sec=5.0
                    )
                except ValueError:
                    self.get_logger().error(f'DoF {dof} not found in received message')
                    continue


    def reset_fault(self):
        """
        Call the state controller's reset_fault service to clear fault conditions.
        
        Returns:
            int: 0 if service call initiated successfully, 
                 1 if already in execution,
                 -1 if service is unavailable
        """
        if self.reset_faults_in_execution:
            return 1
        self.reset_faults_in_execution = True
        fault_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/reset_fault', callback_group=fault_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.reset_fault_callback)
            return 0
        else:
            self.get_logger().error('Service /state_controller/reset_fault not available')
            self.reset_faults_in_execution = False
            return -1


    def reset_fault_callback(self, future):
        """
        Callback for reset_fault service response.
        
        Args:
            future: Async future object containing the service result
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Fault reset successfully')
            else:
                self.get_logger().error('Failed to reset fault')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
        self.reset_faults_in_execution = False


    def try_turn_on(self):
        """
        Call the state controller's try_turn_on service to enable drives.
        
        Returns:
            int: 0 if service call initiated successfully,
                 -1 if service is unavailable
        """
        self.try_turn_on_in_execution = True
        self.get_logger().info('Turning on drives...', once=True)
        try_on_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/try_turn_on', callback_group=try_on_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.try_turn_on_callback)
            return 0
        else:
            self.get_logger().error('Service /state_controller/try_turn_on not available')
            self.try_turn_on_in_execution = False
            return -1


    def try_turn_on_callback(self, future):
        """
        Callback for try_turn_on service response.
        
        Args:
            future: Async future object containing the service result
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turn on command sent successfully')
            else:
                self.get_logger().error('Failed to turn on drives')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
    

    def try_turn_on_checker(self):
        """
        Check if all drives have successfully reached STATE_OPERATION_ENABLED.
        
        Returns:
            bool: True if all drives are in operation enabled state or if try_turn_on
                  is not currently executing, False otherwise
        """
        if not self.try_turn_on_in_execution:
            return True
        if all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values()):
            self.get_logger().info('All drives are in STATE_OPERATION_ENABLED state')
            self.try_turn_on_in_execution = False
            return True
        else:
            return False


    def try_turn_off(self):
        """
        Call the state controller's try_turn_off service to disable drives.
        
        Returns:
            int: 0 if service call initiated successfully,
                 1 if already in execution,
                 -1 if service is unavailable
        """
        if self.try_turn_off_in_execution:
            return 1
        self.try_turn_off_in_execution = True
        self.get_logger().info('Turning OFF drives...', once=True)
        try_on_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/try_turn_off', callback_group=try_on_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.try_turn_off_callback)
            return 0
        else:
            self.get_logger().error('Service /state_controller/try_turn_off not available')      
            return -1


    def try_turn_off_callback(self, future):
        """
        Callback for try_turn_off service response.
        
        Args:
            future: Async future object containing the service result
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turn off command sent successfully')
            else:
                self.get_logger().error('Failed to turn off drives')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
            self.try_turn_off_in_execution = False


    def try_turn_off_checker(self):
        """
        Check if all drives have successfully reached STATE_SWITCH_ON_DISABLED.
        
        Returns:
            bool: True if all drives are disabled, False otherwise or if try_turn_off
                  is not currently executing
        """
        if not self.try_turn_off_in_execution:
            return False
        if all(state == 'STATE_SWITCH_ON_DISABLED' for state in self.state.values()):
            self.get_logger().info('All drives are in STATE_SWITCH_ON_DISABLED (Try Turn Off completed)')
            self.try_turn_off_in_execution = False
            return True
        else:
            return False


    def start_perform_process(self):
        """
        Initiate the homing process via the state controller service.
        
        Sets homing_process_started flag and records the start time. If homing
        is already in progress, this call is skipped.
        """
        if self.homing_process_started:
            self.get_logger().info('Homing process already started, skipping...')
            return
        self.homing_process_started = True
        self.homing_process_starting_time = self.get_clock().now()
        self.get_logger().info('--------------->Performing Homing for all three joints')
        try_on_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/perform_homing', callback_group=try_on_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.perform_homing_callback)
        else:
            self.get_logger().error('Service /state_controller/perform_homing not available')


    def perform_homing_callback(self, future):
        """
        Callback for perform_homing service response.
        
        Args:
            future: Async future object containing the service result
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Homing Process Started successfully')
            else:
                self.get_logger().error('Failed to Start Homing Process')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
            self.homing_process_started = False

    

    def switch_mode(self, dof_name, target_mode):
        """
        Request mode switch for a specific DOF via the state controller service.
        
        Args:
            dof_name (str): Name of the degree of freedom (e.g., 'joint_x')
            target_mode (int): Target mode number from MODE_OF_OPERATION_MAP
            
        Returns:
            int: 0 if service call initiated successfully,
                 1 if already in execution for this DOF,
                 -1 if service is unavailable
        """
        if self.switch_mode_in_execution[dof_name]:
            return 1
        self.switch_mode_in_execution[dof_name] = True
        self.get_logger().info(
            f'>>>>>>>>>>>>>>>>>>>>>>> Switching mode for {dof_name} from {self.mode[dof_name]} to {get_op_mode_string(target_mode)}...')
        switch_mode_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(SwitchDriveModeOfOperation, '/state_controller/switch_mode_of_operation', callback_group=switch_mode_service_group)
        
        if client.wait_for_service(timeout_sec=5.0):
            request = SwitchDriveModeOfOperation.Request()
            request.dof_name = dof_name
            request.mode_of_operation = target_mode
            future = client.call_async(request)
            future.add_done_callback(lambda f: self.switch_mode_callback(f, dof_name))
        else:
            self.get_logger().error(f'Service /state_controller/switch_mode_of_operation not available for {dof_name}')
            return -1
        return 0


    def switch_mode_callback(self, future, dof_name):
        """
        Callback for switch_mode service response.
        
        Args:
            future: Async future object containing the service result
            dof_name (str): Name of the DOF for which mode switch was requested
        """
        try:
            result = future.result()
            if result:
                self.get_logger().info(f'Mode switched for {dof_name}: {result.return_message}')
            else:
                self.get_logger().error(f'Failed to switch mode for {dof_name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')


    def switch_mode_checker(self, dof_name, target_mode):
        """
        Check if a specific DOF has successfully switched to the target mode.
        
        Args:
            dof_name (str): Name of the degree of freedom
            target_mode (int): Target mode number to verify
            
        Returns:
            bool: True if DOF is in target mode or switch not in execution,
                  False otherwise
        """
        if not self.switch_mode_in_execution[dof_name]:
            return False
        if get_op_mode_number(self.mode[dof_name]) == target_mode:
            self.get_logger().info(f'{dof_name} is in target mode {target_mode} (Switch Mode Terminated)')
            self.switch_mode_in_execution[dof_name] = False
            return True
        else:
            return False


    def set_target_moo(self, target_mode):
        """
        Initiate mode switch to target cyclic mode for all DOFs.
        
        This method requests mode switches for all DOFs simultaneously.
        The completion is checked via set_target_moo_checker().
        
        Args:
            target_mode (str): Target mode string (e.g., 'MODE_CYCLIC_SYNC_POSITION')
        """
        if not self.set_target_moo_in_execution:
            self.set_target_moo_in_execution = True
            self.switch_mode_in_execution = {dof: False for dof in self.dof_names}
            self.get_logger().info(f'Setting target cyclic mode to {target_mode} for all DOFs...')
        for curr_dof in self.dof_names:
            self.switch_mode(curr_dof, get_op_mode_number(target_mode))
    

    def set_target_moo_checker(self, target_mode):
        """
        Check if all DOFs have successfully switched to the target cyclic mode.
        
        Args:
            target_mode (int): Target mode number to verify
            
        Returns:
            bool: True if all DOFs are in target mode or operation completed,
                  False if still in progress
        """
        if not self.set_target_moo_in_execution:
            return False
        if all( get_op_mode_number(m) == target_mode for m in self.mode.values()):
            self.get_logger().info(f'Setting he target MOO Attained (Set Target MOO Terminated)')
            for curr_dof in self.dof_names:
                self.switch_mode_in_execution[curr_dof] = False
            self.set_target_moo_in_execution = False
            return True
        return True

    def all_in_no_mode(self):
        """
        Check if all DOFs are in MODE_NO_MODE with appropriate states.
        
        Returns:
            bool: True if all DOFs are in MODE_NO_MODE and valid operational states,
                  False otherwise
        """
        if all(mode == 'MODE_NO_MODE' for mode in self.mode.values()):
            if all(state in ['STATE_SWITCH_ON_ENABLED', 'STATE_SWITCH_ON', 'STATE_OPERATION_ENABLED', 'STATE_READY_TO_SWITCH_ON'] for state in self.state.values()):
                return True
            else:
                for dof, state in self.state.items():
                    if state not in ['STATE_SWITCH_ON_ENABLED', 'STATE_SWITCH_ON', 'STATE_OPERATION_ENABLED']:
                        self.get_logger().info(f'{dof} is in MODE_NO_MODE but has unexpected state: {state}')
        else:
            return False
        

    def all_in_homing_mode(self):
        """
        Check if all DOFs are in MODE_HOMING with STATE_OPERATION_ENABLED.
        
        Returns:
            bool: True if all DOFs are ready for homing, False otherwise
        """
        if all(mode == 'MODE_HOMING' for mode in self.mode.values()):
            return all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values())
        else:
            return False


    def all_in_target_state_mode(self, target_mode):
        """
        Check if all DOFs have reached the target mode and operational state.
        
        Args:
            target_mode (str): Target mode string to verify
            
        Returns:
            bool: True if all DOFs are in target mode and operational,
                  False otherwise
        """
        if all(mode == target_mode for mode in self.mode.values()):
            if all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values()):
                return True
            else:
                for dof, state in self.state.items():
                    if state not in ['STATE_SWITCH_ON_ENABLED', 'STATE_SWITCH_ON', 'STATE_OPERATION_ENABLED']:
                        self.get_logger().info(f'{dof} is in {target_mode} but has unexpected state: {state}')
        else:
            return False
        

    def log_mode_state(self):
        """
        Log the current mode and state of all DOFs for debugging.
        
        Useful for investigating unexpected behavior during boot sequence.
        """
        for dof in self.dof_names:
            self.get_logger().info(f'Details of unexpected behavior: DOF {dof} is mode {self.mode[dof]} and state {self.state[dof]}')
        

    def shutdown_node(self):
        """
        Prepare the node for shutdown.
        
        Currently sets the shutdown_requested flag. Actual cleanup occurs in main().
        """
        self.get_logger().info('Shutting down...')
        self.shutdown_requested = True


def main(args=None):
    """
    Main entry point for the boot hardware node.
    
    Initializes ROS 2, creates the HomingNode, and runs the main control loop.
    The loop continuously monitors drive states and executes the boot sequence:
    1. Fault handling
    2. Drive power-up
    3. Mode switching to homing
    4. Homing execution
    5. Mode switching to target cyclic mode
    6. Graceful shutdown
    
    Args:
        args: Command-line arguments passed to rclpy.init()
        
    The target operation mode can be specified as the first command-line argument.
    Default is MODE_CYCLIC_SYNC_POSITION.
    """
    rclpy.init(args=args)

    # Parse target operation mode from command line
    input_op_mode = sys.argv[1] if len(sys.argv) > 1 else "MODE_CYCLIC_SYNC_POSITION"
    node = HomingNode(input_op_mode)

    # Use multi-threaded executor for handling async service callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():  # Main loop
            executor.spin_once()
            
            ##############################################################
            # Boot sequence before homing process starts
            ##############################################################
            if not node.homing_process_started:
                # Handle fault conditions
                if any( n == 'STATE_FAULT' for n in node.state.values()):
                    node.get_logger().info("Detected FAULT state - Resetting Faults", throttle_duration_sec=5.0)
                    node.reset_fault()

                # Handle not-ready states
                elif any( n == 'STATE_NOT_READY_TO_SWITCH_ON' for n in node.state.values()):
                    node.get_logger().info("Detected NOT_READY_TO_SWITCH_ON state", throttle_duration_sec=5.0)
                    pass
                
                # Enable drives when disabled
                elif any( n in ['STATE_SWITCH_ON_DISABLED', 'STATE_READY_TO_SWITCH_ON'] for n in node.state.values()):
                    node.get_logger().info("Detected DISABLED state - Try to turn on the motors", throttle_duration_sec=5.0)
                    node.try_turn_on()
                
                # Switch to homing mode when drives are enabled
                else:
                    for dof in node.dof_names:
                        if node.mode[dof] == 'MODE_HOMING':
                            pass
                        else:
                            node.get_logger().info('Drives turned on but not in homing mode, setting it.', throttle_duration_sec=5.0)
                            if node.switch_mode(dof, get_op_mode_number('MODE_HOMING'))<0:
                                node.get_logger().error(f'Failed to switch {dof} to homing mode, retrying...')
                                continue
                
                # Start homing when all drives are in homing mode
                if node.all_in_homing_mode():
                    node.start_perform_process()
            
            ##############################################################
            # Homing process monitoring and post-homing mode switching
            ##############################################################
            else:      
                timeout_sec = 20.0
                node.get_logger().info('Homing Process Started, waiting for completion...', once=True)
                
                # Calculate elapsed time since homing started
                current_time = node.get_clock().now()
                elapsed = (current_time - node.homing_process_starting_time).nanoseconds / 1e9  # convert to seconds
                
                # Handle homing timeout
                if elapsed > timeout_sec:
                    node.get_logger().error('Timeout while waiting for homing to complete')
                    node.try_turn_off()
                    if node.try_turn_off_checker():
                        node.get_logger().info('Drives turned off successfully, Try to perform homing again ...')
                        node.homing_process_started = False
                        node.homing_process_attained = False
                
                # Monitor homing completion via status word bit 12
                else:
                    if all((status_word & (1 << 12)) != 0 for status_word in node.status_words.values()):
                        node.homing_process_attained = True
                    
                    # Switch to target mode after homing completion
                    if node.homing_process_attained and not node.set_target_moo_in_execution:
                        node.get_logger().info('!!!!!!  Homing Performed Successfully !!!!!!', once=True)
                        node.set_target_moo(node.target_cyclic_mode)
                    
                    # Monitor target mode switch completion
                    if node.homing_process_attained and node.set_target_moo_in_execution:
                        if node.set_target_moo_checker(node.target_cyclic_mode): # node.set_target_moo_in_execution becames False
                            node.get_logger().info('Homing process ended successfully, switching to target cyclic mode.', throttle_duration_sec=5.0 )
                            node.try_turn_off()
                    
                    # Wait for shutdown after turning off drives
                    if node.homing_process_attained and node.set_target_moo_in_execution:
                        if node.try_turn_off_checker():
                            node.get_logger().info('Drives turned off successfully, exiting...')
                            break                

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        # Cleanup: destroy node and shutdown ROS 2 context
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
