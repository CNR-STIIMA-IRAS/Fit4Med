import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ethercat_controller_msgs.srv import SwitchDriveModeOfOperation
from ethercat_controller_msgs.msg import Cia402DriveStates
from rclpy.duration import Duration
import subprocess
import sys

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
    return MODE_OF_OPERATION_MAP.get(mode_string, 0)

def get_op_mode_string(mode_number):
    if mode_number in MODE_OF_OPERATION_MAP.values():
        return list(MODE_OF_OPERATION_MAP.keys())[list(MODE_OF_OPERATION_MAP.values()).index(mode_number)]
    return MODE_OF_OPERATION_MAP['MODE_NO_MODE']


_executor = None


class HomingNode(Node):

    def __init__(self, target_op_mode):
        super().__init__('homing_node')
        self.get_logger().info('Booting Cia402 drives and performing homing...')
        self.ethercat_ready = False
        # Timer to check EtherCAT slaves' status
        self.ethercat_timer = self.create_timer(5.0, self.check_ethercat_slaves)
        # Define callback groups
        topic_group = MutuallyExclusiveCallbackGroup()
        try_on_service_group = MutuallyExclusiveCallbackGroup()
        switch_mode_service_groupt = MutuallyExclusiveCallbackGroup()
        # Subscription to drive status updates
        self.subscription = self.create_subscription(Cia402DriveStates, '/state_controller/drive_states', self.handle_drive_status, qos_profile=10, callback_group=topic_group)
        # Switch mode service client
        self.switch_mode_client = self.create_client(SwitchDriveModeOfOperation, '/state_controller/switch_mode_of_operation', callback_group=switch_mode_service_groupt)
        if not self.switch_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /state_controller/switch_mode_of_operation not available, exiting...')
            rclpy.shutdown()
            sys.exit(1)
        # Try turn on service client
        self.try_on_client = self.create_client(Trigger, '/state_controller/try_turn_on', callback_group=try_on_service_group)
        if not self.try_on_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /state_controller/try_turn_on not available, exiting...')
            rclpy.shutdown()
            sys.exit(1)
        # Try turn off service client
        self.try_off_client = self.create_client(Trigger, '/state_controller/try_turn_off', callback_group=try_on_service_group)
        if not self.try_off_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /state_controller/try_turn_off not available, exiting...')
            rclpy.shutdown()
            sys.exit(1)
        # Reset faults service client
        self.reset_faults_client = self.create_client(Trigger, '/state_controller/reset_fault', callback_group=try_on_service_group)
        if not self.reset_faults_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /state_controller/reset_fault not available, exiting...')
            rclpy.shutdown()
            sys.exit(1)
        # Service client for performing homing
        self.perform_homing_client = self.create_client(Trigger, '/state_controller/perform_homing', callback_group=switch_mode_service_groupt)
        if not self.perform_homing_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /state_controller/perform_homing not available, exiting...')
            rclpy.shutdown()
            sys.exit(1)
        self.dof_names = ['joint_x', 'joint_y', 'joint_z']
        self.state = {dof: 'STATE_UNDEFINED' for dof in self.dof_names}
        self.mode = {dof: 'MODE_NO_MODE' for dof in self.dof_names}
        self.status_words = {dof: 0 for dof in self.dof_names}
        self.slave_indices = [0, 1, 2]  # Positions of the drives in the slaves list, check in ros2_control file
        # Flag to set true when homing process has completed
        self.homing_process_started = False
        self.homing_process_in_execution = False
        self.homing_process_successful = False
        self.homing_process_starting_time = None
        # Target cyclic mode
        self.target_cyclic_mode = target_op_mode
        self.try_turn_on_in_execution = False
        self.try_turn_off_in_execution = False
        self.reset_faults_in_execution = False
        self.switch_mode_in_execution = {dof: False for dof in self.dof_names}
        self.shutdown_requested = False
        self.moo_switch_starting_time = None
        self.try_turn_off_starting_time = None
        self.set_target_moo_start_time = None

    def check_ethercat_slaves(self):
        eth_states = self.get_ethercat_slaves_status()
        if eth_states and all(info['state'] == 'OP' for info in eth_states.values()):
            self.get_logger().info('All EtherCAT slaves are in OP state.')
            self.ethercat_ready = True
            self.ethercat_timer.cancel()  # Stop the timer
        else:
            self.get_logger().info(f'Not all EtherCAT slaves in OP state. Current states: {eth_states}')

    def get_ethercat_slaves_status(self):
        try:
            result = subprocess.run(['ethercat', 'slaves'],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE,
                                    text=True,
                                    check=True)
            lines = result.stdout.strip().splitlines()
            eth_states = {}
            for idx in self.slave_indices:
                if idx < len(lines):
                    parts = lines[idx].split()
                    if len(parts) > 2:
                        slave_name, state = parts[1], parts[2]
                        eth_states[idx] = {'name': slave_name, 'state': state}
                    else:
                        self.get_logger().warn(f"Formato non valido per la linea dello slave {idx}: {lines[idx]}")
                else:
                    self.get_logger().warn(f"Nessuno slave trovato all'indice {idx}.")
            return eth_states

        except FileNotFoundError:
            self.get_logger().error("'ethercat' command not found. Verify that EtherCAT master is active.")
            return {}

    def handle_drive_status(self, msg):
        if not self.ethercat_ready:
            return
        else:
            for dof in self.dof_names:
                try:
                    index = msg.dof_names.index(dof)
                    self.state[dof] = msg.drive_states[index]
                    self.mode[dof] = msg.modes_of_operation[index]
                    self.status_words[dof] = msg.status_words[index]
                    self.get_logger().info(f'DOF {dof} - State: {self.state[dof]}, Mode: {self.mode[dof]}, Status Word: {self.status_words[dof]:>019_b}', throttle_duration_sec=5.0)
                except ValueError:
                    self.get_logger().error(f'DoF {dof} not found in received message')
                    continue

    def reset_fault(self):
        if self.reset_faults_in_execution:
            return 1
        self.reset_faults_in_execution = True
        request = Trigger.Request()
        future = self.reset_faults_client.call_async(request)
        future.add_done_callback(self.reset_fault_callback)

    def reset_fault_callback(self, future):
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
        if self.try_turn_on_in_execution:
            return 1
        self.try_turn_on_in_execution = True
        self.get_logger().info('Turning on drives...', once = True)

        request = Trigger.Request()
        future = self.try_on_client.call_async(request)
        future.add_done_callback(self.try_turn_on_done)
        
    def try_turn_on_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turn on command sent successfully')
                self.try_turn_on_starting_time = self.get_clock().now()
                self.check_turn_on_timer = self.create_timer(0.1, self.check_if_turned_on)
            else:
                self.get_logger().error('Error trying to turn on drives')
        except Exception as e:
            self.get_logger().error(f'Rased exception during try turn on service: {e}')
        
    def check_if_turned_on(self):
        if all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values()):
            self.get_logger().info('All drives are now ON.')
            self.check_turn_on_timer.cancel()
            self.try_turn_on_in_execution = False
        else:
            elapsed = (self.get_clock().now() - self.try_turn_on_starting_time).nanoseconds / 1e9
            if elapsed > 5.0:
                self.get_logger().error('Timeout while waiting for drives to turn off. Retrying...')
                self.check_turn_on_timer.cancel()
                self.try_turn_on_in_execution = False

    def try_turn_off(self):
        if self.try_turn_off_in_execution:
            return 1
        self.try_turn_off_in_execution = True
        self.get_logger().info('Turning off drives...')
        request = Trigger.Request()
        future = self.try_off_client.call_async(request)
        future.add_done_callback(self.try_turn_off_done)

    def try_turn_off_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives successfully turned off.')
                if self.homing_process_successful:
                    self.try_turn_off_starting_time = self.get_clock().now()
                    self.check_turn_off_timer = self.create_timer(0.1, self.check_if_turned_off)
            else:
                self.get_logger().error('Error trying to turn off drives')
        except Exception as e:
            self.get_logger().error(f'Rased exception during try turn off service: {e}')

    def check_if_turned_off(self):
        if all(state == 'STATE_SWITCH_ON_DISABLED' for state in self.state.values()):
            self.get_logger().info('All drives are now OFF.')
            self.try_turn_off_in_execution = False
            self.check_turn_off_timer.cancel()
            self.shutdown_requested = True
        else:
            elapsed = (self.get_clock().now() - self.try_turn_off_starting_time).nanoseconds / 1e9
            if elapsed > 5.0:
                self.get_logger().error('Timeout while waiting for drives to turn off. Retrying...')
                self.try_turn_off_in_execution = False
                self.check_turn_off_timer.cancel()
                
    def perform_homing(self):
        if self.homing_process_in_execution:
            return 1
        self.homing_process_started = True
        self.homing_process_in_execution = True
        self.get_logger().info('---------------> Performing Homing for all three joints')
        request = Trigger.Request()
        future = self.perform_homing_client.call_async(request)
        future.add_done_callback(self.perform_homing_done)

    def perform_homing_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Perfom homing service succesfully called.')
                self.homing_start_time = self.get_clock().now()
                self.homing_check_timer = self.create_timer(0.1, self._check_homing_status)  
            else:
                self.get_logger().error('Error trying to turn off drives')
        except Exception as e:
            self.get_logger().error(f'Rased exception during try turn off service: {e}')

    def _check_homing_status(self):
        if all((status_word & (1 << 12)) != 0 for status_word in self.status_words.values()):
            self.get_logger().info('✅ Homing performed successfully.')
            self.homing_process_successful = True
            self.homing_process_in_execution = False
            self.homing_check_timer.cancel()
        else:
            elapsed = (self.get_clock().now() - self.homing_start_time).nanoseconds / 1e9
            if elapsed > 10.0:
                self.get_logger().error('❌ Timeout while waiting for homing to complete.')
                self.homing_process_successful = False
                self.homing_process_in_execution = False
            else:
                self.get_logger().info('Waiting for status words to confirm homing...')

    def switch_mode(self, mode_value):
        global _executor
        futures = []
        for dof in self.dof_names:
            if not self.switch_mode_in_execution[dof]:
                req = SwitchDriveModeOfOperation.Request()
                req.dof_name = dof
                req.mode_of_operation = mode_value
                self.get_logger().info(f"Setting mode {mode_value} for {dof}...")
                self.switch_mode_in_execution[dof] = True
                futures.append(self.switch_mode_client.call_async(req))
        for future in futures:                
            _executor.spin_until_future_complete(future)
        self.moo_switch_starting_time = self.get_clock().now()
        self.switch_mode_timer = self.create_timer(0.1, lambda: self._check_switch_mode_reached(mode_value))

    def _check_switch_mode_reached(self, target_mode):
        for dof_name in self.dof_names:
            if get_op_mode_number(self.mode[dof_name]) == target_mode:
                self.get_logger().info(f'{dof_name} is in target mode {target_mode} (Switch Mode Terminated)')
                self.switch_mode_in_execution[dof_name] = False
        if all(self.switch_mode_in_execution[d] == False for d in self.dof_names):
            self.get_logger().info(f'All drives reached {target_mode}!!')
            self.switch_mode_timer.cancel()
        else:
            elapsed = (self.get_clock().now() - self.moo_switch_starting_time).nanoseconds / 1e9
            if elapsed > 5.0:
                self.get_logger().warn(f'Timeout reached for {target_mode} wait. Retrying to set...')
                for dof in self.dof_names:
                    self.switch_mode_in_execution = False
                self.switch_mode_timer.cancel()

    def all_in_no_mode(self):
        if all(mode == 'MODE_NO_MODE' for mode in self.mode.values()):
            if all(state == 'STATE_SWITCH_ON_DISABLED' for state in self.state.values()):
                return True
            else:
                for dof, state in self.state.items():
                    if (state != 'STATE_SWITCH_ON_DISABLED' for state in self.state.values()):
                        self.get_logger().info(f'{dof} is in MODE_NO_MODE but has unexpected state: {state}')
        else:
            return False
        
    def all_in_homing_mode(self):
        if all(mode == 'MODE_HOMING' for mode in self.mode.values()):
            return all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values())
        else:
            return False

    def all_in_target_mode(self, target_mode):
        if all(mode == target_mode for mode in self.mode.values()):
            return True
        else:
            return False


def main(args=None):
    global _executor

    rclpy.init(args=args)

    input_op_mode = sys.argv[1] if len(sys.argv) > 1 else "MODE_CYCLIC_SYNC_POSITION"
    node = HomingNode(input_op_mode)

    _executor = MultiThreadedExecutor(num_threads=4)
    _executor.add_node(node)

    try:
        while rclpy.ok():  # Main loop
            _executor.spin_once()
            ##############################################################
            if node.ethercat_ready:
                if not node.homing_process_started:
                    if any( n in ['STATE_SWITCH_ON_DISABLED', 'STATE_READY_TO_SWITCH_ON'] for n in node.state.values()):
                        node.get_logger().info("(MAIN) Detected DISABLED state - Try to turn on the motors", throttle_duration_sec=5.0)
                        if not node.try_turn_on_in_execution:
                            node.try_turn_on()
                    elif any( n == 'STATE_FAULT' for n in node.state.values()):
                        node.get_logger().info("(MAIN) Detected FAULT state - Resetting Faults", throttle_duration_sec=5.0)
                        if not node.reset_faults_in_execution:
                            node.reset_fault()
                    elif any( n == 'STATE_NOT_READY_TO_SWITCH_ON' for n in node.state.values()):
                        node.get_logger().info("(MAIN) Detected NOT_READY_TO_SWITCH_ON state", throttle_duration_sec=5.0)
                        pass
                    elif all(n == 'STATE_OPERATION_ENABLED' for n in node.state.values()):
                        if not node.all_in_homing_mode() and not all(node.switch_mode_in_execution[d]==False for d in node.dof_names):
                            node.get_logger().info('(MAIN) Setting homing mode.')
                            node.switch_mode(get_op_mode_number('MODE_HOMING'))
                        else:
                            if not node.homing_process_in_execution:
                                node.get_logger().info('(MAIN) Performing homing.')
                                node.perform_homing()
                            else:
                                node.get_logger().info('(MAIN) Perform homing already called, waiting...')
                    else: 
                        node.get_logger().info(f'(MAIN) Drives are in state: {node.state.values()}')
                else:
                    if not node.homing_process_successful and not node.homing_process_in_execution:
                        if not all(state == 'STATE_SWITCH_ON_DISABLED' for state in node.state.values()):
                            node.try_turn_off()
                        if all(state == 'STATE_SWITCH_ON_DISABLED' for state in node.state.values()) and not node.all_in_no_mode():
                            node.switch_mode(0)
                        if node.all_in_no_mode():
                            # node.get_clock().sleep_for(Duration(seconds=1))
                            node.homing_check_timer.cancel()
                            node.homing_process_started = False
                    elif node.homing_process_successful and not node.all_in_target_mode(node.target_cyclic_mode):
                        node.switch_mode(get_op_mode_number(node.target_cyclic_mode))
                    elif node.homing_process_successful and node.all_in_target_mode(node.target_cyclic_mode):
                        node.try_turn_off()
                    elif node.homing_process_in_execution:
                        pass
                    else:
                        node.get_logger().info(f'(MAIN) Homing process output is: {node.homing_process_successful}. Drives in state: {node.state.values()} and mode: {node.mode.values()}')
                    ###########################################################################
                    if node.shutdown_requested:
                        node.get_logger().info('(MAIN) Shutdown requested, exiting main loop.')
                        break

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()     # Shutdown the ROS 2 context

if __name__ == '__main__':
    main()
