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


class HomingNode(Node):

    def __init__(self, target_op_mode):
        super().__init__('homing_node')
        self.get_logger().info('Homing node started')
        self.ethercat_ready = False
        # Timer to check EtherCAT slaves' status
        self.ethercat_timer = self.create_timer(5.0, self.check_ethercat_slaves)
        # Define topic callback group
        topic_group = MutuallyExclusiveCallbackGroup()
        # Subscription to drive status updates
        self.subscription = self.create_subscription(Cia402DriveStates, '/state_controller/drive_states', self.handle_drive_status, qos_profile=10, callback_group=topic_group)
        self.dof_names = ['joint_x', 'joint_y', 'joint_z']
        self.state = {dof: 'STATE_UNDEFINED' for dof in self.dof_names}
        self.mode = {dof: 'MODE_NO_MODE' for dof in self.dof_names}
        self.status_words = {dof: 0 for dof in self.dof_names}
        self.slave_indices = [0, 1, 2]  # Positions of the drives in the slaves list, check in ros2_control file
        # Flag to set true when homing process has completed
        self.cyclic_modes = {'MODE_CYCLIC_SYNC_POSITION', 'MODE_CYCLIC_SYNC_VELOCITY', 'MODE_CYCLIC_SYNC_TORQUE'}
        self.homing_process_started = False
        # Target cyclic mode
        self.target_cyclic_mode = target_op_mode
        self.shutdown_requested = False

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
                except ValueError:
                    self.get_logger().error(f'DoF {dof} not found in received message')
                    continue
                if not self.homing_process_started:
                    if self.state[dof] == 'STATE_FAULT':
                        self.get_logger().info("Detected FAULT state")
                        self.reset_fault()
                    elif self.state[dof] == 'STATE_NOT_READY_TO_SWITCH_ON':
                        self.get_logger().info("Detected NOT_READY_TO_SWITCH_ON state")
                        pass
                    elif self.state[dof] in ['STATE_SWITCH_ON_DISABLED', 'STATE_READY_TO_SWITCH_ON']:
                        self.get_logger().info("Detected DISABLED state")
                        self.try_turn_on()
                    else:
                        if self.mode[dof] == 'MODE_HOMING':
                            pass
                        else:
                            self.get_logger().info('Drives turned on but not in homing mode, setting it.')
                            self.switch_mode(dof, self.get_op_mode_number('MODE_HOMING'))
                if self.all_in_homing_mode() and not self.homing_process_started:
                    self.perform_homing()


    def get_op_mode_number(self, mode):
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

    def reset_fault(self):
        self.get_logger().info('Resetting faults...')
        fault_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/reset_fault', callback_group=fault_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.reset_fault_callback)
        else:
            self.get_logger().error('Service /state_controller/reset_fault not available')

    def reset_fault_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Fault reset successfully')
            else:
                self.get_logger().error('Failed to reset fault')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    def try_turn_on(self):
        self.get_logger().info('Turning on drives...')
        try_on_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/try_turn_on', callback_group=try_on_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.try_turn_on_callback)
        else:
            self.get_logger().error('Service /state_controller/try_turn_on not available')

    def try_turn_on_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turned on successfully')
            else:
                self.get_logger().error('Failed to turn on drives')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    def perform_homing(self):
        self.homing_process_started = True
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
        try:
            result = future.result()
            if result.success:
                start_time = self.get_clock().now()
                timeout_sec = 5.0
                self.get_logger().info('Homing started, waiting for completion...')
                while True:
                    current_time = self.get_clock().now()
                    elapsed = (current_time - start_time).nanoseconds / 1e9  # convert to seconds
                    
                    if all((status_word & (1 << 12)) != 0 for status_word in self.status_words.values()):
                        self.get_logger().info('Homing performed successfully')
                        break
                    elif elapsed > timeout_sec:
                        self.get_logger().error('Timeout while waiting for homing to complete')
                        return False
                    self.get_logger().info("waiting for status worlds to be all true!!")
                    time.sleep(0.1)  # sleep a bit to avoid busy waiting
                self.set_target_moo(self.target_cyclic_mode)
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    def set_target_moo(self, target_mode):
        for curr_dof in self.dof_names:
            self.get_logger().info(f'Setting dof {curr_dof} to final target mode being: {target_mode}.')
            self.switch_mode(curr_dof, self.get_op_mode_number(target_mode))
        while not self.all_in_target_state_mode(self.target_cyclic_mode):
            time.sleep(0.1)
        self.shutdown_node()

    def switch_mode(self, dof_name, target_mode):
        self.get_logger().info(f'Switching mode for {dof_name}...')
        switch_mode_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(SwitchDriveModeOfOperation, '/state_controller/switch_mode_of_operation', callback_group=switch_mode_service_group)
        
        if client.wait_for_service(timeout_sec=5.0):
            request = SwitchDriveModeOfOperation.Request()
            request.dof_name = dof_name
            request.mode_of_operation = target_mode
            self.get_logger().info(f'Sending switch request for {dof_name}...')
            future = client.call_async(request)
            future.add_done_callback(lambda f: self.switch_mode_callback(f, dof_name))
        else:
            self.get_logger().error(f'Service /state_controller/switch_mode_of_operation not available for {dof_name}')
        
    def switch_mode_callback(self, future, dof_name):
        try:
            result = future.result()
            if result:
                self.get_logger().info(f'Mode switched for {dof_name}: {result.return_message}')
                return True
            else:
                self.get_logger().error(f'Failed to switch mode for {dof_name}')
                return False
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    def all_in_no_mode(self):
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
        if all(mode == 'MODE_HOMING' for mode in self.mode.values()):
            if all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values()):
                return True
            else:
                for dof, state in self.state.items():
                    self.get_logger().info(f'{dof} is in MODE_HOMING but has unexpected state: {state}')
        else:
            return False

    def all_in_target_state_mode(self, target_mode):
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
        for dof in self.dof_names:
            self.get_logger().info(f'Details of unexpected behavior: DOF {dof} is mode {self.mode[dof]} and state {self.state[dof]}')
        
    def shutdown_node(self):
        self.get_logger().info('Shutting down...')
        self.shutdown_requested = True
        # self.destroy_node()

        # # Ensure context is valid before shutting down
        # if rclpy.ok():
        #     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    input_op_mode = sys.argv[1] if len(sys.argv) > 1 else "MODE_CYCLIC_SYNC_POSITION"
    node = HomingNode(input_op_mode)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():  # Main loop
            executor.spin_once()
            if node.shutdown_requested:  # Check if shutdown is requested
                break
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()     # Shutdown the ROS 2 context

if __name__ == '__main__':
    main()
