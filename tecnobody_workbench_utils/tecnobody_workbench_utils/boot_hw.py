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


class HomingNode(Node):

    def __init__(self, target_op_mode):
        super().__init__('boot_hw_node')
        self.get_logger().info('Booting Cia402 drives and performing homing...')
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
        self.homing_process_starting_time = None
        self.homing_process_attained = False
        # Target cyclic mode
        self.target_cyclic_mode = target_op_mode

        self.try_turn_off_in_execution = False
        self.try_turn_on_in_execution = False
        self.reset_faults_in_execution = False
        self.switch_mode_in_execution = {dof: False for dof in self.dof_names}
        self.set_target_moo_in_execution = False
        

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
                    self.get_logger().info(f'DOF {dof} - State: {self.state[dof]}, Mode: {self.mode[dof]}, Status Word: {self.status_words[dof]:>019_b}', throttle_duration_sec=5.0    )
                except ValueError:
                    self.get_logger().error(f'DoF {dof} not found in received message')
                    continue


    def reset_fault(self):
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
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turn on command sent successfully')
            else:
                self.get_logger().error('Failed to turn on drives')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
    
    def try_turn_on_checker(self):
        if not self.try_turn_on_in_execution:
            return True
        if all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values()):
            self.get_logger().info('All drives are in STATE_OPERATION_ENABLED state')
            self.try_turn_on_in_execution = False
            return True
        else:
            return False

    def try_turn_off(self):
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
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turn off command sent successfully')
            else:
                self.get_logger().error('Failed to turn on drives')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
            self.try_turn_off_in_execution = False

    def try_turn_off_checker(self):
        if not self.try_turn_off_in_execution:
            return False
        #self.get_logger().info(f'Checking if all drives are in STATE_SWITCH_ON_DISABLED state...{self.state.values()}')
        if all(state == 'STATE_SWITCH_ON_DISABLED' for state in self.state.values()):
            self.get_logger().info('All drives are in STATE_SWITCH_ON_DISABLED (Try Turn Off completed)')
            self.try_turn_off_in_execution = False
            return True
        else:
            return False

    def start_perform_process(self):
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
        try:
            result = future.result()
            if result:
                self.get_logger().info(f'Mode switched for {dof_name}: {result.return_message}')
            else:
                self.get_logger().error(f'Failed to switch mode for {dof_name}')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    def switch_mode_checker(self, dof_name, target_mode):
        if not self.switch_mode_in_execution[dof_name]:
            return False
        if get_op_mode_number(self.mode[dof_name]) == target_mode:
            self.get_logger().info(f'{dof_name} is in target mode {target_mode} (Switch Mode Terminated)')
            self.switch_mode_in_execution[dof_name] = False
            return True
        else:
            return False

    def set_target_moo(self, target_mode):
        if not self.set_target_moo_in_execution:
            self.set_target_moo_in_execution = True
            self.switch_mode_in_execution = {dof: False for dof in self.dof_names}
            self.get_logger().info(f'Setting target cyclic mode to {target_mode} for all DOFs...')
        for curr_dof in self.dof_names:
            self.switch_mode(curr_dof, get_op_mode_number(target_mode))
        # while not self.all_in_target_state_mode(self.target_cyclic_mode):
        #     time.sleep(0.1)
        # self.shutdown_node()
    
    def set_target_moo_checker(self, target_mode):
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
            return all(state == 'STATE_OPERATION_ENABLED' for state in self.state.values())
                # return True
            # else:
            #     for dof, state in self.state.items():
            #         self.get_logger().info(f'{dof} is in MODE_HOMING but has unexpected state: {state}')
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
            ##############################################################
            if not node.homing_process_started:
                if any( n == 'STATE_FAULT' for n in node.state.values()):
                    node.get_logger().info("Detected FAULT state - Resetting Faults", throttle_duration_sec=5.0)
                    node.reset_fault()

                elif any( n == 'STATE_NOT_READY_TO_SWITCH_ON' for n in node.state.values()):
                    node.get_logger().info("Detected NOT_READY_TO_SWITCH_ON state", throttle_duration_sec=5.0)
                    pass
                elif any( n in ['STATE_SWITCH_ON_DISABLED', 'STATE_READY_TO_SWITCH_ON'] for n in node.state.values()):
                    node.get_logger().info("Detected DISABLED state - Try to turn on the motors", throttle_duration_sec=5.0)
                    node.try_turn_on()
                else:
                    for dof in node.dof_names:
                        if node.mode[dof] == 'MODE_HOMING':
                            pass
                        else:
                            node.get_logger().info('Drives turned on but not in homing mode, setting it.', throttle_duration_sec=5.0)
                            if node.switch_mode(dof, get_op_mode_number('MODE_HOMING'))<0:
                                node.get_logger().error(f'Failed to switch {dof} to homing mode, retrying...')
                                continue
                if node.all_in_homing_mode():
                    node.start_perform_process()
            ##############################################################
            else:      
                timeout_sec = 20.0
                node.get_logger().info('Homing Process Started, waiting for completion...', once=True)
                current_time = node.get_clock().now()
                elapsed = (current_time - node.homing_process_starting_time).nanoseconds / 1e9  # convert to seconds
                if elapsed > timeout_sec:
                    node.get_logger().error('Timeout while waiting for homing to complete')
                    node.try_turn_off()
                    if node.try_turn_off_checker():
                        node.get_logger().info('Drives turned off successfully, Try to perform homing again ...')
                        node.homing_process_started = True
                        node.homing_process_attained = False
                else:
                    if all((status_word & (1 << 12)) != 0 for status_word in node.status_words.values()):
                        node.homing_process_attained = True
                    if node.homing_process_attained and not node.set_target_moo_in_execution:
                        node.get_logger().info('!!!!!!  Homing Performed Successfully !!!!!!', once=True)
                        node.set_target_moo(node.target_cyclic_mode)
                    if node.homing_process_attained and node.set_target_moo_in_execution:
                        if node.set_target_moo_checker(node.target_cyclic_mode): # node.set_target_moo_in_execution becames False
                            node.get_logger().info('Homing process ended successfully, switching to target cyclic mode.', throttle_duration_sec=5.0 )
                            node.try_turn_off()
                    if node.homing_process_attained and node.set_target_moo_in_execution:
                        if node.try_turn_off_checker():
                            node.get_logger().info('Drives turned off successfully, exiting...')
                            break                

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    finally:
        node.destroy_node()  # Clean up the node
        rclpy.shutdown()     # Shutdown the ROS 2 context

if __name__ == '__main__':
    main()
