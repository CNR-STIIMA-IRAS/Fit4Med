import rclpy
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ethercat_controller_msgs.msg import Cia402DriveStates
from ethercat_controller_msgs.srv import GetModesOfOperation
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

_shutdown_request = False

class EthercatCheckerNode(Node):
    def __init__(self):
        super().__init__('ethercat_checker_node')
        self.get_logger().info('ethercat checker node started')

        # Define callback groups
        driver_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()
        
        # Subscription to drive status updates
        self.subscription = self.create_subscription(Cia402DriveStates, '/state_controller/drive_states', self.handle_drive_status, qos_profile=10, callback_group=driver_group)

        # Create the no-error publisher
        self.publisher_ = self.create_publisher(Bool, '/ethercat_error_check', 10)

        # Create error checking enable service
        self.enable_error_check_srv = self.create_service(
            SetBool,
            '/ethercat_checker/enable_error_checking',
            self.enable_error_checking_callback
        )

        self.get_mode_of_op_srv = self.create_service(
            GetModesOfOperation,
            '/ethercat_checker/get_drive_mode_of_operation',
            self.get_modes_callback,
            callback_group=service_group
        )

        self.shutdown_service = self.create_service(
            Trigger,
            "/ethercat_checker/request_shutdown", self.shutdown_node
        )

        self.error_checking_enabled = True
        self.dof_names = ['joint_x', 'joint_y', 'joint_z']
        self.state = {dof: 'STATE_UNDEFINED' for dof in self.dof_names}
        self.mode = {dof: 'MODE_NO_MODE' for dof in self.dof_names}
        self.status_words = {dof: 0 for dof in self.dof_names}
        self.fault_reset_in_execution = False
        self.try_turn_off_in_execution = False
        self.try_turn_on_in_execution = False

    def enable_error_checking_callback(self, request, response):
        self.error_checking_enabled = request.data
        response.success = True
        response.message = f"Error checking {'enabled' if request.data else 'disabled'}."
        return response

    def handle_drive_status(self, msg):
        for dof in self.dof_names:
            try:
                index = msg.dof_names.index(dof)
                self.state[dof] = msg.drive_states[index]
                self.mode[dof] = msg.modes_of_operation[index]
                self.status_words[dof] = msg.status_words[index]
            except ValueError:
                self.get_logger().error(f'DoF {dof} not found in received message')

    def get_modes_callback(self, request, response):
        if_names = request.dof_names

        # Se lista vuota, restituisci tutte
        if not if_names:
            if_names = self.dof_names

        response.dof_names = []
        response.values = []
        response.status_words = []

        for name in if_names:
            if name in self.dof_names:
                response.dof_names.append(name)
                response.values.append(self.get_op_mode_number(self.mode[name]))
                response.status_words.append(self.status_words[name])
            else:
                self.get_logger().warn(f"DOF named '{name}' not found")

        return response

    def check_faults(self):
        msg_to_pub = Bool()
        if 'STATE_FAULT' in self.state.values():
            self.get_logger().error("Detected FAULT state - Trying to resetting faults...", throttle_duration_sec=1)
            self.reset_fault()
            msg_to_pub.data = False
            self.publisher_.publish(msg_to_pub)
        # elif 'STATE_SWITCH_ON_DISABLED' in self.state.values():
        #     self.get_logger().info("Detected DISABLED state - Trying to turn on", throttle_duration_sec=1)
        #     self.try_turn_on()
        #     msg_to_pub.data = False
        #     self.publisher_.publish(msg_to_pub)
        else:
            counter = 0
            for dof in self.dof_names:
                if (self.state[dof] == 'STATE_SWITCH_ON') or (self.state[dof] == 'STATE_SWITCH_ON_ENABLED') or (self.state[dof] == 'STATE_OPERATION_ENABLED'):
                    counter += 1
            if counter == len(self.dof_names):
                msg_to_pub.data = True
                self.publisher_.publish(msg_to_pub)
            else:
                self.get_logger().info(f'Found ethercat slave in {self.state[dof]} state. Not managed.', throttle_duration_sec=1)
                msg_to_pub.data = False
                self.publisher_.publish(msg_to_pub)

    def reset_fault(self):
        if self.fault_reset_in_execution:
            self.get_logger().warn('Fault reset already in execution, skipping...', throttle_duration_sec=1)
            return
        
        self.fault_reset_in_execution = True
        fault_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/reset_fault', callback_group=fault_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.reset_fault_callback)
        else:
            self.get_logger().error('Service /state_controller/reset_fault not available')
            self.fault_reset_in_execution = False

    def reset_fault_callback(self, future):
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

    def try_turn_on(self):
        if self.try_turn_on_in_execution:
            self.get_logger().warn('Try turn on already in execution, skipping...', throttle_duration_sec=5)
            return
        self.try_turn_on_in_execution = True
        try_on_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/try_turn_on', callback_group=try_on_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.try_turn_on_callback)
        else:
            self.get_logger().error('Service /state_controller/try_turn_on not available')
            self.try_turn_on_in_execution = False

    def try_turn_on_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turned on successfully')
            else:
                self.get_logger().error('Failed to turn on drives')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
        self.try_turn_on_in_execution = False
    
    def try_turn_off(self):
        if self.try_turn_off_in_execution:
            self.get_logger().warn('Try turn off already in execution, skipping...')
            return
        self.try_turn_off_in_execution = True
        self.get_logger().info('Turning on drives...')
        try_on_service_group = MutuallyExclusiveCallbackGroup()
        client = self.create_client(Trigger, '/state_controller/try_turn_off', callback_group=try_on_service_group)
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.try_turn_off_callback)
        else:
            self.get_logger().error('Service /state_controller/try_turn_off not available')
            self.try_turn_off_in_execution = False

    def try_turn_off_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Drives turned on successfully')
            else:
                self.get_logger().error('Failed to turn on drives')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')
        self.try_turn_off_in_execution = False
    
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
    
    def shutdown_node(self, request, response):
        global _shutdown_request
        _shutdown_request = True
        response.success = True
        return response
    #     self.get_logger().info('Shutting down...')
    #     time.sleep(2)
    #     self.destroy_node()

    #     # Ensure context is valid before shutting down
    #     if rclpy.ok():
    #         rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    import os
    os.sched_setaffinity(0, {6})

    node = EthercatCheckerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while not _shutdown_request:
            executor.spin_once()
            if node.error_checking_enabled:
                node.check_faults()
            
        node.get_logger().info('^^^^^^^^^^^^^^^^^ Shutting down...')
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        executor.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
