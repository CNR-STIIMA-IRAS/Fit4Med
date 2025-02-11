import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from ethercat_controller_msgs.msg import Cia402DriveStates
from std_msgs.msg import Bool


class EthercatCheckerNode(Node):
    def __init__(self):
        super().__init__('ethercat_checker_node')
        self.get_logger().info('ethercat checker node started')

        # Define topic callback group
        topic_group = MutuallyExclusiveCallbackGroup()
        
        # Subscription to drive status updates
        self.subscription = self.create_subscription(Cia402DriveStates, '/state_controller/drive_states', self.handle_drive_status, qos_profile=10, callback_group=topic_group)

        # Create the no-error publisher
        self.publisher_ = self.create_publisher(Bool, '/ethercat_error_check', 10)

        self.dof_names = ['joint_1'] #, 'joint_2', 'joint_3']
        self.state = {dof: 'STATE_UNDEFINED' for dof in self.dof_names}
        self.mode = {dof: 'MODE_NO_MODE' for dof in self.dof_names}
        self.mode_target = 8  # MODE_CYCLIC_SYNC_POSITION
        self.shutdown_initiated = False  # To track if shutdown is already requested

    def handle_drive_status(self, msg):
        msg_to_pub = Bool()
        for dof in self.dof_names:
            try:
                index = msg.dof_names.index(dof)
                self.state[dof] = msg.drive_states[index]
                self.mode[dof] = msg.modes_of_operation[index]
            except ValueError:
                self.get_logger().error(f'DoF {dof} not found in received message')
                continue
        if 'STATE_FAULT' in self.state.values():
            self.get_logger().info("Detected FAULT state")
            self.reset_fault()
            msg_to_pub.data = False
            self.publisher_.publish(msg_to_pub)
        elif 'STATE_SWITCH_ON_DISABLED' in self.state.values():
            self.get_logger().info("Detected DISABLED state")
            self.try_turn_on()
            msg_to_pub.data = False
            self.publisher_.publish(msg_to_pub)
        else:
            counter = 0
            for dof in self.dof_names:
                if (self.state[dof] == 'STATE_SWITCH_ON') or (self.state[dof] == 'STATE_SWITCH_ON_ENABLED') or (self.state[dof] == 'STATE_OPERATION_ENABLED'):
                    counter += 1
            if counter == len(self.dof_names):
                msg_to_pub.data = True
                self.publisher_.publish(msg_to_pub)
            else:
                self.get_logger().info(f'Found ethercat slave in {self.state[dof]} state. Not managed.')
                msg_to_pub.data = False
                self.publisher_.publish(msg_to_pub)

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


def main(args=None):
    rclpy.init(args=args)

    node = EthercatCheckerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

if __name__ == '__main__':
    main()
