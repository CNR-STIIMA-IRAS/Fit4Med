import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from time import sleep


class ControllersChecker(Node):
    def __init__(self):
        super().__init__('ros_controllers_checker')
        
        # Publisher to log active controllers
        self.log_publisher_ = self.create_publisher(String, '/ros_controllers_checker/active_controllers', 10)

        # Publisher for gpio_command_controller
        self.gpio_publisher_ = self.create_publisher(
            DynamicInterfaceGroupValues,
            '/gpio_command_controller/commands',
            10
        )

        # List of active controllers
        self.active_controllers = []
        
        # Create the client for ListControllers service
        self.client_ = self.create_client(ListControllers, '/controller_manager/list_controllers')
        
        # Create a timer to check the status of controllers
        self.timer = self.create_timer(1.0, self.check_controllers_status)

        # Wait for the service to be available at startup
        self.wait_for_service()

    def wait_for_service(self):
        if not self.client_.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/list_controllers not available.')
            rclpy.shutdown()
        else:
            # Call service to list active controllers
            self.list_active_controllers()

    def list_active_controllers(self):
        # Call the service to list controllers
        future = self.client_.call_async(ListControllers.Request())
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.active_controllers = [controller.name for controller in response.controller if controller.state == 'active']
            self.get_logger().info(f"Active controllers: {', '.join(self.active_controllers)}")
            
            # Log active controllers
            log_msg = String()
            log_msg.data = ', '.join(self.active_controllers)
            self.log_publisher_.publish(log_msg)
        except Exception as e:
            self.get_logger().error(f"Error calling service: {e}")

    def check_controllers_status(self):
        if not self.active_controllers:
            self.get_logger().warn("No active controllers found, skipping status check.")
            return
        
        # Call the service periodically to check if controllers are still active
        future = self.client_.call_async(ListControllers.Request())
        future.add_done_callback(self.update_controllers_status)

    def update_controllers_status(self, future):
        try:
            response = future.result()
            current_active_controllers = {controller.name for controller in response.controller if controller.state == 'active'}
            
            # Check if any active controller has changed state to 'inactive' or 'error'
            inactive_controllers = set(self.active_controllers) - current_active_controllers
            if inactive_controllers:
                self.get_logger().warn(f"Controllers {', '.join(inactive_controllers)} are no longer active. Resetting GPIO.")
                self.publish_gpio_command(0.0)  # Set gpio interface to 0.0
                self.active_controllers = list(current_active_controllers)  # Update active controllers list
        except Exception as e:
            self.get_logger().error(f"Error calling service: {e}")

    def publish_gpio_command(self, value):
        msg = DynamicInterfaceGroupValues(
            interface_groups=['safe_output'],
            interface_values=[
                InterfaceValue(
                    interface_names=['d_output.1'],
                    values=[value]
                )
            ]
        )
        self.gpio_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllersChecker() 
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')


if __name__ == '__main__':
    main()
