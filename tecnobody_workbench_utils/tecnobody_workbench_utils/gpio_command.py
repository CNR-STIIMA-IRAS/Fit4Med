import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from time import sleep

class GPIOCommandPublisher(Node):
    def __init__(self):
        super().__init__('gpio_command_publisher')

        # Declare output value param with default value 1.0
        self.declare_parameter('output_value', 1.0)
        output_value = self.get_parameter('output_value').get_parameter_value().double_value
        
        #  Publish active controllers for logging
        self.log_publisher_ = self.create_publisher(String, '/gpio_command_publisher/active_controllers', 10)

        # Create publisher for gpio_command_controller
        self.gpio_publisher_ = self.create_publisher(
            DynamicInterfaceGroupValues,
            '/gpio_command_controller/commands',
            10
        )

        # Create and publish message
        msg = DynamicInterfaceGroupValues(
            interface_groups=['safe_output'],
            interface_values=[
                InterfaceValue(
                    interface_names=['d_output.1'],
                    values=[output_value]
                )
            ]
        )

        self.gpio_publisher_.publish(msg)
        
        self.timer = self.create_timer(0.01, self.list_controllers_sub)

    def list_controllers_sub(self):
        list_service_group = MutuallyExclusiveCallbackGroup()
        # Create client for ListControllers service
        self.client_ = self.create_client(ListControllers, '/controller_manager/list_controllers', callback_group=list_service_group)
        self.future = self.client_.call_async(ListControllers.Request())
        self.future.add_done_callback(self.handle_response)

        if not self.client_.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /controller_manager/list_controllers not available.')
            rclpy.shutdown()
            return
        
    def handle_response(self, future):
            try:
                response = future.result()
                active_controllers = [controller.name for controller in response.controller if controller.state == 'active']

                # Log
                log_msg = String()
                log_msg.data = ', '.join(active_controllers)
                self.log_publisher_.publish(log_msg)
                
                # Gpio
                gpio_msg = DynamicInterfaceGroupValues(
                    interface_groups=['safe_output'],
                    interface_values=[
                        InterfaceValue(
                            interface_names=['d_output.1'],
                            values=[1.0]
                        )
                    ]
                )

                self.gpio_publisher_.publish(gpio_msg)            
            except Exception as e:
                self.get_logger().error(f"Error calling service: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GPIOCommandPublisher() 
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')


if __name__ == '__main__':
    main()
