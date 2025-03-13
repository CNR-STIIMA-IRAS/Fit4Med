import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import String
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue


class GPIOCommandPublisher(Node):
    def __init__(self):
        super().__init__('gpio_command_publisher')

        # Publisher for gpio_command_controller
        self.gpio_publisher_ = self.create_publisher(
            DynamicInterfaceGroupValues,
            '/gpio_command_controller/commands',
            10
        )

        # Create a timer to publish GPIO message
        self.create_timer(0.1, self.publish_gpio_command)
        
        # Create a timer to shutdown the node
        self.create_timer(5.0, self.shutdown_node)
        
    def publish_gpio_command(self):
        msg = DynamicInterfaceGroupValues(
            interface_groups=['safe_output'],
            interface_values=[
                InterfaceValue(
                    interface_names=['d_output.1'],
                    values=[1.0]
                )
            ]
        )
        self.gpio_publisher_.publish(msg)
        
    def shutdown_node(self):
        self.destroy_timer(self.publish_gpio_command)
        self.destroy_timer(self.shutdown_node)
        self.get_logger().info('gpio error state disabled, shutting down...')
        self.destroy_publisher(self.gpio_publisher_)
        self.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


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
