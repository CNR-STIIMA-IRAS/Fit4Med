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

        # Publisher for gpio_command_controller
        self.gpio_publisher_ = self.create_publisher(
            DynamicInterfaceGroupValues,
            '/gpio_command_controller/commands',
            10
        )

        # Create and publish initial GPIO message
        self.publish_gpio_command(1.0)
        
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
        self.shutdown_node()
        
    def shutdown_node(self):
        self.get_logger().info('gpio error state disabled, shutting down...')
        self.destroy_publisher(self.gpio_publisher_)
        self.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    GPIOCommandPublisher() 

if __name__ == '__main__':
    main()
