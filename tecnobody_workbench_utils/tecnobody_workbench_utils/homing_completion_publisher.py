import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HomingCompletionPublisher(Node):
    def __init__(self):
        super().__init__('homing_completion_publisher')
        self.publisher_ = self.create_publisher(Bool, '/homing_done', 10)
        
        timer_group = MutuallyExclusiveCallbackGroup()
        shutdown_group = MutuallyExclusiveCallbackGroup()
        
        self.timer = self.create_timer(0.5, self.publish_completion, callback_group=timer_group)
        self.shutdown_timer = self.create_timer(5.0, self.shutdown_node, callback_group=shutdown_group)
        self.get_logger().info('homing_completion_publisher initialized.')
        
    def publish_completion(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Homing confirmation published: {msg.data}')

    def shutdown_node(self):
        self.get_logger().info('Shutting down homing completion publisher...')
        self.destroy_timer(self.timer)
        self.destroy_timer(self.shutdown_timer)
        self.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    homing_completion_publisher = HomingCompletionPublisher()
    
    executor = MultiThreadedExecutor()
    executor.add_node(homing_completion_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

if __name__ == '__main__':
    main()
