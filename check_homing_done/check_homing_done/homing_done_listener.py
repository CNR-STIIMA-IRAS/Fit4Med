import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class HomingDoneListener(Node):

    def __init__(self):
        super().__init__('homing_done_listener')
        listener_group = MutuallyExclusiveCallbackGroup()
        self.shutdown_group = MutuallyExclusiveCallbackGroup()
        
        self.subscription = self.create_subscription(
            Bool,
            '/homing_done',
            self.listener_callback,
            10,
            callback_group = listener_group
        )
        self.FIRST_RCV = False
        self.get_logger().info("homing_done_listener initialized.")

    def listener_callback(self, msg):
        if msg.data:
            if self.FIRST_RCV:
                self.timer = self.create_timer(5.0, self.trigger_shutdown, callback_group = self.shutdown_group)
                self.get_logger().info("homing_done_listener shutdown triggered.")
            self.FIRST_RCV = True
        else:
            self.get_logger().info("homing_done is False")

    def trigger_shutdown(self):
        self.get_logger().info('Homing completed confirmation received, shutting down...')
        self.destroy_timer(self.timer)
        self.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = HomingDoneListener()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')

if __name__ == '__main__':
    main()
