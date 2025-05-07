import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy

class LaunchStatusPublisher(Node):
    def __init__(self):
        super().__init__('launch_status_publisher')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(String, '/launch_status', qos)
        self.status = ''
        self.timer = self.create_timer(0.5, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.pub.publish(msg)

    def set_ready(self, names):
        self.get_logger().info("Setting launch status to True")
        self.status = ';'.join(names)
