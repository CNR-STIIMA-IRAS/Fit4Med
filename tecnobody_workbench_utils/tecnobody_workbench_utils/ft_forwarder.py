import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class FTForwarder(Node):
    def __init__(self):
        super().__init__('ft_forwarder')
        self.subscription = self.create_subscription(
            JointState,
            '/ft_sensor/joint_states',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )
        self.get_logger().info("FTForwarder node started, forwarding data 1:1")

    def listener_callback(self, msg):
        if not msg.position:
            self.get_logger().warn("Received JointState without position values!")
            return

        output_msg = Float64MultiArray()
        output_msg.data.append(msg.position[0])
        output_msg.data.append(msg.position[1])
        output_msg.data.append(msg.position[2])
        self.publisher.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FTForwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    except Exception as e:
        node.get_logger().error(f"Exception in node: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
