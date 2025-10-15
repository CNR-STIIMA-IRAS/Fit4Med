import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tecnobody_msgs.msg import PlcController
from std_srvs.srv import Trigger

_shutdown_request = False

class SonarTeachNode(Node):
    def __init__(self):
        super().__init__('sonar_teach_node')

        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # Publisher to the PLC controller command interface
        self.command_publisher = self.create_publisher(
            PlcController,
            '/PLC_controller/plc_commands',
            10
        )

        self.trigger_teach_service = self.create_service(
            Trigger,
            "/sonar_teach_node/enable_sonar_teach", 
            self.sonar_teach_enable_callback,
            callback_group=self.service_group
        )

        self.shutdown_service = self.create_service(
            Trigger,
            "/sonar_teach_node/request_shutdown", self.shutdown_node
        )
        self.stay_awake_timer = self.create_timer(10.0, lambda: True, self.timer_group)
        self.get_logger().info("class correctly initialized.")

    def sonar_teach_enable_callback(self, request, response):
        self.get_logger().info("Received sonar teach request")
        self.start_sonar_teach()
        response.success = True
        return response

    def publish_command(self, n, v):
        command_msg = PlcController()
        command_msg.interface_names = [n]
        command_msg.values = [v]
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f"Published command message: {command_msg.values}")

    def start_sonar_teach(self):
        self.get_logger().info("Starting sonar teach process")
        self.sonar_teach_active = True
        start_time = self.get_clock().now()
        self.teach_timer = self.create_timer(0.05, lambda s = start_time : self.teach_progress(s), callback_group=self.timer_group)

    def teach_progress(self, start_time): 
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9
            # Sonar teaching logic
            if elapsed_time < 3.0: 
                self.publish_command('PLC_node/sonar_teach', 1)
            elif elapsed_time > 3.0 and elapsed_time < 5.0:
                self.publish_command('PLC_node/sonar_teach', 0)
            elif elapsed_time > 5.0 and elapsed_time < 8.0:
                self.publish_command('PLC_node/sonar_teach', 1)
            else:
                self.publish_command('PLC_node/sonar_teach', 0)
                self.get_logger().info("Teaching process completed.", once=True)
                self.teach_timer.cancel()

    def shutdown_node(self, request, response):
        global _shutdown_request
        _shutdown_request = True
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SonarTeachNode()

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)

    try:
        while rclpy.ok() and not _shutdown_request:
            executor.spin_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down sonar teaching node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
