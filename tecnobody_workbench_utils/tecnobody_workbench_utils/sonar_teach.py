import time
import rclpy
from rclpy.node import Node
from tecnobody_msgs.msg import PlcController
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

class SonarTeachNode(Node):
    def __init__(self):
        super().__init__('sonar_teach_node')

        self.sonar_teach_enable_sub = self.create_subscription(
            Bool,
            '/sonar_teach/enable',
            self.sonar_teach_enable_callback,
            10
        )


        # Publisher to the PLC controller command interface
        self.command_publisher = self.create_publisher(
            PlcController,
            '/PLC_controller/plc_commands',
            10,
        )

        self.command_msg = PlcController()
        self.command_msg.values = [0] * 8
        self.command_msg.interface_names = [ 'PLC_node/mode_of_operation', 
                                            'PLC_node/power_cutoff', 
                                            'PLC_node/sonar_teach', 
                                            'PLC_node/s_output.4', 
                                            'PLC_node/estop', 
                                            'PLC_node/manual_mode', 
                                            'PLC_node/force_sensors_pwr', 
                                            'PLC_node/s_output.8' ]

        # self.trigger_teach_service = self.create_service(
        #     Trigger,
        #     "/ethercat_checker/request_shutdown", lambda req, resp: self.start_sonar_teach()
        # )

        self.start_time = -999.0
        self.sonar_teach_active = False
        self.sonar_teach_completed = False
        self.sonar_teach_enabled = False

    def sonar_teach_enable_callback(self, msg):
        self.sonar_teach_enabled = msg.data


    def publish_command(self, n, v):
        if n in self.command_msg.interface_names:
            idx = self.command_msg.interface_names.index(n)
            self.command_msg.values[idx] = v
            self.command_publisher.publish(self.command_msg)
            self.get_logger().info(f"Published command message: {self.command_msg.values}")
        else:
            self.get_logger().warn(f"FUNCTION publish_command Interface name '{n}' not found in command message.")

    def start_sonar_teach(self, start_time):
        self.sonar_teach_active = True

        if not self.sonar_teach_completed:
            self.get_logger().info(f"[Sonar Teach] time since start: {time.time() - start_time} secs")
            # Sonar teaching logic
            if time.time()- start_time < 3.0: 
                self.publish_command('PLC_node/sonar_teach', 1)
            elif time.time() - start_time > 3.0 and time.time() - start_time < 4.0:
                self.publish_command('PLC_node/sonar_teach', 0)
            elif time.time() - start_time > 4.0 and time.time() - start_time < 5.0:
                self.publish_command('PLC_node/sonar_teach', 1)
            else:
                self.sonar_teach_completed = True


def main(args=None):
    rclpy.init(args=args)

    node = SonarTeachNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            if node.sonar_teach_enabled and not node.sonar_teach_active and not node.sonar_teach_completed:
                node.get_logger().info("Starting sonar teach process.")
                node.start_time = time.time()
                node.sonar_teach_completed = False
                node.start_sonar_teach(node.start_time)
            elif node.sonar_teach_enabled and node.sonar_teach_active and not node.sonar_teach_completed:
                if node.start_time > 0:
                    node.start_sonar_teach(node.start_time)
                else:
                    node.get_logger().warn("Sonar teach process active but start time not set.")
            elif not node.sonar_teach_enabled and node.sonar_teach_active and not node.sonar_teach_completed:
                node.get_logger().info("Sonar teach process disabled.", once= True)
                node.publish_command('PLC_node/sonar_teach', 0)
            elif node.sonar_teach_enabled and node.sonar_teach_active and node.sonar_teach_completed:
                node.get_logger().info("Sonar teach process finished!", once= True)
                node.publish_command('PLC_node/sonar_teach', 0)
            elif not node.sonar_teach_enabled and node.sonar_teach_active and node.sonar_teach_completed:
                node.sonar_teach_active = False
                node.sonar_teach_completed = False
            else:
                pass

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PLC Controller Interface node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
