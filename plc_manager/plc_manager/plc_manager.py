import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from plc_controller_msgs.msg import PlcController
import os
import signal
import subprocess


class PLCControllerInterface(Node):
    def __init__(self):
        super().__init__('plc_manager')

        # Subscriber to the PLC controller state interface
        self.state_subscriber = self.create_subscription(
            PlcController,
            '/PLC_controller/plc_states',
            self.state_callback,
            10
        )

        # Publisher to the PLC controller command interface
        self.command_publisher = self.create_publisher(
            PlcController,
            '/PLC_controller/plc_commands',
            10
        )

        # Internal storage for PLC interfaces names and values
        self.interface_names = []
        self.state_values = []
        self.pids = []
        self.nodes_diff = []

        self.RESET_MAIN = True

    def state_callback(self, msg):
        """Callback to handle incoming state messages."""
        
        # Store the interface names from the state message
        self.interface_names = list(msg.interface_names)

        # Store the uint8 values from the state message
        self.state_values = list(msg.values)
        
        for int_idx in range(len(self.interface_names)):
            if self.interface_names[int_idx] == "s_input.1":
                if self.state_values[int_idx] == 1 and self.RESET_MAIN:
                    
                    nodes_before_str = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
                    nodes_before = list(filter(None, nodes_before_str.stdout.strip().split('\n')))
                    nodes_before = [node.lstrip("/") for node in nodes_before]
                    # self.get_logger().info("node: {}".format(nodes_before))
                    
                    subprocess.Popen(["/home/tartaglia/ros2_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh"], shell=True, executable="/bin/bash")
                    nodes_after_str = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
                    nodes_after = list(filter(None, nodes_after_str.stdout.strip().split('\n')))
                    nodes_after = [node.lstrip("/") for node in nodes_after]
                    
                    self.nodes_diff = list(set(nodes_after) - set(nodes_before))
                    
                    for node in self.nodes_diff:
                        self.get_logger().info("node: {}".format(node))
                        pid = subprocess.run(f"ps aux | grep {node} | grep -v grep | awk '{{print $2}}'",shell=True, capture_output=True, text=True)
                        self.pids.append(pid.stdout)
                    
                    self.RESET_MAIN = False
        self.get_logger().info("nodes_diff: {}".format(self.nodes_diff))
        self.get_logger().info("PIDS: {}".format(self.pids))
                    
        
                    
    def publish_command(self):
        """Publish a command message based on the stored state values."""
        if not self.state_values:
            self.get_logger().warn("No state values received yet, skipping command publish.")
            return

        # Create a command message
        command_msg = PlcController()
        command_msg.values = self.state_values  # Reuse the state values for now
        command_msg.interface_names = []  # Populate with appropriate interface names if needed

        # Publish the command message
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f"Published command message: {command_msg.values}")


def main(args=None):
    rclpy.init(args=args)
    node = PLCControllerInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PLC Controller Interface node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()