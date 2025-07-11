import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from plc_controller_msgs.msg import PlcController
import time
import subprocess
import os
import signal

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class PLCControllerInterface(Node):
    def __init__(self):
        super().__init__('plc_manager')
        
        plc_group = MutuallyExclusiveCallbackGroup()
        launch_group = MutuallyExclusiveCallbackGroup()

        # Subscriber to the PLC controller state interface
        self.state_subscriber_callback_running = False
        self.state_subscriber = self.create_subscription(
            PlcController,
            '/safety_plc/PLC_controller/plc_states',
            self.state_callback,
            10,
            callback_group=plc_group
        )

        # Publisher to the PLC controller command interface
        self.command_publisher = self.create_publisher(
            PlcController,
            '/safety_plc/PLC_controller/plc_commands',
            10
        )
        
        # Subscriber to the /launch_status topic
        self.launch_status_subscriber = self.create_subscription(
            String,
            '/tecnobody_fake_hardware/launch_status',
            self.launch_status_callback,
            10,
            callback_group=launch_group
        )

        # Internal storage for PLC interfaces names and values
        self.interface_names = []
        self.state_values = []
        self.command_values = []
        self.launch_status = []
        
        self.ESTOP = 0
        
    def launch_status_callback(self, msg):
        """Callback to handle messages from /launch_status."""
        self.launch_status = []
        self.launch_status.extend(msg.data.split(";"))
        
    def state_callback(self, msg):
        """Callback to handle incoming state messages."""

        # Store the interface names from the state message
        self.interface_names = list(msg.interface_names)
        # Store the uint8 values from the state message
        self.state_values = list(msg.values)
        
        for int_idx in range(len(self.interface_names)):
            if self.interface_names[int_idx] == "estop":
                if self.state_values[int_idx] == 1 and self.ESTOP == 0:
                    # self.get_logger().info(bcolors.OKBLUE + "************************ INPUT 1 and ESTOP 0 => ACTIVATE" + bcolors.ENDC)
                    subprocess.Popen([" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh"], shell=True, executable="/bin/bash")
                elif self.state_values[int_idx] == 0 and self.ESTOP  == 1:
                    # self.get_logger().info(bcolors.OKCYAN + "************************ INPUT 0 and ESTOP 1 => KILL " + bcolors.ENDC)
                    if len(self.launch_status) > 1:                       
                        for node in self.launch_status:
                                pid_str = subprocess.run(f"ps aux | grep {node} | grep -v grep | awk '{{print $2}}'", shell=True, capture_output=True, text=True)
                                pid = pid_str.stdout.strip()
                                
                                # Kill the process using SIGINT
                                self.get_logger().info("Killing ROS2 node with pid: {}".format(pid))
                                os.kill(int(pid), signal.SIGINT)               
                        self.get_logger().info("All targeted processes signaled.")
                    else:
                        self.get_logger().info("no nodes in launch status topic.")
                elif self.state_values[int_idx] == 0 and self.ESTOP  == 0:
                    # self.get_logger().info(bcolors.OKGREEN + "************************ INPUT 0 and ESTOP 0 => PASS " + bcolors.ENDC)
                    pass
                elif self.state_values[int_idx] == 1 and self.ESTOP  == 1:
                    # self.get_logger().info(bcolors.FAIL + "************************ INPUT 1 and ESTOP 1 => PASS " + bcolors.ENDC)
                    pass
                self.ESTOP = self.state_values[int_idx]
        
    def publish_command(self, names, values):
        # Create the command message
        command_msg = PlcController()
        command_msg.values = values 
        command_msg.interface_names = names

        # Publish
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f"Published command message: {command_msg.values}")


def main(args=None):
    rclpy.init(args=args)
    node = PLCControllerInterface()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PLC Controller Interface node.")
        pid_str = subprocess.run(f"ps aux | grep {node} | grep -v grep | awk '{{print $2}}'", shell=True, capture_output=True, text=True)
        pid = pid_str.stdout.strip()
        # Kill the process using SIGINT
        node.get_logger().info("Killing ROS2 node with pid: {}".format(pid))
        os.kill(int(pid), signal.SIGINT)
    finally:
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown()

if __name__ == '__main__':
    main()