import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tecnobody_msgs.msg import PlcController, PlcStates
import sys
import subprocess
import threading
import os
import signal
import socket
import socket

class UdpClient:
    def __init__(self, target_ip="127.0.0.1", target_port=5005, local_port=0, timeout=2.0):
        """
        target_ip: indirizzo server
        target_port: porta server
        local_port: porta locale su cui il client riceve (0 = porta casuale)
        timeout: timeout recv (in secondi)
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", local_port))   # necessario per ricevere
        self.sock.settimeout(timeout)

    def send(self, message: bytes):
        """Invia un messaggio al server"""
        self.sock.sendto(message, (self.target_ip, self.target_port))

    def receive(self, bufsize=4096):
        """Aspetta una risposta dal server (bloccante fino a timeout)"""
        try:
            data, addr = self.sock.recvfrom(bufsize)
            print(f"[UdpClient] Received message from {addr}: {data}")
            return data, addr
        except socket.timeout:
            return None, None

    def close(self):
        self.sock.close()


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
    def __init__(self, target_ip):
        super().__init__('plc_manager')
        
        self.shutdown_all = False
        plc_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()

        # Subscriber to the PLC controller state interface
        self.state_subscriber_callback_running = False
        self.state_subscriber = self.create_subscription(
            PlcStates,
            '/PLC_controller/plc_states',
            self.state_callback,
            10,
            callback_group=plc_group
        )

        # Publisher to the PLC controller command interface
        self.command_publisher = self.create_publisher(
            PlcController,
            '/PLC_controller/plc_commands',
            10,
        )

        # Internal storage for PLC interfaces names and values
        self.interface_names = []
        self.state_values = []
        self.command_values = []
        self.launch_status = []
        self.ESTOP = 0
        self.lock = threading.Lock()
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
        
        # Check if the ros2 controllers are launched
        self.ros_launched = False    
        self.ros_launched_prev = False    
        self.check_ros_status_timer = self.create_timer(1.0, self.check_ros_launch_status, timer_group)
        self.FIRST_TIME = False

        time.sleep(2)

        self.publish_command('PLC_node/estop', 1)
        self.publish_command('PLC_node/force_sensors_pwr', 1)

        # UDP client to send status messages
        self.client = UdpClient(target_ip, 5005) #10.2.16.43
        
    def check_ros_launch_status(self):
        launcher_name = "run_platform_control.launch.py"   
        try:
            output = subprocess.check_output(["pgrep", "-af", "ros2 launch"], text=True)
            for line in output.splitlines():
                if launcher_name in line:
                    if not self.ros_launched:
                        self.get_logger().info('✅ Ros control launcher detected!', throttle_duration_sec=20.0)
                        self.ros_launched = True
                        self.ros_launched_prev = self.ros_launched
                    return True
            self.get_logger().info('🕓 Waiting for ros controllers to start!', throttle_duration_sec=5.0)
            self.ros_launched = False
            if (self.ros_launched_prev != self.ros_launched) and (self.ros_launched_prev == 1 and self.ros_launched == 0):
                    self.get_logger().info('Call a SW ESTOP!', throttle_duration_sec=5.0)
                    # open the emergency => emergency status will become 0
                    self.publish_command('PLC_node/estop', 0)
                    self.publish_command('PLC_node/manual_mode', 0)
                    time.sleep(0.05)
                    self.publish_command('PLC_node/estop', 1)
            self.ros_launched_prev = self.ros_launched
            return self.ros_launched
        except subprocess.CalledProcessError:
            # pgrep returns nonzero if no processes found
            self.ros_launched = False
            return False
        
    def state_callback(self, msg):
        """Callback to handle incoming state messages."""
        if not self.lock.acquire(blocking=False):
            self.get_logger().warn("state callback already in execution, ignore.")
            return
        try:
            # Store the interface names from the state message
            self.interface_names = list(msg.interface_names)
            # Store the uint8 values from the state message
            self.state_values = list(msg.values)
            # Check for loaded and active controllers
            for int_idx in range(len(self.interface_names)):
                if self.interface_names[int_idx] == "estop":
                    if self.state_values[int_idx] == 1 and self.ESTOP == 0:
                        self.get_logger().info(bcolors.OKBLUE + "************************ INPUT 1 and ESTOP 0 => ACTIVATE" + bcolors.ENDC)
                        if not self.FIRST_TIME:
                            subprocess.Popen([" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh --perform-homing"], shell=True, executable="/bin/bash")
                            self.FIRST_TIME = True
                        else:
                            subprocess.Popen([" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh"], shell=True, executable="/bin/bash")

                    elif self.state_values[int_idx] == 0 and self.ESTOP == 1:
                        self.get_logger().info(bcolors.OKCYAN + f"************************ INPUT 0 and ESTOP 1 => KILL" + bcolors.ENDC)
                        try:
                            # _ = subprocess.check_output(["pgrep", "-f", "ros2 run rehab"])
                            self.client.send(b"STOP")
                            timeout = time.time() + 10
                            data, addr = self.client.receive()
                            while data != b"STOPPED":
                                time.sleep(0.5)
                                if time.time() > timeout:
                                    self.get_logger().warn("PLC Manager: No acknowledgment from FMRR GUI for STOP command.", throttle_duration_sec=5.0)
                                    break
                        except subprocess.CalledProcessError:
                            self.get_logger().info("No rehab GUI process found.")

                        try:
                            platform_launcher_pid = subprocess.check_output(["pgrep", "-f", "ros2 launch tecnobody_workbench run_platform_control.launch.py"]) 
                            os.kill(int(platform_launcher_pid), signal.SIGINT)
                            time.sleep(2)
                        except subprocess.CalledProcessError:
                            self.get_logger().info("No platform_control.launch.py process found.")
        
                    elif self.state_values[int_idx] == 0 and self.ESTOP  == 0:
                        pass

                    elif self.state_values[int_idx] == 1 and self.ESTOP  == 1:
                        try:
                            node_list = self.get_node_names()
                            if 'tecnobody_ethercat_checker_node' in node_list:
                                self.client.send(b"RUNNING")
                        except Exception as e:
                            self.get_logger().error(f"Exception when sending message to UDP Server: {e}")                            
                        pass
                    self.ESTOP = self.state_values[int_idx]
                
        finally:
            self.lock.release()
        
    def publish_command(self, n, v):
        if n in self.command_msg.interface_names:
            idx = self.command_msg.interface_names.index(n)
            self.command_msg.values[idx] = v
            self.command_publisher.publish(self.command_msg)
            self.get_logger().info(f"Published command message: {self.command_msg.values}")
        else:
            self.get_logger().warn(f"FUNCTION publish_command Interface name '{n}' not found in command message.")
        

def main(args=None):
    rclpy.init(args=args)
    target_ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.0"


    node = PLCControllerInterface(target_ip)
    mt_executor = MultiThreadedExecutor(2)
    mt_executor.add_node(node)

    import os
    os.sched_setaffinity(0, {2})
    try:
        while rclpy.ok() and not node.shutdown_all:
            mt_executor.spin_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PLC Controller Interface node.")
    finally:
        node.client.close()
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()