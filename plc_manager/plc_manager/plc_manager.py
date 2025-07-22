import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from plc_controller_msgs.msg import PlcController
from controller_manager_msgs.srv import ListControllers, UnloadController, SwitchController
from std_srvs.srv import Trigger
import sys
import subprocess
import threading
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
        
        self.shutdown_all = False
        plc_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()

        # Subscriber to the PLC controller state interface
        self.state_subscriber_callback_running = False
        self.state_subscriber = self.create_subscription(
            PlcController,
            '/PLC_controller/plc_states',
            self.state_callback,
            10,
            callback_group=plc_group
        )

        # Publisher to the PLC controller command interface
        self.command_publisher = self.create_publisher(
            PlcController,
            '/PLC_controller/plc_commands',
            10
        )

        # Controller Manager Services subscription
        self.list_controllers_client = self.create_client(ListControllers, '/controller_manager/list_controllers', callback_group=service_group)
        client_success = self.list_controllers_client.wait_for_service(timeout_sec=5.0)
        if not client_success:
            self.get_logger().error("List controllers service is not ready.")
            self.shutdown_all = True
            return

        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        client_success = self.switch_client.wait_for_service(timeout_sec=5.0)
        if not client_success:
            self.get_logger().error("Switch controller service is not ready.")
            self.shutdown_all = True
            return
        
        self.unload_client = self.create_client(UnloadController, '/controller_manager/unload_controller')
        client_success = self.unload_client.wait_for_service(timeout_sec=5.0)
        if not client_success:
            self.get_logger().error("Unload controller service is not ready.")
            self.shutdown_all = True
            return
        
        self.eth_checker_shutdown_client = self.create_client(Trigger, '/ethercat_checker/request_shutdown')
        eth_checker_shutdown_client_success = self.eth_checker_shutdown_client.wait_for_service(timeout_sec=5.0)
        if not eth_checker_shutdown_client_success:
            self.get_logger().error("Ethercat checker shutdown service is not ready.")
            self.shutdown_all = True
            return

        # Internal storage for PLC interfaces names and values
        self.interface_names = []
        self.state_values = []
        self.command_values = []
        self.launch_status = []
        self.ESTOP = 1
        self.lock = threading.Lock()
        self.command_msg = PlcController()
        self.command_msg.values = [0] * 8
        self.command_msg.interface_names = [ 'PLC_node/mode_of_operation', 
                                            'PLC_node/power_cutoff', 
                                            'PLC_node/sonar_teach', 
                                            'PLC_node/s_output.4', 
                                            'PLC_node/s_output.5', 
                                            'PLC_node/s_output.6', 
                                            'PLC_node/s_output.7', 
                                            'PLC_node/s_output.8' ]    
    
    def get_controllers(self):
        req = ListControllers.Request()
        future = self.list_controllers_client.call_async(req)

        while not future.done(): 
            time.sleep(0.01)

        response = future.result()
        if response == None:
            self.get_logger.info("controller not found")
            return False
        
        active_controllers = []
        loaded_controllers = []

        for ctrl in response.controller:
            if ctrl.name != 'PLC_controller':
                if ctrl.state == 'active':
                    active_controllers.append(ctrl.name)
                elif ctrl.state in ['inactive', 'unconfigured']:
                    loaded_controllers.append(ctrl.name)
                else:
                    self.get_logger().info(f'Controller {ctrl.name} is in state {ctrl.state}. Not managed.')
        
        if 'joint_state_broadcaster' in active_controllers:
            active_controllers.pop(active_controllers.index('joint_state_broadcaster'))  # Remove the element by index
            active_controllers.append('joint_state_broadcaster')
        
        if 'joint_state_broadcaster' in loaded_controllers:
            loaded_controllers.pop(loaded_controllers.index('joint_state_broadcaster'))  # Remove the element by index
            loaded_controllers.append('joint_state_broadcaster')
        # DEBUG
        # self.get_logger().info(f'✅ Active: {active_controllers}')
        # self.get_logger().info(f'🕓 Loaded: {loaded_controllers}')
        return active_controllers, loaded_controllers

    def switch_controller(self, controller_to_activate, controller_to_deactivate):
        if controller_to_activate is None and controller_to_deactivate is None:
            return True
        
        switch_req = SwitchController.Request()
        switch_req.activate_controllers = [] if controller_to_activate is None else controller_to_activate
        switch_req.deactivate_controllers = [] if controller_to_deactivate is None else controller_to_deactivate
        
        switch_req.strictness = SwitchController.Request.BEST_EFFORT
        self.get_logger().info(f'Activating controller {controller_to_activate}...') if controller_to_activate else self.get_logger().info('No controller to activate.')
        self.get_logger().info(f'Deactivating controller {controller_to_deactivate}...') if controller_to_deactivate else self.get_logger().info('No controller to deactivate.')
        future = self.switch_client.call_async(switch_req)
        while not future.done(): 
            time.sleep(0.01)
        if future.result().ok:
            _start_time = self.get_clock().now()
            while True:
                active, _ = self.get_controllers()
                if controller_to_activate is None and controller_to_deactivate is not None and all(elem not in active for elem in controller_to_deactivate):
                    return True
                elif controller_to_activate is not None and controller_to_deactivate is None and all(elem in active for elem in controller_to_activate):
                    return True
                elif all(elem not in active for elem in controller_to_deactivate) and all(elem in active for elem in controller_to_activate):
                    return True
                _current_time = self.get_clock().now()
                if _current_time - _start_time > 2.0:
                    self.get_logger().warn("⚠️ The switch of controllers got a strange behavior: the result of the servie is OK, but the list of controller mismatch the request...")
                    return False

    def unload(self, controller_name):
        unload_req = UnloadController.Request()
        unload_req.name = controller_name
        future = self.unload_client.call_async(unload_req)
        while not future.done(): 
            time.sleep(0.01)
        if future.result().ok:
            self.get_logger().info(f'✅ Controller {controller_name} unloaded)')
        else:
            self.get_logger().error(f'❌ Failed to unload controller: {controller_name}')

        
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
                        self.publish_command('PLC_node/power_cutoff', 0)
                        time.sleep(2)
                        subprocess.Popen([" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh"], shell=True, executable="/bin/bash")
                        self.ESTOP = self.state_values[int_idx]
                    elif self.state_values[int_idx] == 0 and self.ESTOP  == 1:
                        self.get_logger().info(bcolors.OKCYAN + f"************************ INPUT 0 and ESTOP 1 => KILL" + bcolors.ENDC)
                        self.get_logger().info(bcolors.OKCYAN + "************************ KILL tecnobody_ethercat_checker_node" + bcolors.ENDC)
                        
                        shutdown_request = Trigger.Request()
                        self.eth_checker_shutdown_client.call_async(shutdown_request)

                        active, loaded = self.get_controllers()
                        if len(active)>0:
                            if not self.switch_controller(None, active):
                                self.get_logger().info("Controller not succesfully, deactivated!!")
                        elif len(active) == 0 and len(loaded) > 0:
                            for controller in loaded:
                                self.unload(controller)
                        else:
                            self.get_logger().info("All targeted processes signaled. Power cut-off")
                            self.publish_command('PLC_node/power_cutoff', 1)
                            self.ESTOP = self.state_values[int_idx]
                    elif self.state_values[int_idx] == 0 and self.ESTOP  == 0:
                        self.ESTOP = self.state_values[int_idx]
                        pass

                    elif self.state_values[int_idx] == 1 and self.ESTOP  == 1:
                        self.ESTOP = self.state_values[int_idx]
                        pass
                    
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
    node = PLCControllerInterface()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(node)


    import os
    os.sched_setaffinity(0, {2,6})
    try:
        while rclpy.ok() and not node.shutdown_all:
            mt_executor.spin_once()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PLC Controller Interface node.")
    finally:
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()