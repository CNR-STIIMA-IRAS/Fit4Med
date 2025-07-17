import sys
from PyQt5.QtCore import QTimer
import time

# mathematics
import numpy as np

#MC Classes/methods

#ROS
import rclpy
from ros2node.api import get_node_names
from rclpy.node import Node
from sensor_msgs.msg import JointState # joints positions, velocities and efforts
from std_msgs.msg import Int16, Float64MultiArray 
from ethercat_controller_msgs.srv import SwitchDriveModeOfOperation, GetModesOfOperation
from std_srvs.srv import Trigger, SetBool
from controller_manager_msgs.srv import SwitchController, ListControllers
from rclpy.parameter_client import AsyncParameterClient


class SyncRosManager:
    _ros_period = 1
    _controller_list_period = 50 # milliseconds
    trajectory_controller_name = 'joint_trajectory_controller'
    forward_command_controller = 'forward_velocity_controller'
    admittance_controller = 'admittance_controller'
    current_controller = None
    jog_cmd_pos = []
    
    def __init__(self, ros_node : Node, joint_names: list):
        ###### super().__init__('FMRR_cell_node') # type: ignore
        self._ros_node = ros_node
        self._ros_timer = QTimer()
        self._ros_timer.timeout.connect(self.spin_ros_once)
        self._ros_timer.start(self._ros_period)
        self._controller_timer = QTimer()
        self._controller_timer.timeout.connect(self.update_current_controller)
        self._joint_state = None
        self._joint_names = joint_names
        #  subscribers        
        self.joint_subscriber = self._ros_node.create_subscription(JointState, 'joint_states', self.getJointState, 1)
        self.tool_subscriber = self._ros_node.create_subscription(JointState, 'joint_states', self.getToolPosition, 1)
        #  publisher  
        self.pub_speed_ovr = self._ros_node.create_publisher(Int16, '/speed_ovr', 10)
        self.jog_cmd_publisher = self._ros_node.create_publisher(Float64MultiArray, f'/{self.forward_command_controller}/commands', 10)
        #  service clients
        self.current_controller_client = self._ros_node.create_client(ListControllers, '/controller_manager/list_controllers')
        client_success = self.current_controller_client.wait_for_service(timeout_sec=5.0)
        if not client_success:
            self._ros_node.get_logger().error("List controllers service is not ready.")
            rclpy.shutdown()
            sys.exit(1)
        self.switch_controller_client = self._ros_node.create_client(SwitchController, '/controller_manager/switch_controller')
        switch_controller_client_success = self.switch_controller_client.wait_for_service(timeout_sec=5.0)
        if not switch_controller_client_success:
            self._ros_node.get_logger().error("Switch controller service is not ready.")
            rclpy.shutdown()
            sys.exit(1)
        self.mode_of_op_client = self._ros_node.create_client(SwitchDriveModeOfOperation, '/state_controller/switch_mode_of_operation')
        mode_of_op_client_success = self.mode_of_op_client.wait_for_service(timeout_sec=5.0)
        if not mode_of_op_client_success:
            self._ros_node.get_logger().error("Switch drive mode of operation service is not ready.")
            rclpy.shutdown()
            sys.exit(1)
        self.get_op_mode_client = self._ros_node.create_client(GetModesOfOperation, '/ethercat_checker/get_drive_mode_of_operation')
        get_op_mode_client_success = self.get_op_mode_client.wait_for_service(timeout_sec=5.0)
        if not get_op_mode_client_success:
            self._ros_node.get_logger().error("Get drive mode of operation service is not ready.")
            rclpy.shutdown()
            sys.exit(1)
        self.enable_eth_error_check = self._ros_node.create_client(SetBool, '/ethercat_checker/enable_error_checking')
        enable_eth_err_chk_success = self.enable_eth_error_check.wait_for_service(timeout_sec=5.0)
        if not enable_eth_err_chk_success:
            self._ros_node.get_logger().error("Switch drive mode of operation service is not ready.")
            rclpy.shutdown()
            sys.exit(1)
        self.cm_param_client = AsyncParameterClient(self._ros_node, 'controller_manager')
        param_client_success= self.cm_param_client.wait_for_services(5.0)
        if not param_client_success:
            self._ros_node.get_logger().error("Parameter client services are not ready.")
            rclpy.shutdown()
            sys.exit(1)
        self.update_rate = -1 
        # Read parameters
        future =  self.cm_param_client.get_parameters(['update_rate'])
        rclpy.spin_until_future_complete(self._ros_node,future)
        result = future.result()
        if result:
            self._ros_node.get_logger().info(f"Read: update rate = {result.values[0].integer_value}")
            self.update_rate = result.values[0].integer_value
        else:
            self._ros_node.get_logger().error("Failed to read update rate parameter.")
            rclpy.shutdown()
            sys.exit(1)
        # catch the controller names
        available_nodes = [ full_name for _, _, full_name in get_node_names(node=self._ros_node, include_hidden_nodes=False) ]
        self.trajectory_controller_name = list(filter(lambda x: x.endswith("_trajectory_controller"), available_nodes))[0].lstrip('/')
        self._ros_node.get_logger().error(f"Trajectory Controller Name: {self.trajectory_controller_name}")
        self._controller_timer.start(self._controller_list_period)
        # soft stop variables
        self._soft_transition_values = []
        self._soft_start_timer = QTimer()
        self._soft_stop_timer = QTimer()
        self._soft_start_timer.timeout.connect(self._soft_start_step)
        self._soft_stop_timer.timeout.connect(self._soft_stop_step)
        self._soft_transition_target = None  # "speed_ovr" or "jog"
        self._soft_transition_joint = None   # only for jog, indicates dof number
        self._prev_jog_direction = None

        # Enable jog
        self.jog_enabled = False
        self.jog_enabled_pressed = False
        # Buttons to enalbe according to active controller name and active mode of operation
        self.enable_jog_buttons = False
        self.enable_zeroing = False
        self.enable_absolute_homing = False
        self.enable_manual_guidance = False
        self.enable_ptp = False

        # Mode of operation action
        self.moo_applied_success = False
    
    def getJointState(self, data):
        name_to_position = dict(zip(data.name, data.position))
        self.RobotJointPosition = [
            name_to_position[joint] for joint in self._joint_names if joint in name_to_position
        ]

    def enable_ethercat_error_checking(self, enable: bool):
        req = SetBool.Request()
        req.data = enable
        future = self.enable_eth_error_check.call_async(req)
        rclpy.spin_until_future_complete(self._ros_node, future)
        if future.result().success:
            self._ros_node.get_logger().info("Ethercat error automatic checking deactivated.")
        else:
            self._ros_node.get_logger().error(f"Service call failed: {future.result().message}")

    def get_drives_mode_of_op(self, dof_names):
        req = GetModesOfOperation.Request()
        req.dof_names = dof_names
        future = self.get_op_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self._ros_node, future)
        if future.result():
            return future.result()
        else:
            self._ros_node.get_logger().error('Service call failed')
            return None

    def trigger_soft_stop(self, start_value: float, steps: int = 10, target='speed_ovr', jog_joint_idx=None):
        if self._soft_stop_timer.isActive():
            self._ros_node.get_logger().warn("Already performing a soft motion. Ignoring new stop request.")
            return
        self._soft_stop_target = target
        self._soft_stop_joint = jog_joint_idx
        sign = np.sign(start_value) if start_value != 0 else 1.0
        amplitude = abs(start_value)
        times = np.linspace(0, 1, steps)
        profile = amplitude * np.cos(0.5 * np.pi * times)
        self._soft_stop_values = (sign * profile).tolist()
        
        if self._soft_start_timer.isActive():
            self._ros_node.get_logger().info("canceling start timer before triggering stop")
            self._soft_start_timer.stop()

        self._soft_stop_timer.start(50)  # ms

    def trigger_soft_start(self, target_value: float, steps: int = 10, target='speed_ovr', jog_joint_idx=None):
        if self._soft_start_timer.isActive():
            self._ros_node.get_logger().warn("Already performing a soft motion. Ignoring new start request.")
            return
        self._soft_start_target = target
        self._soft_start_joint = jog_joint_idx

        sign = np.sign(target_value) if target_value != 0 else 1.0
        amplitude = abs(target_value)
        times = np.linspace(0, 1, steps)
        profile = amplitude * np.sin(0.5 * np.pi * times)
        self._soft_start_values = (sign * profile).tolist()

        if self._soft_stop_timer.isActive():
            self._ros_node.get_logger().info("canceling stop timer before triggering start")
            self._soft_stop_timer.stop()

        self._soft_start_timer.start(50)  # ms

    def _soft_stop_step(self):
        if not self._soft_stop_values:
            self._soft_stop_timer.stop()
            return
        value = self._soft_stop_values.pop(0)
        if self._soft_stop_target == 'speed_ovr':
            msg = Int16()
            msg.data = int(value)
            self.pub_speed_ovr.publish(msg)
            self._ros_node.get_logger().info(f"[SoftStop] speed_ovr: {msg.data}")
        elif self._soft_stop_target == 'jog':
            jog_cmd = [0.0] * len(self._joint_names)
            if self._soft_stop_joint is not None:
                jog_cmd[self._soft_stop_joint] = value
            msg = Float64MultiArray()
            msg.data = jog_cmd
            self.jog_cmd_publisher.publish(msg)
            self._ros_node.get_logger().info(f"[SoftStop] jog velocity: {jog_cmd}")
        else:
            self._ros_node.get_logger().warn("Unknown soft stop target, stopping timer.")
            self._soft_stop_timer.stop()

    def _soft_start_step(self):
        if not self._soft_start_values:
            self._soft_start_timer.stop()
            return
        value = self._soft_start_values.pop(0)
        if self._soft_start_target == 'speed_ovr':
            msg = Int16()
            msg.data = int(value)
            self.pub_speed_ovr.publish(msg)
            self._ros_node.get_logger().info(f"[SoftStart] speed_ovr: {msg.data}")
        elif self._soft_start_target == 'jog':
            jog_cmd = [0.0] * len(self._joint_names)
            if self._soft_start_joint is not None:
                jog_cmd[self._soft_start_joint] = value
            msg = Float64MultiArray()
            msg.data = jog_cmd
            self.jog_cmd_publisher.publish(msg)
            self._ros_node.get_logger().info(f"[SoftStart] jog velocity: {jog_cmd}")
        else:
            self._ros_node.get_logger().warn("Unknown soft start target, stopping timer.")
            self._soft_start_timer.stop()

    def getToolPosition(self, data):
        ToolPosition = data
        self.HandlePosition = [0,0,0]
        name_to_position = dict(zip(data.name, data.position))
        self.HandlePosition = [
            name_to_position[joint] for joint in self._joint_names if joint in name_to_position
        ]
        
    def spin_ros_once(self):
        rclpy.spin_once(self._ros_node, timeout_sec=0)

    def update_current_controller(self):
        req = ListControllers.Request()
        future = self.current_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self._ros_node, future)
        if not future.result():
            self._ros_node.get_logger().error("Failed to call list_controllers service")
            return False
        
        op_response = self.get_drives_mode_of_op(self._joint_names)
        is_csv_mode = (len(op_response.dof_names)==len(self._joint_names)) and all(op_response.values[j] == 9 for j in range(len(self._joint_names)))
        is_csp_mode = (len(op_response.dof_names)==len(self._joint_names)) and all(op_response.values[j] == 8 for j in range(len(self._joint_names)))
        is_hmg_mode = (len(op_response.dof_names)==len(self._joint_names)) and all(op_response.values[j] == 6 for j in range(len(self._joint_names)))
        
        active_controller = None
        for ctrl in future.result().controller:
            if ctrl.name == self.forward_command_controller and ctrl.state == 'active' and is_csv_mode:
                active_controller = self.forward_command_controller
                self.enable_jog_buttons = self.jog_enabled
                self.enable_zeroing = False
                self.enable_absolute_homing = False
                self.enable_manual_guidance = False
                self.enable_ptp = False            
                break
            elif ctrl.name == self.trajectory_controller_name and ctrl.state == 'active' and is_csp_mode:
                active_controller = self.trajectory_controller_name
                self.enable_jog_buttons = False
                self.enable_zeroing = False
                self.enable_absolute_homing = False
                self.enable_manual_guidance = False
                self.enable_ptp = True
                break
            elif ctrl.name == self.admittance_controller and ctrl.state == 'active' and is_csv_mode:
                active_controller = self.admittance_controller
                self.enable_jog_buttons = False
                self.enable_zeroing = False
                self.enable_absolute_homing = True
                self.enable_manual_guidance = True
                self.enable_ptp = False
                break
        if active_controller:
            # self._ros_node.get_logger().info(f'active controller: {active_controller}')
            self.current_controller = active_controller
            return True
        else:
            if is_hmg_mode:
                self.enable_jog_buttons = False
                self.enable_zeroing = True
                self.enable_absolute_homing = False
                self.enable_manual_guidance = False
                self.enable_ptp = False
                return True
            else:
                self._ros_node.get_logger().warn("⚠️ No known controller is currently active!")
                return False

    def switch_controller(self, controller_to_activate, controller_to_deactivate):
        switch_req = SwitchController.Request()
        switch_req.activate_controllers = [] if controller_to_activate is None else [controller_to_activate]
        switch_req.deactivate_controllers = [] if controller_to_deactivate is None else [controller_to_deactivate]
        switch_req.strictness = SwitchController.Request.STRICT
        self._ros_node.get_logger().info(f'Activating controller {controller_to_activate}...') if controller_to_activate else self._ros_node.get_logger().info('No controller to activate.')
        self._ros_node.get_logger().info(f'Deactivating controller {controller_to_deactivate}...') if controller_to_deactivate else self._ros_node.get_logger().info('No controller to deactivate.')
        future = self.switch_controller_client.call_async(switch_req)
        rclpy.spin_until_future_complete(self._ros_node, future)
        return future.result()  # type: ignore

    def start_homing_procedure(self):
        res = self.switch_controller(None, self.current_controller)
        if not res.ok:
            self._ros_node.get_logger().error(f"⚠️ Failed to deactivate controller before homing")
            return
        self.remaining_dofs = set(self._joint_names)
        if self.set_mode_of_operation(6):
            self.perform_homing()

    def perform_homing(self):
        self.homing_process_started = True
        self._ros_node.get_logger().info('--------------->Performing Homing for all three joints')
        client = self._ros_node.create_client(Trigger, '/state_controller/perform_homing')
        if client.wait_for_service(timeout_sec=5.0):
            request = Trigger.Request()
            future = client.call_async(request)
            future.add_done_callback(self.perform_homing_callback)
        else:
            self._ros_node.get_logger().error('Service /state_controller/perform_homing not available')

    def perform_homing_callback(self, future):
        try:
            result = future.result()
            if result.success:
                start_time = self._ros_node.get_clock().now()
                timeout_sec = 5.0
                self._ros_node.get_logger().info('Homing started, waiting for completion...')
                while True:
                    current_time = self._ros_node.get_clock().now()
                    elapsed = (current_time - start_time).nanoseconds / 1e9  # convert to seconds
                    
                    op_response = self.get_drives_mode_of_op(self._joint_names)
                    if all((status_word & (1 << 12)) != 0 for status_word in op_response.status_words):
                        self._ros_node.get_logger().info('Homing performed successfully')
                        break
                    elif elapsed > timeout_sec:
                        self._ros_node.get_logger().error('Timeout while waiting for homing to complete')
                        return False
                    self._ros_node.get_logger().info("waiting for status worlds to be all true!!")
                    time.sleep(0.1)  # sleep a bit to avoid busy waiting

                self.homing_process_started = False
                self.activate_controller_after_homing()
            else:
                self._ros_node.get_logger().error('Failed to perform homing (service returned False)')
        except Exception as e:
            self._ros_node.get_logger().error(f'Service call failed with exception: {e}')

    def activate_controller_after_homing(self):
        res = self.switch_controller(self.current_controller, None)
        if not res.ok:
            self._ros_node.get_logger().error(f"⚠️ Failed to activate controller after homing")
            return
        self.set_mode_of_operation(8) if self.current_controller == self.trajectory_controller_name else self.set_mode_of_operation(9)
    
    def set_mode_of_operation(self, mode_value):
        req = SwitchDriveModeOfOperation.Request()
        for dof in self._joint_names:
            req.dof_name = dof
            req.mode_of_operation = mode_value
            self._ros_node.get_logger().info(f"Setting mode {mode_value} for {dof}...")
            future = self.mode_of_op_client.call_async(req)
            rclpy.spin_until_future_complete(self._ros_node, future)
        
        op_response = self.get_drives_mode_of_op(self._joint_names)
        if (len(op_response.dof_names)==len(self._joint_names)) and all(op_response.values[j] == mode_value for j in range(len(self._joint_names))):
            return True
        else:
            self._ros_node.get_logger().info(f"got response: {op_response} when trying to change OP mode.")
            time.sleep(0.1)
            return self.set_mode_of_operation(mode_value)

    def controller_and_op_mode_switch(self, new_mode, new_controller):
        res = self.switch_controller(new_controller, self.current_controller)
        if not res.ok:
            self._ros_node.get_logger().error(f"⚠️ Failed to deactivate controller before switching mode")
            return False
        if self.set_mode_of_operation(new_mode):
            self._ros_node.get_logger().info(f"✅ Switched to mode of OP {new_mode} mode with controller {new_controller}")
            return True
        else:
            self._ros_node.get_logger().error("Error switching mode of OP!!!")
            return False
        
    def jog_enable(self, pressed):
        # Avoid multiple event
        if self.jog_enabled_pressed == pressed:
            pass

        self.jog_enabled_pressed = pressed
        if pressed:
            if self.current_controller != self.forward_command_controller:
                self._ros_node.get_logger().info(f"Switching to: {self.forward_command_controller}")
                if not self.controller_and_op_mode_switch(9, self.forward_command_controller):
                    self._ros_node.get_logger().error(f"⚠️ Failed to switch to {self.forward_command_controller}!")
                    return
            self.jog_enabled = True
        else:
            if self.current_controller != self.trajectory_controller_name:
                self._ros_node.get_logger().info(f"Switching to: {self.trajectory_controller_name}")
                if not self.controller_and_op_mode_switch(8, self.trajectory_controller_name):
                    self._ros_node.get_logger().error(f"⚠️ Failed to switch to {self.trajectory_controller_name}!")
                    return
            self.jog_enabled = False

    def jog_command(self, direction, joint_to_move):
        velocity_command_ref = 0.1
        if direction not in [-1, 1, 0]:
            self._ros_node.get_logger().error("⚠️ Direction must be -1, 1 or 0.")
            return
        if direction == 0:
            self._ros_node.get_logger().info(f"Stopping JOG movement starting from {velocity_command_ref*self._prev_jog_direction}.")
            self.trigger_soft_stop(start_value=velocity_command_ref*self._prev_jog_direction, steps=15, target='jog', jog_joint_idx=joint_to_move)
            self.enable_ethercat_error_checking(True)
        else:
            self._ros_node.get_logger().info("Starting JOG movement.")
            self.enable_ethercat_error_checking(False)

            target_velocity = velocity_command_ref * direction

            self.trigger_soft_start(target_value=target_velocity, steps=10, target='jog', jog_joint_idx=joint_to_move)
            self._prev_jog_direction = direction

