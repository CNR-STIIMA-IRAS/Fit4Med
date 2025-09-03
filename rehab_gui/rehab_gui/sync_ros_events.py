import sys
from PyQt5.QtCore import QTimer
import time

# mathematics
import numpy as np

#MC Classes/methods

#ROS
import roslibpy


class SyncRosManager:
    _ros_period = 1
    _controller_list_period = 50  # milliseconds
    trajectory_controller_name = 'joint_trajectory_controller'
    forward_command_controller = 'forward_velocity_controller'
    admittance_controller = 'admittance_controller'
    current_controller = None
    jog_cmd_pos = []

    def __init__(self, joint_names: list):
        self._controller_timer = QTimer()
        self._controller_timer.timeout.connect(self.list_active_controllers)
        self._joint_state = None
        self._joint_names = joint_names
        # roslibpy init
        self.ros_client = roslibpy.Ros(host='10.2.16.42', port=9090)
        #  subscribers
        self.RobotJointPosition = [0.0] * len(self._joint_names)
        self.joint_subscriber = roslibpy.Topic(self.ros_client, '/joint_states', 'sensor_msgs/msg/JointState')
        self.joint_subscriber.subscribe(self.getJointState)
        self.HandlePosition = [0.0] * len(self._joint_names)
        self.tool_subscriber = roslibpy.Topic(self.ros_client, '/joint_states', 'sensor_msgs/msg/JointState')
        self.tool_subscriber.subscribe(self.getToolPosition)
        self.fault_state_subscriber = roslibpy.Topic(self.ros_client, '/ethercat_checker/drive_state_flags',
                                                     'ethercat_controller_msgs/msg/DriveStateFlags')
        self.fault_state_subscriber.subscribe(self.getDrivestState)
        #  publisher
        self.pub_speed_ovr = roslibpy.Topic(self.ros_client, '/speed_ovr', 'std_msgs/msg/Int16')
        self.jog_cmd_publisher = roslibpy.Topic(self.ros_client,
                                                f'/{self.forward_command_controller}/commands',
                                                'std_msgs/msg/Float64MultiArray')
        # soft stop variables
        self._soft_transition_values = []
        self._soft_start_timer = QTimer()
        self._soft_stop_timer = QTimer()
        self._soft_start_timer.timeout.connect(self._soft_start_step)
        self._soft_stop_timer.timeout.connect(self._soft_stop_step)
        self._soft_transition_target = None  # "speed_ovr" or "jog"
        self._soft_transition_joint = None  # only for jog, indicates dof number
        self._prev_jog_direction = None
        # Enable jog
        self.jog_enabled = False
        self.jog_enabled_pressed = False
        # Enable manual guidance
        self.manual_guidance_enabled = False
        self.manual_guidance_enabled_pressed = False
        # Enable zeroing
        self.homing_check_enabled_pressed = False
        # Buttons to enalbe according to active controller name and active mode of operation
        self.enable_jog_buttons = False
        self.enable_zeroing = False
        self.enable_manual_guidance = False
        self.enable_ptp = False
        # Motors stop services anti-rebound flags
        self.try_turn_on_in_execution = False
        self.try_turn_off_in_execution = False
        self.are_motors_on = False
        # Reset Faults
        self.manual_reset_faults = True
        self.is_in_fault_state = False
        self.fault_reset_in_execution = False
        # check if ros env is correctly launched 
        self._ros_ready_timer = QTimer()
        self._ros_ready_timer.timeout.connect(self.check_ros_ready)
        self._ros_ready_timer.start(100)
        # check if clients are being destroyed
        self.destroy_clients_init = False
        self.ros_client.run()
        print('SyncRosManager class correctly initialized.')

    def trigger_soft_stop(self, start_value: float, steps: int = 10, target='speed_ovr', jog_joint_idx=None):
        if self._soft_stop_timer.isActive():
            print("Already performing a soft motion. Ignoring new stop request.")
            return
        self._soft_stop_target = target
        self._soft_stop_joint = jog_joint_idx
        sign = np.sign(start_value) if start_value != 0 else 1.0
        amplitude = abs(start_value)
        times = np.linspace(0, 1, steps)
        profile = amplitude * np.cos(0.5 * np.pi * times)
        self._soft_stop_values = (sign * profile).tolist()

        if self._soft_start_timer.isActive():
            print("canceling start timer before triggering stop")
            self._soft_start_timer.stop()

        self._soft_stop_timer.start(50)  # ms

    def trigger_soft_start(self, target_value: float, steps: int = 10, target='speed_ovr', jog_joint_idx=None):
        if self._soft_start_timer.isActive():
            print("Already performing a soft motion. Ignoring new start request.")
            return
        self._soft_start_target = target
        self._soft_start_joint = jog_joint_idx

        sign = np.sign(target_value) if target_value != 0 else 1.0
        amplitude = abs(target_value)
        times = np.linspace(0, 1, steps)
        profile = amplitude * np.sin(0.5 * np.pi * times)
        self._soft_start_values = (sign * profile).tolist()

        if self._soft_stop_timer.isActive():
            print("canceling stop timer before triggering start")
            self._soft_stop_timer.stop()

        self._soft_start_timer.start(50)  # ms

    def _soft_stop_step(self):
        if not self._soft_stop_values:
            self._soft_stop_timer.stop()
            return
        value = self._soft_stop_values.pop(0)
        if self._soft_stop_target == 'speed_ovr':
            msg = roslibpy.Message({'data': int(value)})
            self.pub_speed_ovr.publish(msg)
            print(f"[SoftStop] speed_ovr: {msg.data}")
        elif self._soft_stop_target == 'jog':
            jog_cmd = [0.0] * len(self._joint_names)
            if self._soft_stop_joint is not None:
                jog_cmd[self._soft_stop_joint] = value
            msg = roslibpy.Message({'data': jog_cmd})
            self.jog_cmd_publisher.publish(msg)
            print(f"[SoftStop] jog velocity: {jog_cmd}")
        else:
            print("Unknown soft stop target, stopping timer.")
            self._soft_stop_timer.stop()

    def _soft_start_step(self):
        if not self._soft_start_values:
            self._soft_start_timer.stop()
            return
        value = self._soft_start_values.pop(0)
        if self._soft_start_target == 'speed_ovr':
            msg = roslibpy.Message({'data': int(value)})
            self.pub_speed_ovr.publish(msg)
            print(f"[SoftStart] speed_ovr: {msg.data}")
        elif self._soft_start_target == 'jog':
            jog_cmd = [0.0] * len(self._joint_names)
            if self._soft_start_joint is not None:
                jog_cmd[self._soft_start_joint] = value
            msg = roslibpy.Message({'data': jog_cmd})
            self.jog_cmd_publisher.publish(msg)
            print(f"[SoftStart] jog velocity: {jog_cmd}")
        else:
            print("Unknown soft start target, stopping timer.")
            self._soft_start_timer.stop()

    ##############################################################################################
    #####                                                                                    #####  
    #####                            GENERAL ROS FUNCTIONS                                   ##### 
    #####                                                                                    #####
    ##############################################################################################

    def check_ros_ready(self):
        if not self.ros_client.is_connected:
            print("ROS not connected yet...")
        else:
            self.trajectory_controller_name = 'joint_trajectory_controller'
            if self.init_service_clients():
                self._controller_timer.start(self._controller_list_period)
                self._ros_ready_timer.stop()
                print('ROS class correctly started!')

    def init_service_clients(self):
        self.current_controller_client = roslibpy.Service(self.ros_client, '/controller_manager/list_controllers',
                                                          'controller_manager_msgs/srv/ListControllers')

        self.switch_controller_client = roslibpy.Service(self.ros_client, '/controller_manager/switch_controller',
                                                         'controller_manager_msgs/srv/SwitchController')

        self.motors_on_client = roslibpy.Service(self.ros_client, '/state_controller/try_turn_on',
                                                 'std_srvs/srv/Trigger')

        self.motors_off_client = roslibpy.Service(self.ros_client, '/state_controller/try_turn_off',
                                                  'std_srvs/srv/Trigger')

        self.reset_fault_client = roslibpy.Service(self.ros_client, '/state_controller/reset_faults',
                                                   'std_srvs/srv/Trigger')

        self.mode_of_op_client = roslibpy.Service(self.ros_client, '/state_controller/switch_mode_of_operation',
                                                  'ethercat_controller_msgs/srv/SwitchDriveModeOfOperation')

        self.get_op_mode_client = roslibpy.Service(self.ros_client, '/ethercat_checker/get_drive_mode_of_operation',
                                                   'ethercat_controller_msgs/srv/GetDriveModeOfOperation')

        self.enable_eth_error_check = roslibpy.Service(self.ros_client, '/ethercat_checker/enable_error_checking',
                                                       'std_srvs/srv/SetBool')

        print('All service clients correctly initialized.')
        return True

    def getJointState(self, data):
        if not set(self._joint_names).issubset(data['name']):
            print(
                f"JointState names {data['name']} do not match the expected joint names {self._joint_names}. Ignoring data.")
            return
        name_to_position = dict(zip(data['name'], data['position']))
        self.RobotJointPosition = [
            name_to_position[joint] for joint in self._joint_names if joint in name_to_position
        ]

    def getToolPosition(self, data):
        if not set(self._joint_names).issubset(data['name']):
            print(
                f"Tool Position names {data['name']} do not match the expected joint names {self._joint_names}. Ignoring data.")
            return

        self.HandlePosition = [0, 0, 0]
        name_to_position = dict(zip(data['name'], data['position']))
        self.HandlePosition = [
            name_to_position[joint] for joint in self._joint_names if joint in name_to_position
        ]

    def getDrivestState(self, msg):
        try:
            self.is_in_fault_state = msg['fault_present']
            self.are_motors_on = msg['motors_on']
        except Exception as e:
            print(f'Drives state flags subscription failed with exception: {e}')

    def list_active_controllers(self):
        if not self.destroy_clients_init:
            req = roslibpy.ServiceRequest()
            _ = self.current_controller_client.call(req, callback=self.update_current_controller)

    def update_current_controller(self, res):
        # self.current_controller = self.trajectory_controller_name
        op_response = self.get_drives_mode_of_op(self._joint_names)
        is_csv_mode = (len(op_response['dof_names']) == len(self._joint_names)) and all(
            op_response['values'][j] == 9 for j in range(len(self._joint_names)))
        is_csp_mode = (len(op_response['dof_names']) == len(self._joint_names)) and all(
            op_response['values'][j] == 8 for j in range(len(self._joint_names)))
        is_hmg_mode = (len(op_response['dof_names']) == len(self._joint_names)) and all(
            op_response['values'][j] == 6 for j in range(len(self._joint_names)))

        active_controller = None
        for ctrl in res['controller']:
            if ctrl['name'] == self.forward_command_controller and ctrl['state'] == 'active' and is_csv_mode:
                active_controller = self.forward_command_controller
                self.enable_jog_buttons = self.jog_enabled
                self.enable_zeroing = False
                self.enable_manual_guidance = False
                self.enable_ptp = False
                break
            elif ctrl['name'] == self.trajectory_controller_name and ctrl['state'] == 'active' and is_csp_mode:
                active_controller = self.trajectory_controller_name
                self.enable_jog_buttons = False
                self.enable_zeroing = False
                self.enable_manual_guidance = False
                self.enable_ptp = True
                break
            elif ctrl['name'] == self.admittance_controller and ctrl['state'] == 'active' and is_csv_mode:
                active_controller = self.admittance_controller
                self.enable_jog_buttons = False
                self.enable_zeroing = False
                self.enable_manual_guidance = self.manual_guidance_enabled
                self.enable_ptp = False
                break
        # DEBUG PRINTS
        # print(f"Current controller: {self.current_controller}")
        # print(f'enable_jog_buttons: {self.enable_jog_buttons}, enable_zeroing: {self.enable_zeroing}, enable_manual_guidance: {self.enable_manual_guidance}, enable_ptp: {self.enable_ptp}')
        if active_controller:
            self.current_controller = active_controller
            return True
        else:
            self.current_controller = None
            if is_hmg_mode:
                self.enable_jog_buttons = False
                self.enable_zeroing = True
                self.enable_manual_guidance = False
                self.enable_ptp = False
                return True
            else:
                print(
                    f"⚠️ No known controller is currently active! Current MOO: [{op_response['values'][0]}, {op_response['values'][1]}, {op_response['values'][2]}]")
                return False

    def switch_controller(self, controller_to_activate, controller_to_deactivate):
        switch_req = roslibpy.ServiceRequest({
            'activate_controllers': [] if controller_to_activate is None else [controller_to_activate],
            'deactivate_controllers': [] if controller_to_deactivate is None else [controller_to_deactivate],
            'strictness': 2,  # BEST_EFFORT = 1, STRICT = 2
            'start_asap': True,
            'timeout': 5.0
        })
        print(f'Activating controller {controller_to_activate}...') if controller_to_activate else print(
            'No controller to activate.')
        print(f'Deactivating controller {controller_to_deactivate}...') if controller_to_deactivate else print(
            'No controller to deactivate.')
        result = self.switch_controller_client.call(switch_req)
        return result  # type: ignore

    def start_homing_procedure(self):
        if self.are_motors_on:
            self.perform_homing()
        else:
            self.turn_on_motors()
            time.sleep(0.3)
            if self.are_motors_on:
                self.perform_homing()

    def activate_controller_after_homing(self):
        res = self.switch_controller(self.current_controller, None)
        if not res['ok'] == True:
            print("❌ Failed to activate controller after homing")
            return
        self.set_mode_of_operation(
            8) if self.current_controller == self.trajectory_controller_name else self.set_mode_of_operation(9)

    def destroy(self):
        self.destroy_clients_init = True

        if hasattr(self, '_ros_ready_timer'):
            self._ros_ready_timer.stop()
            self._ros_ready_timer.deleteLater()
        if hasattr(self, '_controller_timer'):
            self._controller_timer.stop()
            self._controller_timer.deleteLater()
        if hasattr(self, '_soft_stop_timer'):
            self._soft_stop_timer.stop()
            self._soft_stop_timer.deleteLater()
        if hasattr(self, '_soft_start_timer'):
            self._soft_start_timer.stop()
            self._soft_start_timer.deleteLater()

        # Terminate roslibpy client
        self.ros_client.terminate()

        # Null out handles to publishers, services, etc.
        self.speed_ovr_publisher = None
        self.jog_publisher = None
        self.list_controllers_cli = None
        self.switch_controller_cli = None
        self.reset_fault_cli = None
        self.try_turn_on_cli = None
        self.try_turn_off_cli = None
        self.switch_mode_of_operation_cli = None
        self.get_drive_mode_of_operation_cli = None
        self.enable_error_checking_cli = None

        try:
            self.comboBox_ResetFaults.currentIndexChanged.disconnect()
        except Exception:
            pass

        print("SyncRosManager correctly destroyed.")
        return True

    ##############################################################################################
    #####                                                                                    #####  
    #####                                   Ethercat Controller                              ##### 
    #####                                        Services                                    #####
    #####                                                                                    #####
    ##############################################################################################

    def controller_and_op_mode_switch(self, new_mode, new_controller):
        if self.current_controller:
            res = self.switch_controller(new_controller, self.current_controller)
        else:
            res = self.switch_controller(new_controller, None)
        if not res['ok'] == True:
            print(f"❌ Failed to deactivate controller before switching mode")
            return False
        if self.set_mode_of_operation(new_mode):
            print(f"✅ Switched to mode of OP {new_mode} mode with controller {new_controller}")
            return True
        else:
            print("Error switching mode of OP!!!")
            return False

    def set_mode_of_operation(self, mode_value):
        for dof in self._joint_names:
            req = roslibpy.ServiceRequest({
                'dof_name': dof,
                'mode_of_operation': mode_value,
            })
            print(f"Setting mode {mode_value} for {dof}...")
            _ = self.mode_of_op_client.call(req)

        op_response = self.get_drives_mode_of_op(self._joint_names)
        if (len(op_response['dof_names']) == len(self._joint_names)) and all(
                op_response['values'][j] == mode_value for j in range(len(self._joint_names))):
            return True
        else:
            print(f"got response: {op_response} when trying to change OP mode.")
            time.sleep(0.1)
            return self.set_mode_of_operation(mode_value)

    def perform_homing(self):
        self.homing_process_started = True
        print('--------------->Performing Homing for all three joints')
        client = roslibpy.Service(self.ros_client, '/state_controller/perform_homing', 'std_srvs/srv/Trigger')
        request = roslibpy.ServiceRequest()
        _ = client.call(request, callback=self.perform_homing_callback)

    def perform_homing_callback(self, res):
        try:
            result = res
            if result.success:
                start_time = time.time()
                timeout_sec = 5.0
                print('Homing started, waiting for completion...')
                while True:
                    current_time = time.time()
                    elapsed = current_time - start_time

                    op_response = self.get_drives_mode_of_op(self._joint_names)
                    if all((status_word & (1 << 12)) != 0 for status_word in op_response.status_words):
                        print('Homing performed successfully')
                        break
                    elif elapsed > timeout_sec:
                        print('Timeout while waiting for homing to complete')
                        for status_word in op_response.status_words:
                            print(
                                f' - STATUS WORD: bit 12 (homing completed): {status_word & (1 << 12)}, '
                                f'bit 13 (error): {status_word & (1 << 13)}'
                            )
                        break
                    print("waiting for status words to be all true!!")
                    time.sleep(0.1)  # evita busy waiting

                self.homing_process_started = False
                self.activate_controller_after_homing()
            else:
                print('Failed to perform homing (service returned False)')
        except Exception as e:
            print(f'Service call failed with exception: {e}')

    def enable_ethercat_error_checking(self, enable: bool):
        req = roslibpy.ServiceRequest({
            'data': enable
        })
        result = self.enable_eth_error_check.call(req)
        if result.success:
            print(f"Ethercat error automatic checking state changed to: {enable}")
        else:
            print(f"Service call failed: {result.message}")

    def get_drives_mode_of_op(self, dof_names):
        req = roslibpy.ServiceRequest({
            'dof_names': dof_names
        })
        result = self.get_op_mode_client.call(req)
        if result is None:
            print('Service call failed')
        return result

    def turn_on_motors(self):
        if self.try_turn_on_in_execution:
            print('Try turn on already in execution, skipping...')
            return
        self.try_turn_on_in_execution = True
        print('Turning on drives...')
        request = roslibpy.ServiceRequest()
        result = self.motors_on_client.call(request)
        self.try_turn_on_in_execution = False
        return result  # type: ignore

    def turn_off_motors(self):
        if self.try_turn_off_in_execution:
            print('Try turn off already in execution, skipping...')
            return
        self.try_turn_off_in_execution = True
        print('Turning off drives...')
        request = roslibpy.ServiceRequest()
        result = self.motors_off_client.call(request)
        self.try_turn_off_in_execution = False
        return result  # type: ignore

    def reset_fault(self):
        if self.fault_reset_in_execution:
            print('Fault reset already in execution, skipping...')
            return
        self.fault_reset_in_execution = True
        request = roslibpy.ServiceRequest()
        _ = self.reset_fault_client.call(request, callback=self.reset_fault_callback)

    def reset_fault_callback(self, future):
        try:
            result = future.result()
            if result.success:
                print('Fault reset successfully')
            else:
                print('Failed to reset faults!')
        except Exception as e:
            print(f'Service call failed with exception: {e}')
        self.fault_reset_in_execution = False

    ##############################################################################################
    #####                                                                                    #####  
    #####                                   BUTTONS CALLBACKS                                ##### 
    #####                                                                                    #####
    ##############################################################################################

    def zeroing_enable(self, pressed):
        # Avoid multiple event
        if self.homing_check_enabled_pressed == pressed:
            return
        self.homing_check_enabled_pressed = pressed
        if pressed:
            if not self.controller_and_op_mode_switch(6, None):
                print(f"❌ Failed to set homing mode!")
                return
        else:
            print(f'>>>>>>>>>>>>>>>>>Disabling homing mode, current controller is: {self.current_controller}')
            if self.current_controller != self.trajectory_controller_name:
                print(f"Switching to: {self.trajectory_controller_name}")
                if not self.controller_and_op_mode_switch(8, self.trajectory_controller_name):
                    print(f"❌ Failed to switch to {self.trajectory_controller_name}!")
                    return

    def manual_guidance_enable(self, pressed):
        if self.manual_guidance_enabled_pressed == pressed:
            pass
        self.manual_guidance_enabled_pressed = pressed
        if pressed:
            if self.current_controller != self.admittance_controller:
                print(f"Switching to: {self.admittance_controller}")
                if not self.controller_and_op_mode_switch(9, self.admittance_controller):
                    print(f"❌ Failed to switch to {self.admittance_controller}!")
                    return
            self.manual_guidance_enabled = True
        else:
            if self.current_controller != self.trajectory_controller_name:
                print(f"Switching to: {self.trajectory_controller_name}")
                if not self.controller_and_op_mode_switch(8, self.trajectory_controller_name):
                    print(f"❌ Failed to switch to {self.trajectory_controller_name}!")
                    return
            self.manual_guidance_enabled = False

    def jog_enable(self, pressed):
        # Avoid multiple event
        if self.jog_enabled_pressed == pressed:
            pass
        self.jog_enabled_pressed = pressed
        if pressed:
            if self.current_controller != self.forward_command_controller:
                print(f"Switching to: {self.forward_command_controller}")
                if not self.controller_and_op_mode_switch(9, self.forward_command_controller):
                    print(f"❌ Failed to switch to {self.forward_command_controller}!")
                    return
            self.jog_enabled = True
        else:
            if self.current_controller != self.trajectory_controller_name:
                print(f"Switching to: {self.trajectory_controller_name}")
                if not self.controller_and_op_mode_switch(8, self.trajectory_controller_name):
                    print(f"❌ Failed to switch to {self.trajectory_controller_name}!")
                    return
            self.jog_enabled = False

    def jog_command(self, direction, joint_to_move):
        velocity_command_ref = 0.1
        if direction not in [-1, 1, 0]:
            print("⚠️ Direction must be -1, 1 or 0.")
            return
        if direction == 0:
            print(f"Stopping JOG movement starting from {velocity_command_ref * self._prev_jog_direction}.")
            self.trigger_soft_stop(start_value=velocity_command_ref * self._prev_jog_direction, steps=15, target='jog',
                                   jog_joint_idx=joint_to_move)
            if not self.manual_reset_faults:
                self.enable_ethercat_error_checking(True)
        else:
            print("Starting JOG movement.")
            self.enable_ethercat_error_checking(False)

            target_velocity = velocity_command_ref * direction

            self.trigger_soft_start(target_value=target_velocity, steps=10, target='jog', jog_joint_idx=joint_to_move)
            self._prev_jog_direction = direction

    def reset_mode_changed(self, index):
        self.manual_reset_faults = (index == 0)
        if self.manual_reset_faults:
            self.enable_ethercat_error_checking(False)
        else:
            self.enable_ethercat_error_checking(True)
