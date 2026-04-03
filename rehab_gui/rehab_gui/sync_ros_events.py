# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from typing import List
import time
import threading

# mathematics
import numpy as np

#ROS
import roslibpy

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color


class RosilibpyServiceHandler(object):
    def __init__(self, ros_client : roslibpy.Ros, namespace : str, msg_type: str):
        self.ros_client : roslibpy.Ros = ros_client
        self.namespace : str = namespace
        self.msg_type : str = msg_type
        self.service_client : roslibpy.Service = roslibpy.Service(self.ros_client, namespace, msg_type)
        self.response : dict = None #type: ignore
        self.error : dict = None #type: ignore
        self.on_done_collback = None #type: ignore
        self.on_error_callback = None #type: ignore

    def __del__(self):
        if self.ros_client.is_connected and self.service_client.is_advertised:
            self.service_client.unadvertise()

    def response_callback(self, response):
        self.response = response
        if self.on_done_collback:
            self.on_done_collback(response) #type: ignore

    def error_callback(self, response):
        self.response = response
        if self.on_error_callback:
            self.on_error_callback(response) #type: ignore

    def call_async(self, req: dict = None, on_done_callback = None, on_error_callback = None) -> None: #type: ignore
        try:
            _req : roslibpy.ServiceRequest = roslibpy.ServiceRequest(req) if req is not None else roslibpy.ServiceRequest()
            self.on_done_collback = on_done_callback
            self.on_error_callback = on_error_callback
            _ = self.service_client.call(_req, self.response_callback, self.error_callback) # type: ignore
        except Exception as e:
            print(f'>>>> Service {self.namespace} [{self.msg_type}] failed with exception: {e}')
            print(f'<<<< Given request: {req}')
        return 
    
    def call(self, req: dict = None, on_error_callback = None) -> dict: #type: ignore
        try:
            _req : roslibpy.ServiceRequest = roslibpy.ServiceRequest(req) if req is not None else roslibpy.ServiceRequest()
            self.on_done_collback = None #type: ignore
            self.on_error_callback = on_error_callback
            self.response = self.service_client.call(_req, errback = self.error_callback) # type: ignore
        except Exception as e:
            print(f'>>>> Service {self.namespace} [{self.msg_type}] failed with exception: {e}')
            print(f'<<<< Given request: {req}')
        return self.response

class ConstRequestServiceHandler(RosilibpyServiceHandler):
    def __init__(self, ros_client: roslibpy.Ros, namespace: str, msg_type: str, req: dict = None): #type: ignore
        super().__init__(ros_client, namespace, msg_type)
        self.req = req

    def __del__(self):
        super().__del__()
         
    def call_async(self, on_done_callback = None, on_error_callback = None) -> None: # type: ignore
        try:
            super().call_async(self.req, on_done_callback, on_error_callback)
        except Exception as e:
            print(f'>>>> Service {self.namespace} [{self.msg_type}] failed with exception: {e}')
            print(f'<<<< Given request: {self.req}')

    def call(self, on_error_callback = None) -> dict: # type: ignore
        try:
            self.response = super().call(self.req, on_error_callback)
        except Exception as e:
            print(f'>>>> Service {self.namespace} [{self.msg_type}] failed with exception: {e}')
            print(f'<<<< Given request: {self.req}')
        return self.response

class CoEDriveStates:
    def __init__(self, lenght : int):
        self.lenght = lenght
        self.dof_names : List[str] = ['n/a'] * lenght
        self.coe_drive_states : List[str] = ['n/a'] * lenght
        self.modes_of_operation : List[str] = ['n/a'] * lenght
        self.status_words : List[int] = [0] * lenght
        self.fault_present : bool = True
        self.drives_on : bool = False
    
    def from_dict(self, msg : dict):
        if msg is not None and 'dof_names' in msg.keys() and len(msg['dof_names'])==self.lenght:
            self.dof_names = msg['dof_names']
            self.coe_drive_states = msg['drive_states']
            self.modes_of_operation = msg['modes_of_operation']
            self.status_words = msg['status_words']
            self.fault_present =  msg['fault_present']
            self.drives_on =  msg['drives_on']
        else:
            self.dof_names = ['n/a'] * self.lenght
            self.coe_drive_states = ['n/a'] * self.lenght
            self.modes_of_operation = ['n/a'] * self.lenght
            self.status_words = [0] * self.lenght
            self.fault_present = True
            self.drives_on = False
            

# Ethercat Controller States, i.e., the state of all the ethercat slaves
class EcSlaveStates:
    def __init__(self, lenght : int):
        self.lenght = lenght
        self.slave_names : List[str] = ['n/a'] * lenght
        self.slave_states : List[str] = ['n/a'] * lenght
    
    def from_dict(self, msg : dict):
        if msg is not None and 'slave_names' in msg.keys() and len(msg['slave_names'])==self.lenght:
            self.slave_names  = [ m.replace('0x000001dd:0x10305070','ASDA2 Delta') for m in msg['slave_names']]
            self.slave_states = msg['slave_states']
        else:
            self.slave_names = ['n/a'] * self.lenght
            self.slave_states = ['n/a'] * self.lenght

class SyncRosManager:

    def __init__(self, expected_number_of_slaves: int, joint_names: List[str], ros_client: roslibpy.Ros):
        self._ros_period = 1
        self._controller_list_period = 50  # milliseconds
        self.trajectory_controller_name : str = 'joint_trajectory_controller'
        self.forward_command_controller_name : str = 'forward_velocity_controller'
        self.admittance_controller_name : str = 'admittance_controller'
        self.current_controller_name : str = None # type: ignore
        self.jog_cmd_pos = []
        self.plc_states = {}

        self.trajectory_completed = False
        self.exercise_completed = False
        self.movement_stopped = False
        self.cancel_movement = False
        self.execution_time_percentage : int = 0
        self.repetition_cnt : int = 0

        self._joint_state = None
        self._joint_names = joint_names
        self._expected_number_of_slaves = expected_number_of_slaves

        self.coe_drive_states : CoEDriveStates = CoEDriveStates(len(self._joint_names))
        self.ec_slave_states : EcSlaveStates = EcSlaveStates(self._expected_number_of_slaves)

        self.ros_client = ros_client

        #  subscribers
        self.RobotJointPosition = [0.0] * len(self._joint_names)
        self.HandlePosition : List[float] = [0.0] * len(self._joint_names)
        
        # soft stop variables
        self._prev_jog_direction : int = None # type: ignore
        # Enable jog
        self.jog_enabled : bool = False
        self.jog_enabled_pressed : bool = False
        # Enable manual guidance
        self.manual_guidance_enabled : bool = False
        # Enable zeroing
        self.homing_process_running : bool = False
        # Buttons to enalbe according to active controller name and active mode of operation
        self.enable_jog_buttons : bool = False
        self.enable_zeroing : bool = False
        self.enable_manual_guidance : bool = False
        self.enable_ptp : bool = False
        # Reset Faults
        self.manual_reset_faults : bool = True
        # check if ros env is correctly launched 
        self.destroy_clients_init : bool = False
        self.lock = threading.Lock()

        self.movement_status : str = 'idle'

        self.init_service_clients()
        self.init_publisher_and_subscribers()
        print('SyncRosManager class correctly initialized.')

    def init_publisher_and_subscribers(self):
        #subscribers
        self.joint_subscriber : roslibpy.Topic = roslibpy.Topic(self.ros_client, '/joint_states', 'sensor_msgs/msg/JointState')
        self.joint_subscriber.subscribe(self.getJointState)

        self.tool_subscriber  : roslibpy.Topic = roslibpy.Topic(self.ros_client, '/joint_states', 'sensor_msgs/msg/JointState')
        self.tool_subscriber.subscribe(self.getToolPosition)

        self.plc_states_subscriber : roslibpy.Topic = roslibpy.Topic(
            self.ros_client,
            '/PLC_controller/plc_states',
            'tecnobody_msgs/msg/PlcStates'
        )
        self.plc_states_subscriber.subscribe(self.getPLCStates)

        self.movement_status_subscriber : roslibpy.Topic = roslibpy.Topic(
            self.ros_client,
            '/tecnobody_workbench_utils/movement_status',
            'std_msgs/msg/String'
        )
        self.movement_status_subscriber.subscribe(self.update_movement_status)  #TODO: check

        #  publishers        
        self.plc_command_publisher : roslibpy.Topic = roslibpy.Topic(
            self.ros_client,
            '/PLC_controller/plc_commands',
            'tecnobody_msgs/msg/PlcController'
        )

        

    def detach_publisher_and_subscribers(self):
        self.joint_subscriber.unsubscribe()
        self.tool_subscriber.unsubscribe()
        self.plc_states_subscriber.unsubscribe()
        self.movement_status_subscriber.unsubscribe()
        
        self.plc_command_publisher.unadvertise()
        
        print('All publisher/subscribers are correctly detached')

    def init_service_clients(self):
        self.current_controller_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, '/controller_manager/list_controllers',
                                                          'controller_manager_msgs/srv/ListControllers')

        self.switch_controller_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, '/controller_manager/switch_controller',
                                                         'controller_manager_msgs/srv/SwitchController')

        self.motors_on_client : ConstRequestServiceHandler = ConstRequestServiceHandler(self.ros_client, '/ethercat_checker/start_motors',
                                                 'std_srvs/srv/Trigger', roslibpy.ServiceRequest())

        self.motors_off_client : ConstRequestServiceHandler = ConstRequestServiceHandler(self.ros_client, '/ethercat_checker/stop_motors',
                                                  'std_srvs/srv/Trigger', roslibpy.ServiceRequest())

        self.reset_fault_client : ConstRequestServiceHandler = ConstRequestServiceHandler(self.ros_client, '/state_controller/reset_fault',
                                                   'std_srvs/srv/Trigger', roslibpy.ServiceRequest())

        self.mode_of_op_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, '/state_controller/switch_mode_of_operation',
                                                  'ethercat_controller_msgs/srv/SwitchDriveModeOfOperation')

        self.enable_eth_error_check : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, '/ethercat_checker/enable_error_checking',
                                                                                        'std_srvs/srv/SetBool')

        self.get_drive_state_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, '/ethercat_checker/get_drive_states',
                                                                                        'ethercat_controller_msgs/srv/GetDriveStates')
        
        self.get_slave_state_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, '/ethercat_checker/get_slave_states',
                                                                                        'tecnobody_msgs/srv/GetSlaveStates')
        
        self.perform_homing_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, '/state_controller/perform_homing', 'std_srvs/srv/Trigger')

        self.set_trajectory_client : RosilibpyServiceHandler= RosilibpyServiceHandler(self.ros_client, "/tecnobody_workbench_utils/set_trajectory",  
                                                                                      "tecnobody_msgs/SetTrajectory")
                
        self.reset_speed_over_client : ConstRequestServiceHandler = ConstRequestServiceHandler(self.ros_client, "/tecnobody_workbench_utils/reset_speed_ovr",
                                                                                               "std_srvs/Trigger", roslibpy.ServiceRequest())

        self.set_rehab_exercise_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, "/tecnobody_workbench_utils/set_rehab_exercise",
                                                                                     "tecnobody_msgs/SetExercise")
        self.set_eeg_exercise_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, "/tecnobody_workbench_utils/set_eeg_exercise",
                                                                                     "tecnobody_msgs/SetExercise")
        self.soft_movement_start_client : RosilibpyServiceHandler = RosilibpyServiceHandler(self.ros_client, "/tecnobody_workbench_utils/soft_movement_start",
                                                                                            'tecnobody_msgs/srv/SoftMovement')
        self.soft_movement_stop_client : ConstRequestServiceHandler = ConstRequestServiceHandler(self.ros_client, "/tecnobody_workbench_utils/soft_movement_stop",
                                                                                                 'std_srvs/srv/Trigger', roslibpy.ServiceRequest())
        
        self.stop_movement_client : ConstRequestServiceHandler = ConstRequestServiceHandler(self.ros_client, "/tecnobody_workbench_utils/stop_movement", 
                                                                                            "std_srvs/Trigger", roslibpy.ServiceRequest())
        self.sonar_bias_client : ConstRequestServiceHandler = ConstRequestServiceHandler(self.ros_client, "/sonar_teach_node/enable_sonar_teach",
                                                                                           "std_srvs/Trigger", roslibpy.ServiceRequest())

        self.on_trajectory_finished_server : roslibpy.Service = roslibpy.Service(self.ros_client, "/rehab_gui/trajectory_finished", "std_srvs/Trigger")
        self.on_trajectory_finished_server.advertise(self.on_trajectory_finished)

        self.on_exercise_finished_server : roslibpy.Service = roslibpy.Service(self.ros_client, "/rehab_gui/exercise_finished", "std_srvs/Trigger")
        self.on_exercise_finished_server.advertise(self.on_exercise_finished)
        
        self.on_exercise_progress_server : roslibpy.Service = roslibpy.Service(self.ros_client, "/rehab_gui/exercise_progress", "tecnobody_msgs/MovementProgress")
        self.on_exercise_progress_server.advertise(self.on_exercise_progress)

        
        self.on_movement_stopped_server : roslibpy.Service  = roslibpy.Service(self.ros_client, "/rehab_gui/movement_stopped", "std_srvs/Trigger")        
        self.on_movement_stopped_server.advertise(self.on_movement_stopped)

        print('All service clients correctly initialized.')
        return True
    
    def detach_services_and_clients(self):
        self.current_controller_client = None #type: ignore
        self.switch_controller_client = None #type: ignore
        self.motors_on_client = None #type: ignore
        self.motors_off_client = None #type: ignore
        self.reset_fault_client = None #type: ignore
        self.mode_of_op_client = None #type: ignore
        self.enable_eth_error_check = None #type: ignore
        self.get_drive_state_client = None #type: ignore
        self.get_slave_state_client = None #type: ignore
        self.perform_homing_client = None #type: ignore
        self.set_trajectory_client = None #type: ignore

        self.reset_speed_over_client = None #type: ignore
        self.set_rehab_exercise_client = None #type: ignore
        self.stop_movement_client = None #type: ignore
        self.sonar_bias_client = None #type: ignore
        self.soft_movement_start_client = None #type: ignore
        self.soft_movement_stop_client = None #type: ignore
        
        self.on_trajectory_finished_server.unadvertise()
        self.on_exercise_finished_server.unadvertise()
        self.on_exercise_progress_server.unadvertise()
        self.on_movement_stopped_server = None #type: ignore
        print('All service/clients correctly deleted.')
        return True
    
    def trigger_soft_movement_start(self, amplitude: float, time_constant: float = 1.0, target='speed_ovr', jog_joint_idx=-1):
        request : dict = {'data': {
            'amplitude': amplitude,
            'time_constant' : time_constant,
            'target' : target, 
            'jog_joint_idx': jog_joint_idx
        }  }
        print(f'>>>> Send the Soft Start Movement command to the Gui Trajectory Manager') 
        response = self.soft_movement_start_client.call(request)  # Pause any ongoing movement
        print(f'<<<< Received: {response}')  # Log the response message

    def trigger_soft_movement_stop(self):
        print(f'>>>> Send the Soft Stop Movement command to the Gui Trajectory Manager') 
        response = self.soft_movement_stop_client.call()  # Pause any ongoing movement
        print(f'<<<< Received: {response}')  # Log the response message

    ##############################################################################################
    #####                                                                                    #####  
    #####                            GENERAL ROS FUNCTIONS                                   ##### 
    #####                                                                                    #####
    ##############################################################################################
    def publish_plc_command(self, name, value):
        command_msg = roslibpy.Message({
            'interface_names' : name,
            'values' : value
        })
        timeout = time.time() + 1
        while time.time() < timeout:
            self.plc_command_publisher.publish(command_msg)


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

    def getPLCStates(self, data):
        self.plc_states = dict(zip(data['interface_names'], data['values']))

    def update_controller_and_driver_states(self) -> None:
        
        if not self.destroy_clients_init:
            
            msg_drive_states = self.get_drive_states()
            self.coe_drive_states.from_dict(msg_drive_states['states'] if msg_drive_states is not None and 'states' in msg_drive_states else None) #type: ignore
            
            if len(self.coe_drive_states.dof_names) != len(self._joint_names):
                print(f'Warning! Get an incomplete list of states (received the data for the axes: {self.coe_drive_states.dof_names}, expected: {self._joint_names}')
                return 
            
            msg_slave_states = self.get_slave_states()
            self.ec_slave_states.from_dict(msg_slave_states) #type: ignore
            
            if len(self.ec_slave_states.slave_names) != self._expected_number_of_slaves:
                print(f'Warning! Get an incomplete list of states (received the data for the axes: {self.ec_slave_states.slave_names}, expected: {self._expected_number_of_slaves}')
                return 
            
            list_controllers_response : dict = self.get_list_controllers()
            if list_controllers_response is None or 'controller' not in list_controllers_response:
                return
            
            ##########################
            moo : List[int] = [ self.get_op_mode_number(mode) for mode in self.coe_drive_states.modes_of_operation ]

            is_csv_mode = all([moo[j] == 9 for j in range(len(self._joint_names))])
            is_csp_mode = all([moo[j] == 8 for j in range(len(self._joint_names))])
            is_hmg_mode = all([moo[j] == 6 for j in range(len(self._joint_names))])


            ###################
            active_controller : str = None #type: ignore
            for ctrl in list_controllers_response['controller']:
                if ctrl['state'] == 'active':
                    if ctrl['name'] == self.forward_command_controller_name:
                        active_controller = self.forward_command_controller_name
                        break
                    elif ctrl['name'] == self.trajectory_controller_name:
                        active_controller = self.trajectory_controller_name
                        break
                    elif ctrl['name'] == self.admittance_controller_name:
                        active_controller = self.admittance_controller_name
                        break
            self.current_controller_name = active_controller
            ###################
            self.enable_jog_buttons = self.current_controller_name == self.forward_command_controller_name and is_csv_mode
            self.enable_manual_guidance = self.current_controller_name == self.admittance_controller_name and is_csv_mode
            self.enable_zeroing = is_hmg_mode
            self.enable_ptp = self.current_controller_name == self.trajectory_controller_name and is_csp_mode
            
    def switch_controller(self, controller_to_activate, controller_to_deactivate) -> dict:
        switch_req = {
            'activate_controllers': [] if controller_to_activate is None else [controller_to_activate],
            'deactivate_controllers': [] if controller_to_deactivate is None else [controller_to_deactivate],
            'strictness': 2,  # BEST_EFFORT = 1, STRICT = 2
            'activate_asap': True,
        }
        if not hasattr(self, 'switch_controller_client'):
            self.init_service_clients()
        return self.switch_controller_client.call(switch_req)  # type: ignore

    def activate_controller_after_homing(self) -> None:
        res : dict = self.switch_controller(self.current_controller_name, None)
        if not res['ok'] == True:
            print("❌ Failed to activate controller after homing")
            return
        self.set_mode_of_operation(8) if self.current_controller_name == self.trajectory_controller_name else self.set_mode_of_operation(9)

    def destroy(self):
        self.destroy_clients_init = True

        # Null out handles to publishers, services, etc.
        self.detach_services_and_clients()
        self.detach_publisher_and_subscribers()

        print("SyncRosManager correctly destroyed.")
        return True

    ##############################################################################################
    #####                                                                                    #####  
    #####                                   Ethercat Controller                              ##### 
    #####                                        Services                                    #####
    #####                                                                                    #####
    ##############################################################################################

    def controller_and_op_mode_switch(self, new_mode: int, new_controller: str, timeout_s: float = 5.0):
        moo = [ self.get_op_mode_number(moo) for moo in self.coe_drive_states.modes_of_operation]
        print(f'{GREEN}>>>>{NC} Switch MOO and controller({YELLOW}{moo}, {self.current_controller_name}{NC}) => ({YELLOW}{new_mode}, {new_controller}{NC})')
        ok = new_controller == self.current_controller_name and\
                all(new_mode == moo[j] for j in range(len(self._joint_names)))
        if not ok: 
            do_switch_controller = new_controller != self.current_controller_name
            do_switch_moo = False
            if do_switch_controller:
                print(f'{GREEN}....{NC} >>>> Switch Controller')
                res = self.switch_controller(new_controller, self.current_controller_name)
                ok = res['ok']
                print(f"{GREEN}....{NC} <<<< Switch Controller [{GREEN+'OK'+NC if ok else RED+'FAILED'+NC}]")
            do_switch_moo = ok and not all(new_mode == moo[j] for j in range(len(self._joint_names)))
            if do_switch_moo:
                print(f'{GREEN}....{NC} >>>> Switch MOO')
                ok = self.set_mode_of_operation(new_mode)
                if ok:
                    start_time = time.time()
                    while True:
                        moo = [ self.get_op_mode_number(moo) for moo in self.coe_drive_states.modes_of_operation]
                        current_time = time.time()
                        elapsed = current_time - start_time
                        if elapsed > timeout_s:
                            ok = False
                            break
                        if all(new_mode == moo[j] for j in range(len(self._joint_names))):
                            ok = True
                            break
                        print(f'{GREEN}....{NC} .... Switch MOO .... waiting the transition (still {YELLOW}{moo})')
                        time.sleep(0.1)

                print(f"{GREEN}....{NC} Switch MOO [{GREEN+'OK'+NC if ok else RED+'FAILED'+NC}]")
            
        print(f"{GREEN}<<<<{NC} Switch  MOO and controller [{GREEN+'OK'+NC if ok else RED+'FAILED'+NC}]")
        return ok

    def set_mode_of_operation(self, mode_value: int) -> bool:
        for idx,dof in enumerate(self._joint_names):
            if self.get_op_mode_number(self.coe_drive_states.modes_of_operation[idx]) == mode_value:
                pass
            req = {'dof_name': dof, 'mode_of_operation': mode_value,}
            _ = self.mode_of_op_client.call(req)

        return True

        # op_response = []
        # for mode in self.coe_drive_states.modes_of_operation:
        #     op_response.append(self.get_op_mode_number(mode))
        # if all(op_response[j] == mode_value for j in range(len(self._joint_names))):
        #     self.op_mode_request_in_execution = False
        #     return True
        # else:
        #     print(f"got response: {self.coe_drive_states.modes_of_operation} when trying to change OP mode.")
        #     time.sleep(0.1)
        #     return self.set_mode_of_operation(mode_value)

    def perform_homing(self) -> bool:
        print('--------------->Performing Homing for all three joints')
        self.turn_on_motors()
        ok : bool = False
        request = roslibpy.ServiceRequest()
        try:
            result : dict = self.perform_homing_client.call(request)
            if result['success']:
                start_time = time.time()
                timeout_sec = 5.0
                print('Homing started, waiting for completion...')
                self.homing_process_running = True
                while True:
                    current_time = time.time()
                    elapsed = current_time - start_time

                    if all((status_word & (1 << 12)) != 0 for status_word in self.coe_drive_states.status_words):
                        print('Homing performed successfully')
                        ok = True
                        break
                    elif elapsed > timeout_sec:
                        print('Timeout while waiting for homing to complete')
                        for status_word in self.coe_drive_states.status_words:
                            print(
                                f' - STATUS WORD: bit 12 (homing completed): {status_word & (1 << 12)}, '
                                f'bit 13 (error): {status_word & (1 << 13)}'
                            )
                        ok = False
                        break
                    time.sleep(0.1)  # evita busy waiting

                self.homing_process_running = False
                self.activate_controller_after_homing()
            else:
                print('Failed to perform homing (service returned False)')
        except Exception as e:
            print(f'Homing service call failed with exception: {e}')

        self.turn_off_motors()
        return ok

    def enable_ethercat_error_checking(self, enable: bool):
        req = roslibpy.ServiceRequest({
            'data': enable
        })
        result = self.enable_eth_error_check.call(req)
        if result['success']==True:
            print(f"Ethercat error automatic checking state changed to: {enable}")
        else:
            print(f"Service call failed: {result['message']}")

    def get_list_controllers(self) -> dict: #type: ignore
        result : dict = None # type: ignore
        try:
            result = self.current_controller_client.call()
            if result is None:
                print('current_controller_client: Service call failed')
        except Exception as e:
            print(f'Get List Controller service call failed with exception: {e}')
        return result

    # def get_drives_mode_of_op(self, dof_names) -> dict:
    #     response : dict = None # type: ignore
    #     try:
    #         req = {'dof_names': dof_names}
    #         response = self.get_op_mode_client.call(req)
    #     except Exception as e:
    #         print(f'Get Op Mode service call failed with exception: {e}')
    #     return response

    def get_drive_states(self) -> dict:
        result : dict = None  # type: ignore
        try:
            result = self.get_drive_state_client.call()
            if result is None:
                print('Drive States Service call failed')

        except Exception as e:
            print(f'Drive States Service call failed with exception: {e}')
        return result

    def get_slave_states(self) -> dict:
        result : dict = None  # type: ignore
        try:
            result = self.get_slave_state_client.call()
            if result is None:
                print('Slave States Service call failed')
        except Exception as e:
            print(f'Slave States Service call failed with exception: {e}')
        return result


    def get_op_mode_number(self, mode: str) -> int:
        op_mode_dict={
            'MODE_NO_MODE': 0,
            'MODE_PROFILED_POSITION': 1,
            'MODE_PROFILED_VELOCITY': 3,
            'MODE_PROFILED_TORQUE': 4,
            'MODE_HOMING': 6,
            'MODE_INTERPOLATED_POSITION': 7,
            'MODE_CYCLIC_SYNC_POSITION': 8,
            'MODE_CYCLIC_SYNC_VELOCITY': 9,
            'MODE_CYCLIC_SYNC_TORQUE': 10
        }
        return op_mode_dict.get(mode, 0)

    def turn_on_motors(self) -> bool:
        result : dict = None  # type: ignore
        # turn on the motors
        print(f'{MAGENTA}>>>>{NC} Turn On Drives Request ...')
        result = self.motors_on_client.call()
        ok = result['success'] if result is not None and 'success' in result.keys() else False
        print(f"{MAGENTA}<<<<{NC} Turn On Drives Request [{GREEN+'OK'+NC if ok else RED+'FAILED'+NC}] {result['message'] if not ok and result is not None and 'message' in result.keys() else ''}")
        return ok

    def turn_off_motors(self) -> bool:
        print(f'{YELLOW}>>>>{NC} Turn Off Drives Request ...')
        result = self.motors_off_client.call()
        ok = result['success'] if result is not None and 'success' in result.keys() else False
        print(f"{YELLOW}<<<<{NC} Turn Off Drives Request [{GREEN+'OK'+NC if ok else RED+'FAILED'+NC}] {result['message'] if not ok and result is not None and 'message' in result.keys() else ''}")
        return ok

    def reset_fault(self):
        result : dict = None # type: ignore
        result = self.reset_fault_client.call()
        try:
            if result['success']:
                print('Fault reset successfully')
            else:
                print('Failed to reset faults!')
        except Exception as e:
            print(f'Service call failed with exception: {e}')

    ##############################################################################################
    #####                                                                                    #####  
    #####                                   BUTTONS CALLBACKS                                ##### 
    #####                                                                                    #####
    ##############################################################################################
    def jog_enable(self, pressed):
        with self.lock:
            # Avoid multiple event
            if self.jog_enabled_pressed == pressed:
                pass
            self.jog_enabled_pressed = pressed
            # print(f"pressed: {pressed}")
            if pressed:
                # print(f"pressed true - current controller: {self.current_controller_name}")
                if self.current_controller_name != self.forward_command_controller_name:
                    print(f"Switching to: {self.forward_command_controller_name}")
                    if not self.controller_and_op_mode_switch(9, self.forward_command_controller_name):
                        print(f"❌ Failed to switch to {self.forward_command_controller_name}!")
                        return
                self.jog_enabled = True
            else:
                # print(f"pressed false - current controller: {self.current_controller_name}")
                if self.current_controller_name != self.trajectory_controller_name:
                    print(f"Switching to: {self.trajectory_controller_name}")
                    if not self.controller_and_op_mode_switch(8, self.trajectory_controller_name):
                        print(f"❌ Failed to switch to {self.trajectory_controller_name}!")
                        return
                self.jog_enabled = False
            # print(f"jog_enabled: {self.jog_enabled}")

    def jog_command(self, direction: int, joint_to_move: int):
        velocity_command_ref : float = 0.1
        if direction not in [-1, 1, 0]:
            print("⚠️ Direction must be -1, 1 or 0.")
            return
        if direction == 0:
            print(f"Stopping JOG movement.")
            self.trigger_soft_movement_stop()
            if not self.manual_reset_faults:
                self.enable_ethercat_error_checking(True)
        else:
            print("Starting JOG movement.")
            self.enable_ethercat_error_checking(False)

            target_velocity = velocity_command_ref * direction

            self.trigger_soft_movement_start(amplitude=target_velocity, time_constant=0.2, target='jog', jog_joint_idx=joint_to_move)
            self._prev_jog_direction = direction

    def reset_mode_changed(self, index):
        self.manual_reset_faults = (index == 0) or (index == 2) or (index == 3)
        if self.manual_reset_faults:
            self.enable_ethercat_error_checking(False)
        else:
            self.enable_ethercat_error_checking(True)

    def send_ptp_trajectory(self, target_point: list, end_time: list) -> bool:
        try:
            _ = self.reset_speed_over_client.call()

            points : List[List[float]]= list(list())
            times : List[float] = list()
            points.append(self.RobotJointPosition)
            times.append(0.0)
            points.append(target_point)
            times.append(end_time) #type: ignore
            req = roslibpy.ServiceRequest({
                'cartesian_positions': [
                    {'point': points[idx], 
                     'time_from_start': times[idx]} for idx in range(len(times))
                    ], 'override': 50
            })
            print(req)
            print(f"{CYAN}>>>>{NC} Set Trajectory Execution Request sent")
            response : dict = self.set_trajectory_client.call(req)
            print(f"{CYAN}<<<<{NC} Set Trajectory Execution Request [{GREEN+'OK'+NC if response is not None and 'success' in response.keys() and response['success'] else RED+'FAILED'+NC}]")
            return response['success'] if response is not None and 'success' in response.keys() else False

        except Exception as e:
            self.stop_movement_client.call()

        return False

    def set_exercise(self, CartesianPositions, TimeFromStart, ovrs: List[float], durations: List[float], eeg_mode: bool) -> bool:
        msg = roslibpy.Message({'factor': int(100)})
        self.repetition_cnt = 0
        self.exercise_completed = False
        response : dict = None #type: ignore
        if len(ovrs) != len(durations):
            print(f"[Set Trajectory] mismatching input dimension!")
            return False
        try:
            _ = self.reset_speed_over_client.call()
            req = roslibpy.ServiceRequest({
                    'cartesian_positions': [{'point': CartesianPositions[idx], 'time_from_start' : TimeFromStart[idx][0]} for idx,val in enumerate(TimeFromStart)], 
                    'repetition_ovrs': ovrs, 'repetition_durations' : durations
                })
            
            if eeg_mode:
                response = self.set_eeg_exercise_client.call(req)  # type: ignore
            else:
                response = self.set_rehab_exercise_client.call(req)  # type: ignore

            return response['success'] if response is not None and 'success' in response.keys() else False
        except Exception as e:
            print(f"[Set Trajectory] Service Call Exception: {e}")

        return False
    
    def stop_movement(self) -> bool:
        print(f'{GREEN}>>>>{NC} Send STOP Movement Request')
        self.movement_stopped = False
        response : dict = None #type: ignore
        ok : bool = False
        msg : str = ""
        try:
            response = self.stop_movement_client.call()
            ok = response['success']
        except Exception as e:
            ok = False
            msg = f'{e}'
        
        print(f"{GREEN}<<<<{NC} STOP Movement Request [{GREEN+'OK'+NC if ok else RED+'FAILED'+NC}]{':{msg}' if len(msg)>0 else ''}")
        
        if ok:
            print(f'{GREEN}<<<<{NC} Wait for the actual movement stop...')
            start_time = time.time()
            while not self.movement_stopped:
                current_time = time.time()
                elapsed = current_time - start_time
                if elapsed > 10.0:
                    print(f'{RED}>>>>{NC} Timeout! Failed to stop the movement in 10s... weird')
                    break
                time.sleep(0.05)

        print(f'{GREEN}>>>>{NC} Send Motor Off Request')
        ok = self.turn_off_motors()
        print(f"{GREEN}<<<<{NC} Motor Off Request [{GREEN+'OK'+NC if ok else RED+'FAILED'+NC}]")
        return ok and self.movement_stopped

    def on_trajectory_finished(self, request, response):
        print(f"[on_trajectory_finished] The Trajectory Execution's just finished {request}")
        self.trajectory_completed = True
        response['success'] = True
        return True       
    
    def on_movement_stopped(self, request, response):
        print(f"[on_movement_stopped] Service Call: {request}")
        self.movement_stopped = True
        response['success'] = True
        return True       
    
    def on_exercise_finished(self, request, response):
        self.repetition_cnt = self.repetition_cnt +1
        self.exercise_completed = True
        response['success'] = True
        return True       

    def on_exercise_progress(self, request, response):
        self.execution_time_percentage = int(request['progress'])  # Get the progress percentage from the worker
        response['additional_speed_override'] =  1.0
        return True

    def update_movement_status(self, data):
        self.movement_status = data['data']

    def get_movement_status(self) -> str:
        return self.movement_status
    
    def sonar_bias(self) :
        print("Setting sonar bias...")
        self.sonar_bias_client.call()