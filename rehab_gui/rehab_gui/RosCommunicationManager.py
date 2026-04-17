# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import gc
import time
from typing import List
from PyQt5.QtWidgets import QMessageBox, QPushButton, QWidget
from PyQt5.QtCore import QThread, QObject, pyqtSignal
import roslibpy
from sync_ros_events import SyncRosManager

class Worker(QThread):
    finished : pyqtSignal = pyqtSignal() #type: ignore
    thread_running : bool = False
    loop_period_s : float = 0.1
    def __init__(self, callback, loop_period_s: float = 0.05):
        super().__init__()
        self.callback = callback
        self.loop_period_s = loop_period_s
    def stop_thread(self):
        self.thread_running = False
    def start_thread(self):
        self.thread_running = True
        self.start()
    def run(self):
        
        while self.thread_running:
            start_time = time.monotonic()
            self.callback()
            end_time = time.monotonic()
        
            elapsed_time = end_time - start_time
            sleep_time = self.loop_period_s - elapsed_time
        
            if sleep_time > 0:
                time.sleep(sleep_time)
            
        self.finished.emit()  # Emit signal when done

class RosCommunicationManager(QObject):
    stop_ros_communication_signal : pyqtSignal = pyqtSignal()
    worker_thread : Worker = None #type: ignore
    
    def __init__(self, joint_names: List[str], number_of_ec_slaves: int, remote_ip: str, remote_port: int, widget: QWidget): #port=9090
        super().__init__()
        
        self.widget = widget
        self.enable_controller_behaviour = None
        self.ROS : SyncRosManager = None #type: ignore

        self.number_of_ec_slaves = number_of_ec_slaves
        self.joint_names = joint_names
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.roslib_first_time_connection = True
        self.manual_mode_activated = False

        self.worker_thread = Worker(self.updateState, loop_period_s=0.2)
        self.worker_thread.finished.connect(self.onUpdateWorkerThreadFinished)
    
    def onUpdateWorkerThreadFinished(self):
        self.stop_ros_communication_signal.emit()

    def startRosCommunication(self) -> None:
        if self.rOk():
            print("ROS_MANAGER already initialized.")
            return 
        # Initialize ROS_MANAGER client
        if self.roslib_first_time_connection:
            self.ros_client = roslibpy.Ros(host=self.remote_ip, port=self.remote_port)
            self.ros_client.run(20)
            self.roslib_first_time_connection = False
        else:
            self.ros_client.connect()

        if self.rOk():
            del self.ROS
            self.ROS = None #type: ignore

        self.ROS = SyncRosManager(self.number_of_ec_slaves, self.joint_names, self.ros_client)

        self.worker_thread.start_thread()

    def stopRosCommunication(self) -> None:
        print("Stopping ROS processes...")
        
        self.ROS_active = False

        # Destroy ROS clients
        if hasattr(self, 'ROS') and self.rOk():
            if not self.ROS.destroy():
                print('Error trying to destroy ROS class.')
                return
            self.ros_client.close()
            
            del self.ROS
            gc.collect()

        #self.ros_waiting_dialog.show()
        print("[MainProgram] ROS processes stopped.")
        self.worker_thread.stop_thread()
        
            
    def isRosCommunicationActive(self) -> bool:
        return hasattr(self, 'ROS') and self.ROS is not None

    def rOk(self) -> bool:
        return self.isRosCommunicationActive()
        # if not self.isRosCommunicationActive():
        #     QMessageBox.warning(self.widget, "Warning",
        #             "Please hold the manual mode emergency button before moving the robot!")
        #     return False

        # return True

    def turnOffMotors(self) -> bool:
        if not self.rOk():
            return False
        self.ROS.publish_plc_command(['PLC_node/manual_mode'], [0])
        return self.ROS.turn_off_motors() if self.areMotorsOn() else True

    def setModeOfOperation(self, mode: int) -> bool:
        if not self.rOk():
            return False
        self.ROS.publish_plc_command(['PLC_node/mode_of_operation'], [mode])
        return True
        
    def turnOnMotors(self) -> bool:
        if self.rOk():
            if self.manual_mode_activated:
                if self.ROS.plc_states['manual_switch_pressed']:
                    # set manual mode during movement
                    self.ROS.publish_plc_command(['PLC_node/manual_mode'], [1])
                else:
                    QMessageBox.warning(self.widget, "Warning",
                                        "Please hold the manual mode emergency button before moving the robot!")
                    return False
            else:
                self.ROS.publish_plc_command(['PLC_node/manual_mode'], [0])
            
            return self.ROS.turn_on_motors() if not self.areMotorsOn() else True
        return False
    
    def enableControllerBehaviour(self, behaviour: str):
        if not self.rOk():
            return
        self.turnOffMotors()
        if behaviour == "Homing":
            return self.ROS.controller_and_op_mode_switch(6, None)  #type: ignore
        elif behaviour == "Jogging":
            return self.ROS.controller_and_op_mode_switch(9, self.ROS.forward_command_controller_name)
        elif behaviour == "ManualGuidance":
            return self.ROS.controller_and_op_mode_switch(9, self.ROS.admittance_controller_name)
        else:
            return self.ROS.controller_and_op_mode_switch(8, self.ROS.trajectory_controller_name)
        
    def toogleJoggingBehaviour(self, axis : int, direction: int):
        return self.ROS.jog_command(direction=direction, joint_to_move=axis) if self.rOk() else None

    def resetModeChanged(self, index):
        if not self.rOk():
            return
        self.ROS.reset_mode_changed(index)

    def SonarBias(self):
        if not self.rOk():
            return
        self.ROS.sonar_bias()

    def setManualMode(self, activate: bool) -> None:
        if not self.rOk():
            return
        self.manual_mode_activated = activate

    def resetFaults(self):
        if not self.rOk():
            return
        self.ROS.reset_fault()

    def areMotorsOn(self) -> bool:
        return self.ROS.coe_drive_states.drives_on if self.rOk() else False
    
    def isManualResetFaults(self) -> bool:
        return self.ROS.manual_reset_faults if self.rOk() else False
            
    def isInFaultState(self) -> bool:
        return self.ROS.coe_drive_states.fault_present if self.rOk() else True
    
    def getCurrentControllerName(self) -> str:
        return self.ROS.current_controller_name if self.rOk() else "n/a"
    
    def getJointTrajectoryControllerName(self) -> str:
        return self.ROS.trajectory_controller_name if self.rOk() else "n/a"

    def getSlaveNames(self) -> List[str]:
        return self.ROS.ec_slave_states.slave_names if self.rOk() else ['n/a'] * self.number_of_ec_slaves
    
    def getSlaveStates(self) -> List[str]:
        return self.ROS.ec_slave_states.slave_states if self.rOk() else ['n/a'] * self.number_of_ec_slaves

    def getDriversStates(self) -> List[str]:
        return self.ROS.coe_drive_states.coe_drive_states if self.rOk() else ['n/a'] * len(self.joint_names)
        
    def getDriversModeOfOperations(self) -> List[str]:
        return self.ROS.coe_drive_states.modes_of_operation if self.rOk() else ['n/a'] * len(self.joint_names)
    
    def getDriversFeedbackPosition(self) -> List[float]:
        return self.ROS.RobotJointPosition if self.rOk() else [0.0] * len(self.joint_names)
    
    def getHandleFeedbackPosition(self) -> List[float]:
        return self.ROS.HandlePosition if self.rOk() else [0.0] * len(self.joint_names)
    
    def getRobotJointPosition(self) -> List[float]:
        return self.ROS.RobotJointPosition if self.rOk() else [0.0] * len(self.joint_names)
    
    def getJointNames(self) -> List[str]:
        return self.ROS._joint_names  if self.rOk() else ['n/a'] * len(self.joint_names)
    
    def performHoming(self) -> bool:
        return self.ROS.perform_homing() if self.rOk() else False
        
    def sendPTPTrajectory(self, target_point, end_time) -> bool:
        if not self.areMotorsOn() and not self.turnOnMotors():
            return False
        result : bool = self.ROS.send_ptp_trajectory(target_point, end_time)
        return result       

    def isJoggingBehaviourEnabled(self) -> bool:
        return self.ROS.enable_jog_buttons if self.rOk() else False
    
    def isManualGuidanceBehaviourEnabled(self) -> bool:
        return self.ROS.enable_manual_guidance if self.rOk() else False

    def isHomingEnabled(self) -> bool:
        if not self.rOk():
            return False
        return self.ROS.enable_zeroing if self.rOk() else False
    
    def isPTPEnabled(self) -> bool:
        return self.ROS.enable_ptp if self.rOk() else False
    
    def triggerSoftMovementStart(self, amplitude: float, time_constant: float, target: str) -> None:
        return self.ROS.trigger_soft_movement_start(amplitude=amplitude, time_constant=time_constant, target=target) if self.rOk() else None      

    def triggerSoftMovementStop(self) -> None:
        return self.ROS.trigger_soft_movement_stop( ) if self.rOk() else None

    def updateState(self) -> None:
        self.ROS.update_controller_and_driver_states() if self.rOk() else False
        return 
    
    # def listActiveControllers(self) -> None:
    #     _ = self.ROS.list_active_controllers() if self.rOk() else False
    #     return 
    
    def setTrajectory(self, CartesianPositions, TimeFromStart, Ovr: list)-> None:
        _ = self.ROS.send_ptp_trajectory(CartesianPositions, TimeFromStart) if self.rOk() else False
        return 
    
    def setExercise(self, CartesianPositions, TimeFromStart, Ovr: list, durations: list, eeg_mode: bool)-> None:
        _ = self.ROS.set_exercise(CartesianPositions, TimeFromStart, Ovr, durations, eeg_mode) if self.rOk() else False
        return 
    
    def getPLCStates(self) -> dict:
        ret : dict = {'s_input.0' : False, 'estop' : False, 'reset': False, 'manual_switch_pressed' : False, 's_input.5' : False, 's_input.6' : False, 's_input.7' : False, 's_input.8' : False }
        return self.ROS.plc_states if self.rOk() else ret

    def isEmergencyActive(self) : 
        return not self.ROS.plc_states['estop'] if self.rOk() and 'estop' in self.ROS.plc_states else True
    
    def driveLogicSwitchOff(self) :
        if not self.rOk():
            return
        self.ROS.publish_plc_command(['PLC_node/power_cutoff'], [1])
    
    def driveLogicSwitchOn(self) :
        if not self.rOk():
            return
        self.ROS.publish_plc_command(['PLC_node/power_cutoff'], [0])

    def FTSensorReset(self, value: int) -> None:
        if not self.rOk():
            return
        self.ROS.publish_plc_command(['PLC_node/force_sensors_pwr'], [value])

    def brakeSwitchOn(self) :
        if not self.rOk():
            return
        self.ROS.publish_plc_command(['PLC_node/brake_disable'], [1])

    def brakeSwitchOff(self) :
        if not self.rOk():
            return
        self.ROS.publish_plc_command(['PLC_node/brake_disable'], [0])

    def stopAnyMovement(self) -> bool:
        return self.ROS.stop_movement() if self.rOk() else False
    
    def getExecutionTimePercentage(self) -> int:
        return self.ROS.execution_time_percentage if self.rOk() else 0
    
    def setExerciseCompleted(self, value : bool) -> None:
        if self.rOk(): 
            self.ROS.exercise_completed = value

    def getExerciseCompleted(self) -> bool:
        return self.ROS.exercise_completed if self.rOk() else False
    
    def getExerciseRepetitionCounter(self) -> int:
        return self.ROS.repetition_cnt if self.rOk() else 0
    
    def setTrajectoryCompleted(self, value : bool) -> None:
        if self.rOk(): 
            self.ROS.trajectory_completed = value

    def getTrajectoryCompleted(self) -> bool:
        return self.ROS.trajectory_completed if self.rOk() else False
    
    def getMovementStopped(self) -> bool:
        return self.ROS.movement_stopped if self.rOk() else False
    
    def getMovementStatus(self) -> str:
        return self.ROS.get_movement_status() if self.rOk() else 'n/a'
