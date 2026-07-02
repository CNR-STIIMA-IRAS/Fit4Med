# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import gc
import time
from typing import Any, List
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
    ros_communication_established_signal : pyqtSignal = pyqtSignal()
    ros_communication_failed_signal : pyqtSignal = pyqtSignal()
    ros_runtime_connection_lost_signal : pyqtSignal = pyqtSignal(str)
    controller_behaviour_ready_signal : pyqtSignal = pyqtSignal(str, bool, str)
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
        self._exercise_in_suspension: bool = False
        self._exercise_type: int = 0
        self._plc_status_provider: Any = None
        self._stop_signal_emitted = False
        self._ros_stop_requested = False
        self._worker_stop_timeout_msec = 5000
        self._worker_force_stop_timeout_msec = 3000
        self._controller_behaviour_timeout_s = 5.0
        self._pending_controller_behaviour = None
        self._pending_controller_mode = None
        self._pending_controller_name = None
        self._pending_controller_deadline = 0.0

        self.worker_thread = Worker(self.updateState, loop_period_s=0.2)
        self.worker_thread.finished.connect(self.onUpdateWorkerThreadFinished)

    def setPlcStatusProvider(self, provider: Any) -> None:
        self._plc_status_provider = provider

    def _is_manual_switch_pressed(self) -> bool:
        if self._plc_status_provider is None:
            return False

        try:
            return bool(self._plc_status_provider.isManualSwitchPressed())
        except Exception as exc:
            print(f"Failed to read manual switch state from UDP status provider: {exc}")
            return False

    def _controller_behaviour_target(self, behaviour: str):
        if not self.rOk():
            return None
        if behaviour == "Homing":
            return 6, None
        if behaviour == "Jogging":
            return 9, self.ROS.forward_command_controller_name
        if behaviour == "ManualGuidance":
            return 9, self.ROS.admittance_controller_name
        if behaviour == "GoToStart":
            return 8, self.ROS.go_to_start_controller_name
        return 8, self.ROS.trajectory_controller_name

    def _clear_pending_controller_behaviour(self) -> None:
        self._pending_controller_behaviour = None
        self._pending_controller_mode = None
        self._pending_controller_name = None
        self._pending_controller_deadline = 0.0

    def _start_pending_controller_behaviour(
        self,
        behaviour: str,
        target_mode: int,
        target_controller: Any,
    ) -> None:
        self._pending_controller_behaviour = behaviour
        self._pending_controller_mode = target_mode
        self._pending_controller_name = target_controller
        self._pending_controller_deadline = (
            time.monotonic() + self._controller_behaviour_timeout_s
        )

    def _finish_pending_controller_behaviour(self, success: bool, reason: str = "") -> None:
        behaviour = self._pending_controller_behaviour
        if behaviour is None:
            return
        self._clear_pending_controller_behaviour()
        self.controller_behaviour_ready_signal.emit(behaviour, success, reason)

    def _drives_are_in_mode(self, mode_value: int) -> bool:
        if not self.rOk():
            return False
        modes = self.ROS.coe_drive_states.modes_of_operation
        if len(modes) < len(self.joint_names):
            return False
        return all(
            self.ROS.get_op_mode_number(modes[idx]) == mode_value
            for idx in range(len(self.joint_names))
        )

    def _pending_controller_behaviour_ready(self) -> bool:
        if (
            self._pending_controller_behaviour is None
            or self._pending_controller_mode is None
            or not self.rOk()
        ):
            return False
        controller_ready = (
            self._pending_controller_name is None
            or self.ROS.current_controller_name == self._pending_controller_name
        )
        return controller_ready and self._drives_are_in_mode(self._pending_controller_mode)

    def _pending_controller_behaviour_timeout_reason(self) -> str:
        behaviour = self._pending_controller_behaviour
        controller = self.getCurrentControllerName()
        modes = self.getDriversModeOfOperations()
        return (
            f"Timed out waiting for {behaviour} controller behaviour.\n"
            f"Active controller: {controller}\n"
            f"Drive modes: {modes}"
        )

    def _check_pending_controller_behaviour(self) -> None:
        if self._pending_controller_behaviour is None:
            return
        if self._pending_controller_behaviour_ready():
            self._finish_pending_controller_behaviour(True)
            return
        if time.monotonic() >= self._pending_controller_deadline:
            self._finish_pending_controller_behaviour(
                False,
                self._pending_controller_behaviour_timeout_reason(),
            )
    
    def onUpdateWorkerThreadFinished(self):
        self._emit_stop_ros_communication_once()

    def _emit_stop_ros_communication_once(self) -> None:
        if self._stop_signal_emitted:
            return

        self._stop_signal_emitted = True
        self.stop_ros_communication_signal.emit()

    def _is_ros_stop_requested(self) -> bool:
        return self._ros_stop_requested or not self.worker_thread.thread_running

    def _request_ros_stop(self) -> None:
        self._ros_stop_requested = True
        ros_manager = self.ROS if hasattr(self, 'ROS') else None
        if ros_manager is not None:
            ros_manager.request_stop()

    def _close_ros_client(self) -> None:
        ros_client = self.ros_client if hasattr(self, 'ros_client') else None #type: ignore
        if ros_client is None:
            return

        try:
            ros_client.close()
        except Exception as exc:
            print(f"Error while closing rosbridge client: {exc}")

        try:
            del self.ros_client
        except AttributeError:
            pass

    def _handle_ros_connection_failed(self, message: str) -> None:
        print(message)
        self._finish_pending_controller_behaviour(False, message)
        self._close_ros_client()
        self.ros_communication_failed_signal.emit()

    def _handle_ros_connection_lost(self, message: str) -> None:
        if self._ros_stop_requested:
            return

        print(message)
        self._finish_pending_controller_behaviour(False, message)
        self._ros_stop_requested = True
        self.worker_thread.stop_thread()
        self.ROS = None  # type: ignore
        self._close_ros_client()
        self.ros_communication_failed_signal.emit()
        self.ros_runtime_connection_lost_signal.emit(message)

    def _stop_update_worker(self) -> bool:
        if not self.worker_thread.isRunning():
            return True

        self.worker_thread.stop_thread()
        if self.worker_thread.wait(self._worker_stop_timeout_msec):
            return True

        print(
            "ROS worker did not stop within "
            f"{self._worker_stop_timeout_msec} ms. Closing rosbridge client to unblock it."
        )
        self._close_ros_client()

        if self.worker_thread.wait(self._worker_force_stop_timeout_msec):
            return True

        print("ROS worker is still running after rosbridge client close.")
        return False

    def _destroy_ros_manager(self, worker_stopped: bool) -> None:
        ros_manager = self.ROS if hasattr(self, 'ROS') else None
        if ros_manager is None:
            return

        if worker_stopped:
            try:
                if not ros_manager.destroy():
                    print('Error trying to destroy ROS class.')
            except Exception as exc:
                # An unexpected rosbridge loss makes unsubscribe/unadvertise fail.
                # Continue clearing the local objects so the next RUNNING message
                # can build a completely new connection.
                print(f"Error while destroying disconnected ROS objects: {exc}")
        else:
            print("Skipping SyncRosManager destroy because the ROS worker is still running.")

        self.ROS = None  # type: ignore
        gc.collect()
        if worker_stopped:
            print("[MainProgram] ROS processes stopped.")
        else:
            print("[MainProgram] ROS manager reference cleared while the worker remains blocked.")

    def startRosCommunication(self) -> None:
        if self.rOk():
            print("ROS_MANAGER already initialized.")
            self.ros_communication_established_signal.emit()
            return 

        if self.worker_thread.isRunning():
            print("ROS worker is still stopping. Completing stop cleanup before retry.")
            self.stopRosCommunication()
            return

        print(f"Connecting to rosbridge at ws://{self.remote_ip}:{self.remote_port}")
        try:
            self.ros_client = roslibpy.Ros(
                host=self.remote_ip,
                port=self.remote_port
            )
            self.ros_client.run(20)
        except Exception as exc:
            if type(exc).__name__ == 'RosTimeoutError':
                self._handle_ros_connection_failed("Failed to connect to rosbridge before timeout.")
            else:
                self._handle_ros_connection_failed(f"Failed to connect to rosbridge: {exc}")
            return

        if not self.ros_client.is_connected:
            self._handle_ros_connection_failed("Failed to connect to rosbridge.")
            return

        self.ROS = SyncRosManager(self.number_of_ec_slaves, self.joint_names, self.ros_client)

        self._clear_pending_controller_behaviour()
        self._stop_signal_emitted = False
        self._ros_stop_requested = False
        self.worker_thread.start_thread()
        self.ros_communication_established_signal.emit()

    def stopRosCommunication(self) -> None:
        print("Stopping ROS processes...")

        self._request_ros_stop()
        self._finish_pending_controller_behaviour(
            False,
            "ROS communication stopped before the controller behaviour became ready.",
        )
        worker_stopped = self._stop_update_worker()

        self.ROS_active = False

        self._destroy_ros_manager(worker_stopped)
        self._close_ros_client()

        if not worker_stopped:
            print("[MainProgram] ROS worker did not stop cleanly after cleanup.")

        # ROS was never started, or its worker already stopped. Emit immediately
        # so plc_manager receives ROS_DISCONNECTED and the UDP flags are reset.
        print("[MainProgram] ROS communication stopped - signaling stop.")
        self._emit_stop_ros_communication_once()
        
            
    def isRosCommunicationActive(self) -> bool:
        return (
            hasattr(self, 'ROS')
            and self.ROS is not None
            and not self._ros_stop_requested
            and hasattr(self, 'ros_client')
            and self.ros_client.is_connected
        )

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

    def setExerciseType(self, mode: int) -> bool:
        if not self.rOk():
            return False
        if self._exercise_type != mode:
            print(f"Setting exercise type to {mode} [2 proximity, 1 proximity, 0 no sensor]...")
        self._exercise_type = mode
        self.ROS.publish_plc_command(['PLC_node/mode_of_operation'], [mode])
        return True
        
    def turnOnMotors(self, show_warning: bool = True) -> bool:
        if self.rOk():
            if self.manual_mode_activated:
                if self._is_manual_switch_pressed():
                    # set manual mode during movement
                    self.ROS.publish_plc_command(['PLC_node/manual_mode'], [1])
                else:
                    if show_warning:
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
        elif behaviour == "GoToStart":
            return self.ROS.controller_and_op_mode_switch(8, self.ROS.go_to_start_controller_name)
        else:
            return self.ROS.controller_and_op_mode_switch(8, self.ROS.trajectory_controller_name)

    def requestControllerBehaviour(self, behaviour: str) -> bool:
        if not self.rOk():
            return False
        target = self._controller_behaviour_target(behaviour)
        if target is None:
            return False
        target_mode, target_controller = target
        self._clear_pending_controller_behaviour()
        self.turnOffMotors()
        ok = self.ROS.request_controller_and_op_mode_switch(target_mode, target_controller)
        if ok:
            self._start_pending_controller_behaviour(
                behaviour,
                target_mode,
                target_controller,
            )
        return ok
        
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
    
    def getGoToStartControllerName(self) -> str:
        return self.ROS.go_to_start_controller_name if self.rOk() else "n/a"

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

    def sendGoToStartPTPTrajectory(self, target_point, end_time) -> bool:
        if not self.areMotorsOn() and not self.turnOnMotors():
            return False
        return self.ROS.send_go_to_start_ptp_trajectory(target_point, end_time) if self.rOk() else False

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

    def isModeSet(self) -> bool:
        """True when all drives are not in MODE_NO_MODE (system initialized beyond startup)."""
        if not self.rOk():
            return False
        modes = self.ROS.coe_drive_states.modes_of_operation
        if not modes:
            return False
        return all(m not in ('MODE_NO_MODE', 'n/a', None, '') for m in modes)
    
    def triggerSoftMovementStart(self, amplitude: float, time_constant: float, target: str) -> None:
        return self.ROS.trigger_soft_movement_start(amplitude=amplitude, time_constant=time_constant, target=target) if self.rOk() else None      

    def triggerSoftMovementStop(self) -> None:
        return self.ROS.trigger_soft_movement_stop( ) if self.rOk() else None

    def updateState(self) -> None:
        ros_manager = self.ROS if hasattr(self, 'ROS') else None
        ros_client = self.ros_client if hasattr(self, 'ros_client') else None #type: ignore
        if self._is_ros_stop_requested() or ros_manager is None or ros_client is None:
            return
        if not ros_client.is_connected:
            self._handle_ros_connection_lost("ROS bridge connection lost during GUI update.")
            return

        try:
            ros_manager.update_controller_and_driver_states()
            self._check_pending_controller_behaviour()
        except Exception as exc:
            if not ros_client.is_connected:
                self._handle_ros_connection_lost(
                    f"ROS bridge connection lost during GUI update: {exc}"
                )
            else:
                print(f"ROS GUI update failed: {exc}")
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

    def setMovementStopped(self, value: bool) -> None:
        if self.rOk():
            self.ROS.movement_stopped = value
    
    def getExerciseSuspended(self) -> bool:
        return self.ROS.exercise_suspended if self.rOk() else False

    def setExerciseSuspended(self, value: bool) -> None:
        if self.rOk():
            self.ROS.exercise_suspended = value

    # ------------------------------------------------------------------ #
    # Suspension state (GUI-level flag — survives ROS flag reset)          #
    # ------------------------------------------------------------------ #
    def setExerciseInSuspension(self, value: bool) -> None:
        """Mark/clear the persistent 'exercise suspended, waiting for resume' state."""
        self._exercise_in_suspension = value

    def isExerciseInSuspension(self) -> bool:
        return self._exercise_in_suspension

    def eegSync(self) -> None:
        """Send a 100 ms HIGH→LOW sync pulse to the EEG system via the PLC."""
        if not self.rOk():
            return
        self.ROS.send_eeg_sync()

    def startBagRecording(self) -> None:
        """Ask the bag_recorder_node on the Linux PC to start recording."""
        if not self.rOk():
            return
        self.ROS.bag_recorder_start_client.call_async()

    def stopBagRecording(self) -> None:
        """Ask the bag_recorder_node on the Linux PC to stop recording."""
        if not self.rOk():
            return
        self.ROS.bag_recorder_stop_client.call_async()

    def turnOffMotorsAsync(self) -> None:
        """Request motor stop without blocking the Qt main thread."""
        if not self.rOk():
            return
        self.ROS.publish_plc_command(['PLC_node/manual_mode'], [0])
        if self.areMotorsOn():
            self.ROS.motors_off_client.call_async()

    def getMovementStatus(self) -> str:
        return self.ROS.get_movement_status() if self.rOk() else 'n/a'
