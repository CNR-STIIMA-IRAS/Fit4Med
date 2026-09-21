# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import os
import sys
from typing import Optional
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox, QPushButton, QProgressBar, QWidget, QSizePolicy
from PyQt5.QtCore import QTimer, QObject, Qt, pyqtSignal, QThread
from ui.uiRobotWindow import Ui_RobotWindow
from ui.uiFMRRMainWindow import Ui_FMRRMainWindow
from RosCommunicationManager import RosCommunicationManager
from UdpCommunicationManager import UdpCommunicationManager
from copy import deepcopy
import time

class Worker(QObject):
    
    def __init__(self, progress_dialog : QProgressBar):
        super().__init__()
        self.stop_thread : bool = False
        self.progress_dialog = progress_dialog
        
    def run(self):
        while not self.stop_thread:
            time.sleep(0.1)
            self.progress_dialog.setValue(self.progress_dialog.value()+1)
        
        self.progress_dialog.setValue(100)

class ProgressBarWorker(QObject):

    def __init__(self, progress_bar : QProgressBar):
        super().__init__()
        self.progress_dialog = progress_bar
        self.progress_dialog.setVisible(True)
        self.progress_dialog.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.progress_dialog.setValue(0)
        self.progress_dialog.hide()

        self.worker_thread = QThread()
        self.worker = Worker(self.progress_dialog)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run) 

    def start(self):
        self.worker.stop_thread = False
        self.worker_thread.start()
        self.progress_dialog.show()

    def stop(self):
        self.worker.stop_thread = True
        self.worker_thread.quit()
        self.worker_thread.wait()
        self.progress_dialog.close()

class RobotWindow(QtWidgets.QDialog):
    _progres_dialog_finished = pyqtSignal(bool)
    _update_window_period = 100
    ROS : RosCommunicationManager = None #type: ignore

    def __init__(self, main_app) -> None:
        super().__init__()
        self.ui = Ui_RobotWindow()
        self.ui.setupUi(self)
        self.main_app = main_app
        self.ui.comboBox_MOO.addItems(['', 'Zeroing', 'JOG', 'Manual Guidance', 'PTP'])
        self.ui.progressBar_RelativeHoming.setVisible(False)

        # Cache for enable states to avoid redundant setStyleSheet calls
        self._last_homing_state = None
        self._last_jog_state = None
        self._last_manual_guidance_state = None
        self._last_ptp_state = None

        # True while an async performHoming() command is in flight, so the periodic
        # updateWindow() polling doesn't re-enable the button out from under it.
        self._homing_in_progress = False
        self._homing_pd: Optional[ProgressBarWorker] = None

    def connect(self, ROS: RosCommunicationManager, UDP: UdpCommunicationManager, parent_timer: QTimer):
        self.ROS = ROS
        self.UDP = UDP
        self.parent_timer = parent_timer

        # Collega i segnali di cambiamento a funzioni di callback
        self.ui.comboBox_MOO.currentIndexChanged.connect(self.onBehaviourOptionChanged)
        self.ui.comboBox_MOO.activated.connect(self.onBehaviourActivation) # type: ignore
        
        self.ui.pushButton_RelativeHoming.clicked.connect(self.relativeHoming) # type: ignore
        self.ui.pushButton_RelativeHoming.setStyleSheet("")

        self.ui.pushButton_MoveRobotManually.setCheckable(True)
        self.ui.pushButton_MoveRobotManually.toggled.connect(self.moveRobotManually) # type: ignore
        self.ui.pushButton_MoveRobotManually.setStyleSheet("")
        self.ui.pushButton_MoveRobotManually.setText("Activate Manual Guidance")

        self.ui.pushButton_JOG.setCheckable(True)
        self.ui.pushButton_JOG.toggled.connect(self.moveJOG)
        self.ui.pushButton_JOG.setStyleSheet("")
        self.ui.pushButton_JOG.setText("Activate JOG")
        
        self.ui.pushButton_Xminus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = -1))
        self.ui.pushButton_Xminus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = 0))
        self.ui.pushButton_Xplus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = 1))
        self.ui.pushButton_Xplus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = 0))

        self.ui.pushButton_Yminus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = -1))
        self.ui.pushButton_Yminus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = 0))
        self.ui.pushButton_Yplus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = 1))
        self.ui.pushButton_Yplus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = 0))

        self.ui.pushButton_Zminus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = -1))
        self.ui.pushButton_Zminus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = 0))
        self.ui.pushButton_Zplus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = 1))
        self.ui.pushButton_Zplus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = 0))

        self.ui.pushButton_ApproachAllJoint.setCheckable(True)
        self.ui.pushButton_ApproachAllJoint.toggled.connect(self.goTo)

        self.ui.pushButton_SensorBias.setCheckable(False)
        self.ui.pushButton_SensorBias.clicked.connect(self.confirmSensorBias)

    def confirmSensorBias(self) -> None:
        decision = QMessageBox.question(
            self,
            "Sensor Calibration",
            "Are you sure you want to start the sensor calibration procedure?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if decision == QMessageBox.Yes:
            self.ROS.SonarBias()

    def onBehaviourOptionChanged(self, index):
        # Handle the change in MOO option here
        selected_option = self.ui.comboBox_MOO.itemText(index)
        print(f"MOO option changed to: {selected_option}")
    
    def onBehaviourActivation(self, index):
        print(f"MOO option activated to: {index}")
        slave_states = self.UDP.getSlaveStates()
        if not all([state == 'OP' for state in slave_states]):
            slave_names = self.UDP.getSlaveNames()
            slave_states_dict = dict(zip(slave_names, slave_states))
            move_states = [
                slave_state for slave_name, slave_state in slave_states_dict.items()
                if 'delta' in slave_name.lower()
            ]
            manual_guidance_states = [
                slave_state for slave_name, slave_state in slave_states_dict.items()
                if 'delta' in slave_name.lower() or 'ati' in slave_name.lower()
            ]
            move_ok = bool(move_states) and all(
                slave_state == 'OP' for slave_state in move_states
            )
            manual_guidance_ok = bool(manual_guidance_states) and all(
                slave_state == 'OP' for slave_state in manual_guidance_states
            )

            if (not move_ok and index in [1,2,4]) or (not manual_guidance_ok and index == 3):
                QMessageBox.warning(self, "Warning", f"Please check for errors the Ethercat Configuration")
                return

        try:
            if index == 1:  # Zeroing
                self.ROS.enableControllerBehaviour("Homing")
            elif index == 2:  # JOG
                self.ROS.setManualMode(True)
                self.ROS.enableControllerBehaviour("Jogging")
            elif index == 3:  # Manual Guidance
                self.ROS.setManualMode(True)
                self.ROS.enableControllerBehaviour("ManualGuidance")
            elif index == 4:  # PTP
                self.ROS.setManualMode(False)
                ActualRobotConfiguration = deepcopy( self.ROS.getHandleFeedbackPosition())
                self.ui.doubleSpin_Joint1_Value.setValue(ActualRobotConfiguration[0])
                self.ui.doubleSpin_Joint2_Value.setValue(ActualRobotConfiguration[1])
                self.ui.doubleSpin_Joint3_Value.setValue(ActualRobotConfiguration[2])
                self.ROS.enableControllerBehaviour("PTP")
        except Exception as e:
            print(f'Exception: {e}')
        
    ###### Callback of buttons to Joint Approach with trajectory controller (TODO)
    def goTo(self, toggled) -> None:
        print(f'toogled the goTo event with toggled value {toggled}')
        if toggled == 0:
            self.ROS.stopAnyMovement()
            return
        
        self.ROS.setManualMode(True)
        QTimer.singleShot(500, self._goTo_afterDelay)

    def _goTo_afterDelay(self) -> None:
        ActualRobotConfiguration  = deepcopy( self.ROS.getHandleFeedbackPosition())
        NewRobotConfiguration = ActualRobotConfiguration
        JointTargetPosition = (float(self.ui.doubleSpin_Joint1_Value.value()), float(self.ui.doubleSpin_Joint2_Value.value()), float(self.ui.doubleSpin_Joint3_Value.value()))
        if any(abs(JointTargetPosition[idx]) > 0.5 for idx in range(len(self.ROS.getJointNames()))):
            QMessageBox.warning(self, "Warning", f"Joint target position is out of range. Please set a value between -0.5 and 0.5.")
            return
        NewRobotConfiguration = JointTargetPosition
        
        target_time = max(abs(NewRobotConfiguration[i]) for i in range(len(self.ROS.getJointNames())))/0.1
        if target_time < 1.0:
            target_time = 2.0
        
        if self.ROS.turnOnMotors():
            print(f'Go To {NewRobotConfiguration} from {self.ROS.getHandleFeedbackPosition()}')
            self.ROS.sendPTPTrajectory(NewRobotConfiguration, target_time)
        else:
            QMessageBox.warning(self, "Warning", "Failed in switching on the motors")
        
    def moveRobotManually(self, activate):
        if activate:
            if not self.ROS.turnOnMotors():
                self.ui.pushButton_MoveRobotManually.setChecked(False)
        else:
            self.ROS.turnOffMotors()
    
    def moveJOG(self, activate):
        if activate:
            if not self.ROS.turnOnMotors():
                self.ui.pushButton_JOG.setChecked(False)
        else:
            self.ROS.turnOffMotors()

    def relativeHoming(self) -> None:
        pb = self.ui.pushButton_RelativeHoming
        previous_state = pb.isChecked()

        if not os.path.exists(os.path.join("/", "tmp")):
            os.makedirs(os.path.join("/", "tmp"))

        for file_name in ("absolute_homing_performed", "relative_homing_performed"):
            file_path = os.path.join("/", "tmp", file_name)
            if os.path.exists(file_path):
                try:
                    print("Removing file: ", file_path)
                    os.remove(file_path)
                except Exception as e:
                    print(f"Failed to remove {file_path}: {e}")
                    pb.setChecked(previous_state)
                    return

        self._homing_in_progress = True
        pb.setEnabled(False)
        self._homing_pd = ProgressBarWorker(self.ui.progressBar_RelativeHoming)
        self._homing_pd.start()

        # perform_homing() itself stays synchronous (service call + a bounded wait
        # loop on the drive state); only where it runs moves off the Qt thread.
        started = self.ROS.runCommandAsync(
            self.ROS.performHoming,
            on_success=lambda ok: self._onRelativeHomingDone(ok, previous_state),
            on_error=lambda msg: self._onRelativeHomingFailed(msg, previous_state),
        )
        if not started:
            self._homing_in_progress = False
            self._homing_pd.stop()
            pb.setEnabled(True)
            pb.setChecked(previous_state)
            QMessageBox.warning(self, "Busy", "Another robot command is already in progress.")

    def _onRelativeHomingDone(self, ok: bool, previous_state: bool) -> None:
        self._homing_in_progress = False
        self._homing_pd.stop()  # type: ignore
        self.ui.pushButton_RelativeHoming.setEnabled(True)
        if ok:
            with open(os.path.join("/", "tmp", "relative_homing_performed"), 'w') as f:
                f.write("homing performed")
            self.ui.pushButton_RelativeHoming.setChecked(not previous_state)
        else:
            self.ui.pushButton_RelativeHoming.setChecked(previous_state)
            QMessageBox.warning(self, "Error", "Homing failed...")

    def _onRelativeHomingFailed(self, msg: str, previous_state: bool) -> None:
        self._homing_in_progress = False
        self._homing_pd.stop()  # type: ignore
        self.ui.pushButton_RelativeHoming.setEnabled(True)
        self.ui.pushButton_RelativeHoming.setChecked(previous_state)
        QMessageBox.warning(self, "Exception", msg)

    ##############################################################################################
    #####                                                                                    #####
    #####                               UPDATE FUNCTION                                      #####
    #####                                                                                    #####
    #######################à######################################################################  
    def updateWindow(self):
        
        self.enableRelativeHomingButton(self.ROS.isHomingEnabled() and self.ui.comboBox_MOO.currentIndex() == 1)
        self.enableJogButton(self.ROS.isJoggingBehaviourEnabled() and self.ui.comboBox_MOO.currentIndex() == 2)
        self.enableManualGuidanceButton(self.ROS.isManualGuidanceBehaviourEnabled() and self.ui.comboBox_MOO.currentIndex() == 3)
        self.enablePTPFrame(self.ROS.isPTPEnabled() and self.ui.comboBox_MOO.currentIndex() == 4)

        self.ROS.setExerciseType(0)
        _result = self.ROS.consumeTrajectoryResult("ptp")
        if _result is not None:
            self.ui.pushButton_ApproachAllJoint.setChecked(False)

            if self.ROS.areMotorsOn():
                self.ROS.turnOffMotors()

            if not _result["success"]:
                QMessageBox.warning(
                    self,
                    "Trajectory failed",
                    f"{_result['message']}\nError code: {_result['error_code']}"
                )

    def enableRelativeHomingButton(self, activate: bool):
        if self._homing_in_progress or activate == self._last_homing_state:
            return
        self._last_homing_state = activate
        self.ui.frame_ReativeHoming.setEnabled(activate)
        self.ui.pushButton_RelativeHoming.setEnabled(activate)
        if activate: 
            self.ui.pushButton_RelativeHoming.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
        else:
            self.ui.pushButton_RelativeHoming.setStyleSheet("")

    def enableJogButton(self, activate: bool):
        motors_on = self.ROS.areMotorsOn() if activate else False
        state_key = (activate, motors_on)
        if state_key == self._last_jog_state:
            return
        self._last_jog_state = state_key
        self.ui.frame_JOG.setEnabled(activate)
        self.ui.pushButton_JOG.setEnabled(activate)
        self.ui.pushButton_Xminus.setEnabled(activate)
        self.ui.pushButton_Xplus.setEnabled(activate)
        self.ui.pushButton_Yminus.setEnabled(activate)
        self.ui.pushButton_Yplus.setEnabled(activate)
        self.ui.pushButton_Zminus.setEnabled(activate)
        self.ui.pushButton_Zplus.setEnabled(activate)
        if activate: 
            jog_axis_style = "background-color: rgb(255,255,224); color: black;"
            if motors_on:
                self.ui.pushButton_JOG.setStyleSheet("background-color: rgb(255, 69, 0); color: black;")
                self.ui.pushButton_JOG.setText("Deactivate Jog")
            else:
                self.ui.pushButton_JOG.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
                self.ui.pushButton_JOG.setText("Activate Jog")
            self.ui.pushButton_Xminus.setStyleSheet(jog_axis_style)
            self.ui.pushButton_Xplus.setStyleSheet(jog_axis_style)
            self.ui.pushButton_Yminus.setStyleSheet(jog_axis_style)
            self.ui.pushButton_Yplus.setStyleSheet(jog_axis_style)
            self.ui.pushButton_Zminus.setStyleSheet(jog_axis_style)
            self.ui.pushButton_Zplus.setStyleSheet(jog_axis_style)
        else:
            self.ui.pushButton_JOG.setStyleSheet("")
            self.ui.pushButton_Xminus.setStyleSheet("")
            self.ui.pushButton_Xplus.setStyleSheet("")
            self.ui.pushButton_Yminus.setStyleSheet("")
            self.ui.pushButton_Yplus.setStyleSheet("")
            self.ui.pushButton_Zminus.setStyleSheet("")
            self.ui.pushButton_Zplus.setStyleSheet("")


    def enableManualGuidanceButton(self, activate: bool):
        motors_on = self.ROS.areMotorsOn() if activate else False
        state_key = (activate, motors_on)
        if state_key == self._last_manual_guidance_state:
            return
        self._last_manual_guidance_state = state_key
        self.ui.frame_MoveRobotManually.setEnabled(activate)
        self.ui.pushButton_MoveRobotManually.setEnabled(activate)
        if activate: 
            if motors_on:
                self.ui.pushButton_MoveRobotManually.setStyleSheet("background-color: rgb(255, 69, 0); color: black;")
                self.ui.pushButton_MoveRobotManually.setText("Deactivate Manual Guidance")
            else:
                self.ui.pushButton_MoveRobotManually.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
                self.ui.pushButton_MoveRobotManually.setText("Activate Manual Guidance")
        else:
            self.ui.pushButton_MoveRobotManually.setStyleSheet("")

    def enablePTPFrame(self, activate: bool):
        motors_on = self.ROS.areMotorsOn() if activate else False
        state_key = (activate, motors_on)
        if state_key == self._last_ptp_state:
            return
        self._last_ptp_state = state_key
        self.ui.frame_GoalPosition.setEnabled(activate)
        if activate:
            if motors_on:
                self.ui.pushButton_ApproachAllJoint.setStyleSheet("background-color: rgb(255, 69, 0); color: black;")
                self.ui.pushButton_ApproachAllJoint.setText("Interrupt")
            else:
                self.ui.pushButton_ApproachAllJoint.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
                self.ui.pushButton_ApproachAllJoint.setText("GoTo")
        else:
            self.ui.pushButton_ApproachAllJoint.setStyleSheet("")
            self.ui.pushButton_ApproachAllJoint.setText("GoTo")

def main():
    app = QtWidgets.QApplication(sys.argv)
    ui = RobotWindow(None)
    ui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
