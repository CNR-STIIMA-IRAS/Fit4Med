# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from GuiRosTasks import Call, gui_task
import math
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox, QPushButton, QProgressBar, QWidget, QSizePolicy
from PyQt5.QtCore import QTimer, QObject, Qt, pyqtSignal, QThread
from ui.uiRobotWindow import Ui_RobotWindow
from ui.uiFMRRMainWindow import Ui_FMRRMainWindow
from RosCommunicationManager import RosCommunicationManager
from UdpCommunicationManager import UdpCommunicationManager
from copy import deepcopy
import time

# GoTo (PTP) timing. The ROS side interpolates with a clamped cubic spline, so
# the peak speed is 1.5x the average one.
PTP_MAX_SPEED = 0.1   # m/s, average handle speed along the straight path
PTP_MIN_TIME_S = 2.0

class ProgressBarWorker(QObject):
    def __init__(self, progress_bar):
        super().__init__(progress_bar)
        self.progress_dialog = progress_bar
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._tick)

    def _tick(self):
        self.progress_dialog.setValue(min(99, self.progress_dialog.value() + 1))

    def start(self):
        self.progress_dialog.setValue(0)
        self.progress_dialog.show()
        self.timer.start()

    def stop(self):
        self.timer.stop()
        self.progress_dialog.setValue(100)
        self.progress_dialog.hide()
        self.deleteLater()

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

        # Debounce for JOG button release: this GUI runs on a touchscreen/
        # stylus panel where a "hold" can arrive as pressed+released only a
        # few ms apart (spurious release), so a held press never moves the
        # robot. Delay the actual stop by this many ms; a new press on the
        # same axis before it fires cancels it, treating the tap as a
        # continued hold instead of a release.
        self._JOG_RELEASE_DEBOUNCE_MS = 200
        self._jog_stop_timers = {}
        self._last_ptp_state = None

    def handleButtonCallbackFailure(self, pb, callback, error_msg):
        # Only homing uses this helper. Its result is handled after completion.
        self.relativeHoming()


    def connect(self, ROS: RosCommunicationManager, UDP: UdpCommunicationManager, parent_timer: QTimer):
        self.ROS = ROS
        self.UDP = UDP
        self.parent_timer = parent_timer

        # Collega i segnali di cambiamento a funzioni di callback
        self.ui.comboBox_MOO.currentIndexChanged.connect(self.onBehaviourOptionChanged)
        self.ui.comboBox_MOO.activated.connect(self.onBehaviourActivation) # type: ignore
        
        self.ui.pushButton_RelativeHoming.clicked.connect(lambda :self.handleButtonCallbackFailure(self.ui.pushButton_RelativeHoming, self.relativeHoming, "Homing failed...")) # type: ignore
        self.ui.pushButton_RelativeHoming.setStyleSheet("")

        self.ui.pushButton_MoveRobotManually.setCheckable(True)
        self.ui.pushButton_MoveRobotManually.toggled.connect(self.moveRobotManually) # type: ignore
        self.ui.pushButton_MoveRobotManually.setStyleSheet("")
        self.ui.pushButton_MoveRobotManually.setText("Activate Manual Guidance")

        self.ui.pushButton_JOG.setCheckable(True)
        self.ui.pushButton_JOG.toggled.connect(self.moveJOG)
        self.ui.pushButton_JOG.setStyleSheet("")
        self.ui.pushButton_JOG.setText("Activate JOG")
        
        self.ui.pushButton_Xminus.pressed.connect(lambda : self.jogDirection(axis=0, direction = -1))
        self.ui.pushButton_Xminus.released.connect(lambda : self.jogDirection(axis=0, direction = 0))
        self.ui.pushButton_Xplus.pressed.connect(lambda : self.jogDirection(axis=0, direction = 1))
        self.ui.pushButton_Xplus.released.connect(lambda : self.jogDirection(axis=0, direction = 0))

        self.ui.pushButton_Yminus.pressed.connect(lambda : self.jogDirection(axis=1, direction = -1))
        self.ui.pushButton_Yminus.released.connect(lambda : self.jogDirection(axis=1, direction = 0))
        self.ui.pushButton_Yplus.pressed.connect(lambda : self.jogDirection(axis=1, direction = 1))
        self.ui.pushButton_Yplus.released.connect(lambda : self.jogDirection(axis=1, direction = 0))

        self.ui.pushButton_Zminus.pressed.connect(lambda : self.jogDirection(axis=2, direction = -1))
        self.ui.pushButton_Zminus.released.connect(lambda : self.jogDirection(axis=2, direction = 0))
        self.ui.pushButton_Zplus.pressed.connect(lambda : self.jogDirection(axis=2, direction = 1))
        self.ui.pushButton_Zplus.released.connect(lambda : self.jogDirection(axis=2, direction = 0))

        self.ui.pushButton_ApproachAllJoint.setCheckable(True)
        self.ui.pushButton_ApproachAllJoint.toggled.connect(self.goTo)

        self.ui.pushButton_SensorBias.setCheckable(False)
        self.ui.pushButton_SensorBias.clicked.connect(self.confirmSensorBias)

    def jogDirection(self, axis, direction):
        # Cancel any pending debounced stop for this axis: a fresh press
        # (same or opposite direction) means the earlier release was either
        # spurious or the operator changed direction -- either way the axis
        # keeps moving and _startJog below will (re)send the right command.
        pending_stop = self._jog_stop_timers.pop(axis, None)
        if pending_stop is not None:
            pending_stop.stop()
            pending_stop.deleteLater()

        if direction == 0:
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(lambda: self._confirmJogStop(axis))
            self._jog_stop_timers[axis] = timer
            timer.start(self._JOG_RELEASE_DEBOUNCE_MS)
        else:
            self._startJog(axis, direction)

    def _confirmJogStop(self, axis):
        self._jog_stop_timers.pop(axis, None)
        self.ROS.requestStopAnyMovement(jog_axis=axis)

    @gui_task
    def _startJog(self, axis, direction):
        yield Call(self.ROS.toogleJoggingBehaviour, axis=axis, direction=direction)

    @gui_task
    def confirmSensorBias(self) -> None:
        decision = QMessageBox.question(self, 'Sensor Calibration', 'Are you sure you want to start the sensor calibration procedure?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if decision == QMessageBox.Yes:
            yield Call(self.ROS.SonarBias)

    def onBehaviourOptionChanged(self, index):
        # Handle the change in MOO option here
        selected_option = self.ui.comboBox_MOO.itemText(index)
        print(f"MOO option changed to: {selected_option}")
    
    @gui_task
    def onBehaviourActivation(self, index):
        print(f'MOO option activated to: {index}')
        slave_states = self.UDP.getSlaveStates()
        if not all([state == 'OP' for state in slave_states]):
            slave_names = self.UDP.getSlaveNames()
            slave_states_dict = dict(zip(slave_names, slave_states))
            move_states = [slave_state for slave_name, slave_state in slave_states_dict.items() if 'delta' in slave_name.lower()]
            manual_guidance_states = [slave_state for slave_name, slave_state in slave_states_dict.items() if 'delta' in slave_name.lower() or 'ati' in slave_name.lower()]
            move_ok = bool(move_states) and all((slave_state == 'OP' for slave_state in move_states))
            manual_guidance_ok = bool(manual_guidance_states) and all((slave_state == 'OP' for slave_state in manual_guidance_states))
            if not move_ok and index in [1, 2, 4] or (not manual_guidance_ok and index == 3):
                QMessageBox.warning(self, 'Warning', f'Please check for errors the Ethercat Configuration')
                return
        try:
            if index == 1:
                yield Call(self.ROS.enableControllerBehaviour, 'Homing')
            elif index == 2:
                self.ROS.setManualMode(True)
                yield Call(self.ROS.enableControllerBehaviour, 'Jogging')
            elif index == 3:
                self.ROS.setManualMode(True)
                yield Call(self.ROS.enableControllerBehaviour, 'ManualGuidance')
            elif index == 4:
                self.ROS.setManualMode(False)
                ActualRobotConfiguration = deepcopy(self.ROS.getHandleFeedbackPosition())
                self.ui.doubleSpin_Joint1_Value.setValue(ActualRobotConfiguration[0])
                self.ui.doubleSpin_Joint2_Value.setValue(ActualRobotConfiguration[1])
                self.ui.doubleSpin_Joint3_Value.setValue(ActualRobotConfiguration[2])
                yield Call(self.ROS.enableControllerBehaviour, 'PTP')
        except Exception as e:
            print(f'Exception: {e}')
        
    ###### Callback of buttons to Joint Approach with trajectory controller (TODO)
    def goTo(self, toggled) -> None:
        print(f'toogled the goTo event with toggled value {toggled}')
        if toggled == 0:
            self.ROS.requestStopAnyMovement()
            return
        
        self.ROS.setManualMode(True)
        self._goTo_afterDelay()

    @gui_task
    def _goTo_afterDelay(self) -> None:
        yield Call(time.sleep, 0.5)
        if not self.ui.pushButton_ApproachAllJoint.isChecked():
            return
        target = (float(self.ui.doubleSpin_Joint1_Value.value()),
                  float(self.ui.doubleSpin_Joint2_Value.value()),
                  float(self.ui.doubleSpin_Joint3_Value.value()))
        if any(abs(value) > 0.5 for value in target):
            self._resetGoToButton()
            QMessageBox.warning(self, 'Warning', f'Joint target position is out of range. Please set a value between -0.5 and 0.5.')
            return
        if not (yield Call(self.ROS.turnOnMotors)):
            self._resetGoToButton()
            QMessageBox.warning(self, 'Warning', 'Failed in switching on the motors')
            return
        # Duration from the distance to where the trajectory really starts:
        # send_ptp_trajectory uses the current position as its first point
        # (read after switching the motors on).
        current = self.ROS.getRobotJointPosition()
        try:
            distance = math.dist(target, current)
            if not math.isfinite(distance):
                # max(PTP_MIN_TIME_S, nan) would silently give PTP_MIN_TIME_S.
                raise ValueError(f'distance is {distance}')
        except (TypeError, ValueError) as exc:
            # Motors are already on: switch them off before giving up.
            yield Call(self.ROS.turnOffMotors)
            self._resetGoToButton()
            QMessageBox.warning(self, 'Warning', f'Cannot compute the PTP movement from the current position {current}: {exc}')
            return
        target_time = max(PTP_MIN_TIME_S, distance / PTP_MAX_SPEED)
        print(f'Go To {target} from {current}: {distance:.3f} m in {target_time:.1f} s')
        if not (yield Call(self.ROS.sendPTPTrajectory, target, target_time)):
            yield Call(self.ROS.turnOffMotors)
            self._resetGoToButton()
            QMessageBox.warning(self, 'Warning',
                                'The PTP movement could not be sent to the robot: motors switched off.\n'
                                'Check the robot state and the logs, then try again.')

    def _resetGoToButton(self):
        # Un-press GoTo without triggering goTo(False), which would send a stop.
        self.ui.pushButton_ApproachAllJoint.blockSignals(True)
        self.ui.pushButton_ApproachAllJoint.setChecked(False)
        self.ui.pushButton_ApproachAllJoint.blockSignals(False)
        
    @gui_task
    def moveRobotManually(self, activate):
        if activate:
            if not (yield Call(self.ROS.turnOnMotors)):
                self.ui.pushButton_MoveRobotManually.setChecked(False)
        else:
            yield Call(self.ROS.turnOffMotors)
    
    @gui_task
    def moveJOG(self, activate):
        if activate:
            if not (yield Call(self.ROS.turnOnMotors)):
                self.ui.pushButton_JOG.setChecked(False)
        else:
            yield Call(self.ROS.turnOffMotors)

    @gui_task
    def relativeHoming(self) -> bool:
        pd = ProgressBarWorker(self.ui.progressBar_RelativeHoming)
        pd.start()
        if not os.path.exists(os.path.join('/', 'tmp')):
            os.makedirs(os.path.join('/', 'tmp'))
        file_path = os.path.join('/', 'tmp', 'absolute_homing_performed')
        if os.path.exists(file_path):
            try:
                print('Removing file: ', file_path)
                os.remove(file_path)
            except Exception as e:
                print(f'Failed to remove {file_path}: {e}')
                pd.stop()
                return False
        file_path = os.path.join('/', 'tmp', 'relative_homing_performed')
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
            except Exception as e:
                print(f'Failed to remove {file_path}: {e}')
                pd.stop()
                return False
        print(f'Files removed {not os.path.exists(file_path)}')
        try:
            if not (yield Call(self.ROS.performHoming)):
                QMessageBox.warning(self, 'Homing', 'Homing failed.')
                return False
            with open(os.path.join('/', 'tmp', 'relative_homing_performed'), 'w') as f:
                f.write('homing performed')
            return True
        finally:
            pd.stop()

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
        if self.main_app.ui.tabWidget.currentIndex() == 0 and\
            self.ROS.areMotorsOn() and self.ROS.getTrajectoryCompleted():
            self.ui.pushButton_ApproachAllJoint.blockSignals(True)
            self.ui.pushButton_ApproachAllJoint.setChecked(False)
            self.ui.pushButton_ApproachAllJoint.blockSignals(False)
            self.ROS.setTrajectoryCompleted(False)
            self._finishTrajectory()

    @gui_task
    def _finishTrajectory(self):
        yield Call(self.ROS.turnOffMotors)

    def enableRelativeHomingButton(self, activate: bool):
        if activate == self._last_homing_state:
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
