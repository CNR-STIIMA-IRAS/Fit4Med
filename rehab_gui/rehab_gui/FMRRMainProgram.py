# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import signal
import threading
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QLabel, QDialog, QVBoxLayout, QPushButton
from PyQt5.QtCore import QObject, Qt, QThread, QTimer, pyqtSignal, pyqtSlot

# mathematics
import numpy as np

from rich.traceback import install
install(show_locals=True)

#MC Classes/methods
from ui.uiFMRRMainWindow import Ui_FMRRMainWindow # import from file ui the class Ui_ui
from MotorsWindow import MotorsWindow
from RobotWindow import RobotWindow
from RehabilitationMovementWindow import RehabilitationMovementWindow
from TrainingProtocolWindow import TrainingProtocolWindow

#ROS
from UdpCommunicationManager import UdpCommunicationManager
from RosCommunicationManager import RosCommunicationManager


JOINT_NAMES = [
    'joint_x',
    'joint_y',
    'joint_z'
]

#########################################################################
##
##
##
##
##
#########################################################################
class WaitingDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Alert")
        self.setModal(True)
        self.setWindowFlags(Qt.Dialog | Qt.WindowStaysOnTopHint)

        label = QLabel("Waiting for robot connection...")
        label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(label)
        self.setLayout(layout)
        self.resize(300, 100)


class ZRecoveryJogWorker(QObject):
    """Run blocking Z-recovery ROS commands away from the Qt GUI thread."""

    shutdown_finished = pyqtSignal()

    def __init__(self, ros_manager):
        super().__init__()
        self.ROS = ros_manager
        self._motors_started = False
        self._jog_active = False

    def _stop_motion(self):
        if not self._motors_started and not self._jog_active:
            return
        if not self.ROS.isRosCommunicationActive():
            return
        if self._jog_active:
            self.ROS.toogleJoggingBehaviour(axis=2, direction=0)
            self._jog_active = False
        if self._motors_started:
            self.ROS.turnOffMotors()
            self._motors_started = False

    @pyqtSlot(object)
    def start_z_plus(self, hold_event):
        try:
            if not hold_event.is_set() or not self.ROS.isRosCommunicationActive():
                return

            self.ROS.setManualMode(True)
            if not self.ROS.turnOnMotors(show_warning=False):
                return
            self._motors_started = True

            # The operator may have released while the motor service was running.
            if not hold_event.is_set():
                self._stop_motion()
                return

            self._jog_active = True
            self.ROS.toogleJoggingBehaviour(axis=2, direction=1)

            # The release may also have occurred during the jog service call.
            if not hold_event.is_set():
                self._stop_motion()
        except Exception as exc:
            print(f"Z+ recovery start failed: {exc}")
            try:
                self._stop_motion()
            except Exception as stop_exc:
                print(f"Z+ recovery cleanup failed: {stop_exc}")

    @pyqtSlot()
    def stop_z_plus(self):
        try:
            self._stop_motion()
        except Exception as exc:
            print(f"Z+ recovery stop failed: {exc}")

    @pyqtSlot()
    def shutdown(self):
        self.stop_z_plus()
        self.shutdown_finished.emit()


#########################################################################
##
##  Z-Axis Limit Recovery Dialog
##  Shown when an emergency is caused by the Z-axis reaching its limit.
##  Guides the operator to move Z+ until the limit switch clears.
##
#########################################################################
class ZRecoveryDialog(QDialog):
    """Modal dialog that guides the operator through Z-axis limit recovery.

    Phases:
      0. Limit hit  – system just stopped; operator must turn the reset key.
      1. Waiting    – key turned, platform restarting; waiting for ROS connection.
      2. Active     – jogging mode ready; operator holds manual switch + Z+ button.
      3. Done       – second emergency cleared; operator turns key to resume.
    """

    _start_jog_requested = pyqtSignal(object)
    _stop_jog_requested = pyqtSignal()
    _shutdown_jog_requested = pyqtSignal()

    def __init__(self, ros_manager, parent=None):
        super().__init__(parent)
        self.ROS = ros_manager
        self._done = False
        self._in_jog_phase = False
        self._jogging_ready = False
        self._jog_button_held = False
        self._jog_hold_event = None
        self._jog_worker_shutting_down = False

        self._jog_thread = QThread(self)
        self._jog_worker = ZRecoveryJogWorker(self.ROS)
        self._jog_worker.moveToThread(self._jog_thread)
        self._start_jog_requested.connect(self._jog_worker.start_z_plus)
        self._stop_jog_requested.connect(self._jog_worker.stop_z_plus)
        self._shutdown_jog_requested.connect(self._jog_worker.shutdown)
        self._jog_worker.shutdown_finished.connect(self._jog_thread.quit)
        self._jog_thread.finished.connect(self._jog_worker.deleteLater)
        self._jog_thread.start()

        self.setWindowTitle("Z-Axis Limit Recovery")
        self.setWindowFlags(
            Qt.Dialog | Qt.WindowStaysOnTopHint | Qt.CustomizeWindowHint | Qt.WindowTitleHint
        )
        self.setWindowModality(Qt.ApplicationModal)

        layout = QVBoxLayout()
        layout.setSpacing(16)
        layout.setContentsMargins(24, 24, 24, 24)

        self._status_label = QLabel(
            "⚠  Z-axis limit switch reached.\n\n"
            "The system has been stopped.\n"
            "Turn the reset key to begin recovery."
        )
        self._status_label.setAlignment(Qt.AlignCenter)
        self._status_label.setWordWrap(True)
        self._status_label.setStyleSheet(
            "font-size: 13px; color: rgb(200,100,0); font-weight: bold; padding: 8px;"
        )
        layout.addWidget(self._status_label)

        self._jog_btn = QPushButton("HOLD TO MOVE Z+")
        self._jog_btn.setVisible(False)
        self._jog_btn.setEnabled(False)
        self._jog_btn.setStyleSheet(
            """
            QPushButton {
                background-color: rgb(85,255,127);
                color: black;
                font-size: 15px;
                padding: 18px;
            }
            QPushButton:pressed {
                background-color: rgb(200,100,0);
                color: black;
            }
            """
        )
        self._jog_btn.pressed.connect(self._start_z_plus)
        self._jog_btn.released.connect(self._stop_z_plus)
        layout.addWidget(self._jog_btn)

        self._ok_btn = QPushButton("OK")
        self._ok_btn.setVisible(False)
        self._ok_btn.setStyleSheet(
            "background-color: rgb(85,255,127); color: black; font-size: 13px; padding: 10px;"
        )
        self._ok_btn.clicked.connect(self.accept)
        layout.addWidget(self._ok_btn)

        self.setLayout(layout)
        self.resize(520, 280)

        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_connection)
        self.finished.connect(self._on_dialog_finished)
        # Poll timer is started only when key is turned (enter_jog_phase)

    # ------------------------------------------------------------------
    def enter_jog_phase(self):
        """Called when plc_manager sends Z_RECOVERY_RUNNING (key was turned, platform restarting)."""
        if self._done or self._in_jog_phase:
            return
        self._in_jog_phase = True
        self._status_label.setText(
            "Press the button to initiate Z+ recovery..."
        )
        self._status_label.setStyleSheet(
            "font-size: 13px; color: rgb(200,100,0); font-weight: bold; padding: 8px;"
        )
        self._jog_btn.setVisible(True)
        self._poll_timer.start(500)

    def _poll_connection(self):
        """Enable Z+ button once ROS connects and jogging mode is ready."""
        if self._done:
            return
        ros_ok = self.ROS.isRosCommunicationActive()
        if ros_ok and not self._jogging_ready:
            if self.ROS.enableControllerBehaviour("Jogging"):
                self._jogging_ready = True
                self._status_label.setText(
                    "Z-axis is outside the allowed range.\n"
                    "Hold the manual safety switch and press the button\n"
                    "below to move Z+ until the limit switch triggers."
                )
                self._status_label.setStyleSheet(
                    "font-size: 13px; color: black; padding: 8px;"
                )
                self._jog_btn.setEnabled(True)
        elif not ros_ok and self._jogging_ready:
            self._jogging_ready = False
            self._request_jog_stop()
            self._jog_btn.setEnabled(False)
            self._status_label.setText(
                "Robot connection lost.\nWaiting for reconnection..."
            )
            self._status_label.setStyleSheet(
                "font-size: 13px; color: rgb(200,100,0); font-weight: bold; padding: 8px;"
            )

    def _start_z_plus(self):
        if (
            self._done
            or self._jog_worker_shutting_down
            or self._jog_button_held
            or not self.ROS.isRosCommunicationActive()
        ):
            return

        self._jog_button_held = True
        self._jog_hold_event = threading.Event()
        self._jog_hold_event.set()
        self._start_jog_requested.emit(self._jog_hold_event)

    def _stop_z_plus(self):
        self._request_jog_stop()

    def _request_jog_stop(self):
        self._jog_button_held = False
        if self._jog_hold_event is not None:
            self._jog_hold_event.clear()
            self._jog_hold_event = None

        if not self._jog_worker_shutting_down and self._jog_thread.isRunning():
            self._stop_jog_requested.emit()

    def _shutdown_jog_worker(self):
        if self._jog_worker_shutting_down or not self._jog_thread.isRunning():
            return

        self._jog_button_held = False
        if self._jog_hold_event is not None:
            self._jog_hold_event.clear()
            self._jog_hold_event = None

        self._jog_worker_shutting_down = True
        self._shutdown_jog_requested.emit()

    @pyqtSlot(int)
    def _on_dialog_finished(self, _result):
        self._poll_timer.stop()
        self._shutdown_jog_worker()

    def on_recovery_done(self):
        """Called by the main window when Z_RECOVERY_DONE is received."""
        self._done = True
        self._poll_timer.stop()
        self._request_jog_stop()
        self._jog_btn.setEnabled(False)
        self._jog_btn.setVisible(False)
        self._status_label.setText(
            "Z-axis is back within the allowed range.\n\n"
        )
        self._status_label.setStyleSheet(
            "font-size: 13px; color: green; font-weight: bold; padding: 8px;"
        )
        self._ok_btn.setVisible(True)

    def closeEvent(self, event):
        if not self._done:
            event.ignore()  # cannot be dismissed until recovery is complete
        else:
            self._poll_timer.stop()
            self._shutdown_jog_worker()
            event.accept()


#########################################################################
##
##
##
##
##
#########################################################################
class MainProgram(QMainWindow):

    def __init__(self, remote_ip, udp_port = 5005, roslibpy_port=9090):
        super().__init__()

        self._update_window_period = 100
        self._update_TrainingTime = 100
        self._toolPosCovFact = 100 # to display coordinatates in centimeters (are given in meters in the yaml files) (used in MovementWindow to display data)
        self._jointPosConvFact = 180/np.pi # conversion from radiants to degrees (used in MovementWindow to display data)
        self.trigger_pause = pyqtSignal(bool) # signal to pause the worker thread

        ###############################################
        self.ui = Ui_FMRRMainWindow()
        self.ui.setupUi(self)
        ###############################################

        self.number_of_ec_slaves : int = 4
        self.remote_ip : str = remote_ip
        self.udp_port : int = udp_port
        self.roslibpy_port : int = roslibpy_port
        print(f"FMRRMainProgram: remote_ip={self.remote_ip}, udp_port={self.udp_port}, roslibpy_port={self.roslibpy_port}")
        self.udp = UdpCommunicationManager(self.remote_ip, self.udp_port)
        print(f"upd:{self.udp}")
        
        self.ros_manager = RosCommunicationManager(JOINT_NAMES,  self.number_of_ec_slaves,  self.remote_ip, self.roslibpy_port, self)


        self.motorWindow = MotorsWindow()
        self.robotWindow = RobotWindow(self)
        self.rehabMovementWindow = RehabilitationMovementWindow(self)
        self.trainingProtocolWindow = TrainingProtocolWindow(self)

        self.ui.verticalLayout_MotorsManagement.addWidget(self.motorWindow)
        self.ui.verticalLayout_RobotMovement.addWidget(self.robotWindow)
        self.ui.verticalLayout_RehabilitationMovement.addWidget(self.rehabMovementWindow)
        self.ui.verticalLayout_TrainingProtocol.addWidget(self.trainingProtocolWindow)

        # Connect the currentChanged signal to the slot
        #self.ros_waiting_dialog = WaitingDialog(self)

        print("FMRR cell node started.")

        self.FMRR_Paths = dict() 
        current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current file's directory
        parent_directory = os.path.join(current_directory, '..')  # Step to the parent folder
        self.FMRR_Paths['Root'] = os.getcwd()
        self.FMRR_Paths['Protocols'] = parent_directory + '/Protocols'
        self.FMRR_Paths['Movements'] = parent_directory + '/Movements'  
        self.FMRR_Paths['Joint_Configuration'] = parent_directory + '/config'
        self.FMRR_Paths['Data'] = os.path.normpath(os.path.join(parent_directory, 'Data'))
        os.makedirs(self.FMRR_Paths['Data'], exist_ok=True)

        self.movement_loaded : bool = False
        self._z_recovery_dlg = None

    def _on_z_recovery_start(self):
        if self._z_recovery_dlg is not None and self._z_recovery_dlg.isVisible():
            return
        self._z_recovery_dlg = ZRecoveryDialog(self.ros_manager, self)
        self._z_recovery_dlg.show()

    def _on_z_recovery_mode(self):
        if self._z_recovery_dlg is not None:
            self._z_recovery_dlg.enter_jog_phase()

    def _on_z_recovery_done(self):
        if self._z_recovery_dlg is not None:
            self._z_recovery_dlg.on_recovery_done()

    def connect(self):
        self.update_window_timer = QTimer()
        self.motorWindow.connect(self.ros_manager, self.update_window_timer)
        self.robotWindow.connect(self.ros_manager, self.update_window_timer)
        self.rehabMovementWindow.connect(self.ros_manager, self.update_window_timer)
        self.trainingProtocolWindow.connect(self.ros_manager, self.update_window_timer)

        ## 
        self.ui.tabWidget.currentChanged.connect(self.onTabChange)
        self.ui.pushButton_CloseProgram.pressed.connect(self.closeProgram)
        ## Connect the 
        self.udp.start_ros_communication.connect(self.ros_manager.startRosCommunication)
        self.udp.stop_ros_communication.connect(self.ros_manager.stopRosCommunication)
        self.udp.udp_message_received.connect(self.motorWindow.onUdpMessageReceived)
        self.udp.plc_status_payload_received.connect(self.ros_manager.updatePlcStatusPayload)
        self.ros_manager.stop_ros_communication_signal.connect(self.udp.onResetRosCommunication)
        self.ros_manager.ros_communication_established_signal.connect(self.udp.onRosCommunicationEstablished)
        self.ros_manager.ros_communication_failed_signal.connect(self.udp.onRosCommunicationFailed)

        self.udp.z_recovery_start_signal.connect(self._on_z_recovery_start)
        self.udp.z_recovery_mode_signal.connect(self._on_z_recovery_mode)
        self.udp.z_recovery_done_signal.connect(self._on_z_recovery_done)

        self.update_window_timer.timeout.connect(self.updateWindow)
        self.update_window_timer.start(self._update_window_period)

    def updateWindow(self):
        current_tab = self.ui.tabWidget.currentIndex()
        # Always update motors window (visible across all tabs)
        self.motorWindow.updateWindow()
        # Only update the sub-window for the currently visible tab
        if current_tab == 0:
            self.robotWindow.updateWindow()
        elif current_tab == 1:
            self.rehabMovementWindow.updateWindow()
        elif current_tab == 2:
            self.trainingProtocolWindow.updateWindow()

    def _shutdown_communications(self):
        self.ros_manager.turnOffMotors()
        self.ros_manager.stopRosCommunication()
        self.udp.onResetRosCommunication()
        self.udp.shutdown()

    def closeEvent(self, event):
        MyString = "Do you want to exit?"
        question = QMessageBox(QMessageBox.Question, "Exit Program", MyString, QMessageBox.Yes | QMessageBox.No)
        decision = question.exec_()
        if decision == QMessageBox.Yes:
            self._shutdown_communications()
            event.accept() 
        else:
            event.ignore()  # Ignore the close event

    def closeProgram(self):
        MyString = "Do you want to exit?"
        question = QMessageBox(QMessageBox.Question, "Exit Program", MyString, QMessageBox.Yes | QMessageBox.No)
        decision = question.exec_()
        if decision == QMessageBox.Yes:
            self._shutdown_communications()
            QApplication.quit()

    def onTabChange(self, value: int):
        # This method is called whenever the current tab is changed
        # value is the index of the newly selected tab
        print(f"Tab changed to index: {value}")
        if value == 0:  # Assuming the second tab is the Motors tab
            if not self.ros_manager.isRosCommunicationActive():
                QMessageBox.warning(self, "Warning", f"Please check the controller configuration. Current Active Controller: {self.ros_manager.getCurrentControllerName()}")
                return
                
        elif value == 2:  # Assuming the third tab is the Robot tab
            pass
        elif value == 3:  # Assuming the third tab is the Robot tab
            pass
###########################
##
##
###########################
def main(remote_ip : str, maximise_window : bool):

    app = QtWidgets.QApplication(sys.argv)
    
    ui = MainProgram(remote_ip)
    ui.connect()
    if maximise_window:
        ui.showMaximized()
    else:
        ui.show()

    # ---------------------------
    # Event loop Qt
    # ---------------------------
    return app.exec_()

def analyze_profile():
    import pstats
    p = pstats.Stats('app_profile.prof')
    p.sort_stats('cumulative')
    p.print_stats()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Definition of the inputs argments')
    parser.add_argument('--remote-ip', metavar='xx.xx.xx.xx', required=False, default='127.0.0.1',
                        help='Address of the machine with the rosbridge websocket running')
    parser.add_argument('--profile', metavar='<true|false>', required=False, type=bool, default=False, 
                        help='path to schema')
    parser.add_argument('--maximise-window', required=False, action='store_true')
    args = parser.parse_args()

    print(args)
    
    profiler = None
    if args.profile:
        import cProfile
        print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Profiling activated')
        profiler = cProfile.Profile()
        profiler.enable()  # Start profiling
    
    exit_code = main(remote_ip=args.remote_ip, maximise_window=args.maximise_window) # Run your application
    
    if args.profile:
        profiler.disable()  # Stop profiling
        print('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Getting statistics')
        # Save profile data to a file
        profiler.dump_stats('app_profile.prof')
        analyze_profile()  # Analyze the profile

    sys.exit(exit_code)  # Exit with the application's exit code
