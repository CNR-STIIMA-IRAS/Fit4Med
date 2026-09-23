# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import faulthandler
import os
import sys
import signal
import threading
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QAbstractButton, QAbstractSpinBox, QComboBox, QLineEdit, QWidget, QApplication, QMainWindow, QMessageBox, QLabel, QDialog, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from GuiRosTasks import Call, CallThread

# mathematics
import numpy as np

from rich.traceback import install
install(show_locals=True)

#MC Classes/methods
from ui.uiFMRRMainWindow import Ui_FMRRMainWindow # import from file ui the class Ui_ui
from MotorsWindow import MotorsWindow
from RobotWindow import RobotWindow
from RehabilitationMovementWindow import RehabilitationMovementWindow, ExerciseType
from TrainingProtocolWindow import TrainingProtocolWindow

#ROS
from UdpCommunicationManager import UdpCommunicationManager
from RosCommunicationManager import RosCommunicationManager


JOINT_NAMES = [
    'joint_x',
    'joint_y',
    'joint_z'
]
log = open(r"C:\\temp\\pyqt_hang.log", "a", buffering=1)
faulthandler.enable(file=log)
faulthandler.dump_traceback_later(10,repeat=True,file=log)

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

        self._closing = False
        self._close_ready = False
        self._udp_shutdown_worker = None
        self._busy_widgets = {}
        self._update_window_period = 100
        self._update_TrainingTime = 100
        self._plc_udp_watchdog_timeout_s = 5.0
        self._plc_udp_watchdog_period_ms = 1000
        self._toolPosCovFact = 100 # to display coordinatates in centimeters (are given in meters in the yaml files) (used in MovementWindow to display data)
        self._jointPosConvFact = 180/np.pi # conversion from radiants to degrees (used in MovementWindow to display data)
        #self.trigger_pause = pyqtSignal(bool) # signal to pause the worker thread

        ###############################################
        self.ui = Ui_FMRRMainWindow()
        self.ui.setupUi(self)
        ###############################################

        self.number_of_ec_slaves : int = 4
        self.remote_ip : str = remote_ip
        self.udp_port : int = udp_port
        self.roslibpy_port : int = roslibpy_port
        print(f"FMRRMainProgram: remote_ip={self.remote_ip}, udp_port={self.udp_port}, roslibpy_port={self.roslibpy_port}")
        self.udp = UdpCommunicationManager(self.remote_ip, self.udp_port, self.number_of_ec_slaves)
        print(f"upd:{self.udp}")
        
        self.ros_manager = RosCommunicationManager(JOINT_NAMES,  self.number_of_ec_slaves,  self.remote_ip, self.roslibpy_port, self)


        self.motorWindow = MotorsWindow()
        self.robotWindow = RobotWindow(self)
        self.rehabMovementWindow = RehabilitationMovementWindow(self)
        self.trainingProtocolWindow = TrainingProtocolWindow(self)
        self._ros_runtime_loss_dialog_shown = False
        self._udp_bind_failed_dialog_shown = False
        self._last_allowed_tab_index = 0

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

    def connect(self):
        self.update_window_timer = QTimer()
        self.ros_manager.setPlcStatusProvider(self.udp)
        self.motorWindow.connect(self.ros_manager, self.udp, self.update_window_timer)
        self.robotWindow.connect(self.ros_manager, self.udp, self.update_window_timer)
        self.rehabMovementWindow.connect(self.ros_manager, self.update_window_timer)
        self.trainingProtocolWindow.connect(self.ros_manager, self.update_window_timer)

        ## 
        self.ui.tabWidget.currentChanged.connect(self.onTabChange)
        self.ui.pushButton_CloseProgram.pressed.connect(self.closeProgram)
        ## Connect the 
        self.udp.setRosCommunicationActiveChecker(self.ros_manager.isRosCommunicationActive)
        self.udp.start_ros_communication.connect(self.ros_manager.startRosCommunication)
        self.udp.stop_ros_communication.connect(self.ros_manager.stopRosCommunication)
        self.udp.udp_message_received.connect(self.motorWindow.onUdpMessageReceived)
        self.udp.udp_bind_failed.connect(self._onUdpBindFailed)
        self.ros_manager.stop_ros_communication_signal.connect(self.udp.onResetRosCommunication)
        self.ros_manager.ros_communication_established_signal.connect(self.udp.onRosCommunicationEstablished)
        self.ros_manager.ros_communication_failed_signal.connect(self.udp.onRosCommunicationFailed)
        self.ros_manager.ros_runtime_connection_lost_signal.connect(self.onRuntimeRosCommunicationLost)
        self.ros_manager.commandsBusyChanged.connect(self._onCommandsBusyChanged)
        self.ros_manager.commandFailed.connect(self._onCommandFailed)
        self.ros_manager.stopFailed.connect(self._onCommandFailed)
        self.ros_manager.shutdownFinished.connect(self._onRosShutdownFinished)

        self.update_window_timer.timeout.connect(self.updateWindow)
        self.update_window_timer.start(self._update_window_period)

        self.plc_udp_watchdog_timer = QTimer()
        self.plc_udp_watchdog_timer.timeout.connect(self.checkPlcUdpWatchdog)
        self.plc_udp_watchdog_timer.start(self._plc_udp_watchdog_period_ms)

    def updateWindow(self):
        if self.ros_manager.isCommandBusy() or self._closing:
            return
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

    def syncExerciseTypeToPLC(self, force: bool = False) -> bool:
        movement_type = self.rehabMovementWindow.TypeOfMovement
        if movement_type == ExerciseType.REACHING:
            return self.ros_manager.setExerciseType(2, force=force) # 2: switch sensor
        if movement_type == ExerciseType.HAND_TO_MOUTH:
            return self.ros_manager.setExerciseType(1, force=force) # 1: proximity sensor
        return self.ros_manager.setExerciseType(0, force=force) # 0: no sensor

    def checkPlcUdpWatchdog(self):
        self.motorWindow.updateUdpWatchdog(self._plc_udp_watchdog_timeout_s)

    def onRuntimeRosCommunicationLost(self, message: str) -> None:
        self.ros_manager.stopRosCommunication()
        if self._ros_runtime_loss_dialog_shown:
            return

        self._ros_runtime_loss_dialog_shown = True
        dialog = QMessageBox(
            QMessageBox.Critical,
            "Restart GUI Required",
            (
                "ROS communication was lost after the GUI was connected.\n\n"
                "Restart the GUI before using the system again."
            ),
            QMessageBox.Ok,
            self,
        )
        dialog.setInformativeText("The current GUI state may no longer be reliable.")
        if message:
            dialog.setDetailedText(message)
        dialog.setWindowModality(Qt.ApplicationModal)
        dialog.setWindowFlags(dialog.windowFlags() | Qt.WindowStaysOnTopHint)
        dialog.exec_()

    def _onUdpBindFailed(self, reason: str) -> None:
        # The UDP layer talking to the embedded controller is dead until this
        # process (or whatever else holds the port) is restarted: nothing
        # from the controller will ever arrive, with no other visible symptom.
        if self._udp_bind_failed_dialog_shown:
            return

        self._udp_bind_failed_dialog_shown = True
        dialog = QMessageBox(
            QMessageBox.Critical,
            "Comunicazione UDP non disponibile",
            (
                "Impossibile aprire la porta UDP verso il controllore embedded.\n\n"
                "Probabilmente un processo precedente la sta ancora occupando. "
                "Chiudere questa GUI, verificare/terminare eventuali processi residui "
                "e riavviare anche il controllore embedded prima di continuare."
            ),
            QMessageBox.Ok,
            self,
        )
        dialog.setDetailedText(reason)
        dialog.setWindowModality(Qt.ApplicationModal)
        dialog.setWindowFlags(dialog.windowFlags() | Qt.WindowStaysOnTopHint)
        dialog.exec_()

    def _onCommandFailed(self, message):
        print("[GUI ROS] " + message)
        self.statusBar().showMessage(message, 15000)

    def _onCommandsBusyChanged(self, busy):
        # Preserve Stop/Interrupt and jog releases during an in-flight command.
        # Other editable inputs are frozen to prevent a command from using
        # values changed halfway through its GUI continuation.
        if busy:
            if self._busy_widgets:
                return
            allowed = set()
            for widget in (self.trainingProtocolWindow.ui.pushButton_STARTtrainig,
                           self.robotWindow.ui.pushButton_ApproachAllJoint):
                if widget.isChecked():
                    allowed.add(widget)
            for name in ('Xminus', 'Xplus', 'Yminus', 'Yplus', 'Zminus', 'Zplus'):
                widget = getattr(self.robotWindow.ui, 'pushButton_' + name)
                if widget.isDown():
                    allowed.add(widget)
            for window in (self.motorWindow, self.robotWindow,
                           self.rehabMovementWindow, self.trainingProtocolWindow):
                for widget in window.findChildren(QWidget):
                    if isinstance(widget, (QAbstractButton, QAbstractSpinBox, QComboBox, QLineEdit)) and widget not in allowed:
                        self._busy_widgets[widget] = widget.isEnabled()
                        widget.setEnabled(False)
            self.ui.tabWidget.tabBar().setEnabled(False)
            self.statusBar().showMessage("Operazione ROS in corso...")
        else:
            for widget, enabled in self._busy_widgets.items():
                widget.setEnabled(enabled)
            self._busy_widgets.clear()
            self.ui.tabWidget.tabBar().setEnabled(not self._closing)
            # Cached enable-state decisions must be recalculated after restore.
            for window in (self.motorWindow, self.robotWindow,
                           self.rehabMovementWindow, self.trainingProtocolWindow):
                for name in tuple(vars(window)):
                    if name.startswith('_last_'):
                        setattr(window, name, None)
            if not self._closing:
                self.updateWindow()
                training = self.trainingProtocolWindow
                for spin in training.spinBoxSpeedOvr + training.spinBoxDuration:
                    spin.setEnabled(not training.Training_ON and not training._stop_pending)
                training.ui.pushButton_PauseTrainig.setEnabled(training.Training_ON and not training._training_paused)
                training.ui.pushButton_ResumeTraining.setEnabled(training.Training_ON and training._training_paused)

    def _shutdown_communications(self):
        if self._closing:
            return
        self._closing = True
        self.update_window_timer.stop()
        self.plc_udp_watchdog_timer.stop()
        self.statusBar().showMessage("Arresto e chiusura comunicazioni...")
        # Completion drives the remaining shutdown; never wait in closeEvent.
        self.ros_manager.requestShutdown(stop_motion=True)

    @pyqtSlot(bool)
    def _onRosShutdownFinished(self, ok):
        if not self._closing:
            return
        if not ok:
            self._closing = False
            self.update_window_timer.start(self._update_window_period)
            self.plc_udp_watchdog_timer.start(self._plc_udp_watchdog_period_ms)
            QMessageBox.warning(self, "Chiusura non completata",
                                "Arresto robot o worker ROS non confermato. Controllare i log prima di riprovare.")
            return
        self.udp.onResetRosCommunication()
        self._udp_shutdown_worker = CallThread(Call(self.udp.shutdown), self)
        self._udp_shutdown_worker.finished.connect(self._onUdpShutdownFinished)
        self._udp_shutdown_worker.start()

    @pyqtSlot()
    def _onUdpShutdownFinished(self):
        worker = self._udp_shutdown_worker
        self._udp_shutdown_worker = None
        error = worker.error
        worker.deleteLater()
        if error is not None or self.udp.udp_thread.isRunning():
            self._closing = False
            self.update_window_timer.start(self._update_window_period)
            self.plc_udp_watchdog_timer.start(self._plc_udp_watchdog_period_ms)
            QMessageBox.warning(self, "Chiusura non completata", "Il thread UDP non si è arrestato.")
            return
        self._close_ready = True
        self.close()

    def closeEvent(self, event):
        if self._close_ready:
            event.accept()
            return
        event.ignore()
        if self._closing:
            return
        decision = QMessageBox.question(self, "Exit Program", "Do you want to exit?",
                                        QMessageBox.Yes | QMessageBox.No)
        if decision == QMessageBox.Yes:
            self._shutdown_communications()

    def closeProgram(self):
        self.close()

    def onTabChange(self, value: int):
        # This method is called whenever the current tab is changed
        # value is the index of the newly selected tab
        if self.trainingProtocolWindow.Training_ON and value != self._last_allowed_tab_index:
            self.ui.tabWidget.blockSignals(True)
            self.ui.tabWidget.setCurrentIndex(self._last_allowed_tab_index)
            self.ui.tabWidget.blockSignals(False)
            QMessageBox.warning(self, "Training Active", "Stop training before changing tab.")
            return

        self._last_allowed_tab_index = value
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
