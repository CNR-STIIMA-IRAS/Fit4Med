import cProfile
import pstats

import os
import sys
import signal
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QLabel, QDialog, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

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

        self._update_window_period = 20
        self._update_TrainingTime = 20
        self._toolPosCovFact = 100 # to display coordinatates in centimeters (are given in meters in the yaml files) (used in MovementWindow to display data)
        self._jointPosConvFact = 180/np.pi # conversion from radiants to degrees (used in MovementWindow to display data)
        self.trigger_pause = pyqtSignal(bool) # signal to pause the worker thread

        ###############################################
        self.ui = Ui_FMRRMainWindow()
        self.ui.setupUi(self)
        ###############################################

        self.remote_ip = remote_ip
        self.udp_port = udp_port
        self.roslibpy_port = roslibpy_port
        self.udp = UdpCommunicationManager(self.remote_ip, self.udp_port)
        
        self.ros_manager = RosCommunicationManager(JOINT_NAMES, self.remote_ip, self.roslibpy_port, self)


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

        self.movement_loaded : bool = False

    def connect(self):
        self.update_window_timer = QTimer()
        self.update_window_timer.timeout.connect(self.updateWindow)
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
        self.ros_manager.stop_ros_communication_signal.connect(self.udp.onResetRosCommunication)
        #self.update_window_timer.timeout.connect(self.listActiveControllers) #type: ignore

        self.update_window_timer.start(self._update_window_period)

    def updateWindow(self):
        pass
        # if not self.ros_manager.isRosCommunicationActive():
        #     self.ros_waiting_dialog.show()
        # else:
        #     self.ros_waiting_dialog.hide()

    def closeEvent(self, event):
        MyString = "Do you want to exit?"
        question = QMessageBox(QMessageBox.Question, "Exit Program", MyString, QMessageBox.Yes | QMessageBox.No)
        decision = question.exec_()
        if decision == QMessageBox.Yes:
            self.ros_manager.turnOffMotors()
            self.ros_manager.stopRosCommunication()
            event.accept() 
        else:
            event.ignore()  # Ignore the close event

    def closeProgram(self):
        MyString = "Do you want to exit?"
        question = QMessageBox(QMessageBox.Question, "Exit Program", MyString, QMessageBox.Yes | QMessageBox.No)
        decision = question.exec_()
        if decision == QMessageBox.Yes:
              self.ros_manager.turnOffMotors()
              self.ros_manager.stopRosCommunication()
              quit()

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
    
    profiler : cProfile.Profile = None #type: ignore
    if args.profile: 
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