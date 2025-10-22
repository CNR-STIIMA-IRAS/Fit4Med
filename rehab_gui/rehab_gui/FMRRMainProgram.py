import gc
import os
import sys
import signal
import time
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QMessageBox, QPushButton
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from functools import partial

# mathematics
import numpy as np

from rich.traceback import install
install(show_locals=True)

#MC Classes/methods
from MovementProgram import FMRR_Ui_MovementWindow
from RobotProgram import FMRR_Ui_RobotWindow
from FMRRMainWindow import Ui_FMRRMainWindow # import from file FMRRMainWindow the class Ui_FMRRMainWindow
import MC_Tools
import yaml
from yaml.loader import SafeLoader

#ROS
import roslibpy
from sync_ros_events import SyncRosManager
from ros_network_checker import WaitingDialog
from RosReadyUDPServer import UdpServer

JOINT_NAMES = [
    'joint_x',
    'joint_y',
    'joint_z'
]

class MainProgram(Ui_FMRRMainWindow, QtCore.QObject): 
    _update_windows_period = 500
    _update_TrainingTime = 20
    _toolPosCovFact = 100 # to display coordinatates in centimeters (are given in meters in the yaml files) (used in MovementWindow to display data)
    _jointPosConvFact = 180/np.pi # conversion from radiants to degrees (used in MovementWindow to display data)
    trigger_pause = pyqtSignal(bool) # signal to pause the worker thread

    def __init__(self):
        QtCore.QObject.__init__(self)
        self.DialogMovementWindow = QtWidgets.QDialog()
        self.DialogRobotWindow = QtWidgets.QDialog()
        self.FMRRMainWindow = QtWidgets.QMainWindow()
    
        self.ROS = None
        self.ROS_active = False
        self.ROS_is_quitting = False
        self.ros_waiting_dialog = WaitingDialog(self.FMRRMainWindow)
        self.waiting_dialog_timer = QTimer()
        self.waiting_dialog_timer.timeout.connect(lambda: self.check_ros_status())
        self.waiting_dialog_timer.start(self._update_windows_period)

        self.udp_thread = QThread()
        self.server = UdpServer(port=5005)
        self.server.moveToThread(self.udp_thread)
        self.udp_thread.started.connect(self.server.start)
        self.server.message_received.connect(lambda d,a : self.udp_request_received(d))
        self.udp_thread.start()

        self.first_time = True
        self.Training_ON = False
        print("FMRR cell node started.")

    def check_ros_status(self):
        if not hasattr(self, 'ROS') or self.ROS is None:
            self.ros_waiting_dialog.show()
        else:
            self.ros_waiting_dialog.hide()

    def initializeRosProcesses(self):
        if hasattr(self, 'ROS') and self.ROS is not None:
            print("[MainProgram] ROS already initialized.")
            return 0
        # Initialize ROS client
        if self.first_time:
            self.ros_client = roslibpy.Ros(host='127.0.0.1', port=9090)
            self.ros_client.run()
        else:
            self.ros_client.connect()
        self.ROS = SyncRosManager(JOINT_NAMES, self.ros_client)
        if self.first_time:
            self.startMovementWindow()
            self.startRobotWindow()
        else:
            self.uiRobotWindow.reconnect_ROS_callbacks()
        self._update_robot_window_callback = partial(self.uiRobotWindow.updateRobotWindow, self.DialogRobotWindow)
        self._update_movement_window_callback = partial(self.uiMovementWindow.updateMovementWindow, self.DialogMovementWindow)
        self.comboBox_ResetFaults.currentIndexChanged.connect(self.ROS.reset_mode_changed)
        self.ros_waiting_dialog.hide()
        self.ROS_active = True
        self._update_windows_timer.timeout.connect(self._update_robot_window_callback)
        self._update_windows_timer.timeout.connect(self._update_movement_window_callback)
        self.on_fct_finished_server = roslibpy.Service(self.ros_client, "/rehab_gui/fct_finished", "std_srvs/Trigger")
        self.on_fct_finished_server.advertise(self.on_fct_worker_finished)
        self.on_fct_progress_server = roslibpy.Service(self.ros_client, "/rehab_gui/fct_progress", "tecnobody_msgs/MovementProgress")
        self.on_fct_progress_server.advertise(self.on_fct_worker_progress)
        self.first_time = False
        return True

    def udp_request_received(self, data):
        if data == b'STOP':
            print("[UdpServer] UDP STOP request received.")
            if self.ROS_active and not self.ROS_is_quitting:
                print("[UdpServer] Processing request to stop ROS processes.")
                self.stopRosProcesses()
        elif data == b'RUNNING':
            if not hasattr(self, 'ROS') or self.ROS is None:
                print("[UdpServer] Starting ROS processes...")
                self.initializeRosProcesses()
        else:
            print(f"[UdpServer] Unknown UDP packet received from UDP client: {data}")


    def stopRosProcesses(self):
        if self.ROS_is_quitting:
            print('Already stopping ROS objects, waiting for completion...')
            return 0
        
        print("Stopping ROS processes...")
        self.ROS_is_quitting = True
        self.ROS_active = False

        # Destroy ROS clients
        if hasattr(self, 'ROS') and self.ROS is not None:
            if not self.ROS.destroy():
                print('Error trying to destroy ROS class.')
                return -1
            self.uiRobotWindow.disconnect_ROS_callbacks()
            self.ros_client.close()
            
            # Disconnect GUI signals
            for signal in [
                self.comboBox_ResetFaults.currentIndexChanged,
            ]:
                try:
                    signal.disconnect()
                except (TypeError, RuntimeError):
                    pass

            try:
                self._update_windows_timer.timeout.disconnect(self._update_robot_window_callback)
                self._update_windows_timer.timeout.disconnect(self._update_movement_window_callback)
            except (TypeError, RuntimeError, AttributeError):
                print("Error disconnecting robot window updating timer.")
                pass

            del self.ROS
            gc.collect()

        self.ros_waiting_dialog.show()
        self.ROS_is_quitting = False
        print("[MainProgram] ROS processes stopped.")
        self.server.send_response(b"STOPPED")
        return 1

    def initializeVariables(self):
        # self.FIRST_TIME = True
        self.NumberExecMovements = 0
        self.TotalTrainingTime = 0
        self.ActualTrainingTime = 0
        self.execution_time_percentage = 0
        self.iPhase = 0
        self._counter_request_homing_procedure = 0 # counter to avoid multiple request of homing procedure

    def updateWindowTimerCallback(self):
        self._update_windows_timer = QTimer()                                           # creo l'oggetto
        self._update_windows_timer.timeout.connect( lambda: self.updateFMRRWindow() )                    # lo collego ad una callback
        self._update_windows_timer.start(self._update_windows_period)

    def updateTrainingTimer(self):
        self._update_TrainingTimer = QTimer()                                           # creo l'oggetto
        self._update_TrainingTimer.timeout.connect(lambda: self.update_TrainingParameters())
        self._update_TrainingTimer.start(self._update_TrainingTime)
        print("Update Training TImer Started (event loop in the main thread)")

    def updateFMRRWindow(self):
        self.lcdNumber_MovementCOUNT.display( self.NumberExecMovements ) 
        self.lcdNumberExerciseTotalTime.display( np.floor((self.TotalTrainingTime - self.ActualTrainingTime)/60) )
        if self.ROS_active:
            self.pushButton_StartMotors.setEnabled(not self.ROS.are_motors_on)
            self.pushButton_StopMotors.setEnabled(self.ROS.are_motors_on)
            self.pushButton_ResetFaults.setEnabled(self.ROS.manual_reset_faults)
            if self.ROS.is_in_fault_state:
                self.frame_FaultDetected.setStyleSheet("background-color: red; border-radius: 10px;")
            else:
                self.frame_FaultDetected.setStyleSheet("background-color: green; border-radius: 10px;")
        
    def definePaths(self):
        self.FMRR_Paths = dict() 
        current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current file's directory
        parent_directory = os.path.join(current_directory, '..')  # Step to the parent folder
        self.FMRR_Paths['Root'] = os.getcwd()
        self.FMRR_Paths['Protocols'] = parent_directory + '/Protocols'
        self.FMRR_Paths['Movements'] = parent_directory + '/Movements'  
        self.FMRR_Paths['Joint_Configuration'] = parent_directory + '/config'

    def startRobotWindow(self):
        self.uiRobotWindow = FMRR_Ui_RobotWindow()
        self.uiRobotWindow.ui_FMRRMainWindow = self  # type: ignore #Needed to be able to chenge MainWindow Widgets from Movement Window
        self.uiRobotWindow.setupUi_RobotWindow(self.DialogRobotWindow)
        self.uiRobotWindow.retranslateUi_RobotWindow(self.DialogRobotWindow)
        self.uiRobotWindow.DialogRobotWindow = self.DialogRobotWindow # type: ignore # Needed to be able to hide window from Movement Window
        self.uiRobotWindow.DialogFMRRMainWindow = self.FMRRMainWindow # type: ignore
        
    def startMovementWindow(self):
        self.uiMovementWindow = FMRR_Ui_MovementWindow()
        self.uiMovementWindow.ui_FMRRMainWindow = self  # type: ignore #Needed to be able to chenge MainWindow Widgets from Movement Window
        self.uiMovementWindow.setupUi_MovementWindow(self.DialogMovementWindow)
        self.uiMovementWindow.retranslateUi_MovementWindow(self.DialogMovementWindow)
        self.uiMovementWindow.DialogMovementWindow = self.DialogMovementWindow # type: ignore # Needed to be able to hide window from Movement Window
        self.uiMovementWindow.DialogFMRRMainWindow = self.FMRRMainWindow # type: ignore
        
    def clbk_BtnMoveRobot(self):
        if self.ROS_active:
            if self.ROS.are_motors_on:
                QMessageBox.warning(self.DialogRobotWindow, "Warning", "Motors are still active. Please turn them off before exiting this window!")
                return
            self.FMRRMainWindow.hide()
            self.DialogRobotWindow.show()
                
    def clbk_BtnLoadCreateMovement(self):
        self.FMRRMainWindow.hide()
        self.DialogMovementWindow.show()

    def clbk_BtnLoadCreateProtocol(self):
        filename = QtWidgets.QFileDialog.getOpenFileName(None, "Load Protocol", self.FMRR_Paths['Protocols'] , "*.yaml")
        if bool(filename[0]):
            # load data  
            self.ProtocolData = yaml.load(open(filename [0]), Loader=SafeLoader) # type: ignore
            # get values       
            self.Vmax = -99 # self.ProtocolData["V_max"] [0] [0]
            self.PhaseIsEnabled = self.ProtocolData["Phases"].get('PhaseIsEnabled')[0]
            self.NrEnabledPhases = sum( self.PhaseIsEnabled ) #Sistemare se non si usa
            self.TotalTrainingTime =  self.PhaseDuration * self.NrEnabledPhases
            self.Modalities = self.ProtocolData["Phases"].get('Modalities')[0]
            self.Percentage = self.ProtocolData["Phases"].get('Percentage')[0] 
            self.doubleSpinBox_SinglePhaseDuration.setValue(self.PhaseDuration)
            self.lcdNumberExerciseTotalTime.display( np.floor(self.TotalTrainingTime/60) )
            self.spinBox_MaxVel.setValue(self.Vmax)
            
            for iPhases in range(20):
                self.lcdNumberPhases[iPhases].setNumDigits(3) # type: ignore
                if iPhases==0:
                    self.Percentage[iPhases]=50
                iPhaseVel = int( float(self.Percentage[iPhases]) /100 * self.Vmax )
                self.lcdNumberPhases[iPhases].display( iPhaseVel ) # type: ignore
                self.spinBoxSpeedOvr[iPhases].setValue(self.Percentage[iPhases]) # type: ignore
                self.spinBoxSpeedOvr[self.iPhase].enabled = True
                    
            for iProgressBar in self.progressBarPhases:
                iProgressBar.setValue(0) # type: ignore
                
            self.pushButton_STARTtrainig.enablePushButton(1)

    def clbk_BtnCLOSEprogram(self):
        MyString = "Do you want to exit?"
        question = QMessageBox(QMessageBox.Question, "Exit Program", MyString, QMessageBox.Yes | QMessageBox.No)
        decision = question.exec_()
        if decision == QMessageBox.Yes:
              quit()

    def clbk_STARTtrainig(self):
        if self.ROS_active:
            self.Training_ON = True
            self.ActualTrainingTime = 0
            self.movement_completed = False
            self.iPhase = 0
            if not self.ROS.are_motors_on:
                self.ROS.turn_on_motors()
            if self.ROS.current_controller != self.ROS.trajectory_controller_name:
                    if not self.ROS.controller_and_op_mode_switch(8, self.ROS.trajectory_controller_name):
                        print(f"Failed to set position mode and switch from {self.ROS.current_controller} to {self.ROS.trajectory_controller_name}!")
                        return
            self.sendTrajectoryFCT()
            self.updateTrainingTimer()
            self.ModalityActualValue = self.Modalities[0]
            # print(f'------------------------------------------{self.spinBoxSpeedOvr[0].value()}')
            # self.startMovementFCT(self.spinBoxSpeedOvr[0].value())
            self.pushButton_PAUSEtrainig.enablePushButton(1)
            self.pushButton_STOPtrainig.enablePushButton(1)
    
    def clbk_PAUSEtrainig(self):
        if self.ROS_active:
            _translate = QtCore.QCoreApplication.translate
            current_speed_ovr = self.spinBoxSpeedOvr[self.iPhase].value()
            if self.pushButton_PAUSEtrainig.State:
                self.pushButton_PAUSEtrainig.State = 0    
                self.pushButton_PAUSEtrainig.setText( _translate("FMRRMainWindow", "RESUME training") )
                self.trigger_pause.emit(True)
                self.ROS.trigger_soft_stop(start_value=current_speed_ovr, steps=10, target='speed_ovr')
                print('The robot movement was successfully stopped')
            else:
                self.pushButton_PAUSEtrainig.State = 1    
                self.pushButton_PAUSEtrainig.setText( _translate("FMRRMainWindow", "PAUSE training") )
                self.ROS.trigger_soft_start(target_value=current_speed_ovr, steps=10, target='speed_ovr')
                self.trigger_pause.emit(False)
                print('The robot movement was successfully resumed')

    def clbk_STOPtrainig(self):   
        if self.ROS_active:
            self.Training_ON = False
            client = roslibpy.Service(self.ros_client, "/tecnobody_workbench_utils/stop_movement", "std_srvs/Trigger")
            req = roslibpy.ServiceRequest()
            client.call(req)
            for iProgBar in self.progressBarPhases:
                iProgBar = 0
            self.pushButton_PAUSEtrainig.enablePushButton(0)
            self.pushButton_STOPtrainig.enablePushButton(0)
            self._update_TrainingTimer.stop()
            self.ROS.publish_plc_command(['PLC_node/manual_mode'], [0])
            self.ROS.turn_off_motors()            
            
    def clbk_StartMotors(self):
        if self.ROS_active:
            if self.ROS.plc_states['manual_switch_pressed']:
                # set manual mode during movement
                self.ROS.publish_plc_command(['PLC_node/manual_mode'], [1])
                self.ROS.turn_on_motors()
            else:
                QMessageBox.warning(self.DialogRobotWindow, "Warning",
                                    "Please hold the manual mode emergency button before moving the robot!")

    def clbk_StopMotors(self):
        if self.ROS_active:
            # set automatic mode after movement
            self.ROS.publish_plc_command(['PLC_node/manual_mode'], [0])
            self.ROS.turn_off_motors()

    def clbk_BtnResetFaults(self):
        if self.ROS_active:
            self.ROS.reset_fault()

    def update_TrainingParameters(self):
        if self.iPhase <= 19:
            self.progressBarPhases[self.iPhase].setValue(self.execution_time_percentage)
            if self.iPhase > 0:
                self.progressBarPhases[self.iPhase - 1].setValue(100) # type: ignore
            if self.movement_completed:                
                self.ModalityActualValue = self.Modalities[self.iPhase] # change here the modality 
                self.NumberExecMovements += 1
                print(f'Number of movements: {self.NumberExecMovements} - Ovr: {self.spinBoxSpeedOvr[self.iPhase].value()}')
                # self.startMovementFCT(self.spinBoxSpeedOvr[self.iPhase].value())
                self.spinBoxSpeedOvr[self.iPhase].enabled = False
        else:            
            self.progressBarPhases[19].setValue(100)
            self.clbk_STOPtrainig()
        
    def on_fct_worker_finished(self, request, response):
        if self.Training_ON:
            self.iPhase += 1
            self.movement_completed = True
        response['success'] = True
        return True
    
    def on_fct_worker_progress(self, request, response):
        self.execution_time_percentage = int(request['progress'])  # Get the progress percentage from the worker
        response['repetition_ovrs'] =  [sp.value() for sp in self.spinBoxSpeedOvr]
        return True

    def clbk_spinBox_MaxVel(self):
        speed_ovr_Value = self.spinBox_MaxVel.value()
        speed_ovr_msg = roslibpy.Message({'data': int(speed_ovr_Value)})
        if self.ROS_active:
            self.ROS.pub_speed_ovr.publish(speed_ovr_msg)

    def setupUi_MainWindow(self):
        Ui_FMRRMainWindow.setupUi(self, self.FMRRMainWindow)
    
    def retranslateUi_MainWindow(self, app): 
        #   GENERAL
        self.progressBarPhases = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19] 
        self.spinBoxSpeedOvr =  [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
        self.lcdNumberPhases =  [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
        self.comboBoxPhases =  [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
        for widget in app.allWidgets():
            NameStr = widget.objectName()
            lenstr= len(NameStr)
            if NameStr[lenstr-7:lenstr-2] == 'Phase':
                WidgetItem = int(NameStr[-2:])-1            
                if isinstance(widget, QtWidgets.QProgressBar):
                    self.progressBarPhases[WidgetItem] = widget
                    self.progressBarPhases[WidgetItem].setValue(0) # type: ignore
                if isinstance(widget, QtWidgets.QSpinBox):
                    self.spinBoxSpeedOvr[WidgetItem] = widget
                if isinstance(widget, QtWidgets.QLCDNumber):
                    self.lcdNumberPhases[WidgetItem] = widget
                if isinstance(widget, QtWidgets.QComboBox):
                    self.comboBoxPhases[WidgetItem] = widget
        for WidgetItem in range(0,20):
            self.lcdNumberPhases[WidgetItem].setSegmentStyle(QtWidgets.QLCDNumber.Flat) # type: ignore            
        #   ENABLE Buttons      
        self.pushButton_LoadCreateMovement.enablePushButton(1)
        self.pushButton_MoveRobot.enablePushButton(1)
        self.pushButton_CLOSEprogram.enablePushButton(1)
        self.pushButton_StartMotors.enablePushButton(1)
        self.pushButton_ResetFaults.enablePushButton(1)
        #   DISABLE Buttons
        self.pushButton_STARTtrainig.enablePushButton(0)
        self.pushButton_PAUSEtrainig.enablePushButton(0)
        self.pushButton_STOPtrainig.enablePushButton(0)
        self.pushButton_SaveProtocol.enablePushButton(0)
        self.pushButton_LoadCreateProtocol.enablePushButton(0)
        self.pushButton_DATAacquisition.enablePushButton(0)
        self.pushButton_StopMotors.enablePushButton(0)
        #   CALLBACKS 
        #    Buttons
        self.pushButton_LoadCreateMovement.clicked.connect(self.clbk_BtnLoadCreateMovement)
        self.pushButton_MoveRobot.clicked.connect(self.clbk_BtnMoveRobot)
        self.pushButton_LoadCreateProtocol.clicked.connect(self.clbk_BtnLoadCreateProtocol)
        self.pushButton_CLOSEprogram.clicked.connect(self.clbk_BtnCLOSEprogram)
        self.pushButton_STARTtrainig.clicked.connect(self.clbk_STARTtrainig)        
        self.pushButton_PAUSEtrainig.clicked.connect(self.clbk_PAUSEtrainig)
        self.pushButton_STOPtrainig.clicked.connect(self.clbk_STOPtrainig)
        self.pushButton_StartMotors.clicked.connect(self.clbk_StartMotors)
        self.pushButton_StopMotors.clicked.connect(self.clbk_StopMotors)
        self.pushButton_ResetFaults.clicked.connect(self.clbk_BtnResetFaults)
        #    Spinbox
        self.spinBox_MaxVel.valueChanged.connect(self.clbk_spinBox_MaxVel)      
        #    CAHNGES
        self.lcdNumberExerciseTotalTime.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_MovementCOUNT.setSegmentStyle(QtWidgets.QLCDNumber.Flat)

    def startMovementFCT(self, default_speed_override=100):
        self.movement_completed = False
        client = roslibpy.Service(self.ros_client, "/tecnobody_workbench_utils/start_movement", "tecnobody_msgs/StartMovement")
        req = roslibpy.ServiceRequest({
            'speed_ovr': default_speed_override
        })
        client.call(req)  # Trigger the worker to start the movement
        
    def clbk_ApproachPoint(self, target_point, end_time):
        if self.ROS_active:
            try:
                client = roslibpy.Service(self.ros_client, "/tecnobody_workbench_utils/set_trajectory",
                                          "tecnobody_msgs/SetTrajectory")
                points = []
                times = []
                points.append(self.ROS.RobotJointPosition)
                times.append(0.0)
                points.append(target_point)
                times.append(end_time)
                req = roslibpy.ServiceRequest({
                    'cartesian_positions': [
                        {'point': points[idx], 'time_from_start': times[idx]} for idx in range(len(times))],
                        'repetition_ovrs': [50]
                })
                response = client.call(req)
                if not response['success']:
                    print("Failed to approach point!")
            except Exception as e:
                print(f"Exception during go home service call: {e}")

    def sendTrajectoryFCT(self):       
        if self.ROS_active:
            TrjYamlData = self.uiMovementWindow.TrjYamlData
            self.CartesianPositions = TrjYamlData.get("cart_trj3").get("cart_positions")
            self.TimeFromStart = TrjYamlData.get("cart_trj3").get("time_from_start")
            client = roslibpy.Service(self.ros_client, "/tecnobody_workbench_utils/set_trajectory", "tecnobody_msgs/SetTrajectory")
            req = roslibpy.ServiceRequest({
                'cartesian_positions': [
                    {'point': self.CartesianPositions[idx], 'time_from_start' : self.TimeFromStart[idx][0]} for idx,val in enumerate(self.TimeFromStart)], 
                    'repetition_ovrs': [ sp.value() for sp in self.spinBoxSpeedOvr]
            })
            client.call(req)  # type: ignore

        
def main(args=None):
    QPushButton.enablePushButton = MC_Tools.enablePushButton #Adding a new method defined in MC_Tools file
    app = QtWidgets.QApplication(sys.argv)
    ui = MainProgram()
    ui.setupUi_MainWindow()
    ui.retranslateUi_MainWindow(app)
    ui.definePaths()
    ui.updateWindowTimerCallback()
    ui.initializeVariables()
    ui.FMRRMainWindow.show()

    signal.signal(signal.SIGINT, lambda *args: app.quit())

    timer = QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None) 

    # ---------------------------
    # Event loop Qt
    # ---------------------------
    exit_code = app.exec_()

    if hasattr(ui, "ros_checker"):
        ui.ros_checker.stop()

    if hasattr(ui, "worker_thread"):
        ui.worker_thread.quit()
        ui.worker_thread.wait()

    sys.exit(exit_code)

if __name__ == "__main__":
    main()