import os
import sys
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QTimer, QThread, pyqtSignal

# mathematics
import numpy as np

#MC Classes/methods
from .MovementProgram import FMRR_Ui_MovementWindow 
from .RobotProgram import FMRR_Ui_RobotWindow 
from .FMRRMainWindow import Ui_FMRRMainWindow # import from file FMRRMainWindow the class Ui_FMRRMainWindow  
from . import MC_Tools
import yaml
from yaml.loader import SafeLoader

#ROS
import rclpy
from ros2node.api import get_node_names
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus

from .async_ros_events import ASyncRosManager
from .sync_ros_events import SyncRosManager

DEBUG = False

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
    trigger_worker = pyqtSignal(int) 
    trigger_pause = pyqtSignal(bool) # signal to pause the worker thread

    def __init__(self):
        QtCore.QObject.__init__(self)
        self._ros_node = Node('FMRR_cell_node')
        
        self.DialogMovementWindow = QtWidgets.QDialog()
        self.DialogRobotWindow = QtWidgets.QDialog()
        self.FMRRMainWindow = QtWidgets.QMainWindow() # FMRRMainWindow is an instance of the class QtWidgets.QMainWindow. Not to be confuse with the namof the file FMRRMainWindow.py
        self._ros_node.get_logger().info("FMRR cell node started.")

        # Set up Movement Worker
        self._ros_network_status_timer = QTimer()                                           # creo l'oggetto
        # self._ros_network_status_timer.timeout.connect( lambda: self.updateROSNetworkStatus() )        
        self._ros_network_status_timer.start(self._update_windows_period)
    
        self.ROS = None
        self.MovementWorker = None

    def initializeRosProcesses(self):
        self.worker_thread = QThread()
        self.ROS = SyncRosManager(self._ros_node, JOINT_NAMES)
        self.MovementWorker = ASyncRosManager(self._ros_node, JOINT_NAMES, self.ROS.trajectory_controller_name)
        self.MovementWorker.moveToThread(self.worker_thread)

    # def stopRosProcesses(self):
    #     self.worker_thread = QThread()
    #     self.ROS = None

    def initializeVariables(self):
        # self.FIRST_TIME = True
        self.NumberExecMovements = 0
        self.TotalTrainingTime = 0
        self.ActualTrainingTime = 0
        self.execution_time_percentage = 0
        self.iPhase = 0
        self._counter_request_homing_procedure = 0 # counter to avoid multiple request of homing procedure
        self.trigger_worker.connect(self.MovementWorker.fct().startMovement)# type: ignore
        self.trigger_pause.connect(self.MovementWorker.fct().is_paused) # type: ignore
        self.MovementWorker.fct().finished.connect(self.on_fct_worker_finished)
        self.MovementWorker.fct().progress.connect(self.on_fct_worker_progress)
        #self.MovementWorker.moo().finished.connect(self.on_moo_worker_finished)
        
        self.worker_thread.start()
        self.MovementWorker.fct().clear()

    # def updateROSNetworkStatus(self):
    #     available_nodes = [ full_name for _, _, full_name in self._ros_node.get_node_names(node=self, include_hidden_nodes=False) ]
    #     if self.ROS is not None and 'state_controlle' is not in available_nodes:
    #         pass
    #     elif self.ROS is not None and 'state_controlle' is not in available_nodes:
        
    def updateWindowTimerCallback(self):
        self._update_windows_timer = QTimer()                                           # creo l'oggetto
        self._update_windows_timer.timeout.connect( lambda: self.updateFMRRWindow() )                    # lo collego ad una callback
        self._update_windows_timer.timeout.connect( lambda: self.uiRobotWindow.updateRobotWindow( self.DialogRobotWindow ) ) # lo collego ad una seconda  callback
        self._update_windows_timer.start(self._update_windows_period)

    def updateTrainingTimer(self):
        self._update_TrainingTimer = QTimer()                                           # creo l'oggetto
        self._update_TrainingTimer.timeout.connect(lambda: self.update_TrainingParameters())
        self._update_TrainingTimer.start(self._update_TrainingTime)
        self._ros_node.get_logger().info("Update Training TImer Started (event loop in the main thread)")

    def updateFMRRWindow(self):
        self.lcdNumber_MovementCOUNT.display( self.NumberExecMovements ) 
        self.lcdNumberExerciseTotalTime.display( np.floor((self.TotalTrainingTime - self.ActualTrainingTime)/60) )
        self.pushButton_StartMotors.enablePushButton(not self.ROS.are_motors_on)
        self.pushButton_StopMotors.enablePushButton(self.ROS.are_motors_on)
        self.pushButton_ResetFaults.setEnabled(self.ROS.manual_reset_faults)
        if self.ROS.is_in_fault_state:
            self.frame_FaultDetected.setStyleSheet("background-color: red; border-radius: 10px;")
        else:
            self.frame_FaultDetected.setStyleSheet("background-color: green; border-radius: 10px;")
    
    def definePaths(self):
        self.FMRR_Paths = dict() 
        current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current file's directory
        parent_directory = '/home/fit4med/fit4med_ws/src/Fit4Med/rehab_gui'  # Step to the parent folder
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
        self.ActualTrainingTime = 0
        self.movement_completed = False
        self.iPhase = 0
        self._home_goal_sent = False
        if self.ROS.current_controller != self.ROS.trajectory_controller_name:
                if not self.ROS.controller_and_op_mode_switch(8, self.ROS.trajectory_controller_name):
                    self._ros_node.get_logger().error(f"Failed to set position mode and switch from {self.ROS.current_controller} to {self.ROS.trajectory_controller_name}!")
                    return
        self.sendTrajectoryFCT()
        self.updateTrainingTimer()
        self.ModalityActualValue = self.Modalities[0]
        print(f'------------------------------------------{self.spinBoxSpeedOvr[0].value()}')
        self.startMovementFCT(self.spinBoxSpeedOvr[0].value()) # type: ignore
        self.pushButton_PAUSEtrainig.enablePushButton(1)
        self.pushButton_STOPtrainig.enablePushButton(1)
    
    def clbk_PAUSEtrainig(self):
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
        self.MovementWorker.fct().stop() 
        for iProgBar in self.progressBarPhases:
            iProgBar = 0
        self.pushButton_PAUSEtrainig.enablePushButton(0)
        self.pushButton_STOPtrainig.enablePushButton(0)
        self._update_TrainingTimer.stop()
    
    def clbk_StartMotors(self):
        self.pushButton_StartMotors.enablePushButton(0)
        self.pushButton_StopMotors.enablePushButton(1)
        self.ROS.turn_on_motors()

    def clbk_StopMotors(self):
        self.uiRobotWindow.pushButton_StartMotors.enablePushButton(1)
        self.uiRobotWindow.pushButton_StopMotors.enablePushButton(0)
        self.ROS.turn_off_motors()

    def clbk_BtnResetFaults(self):
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
                self.startMovementFCT(self.spinBoxSpeedOvr[self.iPhase].value())
                self.spinBoxSpeedOvr[self.iPhase].enabled = False
        else:            
            self.progressBarPhases[19].setValue(100)
            self.clbk_STOPtrainig()
        
    def on_fct_worker_finished(self):
        self.iPhase += 1
        self.movement_completed = True

    # def on_moo_worker_finished(self):
    #     self.ROS.moo_applied_success = True
    
    def on_fct_worker_progress(self,value):
        self.execution_time_percentage = int(value)  # Get the progress percentage from the worker

    def clbk_spinBox_MaxVel(self):
        speed_ovr_Value = self.spinBox_MaxVel.value()
        speed_ovr_msg = Int16()
        speed_ovr_msg.data = speed_ovr_Value
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
        self.comboBox_ResetFaults.currentIndexChanged.connect(self.ROS.reset_mode_changed)
        #    CAHNGES
        self.lcdNumberExerciseTotalTime.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_MovementCOUNT.setSegmentStyle(QtWidgets.QLCDNumber.Flat)

    def startMovementFCT(self, default_speed_override=100):
        self.movement_completed = False
        self._home_status = GoalStatus.STATUS_UNKNOWN
        self._home_goal_sent = False
        self.trigger_worker.emit(default_speed_override)  # Trigger the worker to start the movement
        
    def clbk_ApproachPoint(self, point):
        self._ros_node.get_logger().info("Approaching desired position...")
        approach_point_goal = FollowJointTrajectory.Goal()
        approach_point_goal.trajectory.joint_names = JOINT_NAMES

        init_point = JointTrajectoryPoint()
        init_point.positions = self.ROS.RobotJointPosition
        init_point.velocities = [0.0] * len(JOINT_NAMES)
        init_point.accelerations = [0.0] * len(JOINT_NAMES)
        init_point.effort = []
        init_point.time_from_start = Duration(sec=0, nanosec=0)
        approach_point_goal.trajectory.points.append(init_point)  # type: ignore

        final_point = JointTrajectoryPoint()
        final_point.positions = point
        final_point.velocities = [0.0] * len(JOINT_NAMES)
        final_point.accelerations = [0.0] * len(JOINT_NAMES)
        final_point.effort = []
        final_point.time_from_start = Duration(sec=3, nanosec=0)
        approach_point_goal.trajectory.points.append(final_point) # type: ignore
        
        approach_point_goal.trajectory.header.stamp = self._ros_node.get_clock().now().to_msg()
        home_future = self.MovementWorker.fct().client.send_goal_async(approach_point_goal)
        #home_future.add_done_callback(self.home_cancel_callback)
        return True

    def sendTrajectoryFCT(self):       
        TrjYamlData = self.uiMovementWindow.TrjYamlData
        self.CartesianPositions = TrjYamlData.get("cart_trj3").get("cart_positions")
        self.TimeFromStart = TrjYamlData.get("cart_trj3").get("time_from_start")
        self.MovementWorker.fct().set(self.CartesianPositions, self.TimeFromStart)  # type: ignore    

        
def main(args=None):
    rclpy.init(args=args)
    QPushButton.enablePushButton = MC_Tools.enablePushButton #Adding a new method defined in MC_Tools file
    app = QtWidgets.QApplication(sys.argv)
    ui = MainProgram()
    ui.setupUi_MainWindow()
    ui.initializeRosProcesses()
    ui.retranslateUi_MainWindow(app)
    ui.definePaths()
    ui.startRobotWindow()
    ui.startMovementWindow()
    ui.updateWindowTimerCallback()
    ui.initializeVariables()
    ui.FMRRMainWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
    rclpy.shutdown()
