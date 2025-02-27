import os
import sys
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QTimer

# mathematics
import numpy as np
from sympy import true

import rclpy.action
import rclpy.duration
import rclpy.time

#MC Classes/methods
from .MovementProgram import FMRR_Ui_MovementWindow 
from .FMRRMainWindow import Ui_FMRRMainWindow # import from file FMRRMainWindow the class Ui_FMRRMainWindow  
from . import MC_Tools
import yaml
from yaml.loader import SafeLoader

#ROS
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint  

from sensor_msgs.msg import JointState # joints positions, velocities and efforts
from std_msgs.msg import Int16 
from geometry_msgs.msg import Point

import debugpy

DEBUG = False

JOINT_NAMES = [
    'joint_1',
    'joint_2',
    'joint_3'
]
     

class MainProgram(Node, Ui_FMRRMainWindow): 
    _update_period = 500
    _update_TrainingTime = 500
    _ros_period = 1
    _toolPosCovFact = 100 # to display coordinatates in centimeters (are given in meters in the yaml files) (used in MovementWindow to display data)
    _jointPosConvFact = 180/np.pi # conversion from radiants to degrees (used in MovementWindow to display data)

    def __init__(self):
        Node.__init__(self, 'FMRR_cell_node') # type: ignore

        if DEBUG:
            # Start debugpy listener to enable attaching debugger
            debugpy.listen(("0.0.0.0", 5678))
            print("Waiting for debugger to attach...")
            debugpy.wait_for_client()  # Pause execution until debugger attaches
            print("Debugger attached. Starting node.")

        self.DialogMovementWindow = QtWidgets.QDialog()
        self.FMRRMainWindow = QtWidgets.QMainWindow() # FMRRMainWindow is an instance of the class QtWidgets.QMainWindow. Not to be confuse with the namof the file FMRRMainWindow.py
        self.get_logger().info("FMRR cell node started.")
        self._joint_state = None

    def initializeVariables(self):
        self.AnswerPauseService = False
        self.trainingOn = False
        self.FIRST_TIME = True
        self.JogOn = False

         
    def updateTimer(self):
        self._update_timer = QTimer()                                           # creo l'oggetto
        self._update_timer.timeout.connect( 
        lambda: self.uiMovementWindow.updateMovementWindow( self.DialogMovementWindow ) ) # lo collego ad una seconda  callback
        self._update_timer.start(self._update_period)

        
    def updateTrainingTimer(self):
        self._update_TrainingTimer = QTimer()                                           # creo l'oggetto
        self._update_TrainingTimer.timeout.connect(lambda: self.update_TrainingParameters())
        self._update_timer.timeout.connect( lambda: self.updateFMRRWindow() )                    # lo collego ad una callback
        self._update_TrainingTimer.start(self._update_TrainingTime)
        self.get_logger().info("Thread started")


    def rosTimer(self):
        self._ros_timer = QTimer()
        self._ros_timer.timeout.connect(self.spin_ros_once)
        self._ros_timer.start(self._ros_period)


    def spin_ros_once(self):
        rclpy.spin_once(self, timeout_sec=0)

        
    def updateFMRRWindow(self):
        self.lcdNumber_MovementCOUNT.display( self.NumberExecMovements ) 
        self.lcdNumberExerciseTotalTime.display( np.floor((self.TotalTrainingTime - self.ActualTrainingTime)/60) )

    
    def definePaths(self):
        self.FMRR_Paths = dict() 
        self.FMRR_Paths['Root'] = os.getcwd()
        self.FMRR_Paths['Protocols'] = '/home/adriano/projects/ros2_ws/src/Fit4Med/rehab_gui/config'
        self.FMRR_Paths['Movements'] = '/home/adriano/projects/ros2_ws/src/Fit4Med/rehab_gui/config'  
        self.FMRR_Paths['Joint_Configuration'] = 'config'

        
    def startMovementWindow(self):
        self.uiMovementWindow = FMRR_Ui_MovementWindow()
        self.uiMovementWindow.ui_FMRRMainWindow = self  # type: ignore #Needed to be able to chenge MainWindow Widgets from Movement Window
        self.uiMovementWindow.setupUi_MovementWindow(self.DialogMovementWindow)
        self.uiMovementWindow.retranslateUi_MovementWindow(self.DialogMovementWindow)
        self.uiMovementWindow.DialogMovementWindow = self.DialogMovementWindow # type: ignore # Needed to be able to hide window from Movement Window
        self.uiMovementWindow.DialogFMRRMainWindow = self.FMRRMainWindow # type: ignore

        
    def clbk_BtnLoadCreateMovement(self):
        self.FMRRMainWindow.hide()
        self.DialogMovementWindow.show()

        
    def clbk_BtnLoadCreateProtocol(self):
        filename = QtWidgets.QFileDialog.getOpenFileName(None, "Load Protocol", self.FMRR_Paths['Protocols'] , "*.yaml")
        if bool(filename[0]):
           # load data  
            self.ProtocolData = yaml.load(open(filename [0]), Loader=SafeLoader) # type: ignore
    #      # get values       
            self.PhaseDuration = self.ProtocolData["PhaseDuration"] [0] [0]
            self.Vmax = self.ProtocolData["V_max"] [0] [0]
            self.PhaseIsEnabled = self.ProtocolData["Phases"].get('PhaseIsEnabled')[0]
            self.NrEnabledPhases = sum( self.PhaseIsEnabled ) #Sistemare se non si usa
            self.TotalTrainingTime =  self.PhaseDuration * self.NrEnabledPhases
            self.Modalities = self.ProtocolData["Phases"].get('Modalities')[0]
            self.Percentage = self.ProtocolData["Phases"].get('Percentage')[0]
    
    #     # set values/properties        
            self.spinBox_SinglePhaseDuration.setValue(self.PhaseDuration)
            self.lcdNumberExerciseTotalTime.display( np.floor(self.TotalTrainingTime/60) )
            self.spinBox_MaxVel.setValue(self.Vmax)
            
            for iPhases in range(20):
                self.lcdNumberPhases[iPhases].setNumDigits(3) # type: ignore
                iPhaseVel = int( float(self.Percentage[iPhases]) /100 * self.Vmax )
                self.lcdNumberPhases[iPhases].display( iPhaseVel ) # type: ignore
                self.spinBoxPhases[iPhases].setValue(self.Percentage[iPhases]) # type: ignore
                    
            for iProgressBar in self.progressBarPhases:
                iProgressBar.setValue(0) # type: ignore
                
            self.pushButton_STARTtrainig.enablePushButton(1)

        
    def clbk_BtnCLOSEprogram(self):
        MyString = "Do you want to exit?"
        question = QMessageBox(QMessageBox.Question, "Question", MyString, QMessageBox.Yes | QMessageBox.No)
        decision = question.exec_()
        if decision == QMessageBox.Yes:
#              app.activeWindow().close()
              quit()
    

    def clbk_STARTtrainig(self):
        self.trainingOn = True        
        self.ActualTrainingTime = 0
        self.iPhase = 0
        self.NumberExecMovements = 0
        # self.AnswerPauseService = self.pauseService(False)
        self.sendTrajectoryFCT()
        self.updateTrainingTimer()
        self.OverrideActualValue = self.Percentage[0]
        self.ModalityActualValue = self.Modalities[0]
        self.KeepOnMovingBool = True
#
        # print(type(self.pub_speed_ovr))   
        self.startMovementFCT()        
#        
        self.pushButton_PAUSEtrainig.enablePushButton(1)
        self.pushButton_STOPtrainig.enablePushButton(1)
        # print('End of StartTraining.')
    
    
    def clbk_PAUSEtrainig(self):
        _translate = QtCore.QCoreApplication.translate
        if self.pushButton_PAUSEtrainig.State:
            self.pushButton_PAUSEtrainig.State = 0    
            self.pushButton_PAUSEtrainig.setText( _translate("FMRRMainWindow", "RESUME training") )
            self.AnswerPauseService = self.pauseService(True) # type: ignore
            self.KeepOnMovingBool = False
            print( 'The robot movement was successfully stopped: %s' % self.AnswerPauseService )
            
        else:
            self.pushButton_PAUSEtrainig.State = 1    
            self.pushButton_PAUSEtrainig.setText( _translate("FMRRMainWindow", "PAUSE training") )
            self.AnswerPauseService = self.pauseService(False) # type: ignore
            self.KeepOnMovingBool = True
            print( 'The robot movement was successfully resumed: %s' % self.AnswerPauseService )
    

    def clbk_STOPtrainig(self):
        self.trainingOn = False        
        self.KeepOnMovingBool = True
        self.stopProtocolFCT() 
        for iProgBar in self.progressBarPhases:
            iProgBar = 0
        self.pushButton_PAUSEtrainig.enablePushButton(0)
        self.pushButton_STOPtrainig.enablePushButton(0)
        self._update_TrainingTimer.stop()
    
              
    def update_TrainingParameters(self):
        if self.trainingOn:
            if self.KeepOnMovingBool:
                self.ActualTrainingTime += 1 * int(self.OverrideActualValue)/100 # +1 second infact _update_TrainingTime = 1000, frequency of the training timer
            # print( 'Actual training time: %s' % self.ActualTrainingTime )
            # print( 'Total remaining training time: %s' % (self.TotalTrainingTime - self.ActualTrainingTime) )
            if  self.ActualTrainingTime < self.TotalTrainingTime:        
                iPhase = int ( np.floor( self.ActualTrainingTime / self.PhaseDuration) )
                _ActualPhaseTime =  self.ActualTrainingTime -  iPhase *self.PhaseDuration
                _ActualPhasePercTime = int( float(_ActualPhaseTime / self.PhaseDuration) *100 )
                if iPhase <= 19:
                    self.progressBarPhases[iPhase].setValue(_ActualPhasePercTime) # type: ignore
                    if iPhase > 0:
                        self.progressBarPhases[iPhase - 1].setValue(100) # type: ignore
                        self.OverrideActualValue = self.Percentage[iPhase]
                    if self.iPhase != iPhase:
                        if self.goHome():
                            self.iPhase = iPhase
                            self.OverrideActualValue = self.Percentage[iPhase] # Change here the topic value
                            self.ModalityActualValue = self.Modalities[iPhase] # change here the modality 
                            self.NumberExecMovements += 1
                            print('Number of movements: %d' %self.NumberExecMovements)
                            self.startMovementFCT()
            else:            
                self.progressBarPhases[19].setValue(100) # type: ignore
                self.clbk_STOPtrainig()
        else:
            return
    

    def SendNewMovementGoal(self):
        if self.KeepOnMovingBool: # and self._clientFollowCartTraj.wait_for_result():
            print('sendNewMovementGoal ok.')
            self.startMovementFCT()
    

    def clbk_spinBox_MaxVel(self):
        speed_ovr_Value = self.spinBox_MaxVel.value()
        speed_ovr_msg = Int16()
        speed_ovr_msg.data = speed_ovr_Value
        self.pub_speed_ovr.publish(speed_ovr_msg)
           
              
######################## MAIN FUNCTIONS ###########################
########
######## setupUi_MainWindow and retranslateUi_MainWindow start the FMRR windox and comnnects callbacks to widgets               
########
######## start_ROS_functions: start ROS node and subscribers                
########
###################################################################

    def setupUi_MainWindow(self):
        Ui_FMRRMainWindow.setupUi(self, self.FMRRMainWindow)
    
    
### My chnages in FMRR MainWindow
    def retranslateUi_MainWindow(self, app): 

#   GENERAL
        self.progressBarPhases = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19] 
        self.spinBoxPhases =  [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19]
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
                    self.spinBoxPhases[WidgetItem] = widget
                if isinstance(widget, QtWidgets.QLCDNumber):
                    self.lcdNumberPhases[WidgetItem] = widget
                if isinstance(widget, QtWidgets.QComboBox):
                    self.comboBoxPhases[WidgetItem] = widget
                    
        for WidgetItem in range(0,20):
            self.lcdNumberPhases[WidgetItem].setSegmentStyle(QtWidgets.QLCDNumber.Flat) # type: ignore
                    
#   ENABLE Buttons      
        self.pushButton_LoadCreateMovement.enablePushButton(1)
        self.pushButton_CLOSEprogram.enablePushButton(1)

#   DISABLE Buttons
        self.pushButton_STARTtrainig.enablePushButton(0)
        self.pushButton_PAUSEtrainig.enablePushButton(0)
        self.pushButton_STOPtrainig.enablePushButton(0)
        self.pushButton_SaveProtocol.enablePushButton(0)
        self.pushButton_LoadCreateProtocol.enablePushButton(0)
        self.pushButton_DATAacquisition.enablePushButton(0)

#   CALLBACKS 
#    Buttons
        self.pushButton_LoadCreateMovement.clicked.connect(self.clbk_BtnLoadCreateMovement)
        self.pushButton_LoadCreateProtocol.clicked.connect(self.clbk_BtnLoadCreateProtocol)
        self.pushButton_CLOSEprogram.clicked.connect(self.clbk_BtnCLOSEprogram)
        self.pushButton_STARTtrainig.clicked.connect(self.clbk_STARTtrainig)        
        self.pushButton_PAUSEtrainig.clicked.connect(self.clbk_PAUSEtrainig)
        self.pushButton_STOPtrainig.clicked.connect(self.clbk_STOPtrainig)
#    Spinbox
        self.spinBox_MaxVel.valueChanged.connect(self.clbk_spinBox_MaxVel)      
#        self.pushButton_LoadCreateMovement.setStyleSheet("Background: #cce4f7")
#        self.pushButton_LoadCreateMovement.setStyleSheet("Background: light blue")
#        self.pushButton_LoadCreateMovement.setStyleSheet("Background:#0c0")
#        self.pushButton_CLOSEprogram.setCheckable(True)

#    CAHNGES
#        self.lcdNumberExerciseTotalTime.setDigitCount(3)
        self.lcdNumberExerciseTotalTime.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.lcdNumber_MovementCOUNT.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
         
#                
##############################      ROS     ###################################

    def start_ROS_functions(self):
    #   subscribers        
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.getJointState, 1)
        self.tool_subscriber = self.create_subscription(JointState, 'joint_states', self.getToolPosition, 1)
        # rospy.Subscriber( 'speed_ovr', Int16, self.getSpeedOveride )

    #   publishers        
        self.pub_speed_ovr = self.create_publisher(Int16, '/speed_ovr', 10)
        self.pub_start_point = self.create_publisher(Point, '/start_point', 10)
        self.pub_end_point = self.create_publisher(Point, '/end_point', 10)

    #   action client
        self._clientFollowCartTraj = ActionClient(self, FollowJointTrajectory, '/scaled_trajectory_controller/follow_joint_trajectory')
     

    def startPauseService(self):
        self.get_logger().info("Pause service is not implemented yet.")
        # rospy.wait_for_service('pause')
        # try:
        #     self.pauseService = rospy.ServiceProxy('pause', SetBool)
        # except rospy.ROSException as e:
        #     print ("The pause service is not ready: %s"%e)


    def FMRR_clientFollowCartTraj(self):
        # Wait for action server to be ready
        # timeout = Duration(seconds=5, nanoseconds=0)
        if not self._clientFollowCartTraj.wait_for_server(5):
            self.get_logger().error("Could not reach controller action server.")
            rclpy.shutdown()
            sys.exit(1)
        self.clearFCT()     

    
    def add_pointFCT(self, positions, time, **kwargs):
        point=JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.effort = []
        point.time_from_start = time
        self._goalFCT.trajectory.points.append(point) # type: ignore
 

    def startMovementFCT(self):
        if self.FIRST_TIME:
            self.OverrideActualValue = 50
            self.FIRST_TIME = False
        self._goalFCT.trajectory.header.stamp = self.get_clock().now().to_msg()
        # sleep_duration = rclpy.duration.Duration(seconds=0, nanoseconds=int(5 * 1e9))
        # self.get_clock().sleep_for(sleep_duration)
        self._send_goal_future = self._clientFollowCartTraj.send_goal_async(self._goalFCT)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        speed_ovr_msg = Int16()
        speed_ovr_msg.data = int(self.OverrideActualValue)
        self.pub_speed_ovr.publish(speed_ovr_msg) 
        print('Speed PCT: %d' %speed_ovr_msg.data)


    def goHome(self):
        home_from_yaml = self._goalFCT.trajectory.points[0].positions.tolist() # type: ignore
        tolerance = 1e-4
        if all(abs(a - b) < tolerance for a, b in zip(self.RobotJointPosition, home_from_yaml)):
            self.KeepOnMovingBool = True
            return True
        else:
            self.KeepOnMovingBool = False

            home_goal = FollowJointTrajectory.Goal()
            home_goal.trajectory.joint_names = JOINT_NAMES

            home_init_point = JointTrajectoryPoint()
            # print("go home")
            # print(self.RobotJointPosition)
            # print(self._joint_state)
            # print("----------")
            home_init_point.positions = self.RobotJointPosition
            home_init_point.velocities = [0.0] * len(home_from_yaml)
            home_init_point.accelerations = [0.0] * len(home_from_yaml)
            home_init_point.effort = []
            home_init_point.time_from_start = Duration(sec=0, nanosec=0)
            home_goal.trajectory.points.append(home_init_point)  # type: ignore

            home_point = JointTrajectoryPoint()
            home_point.positions = home_from_yaml
            home_point.velocities = [0.0] * len(home_from_yaml)
            home_point.accelerations = [0.0] * len(home_from_yaml)
            home_point.effort = []
            home_point.time_from_start = Duration(sec=1, nanosec=0)
            home_goal.trajectory.points.append(home_point) # type: ignore
            
            home_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            home_future = self._clientFollowCartTraj.send_goal_async(home_goal)

            speed_ovr_msg = Int16()
            speed_ovr_msg.data = 100
            self.pub_speed_ovr.publish(speed_ovr_msg) 
            print('Going home with speed PCT: %d' %speed_ovr_msg.data)

            # rclpy.spin_until_future_complete(self, home_future)
            return False
     
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!!')
            return
        self._goal_handle = goal_handle
        # self.get_logger().info('Goal accepted')
        # self._get_result_future.add_done_callback(self._done_callback)


    # def _done_callback(self, future):
    #     try:
    #         # result = future.result()
    #         # self.get_logger().info(f'Result: {result.sequence}')
    #         if not self.KeepOnMovingBool: # and self._clientFollowCartTraj.wait_for_result():
    #             return
    #         elif self.trainingOn: 
    #             self.NumberExecMovements += 1
    #             # self.startMovementFCT()
    #             # self.trainingOn = False
    #             print('Number of movements: %d' %self.NumberExecMovements)
    #     except Exception as e:
    #         self.get_logger().error(f'Exception in _done_callback: {e}')

    
    def stopProtocolFCT(self):
        if hasattr(self, '_goal_handle'):
            cancel_future = self._goal_handle.cancel_goal_async()
            self.clearFCT()
        else:
            self.get_logger().info('goal handle has not been created yet')

    
    def clearFCT(self):
        self._goalFCT = FollowJointTrajectory.Goal()
        self._goalFCT.trajectory.joint_names = JOINT_NAMES
        # self._goalFCT.path_tolerance.position_error.x = 0.01
        # self._goalFCT.path_tolerance.position_error.y = 0.01
        # self._goalFCT.path_tolerance.position_error.z = 0.01
        # self._goalFCT.relative=True


    def sendTrajectoryFCT(self):       
        CartesianMovementData = self.uiMovementWindow.CartesianMovementData
        self.CartesianPositions = CartesianMovementData.get("cart_trj3").get("cart_positions")
        self.TimeFromStart = CartesianMovementData.get("cart_trj3").get("time_from_start")
        self.clearFCT()
        _numSample = len(self.TimeFromStart)
        for iPoint in range(0, _numSample): 
            _TimeFromStart = Duration(sec=int(self.TimeFromStart[iPoint][0]),nanosec=int((self.TimeFromStart[iPoint][0] - int(self.TimeFromStart[iPoint][0])) * 1e9))           
            self.add_pointFCT(self.CartesianPositions[iPoint], _TimeFromStart)    


    def getJointState(self, data):
        self._joint_state = data
        self.RobotJointPosition = list(self._joint_state.position)


    def getToolPosition(self, data):
        ToolPosition = data
        self.HandlePosition = [0,0,0]
        self.HandlePosition[0] = ToolPosition.position[0]
        self.HandlePosition[1] = ToolPosition.position[1]
        self.HandlePosition[2] = ToolPosition.position[2]
 
    # def getSpeedOveride(self, data):
    #     SpeedOverride = data
    #     # print(SpeedOverride)
        


def main(args=None):
    rclpy.init(args=args)
    QPushButton.enablePushButton = MC_Tools.enablePushButton #Adding a new method defined in MC_Tools file
    app = QtWidgets.QApplication(sys.argv)
    ui = MainProgram()
    ui.setupUi_MainWindow()
    ui.retranslateUi_MainWindow(app)
    ui.definePaths()
    ui.startMovementWindow()
    ui.start_ROS_functions()
    ui.rosTimer()
    # rclpy.spin_once(ui)
    ui.FMRR_clientFollowCartTraj()
    ui.updateTimer()
    ui.initializeVariables()
    ui.FMRRMainWindow.show()
    sys.exit(app.exec_())
    

if __name__ == "__main__":
    main()
    rclpy.shutdown()
