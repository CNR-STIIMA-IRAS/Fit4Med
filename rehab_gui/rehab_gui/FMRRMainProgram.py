import os
import sys
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QTimer

# mathematics
import numpy as np
# from sympy import true

# import rclpy.action
# import rclpy.duration
# import rclpy.time

#MC Classes/methods
from .MovementProgram import FMRR_Ui_MovementWindow 
from .RobotProgram import FMRR_Ui_RobotWindow 
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
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState # joints positions, velocities and efforts
from std_msgs.msg import Int16 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ethercat_controller_msgs.srv import SwitchDriveModeOfOperation
from controller_manager_msgs.srv import UnloadController, LoadController, ConfigureController, SwitchController
# from geometry_msgs.msg import Point
# import debugpy
import time 

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

        self.DialogMovementWindow = QtWidgets.QDialog()
        self.DialogRobotWindow = QtWidgets.QDialog()
        self.FMRRMainWindow = QtWidgets.QMainWindow() # FMRRMainWindow is an instance of the class QtWidgets.QMainWindow. Not to be confuse with the namof the file FMRRMainWindow.py
        self.get_logger().info("FMRR cell node started.")
        self._joint_state = None

    def initializeVariables(self):
        self.AnswerPauseService = False
        self.trainingOn = False
        self.FIRST_TIME = True
        self.JogOn = False
        
    def updateTimercallback(self):
        self.uiRobotWindow.updateRobotWindow( self.DialogRobotWindow )
        
         
    def updateTimer(self):
        self._update_timer = QTimer()                                           # creo l'oggetto
        self._update_timer.timeout.connect( 
        self.updateTimercallback  ) # lo collego ad una seconda  callback
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
        question = QMessageBox(QMessageBox.Question, "Exit Program", MyString, QMessageBox.Yes | QMessageBox.No)
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

    def start_homing_procedure(self):
        switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        if switch_client.wait_for_service(timeout_sec=5.0):
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = []
            switch_req.deactivate_controllers = [self.controller_name]
            switch_req.strictness = SwitchController.Request.STRICT
            self.get_logger().info(f'Deactivating controller {self.controller_name}...')
            future = switch_client.call_async(switch_req)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.3)
            self.unload_controller_before_homing()
        else:
            self.get_logger().error('Failed to deactivate controller before homing.')

    def unload_controller_before_homing(self):
        client = self.create_client(UnloadController, '/controller_manager/unload_controller')
        if client.wait_for_service(timeout_sec=5.0):
            request = UnloadController.Request()
            request.name = self.controller_name
            self.get_logger().info(f'Unloading controller {self.controller_name}...')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.3)
            self.set_homing_mode_all_dofs()
        else:
            self.get_logger().error('Error unloading the controller before homing.')

    def set_homing_mode_all_dofs(self):
        self.remaining_homing_dofs = set(JOINT_NAMES)
        self._set_mode_for_next_dof(6, after_all_done=self.load_controller_after_homing)

    def load_controller_after_homing(self):
        time.sleep(0.5)
        load_client = self.create_client(LoadController, '/controller_manager/load_controller')
        if load_client.wait_for_service(timeout_sec=5.0):
            load_req = LoadController.Request()
            load_req.name = self.controller_name
            self.get_logger().info(f'Loading controller {self.controller_name}...')
            future = load_client.call_async(load_req)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.3)
            self.configure_controller_after_loading()
        else:
            self.get_logger().error('Failed to load controller after homing.')

    def configure_controller_after_loading(self):
        config_client = self.create_client(ConfigureController, '/controller_manager/configure_controller')
        if config_client.wait_for_service(timeout_sec=5.0):
            config_req = ConfigureController.Request()
            config_req.name = self.controller_name
            self.get_logger().info(f'Configuring controller {self.controller_name}...')
            future = config_client.call_async(config_req)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.3)
            self.activate_controller_after_config()
        else:
            self.get_logger().error('Failed to configure controller after loading.')

    def activate_controller_after_config(self):
        switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        if switch_client.wait_for_service(timeout_sec=5.0):
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = [self.controller_name]
            switch_req.deactivate_controllers = []
            switch_req.strictness = SwitchController.Request.STRICT
            self.get_logger().info(f'Activating controller {self.controller_name}...')
            future = switch_client.call_async(switch_req)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.5)
            self.set_cyclic_mode_all_dofs()
        else:
            self.get_logger().error('Failed to activate controller after configure.')

    def set_cyclic_mode_all_dofs(self):
        self.remaining_cyclic_dofs = set(JOINT_NAMES)
        self._set_mode_for_next_dof(8, after_all_done=self.confirm_homing_completed)

    def _set_mode_for_next_dof(self, mode_value, after_all_done):
        target_dofs = self.remaining_homing_dofs if mode_value == 6 else self.remaining_cyclic_dofs

        if not target_dofs:
            after_all_done()
            return

        dof = target_dofs.pop()
        client = self.create_client(SwitchDriveModeOfOperation, '/state_controller/switch_mode_of_operation')
        if client.wait_for_service(timeout_sec=5.0):
            req = SwitchDriveModeOfOperation.Request()
            req.dof_name = dof
            req.mode_of_operation = mode_value
            self.get_logger().info(f"Setting mode {mode_value} for {dof}...")
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.2)
            self._set_mode_for_next_dof(mode_value, after_all_done)
        else:
            self.get_logger().error(f"Service unavailable to set mode {mode_value} for {dof}")
            self._set_mode_for_next_dof(mode_value, after_all_done)

    def confirm_homing_completed(self):
        self.get_logger().info('✅ Homing procedure completed successfully.')
     
           
              
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
        self.pushButton_MoveRobot.enablePushButton(1)
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
        self.pushButton_MoveRobot.clicked.connect(self.clbk_BtnMoveRobot)
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
        self.controller_name = 'joint_trajectory_controller'

    #   subscribers        
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.getJointState, 1)
        self.tool_subscriber = self.create_subscription(JointState, 'joint_states', self.getToolPosition, 1)
        # rospy.Subscriber( 'speed_ovr', Int16, self.getSpeedOveride )

    #   publishers        
        self.pub_speed_ovr = self.create_publisher(Int16, '/speed_ovr', 10)
        # self.pub_start_point = self.create_publisher(Point, '/start_point', 10)
        # self.pub_end_point = self.create_publisher(Point, '/end_point', 10)

    #   action client
        self._clientFollowCartTraj = ActionClient(self, FollowJointTrajectory, f'/{self.controller_name}/follow_joint_trajectory')
     

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
    ui.startRobotWindow()
    ui.startMovementWindow()
    ui.start_ROS_functions()
    ui.rosTimer()

    ui._ros_timer = QTimer()
    ui._ros_timer.timeout.connect(lambda: rclpy.spin_once(ui, timeout_sec=0))
    ui._ros_timer.start(ui._ros_period)


    # rclpy.spin_once(ui)
    ui.FMRR_clientFollowCartTraj()
    ui.updateTimer()
    ui.initializeVariables()
    ui.FMRRMainWindow.show()
    sys.exit(app.exec_())
    

if __name__ == "__main__":
    main()
    rclpy.shutdown()
