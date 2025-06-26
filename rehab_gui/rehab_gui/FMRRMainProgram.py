import os
import sys
from PyQt5 import QtWidgets, QtCore 
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject
from enum import Enum
import time

# mathematics
import numpy as np
from scipy.interpolate import CubicSpline

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
from sensor_msgs.msg import JointState # joints positions, velocities and efforts
from std_msgs.msg import Int16 
from ethercat_controller_msgs.srv import SwitchDriveModeOfOperation
from controller_manager_msgs.srv import SwitchController
# from geometry_msgs.msg import Point
# import debugpy
from rclpy.parameter_client import AsyncParameterClient
from action_msgs.msg import GoalStatus

DEBUG = False

JOINT_NAMES = [
    'joint_1',
    'joint_2',
    'joint_3'
]


class MovementActionWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)
    
    def __init__(self, ros_node):
        super(MovementActionWorker, self).__init__()
        self._node = ros_node
        self._init_time_s = 0
        #   action client
        self.controller_name = 'joint_trajectory_controller'
        self._clientFollowCartTraj = ActionClient(self._node, FollowJointTrajectory, f'/{self.controller_name}/follow_joint_trajectory')
        if not self._clientFollowCartTraj.wait_for_server(2):
            print("Action server not available, exiting...")
            rclpy.shutdown()
            sys.exit(1)
        self.clearFCT() 

    def clearFCT(self):
        self._goalFCT = FollowJointTrajectory.Goal()
        self._goalFCT.trajectory.joint_names = JOINT_NAMES

    def setFCT(self, cartesian_positions, time_from_start,**kwargs):
        # Called only at the beginning of the training
        self.clearFCT()
        if type(time_from_start[0]) is not Duration: 
            self._total_time_s = (time_from_start[-1][0] - time_from_start[0][0])
        else:
            self._total_time_s = time_from_start[-1].sec - time_from_start[0].sec + (time_from_start[-1].nanosec - time_from_start[0].nanosec) * 1e-9
        if self._total_time_s <= 0:
            self._total_time_s = 1.0
        
        _numSample = len(time_from_start)
        t_spl = np.zeros(_numSample)
        self._time_from_start_duration = [0] * _numSample
        self._point_velocities = [0.0] * _numSample
        self._point_accelerations= [0.0] * _numSample
        for iPoint in range(0, _numSample):
            if type(time_from_start[iPoint]) is not Duration:
                _time_from_start_s = time_from_start[iPoint][0]
            else:
                _time_from_start_s = time_from_start[iPoint].sec + time_from_start[iPoint].nanosec * 1e-9
            t_spl[iPoint] = float(_time_from_start_s)

        # Interpolate trajectory and calculate velocities and accelerations
        P = np.array(cartesian_positions)
        splines = [CubicSpline(t_spl, P[:, i], bc_type='clamped') for i in range(3)]

        for iPoint in range(_numSample):
            t_iPoint = t_spl[iPoint]
            pos = np.array([s(t_iPoint) for s in splines])
            vel = np.array([s.derivative(1)(t_iPoint) for s in splines])
            acc = np.array([s.derivative(2)(t_iPoint) for s in splines])
            self._time_from_start_duration[iPoint] = Duration(sec=int(t_iPoint),nanosec=int((t_iPoint - int(t_iPoint)) * 1e9))
            self._point_velocities[iPoint] = vel

            self.add_pointFCT(pos, vel, acc, t_iPoint, **kwargs)

    def setSpeedOverrideFCT(self, speed_ovr):
        self.speed_scale = speed_ovr / 100.0
        vel_scaled = [v * self.speed_scale for v in self._point_velocities]

        for iPoint in range(len(self._goalFCT.trajectory.points)):
            _d = self._time_from_start_duration[iPoint]
            _time_from_start_s = (_d.sec + (_d.nanosec / 1e9)) / self.speed_scale

            self._goalFCT.trajectory.points[iPoint].time_from_start = Duration(
                sec=int(_time_from_start_s),
                nanosec=int((_time_from_start_s - int(_time_from_start_s)) * 1e9)
            )
            self._goalFCT.trajectory.points[iPoint].velocities = vel_scaled[iPoint]
            if iPoint==0 or iPoint==len(self._goalFCT.trajectory.points):
                if vel_scaled[iPoint].any() != 0:
                    print(f"Warning: Velocity at point {iPoint} is not zero, but it should be. Vel: {vel_scaled[iPoint]}")
            with open("trajectory_log.txt", "a") as log_file:
                log_file.write(
                    f'SET SPEED OVR {speed_ovr}:\t TIME {self._goalFCT.trajectory.points[iPoint].time_from_start}\t POS:{self._goalFCT.trajectory.points[iPoint].positions}\t VEL:{self._goalFCT.trajectory.points[iPoint].velocities}\t ACC:{self._goalFCT.trajectory.points[iPoint].accelerations}\n'
                )

    def add_pointFCT(self, positions, velocities, accelerations, time, **kwargs):
        point=JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.effort = [] * len(positions)
        point.time_from_start = time
        self._goalFCT.trajectory.points.append(point)
         
    def start_movementFCT(self, default_speed_ovr):
        # Called at the beginning of each movement
        self.setSpeedOverrideFCT(default_speed_ovr)
        self._init_time_s = time.time()
        self._goalFCT.trajectory.header.stamp = self._node.get_clock().now().to_msg()
        self._send_goal_future = self._clientFollowCartTraj.send_goal_async(self._goalFCT)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._node.get_logger().info('Goal rejected!!')
            return
        self._node.get_logger().info('Goal accepted')
        self._get_goal_result_future = self._goal_handle.get_result_async()
        self._get_goal_result_future.add_done_callback(self._done_callback)
        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self.timer = self._node.create_timer(0.02, lambda: self.check_status())

    def _done_callback(self, future):
        try:
            self.finished.emit()
            self.timer.cancel()
        except Exception as e:
            self._node.get_logger().error(f'Exception in _done_callback: {e}')

    def check_status(self):
        old_status = self._goal_status
        self._goal_status = self._goal_handle.status
        
        if old_status != self._goal_status:
            old_status_string = self.get_status_string(old_status)
            status_string = self.get_status_string(self._goal_status)
            self._node.get_logger().info(f'Action transition from {old_status_string} to {status_string}')

        actual_time = time.time()
        actual_time_from_start_percentage = float(actual_time - self._init_time_s)/float(self._total_time_s/self.speed_scale) * 100
        self.progress.emit(int(actual_time_from_start_percentage))
            
    @staticmethod
    def get_status_string(status):
        status_dict = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED'
        }
        return status_dict.get(status, 'UNKNOWN')

    def stopFCT(self):
        if hasattr(self, '_goal_handle'):
            cancel_future = self._goal_handle.cancel_goal_async()
            self.clearFCT()
        else:
            self._node.get_logger().info('goal handle has not been created yet')


class RosManager(Node):
    _ros_period = 1
    
    def __init__(self):
        Node.__init__(self, 'FMRR_cell_node') # type: ignore
        self._ros_timer = QTimer()
        self._ros_timer.timeout.connect(self.spin_ros_once)
        self._ros_timer.start(self._ros_period)
    #   subscribers        
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.getJointState, 1)
        self.tool_subscriber = self.create_subscription(JointState, 'joint_states', self.getToolPosition, 1)
        # rospy.Subscriber( 'speed_ovr', Int16, self.getSpeedOveride )
    #   publishers        
        self.pub_speed_ovr = self.create_publisher(Int16, '/speed_ovr', 10)

        self.cm_param_client = AsyncParameterClient(self, 'controller_manager')
        param_client_success= self.cm_param_client.wait_for_services(5.0)

        if not param_client_success:
            self.get_logger().error("Parameter client services are not ready.")
            rclpy.shutdown()
            sys.exit(1)
        self.update_rate = -1 
        future =  self.cm_param_client.get_parameters(['update_rate'])
        rclpy.spin_until_future_complete(self,future)
        result = future.result()   #rcl_interfaces.srv.GetParameters_Response
        if result:
            self.get_logger().info(f"Read: update rate = {result.values[0].integer_value}")
            self.update_rate = result.values[0].integer_value
        else:
            self.get_logger().error("Failed to read update rate parameter.")
            rclpy.shutdown()
            sys.exit(1)
        self._joint_state = None

    def getJointState(self, data):
        self._joint_state = data
        self.RobotJointPosition = list(self._joint_state.position)

    def getToolPosition(self, data):
        ToolPosition = data
        self.HandlePosition = [0,0,0]
        self.HandlePosition[0] = ToolPosition.position[0]
        self.HandlePosition[1] = ToolPosition.position[1]
        self.HandlePosition[2] = ToolPosition.position[2]
        
    def spin_ros_once(self):
        rclpy.spin_once(self, timeout_sec=0)

     

class MainProgram(Ui_FMRRMainWindow, QtCore.QObject): 
    _update_windows_period = 500
    _update_TrainingTime = 20
    _toolPosCovFact = 100 # to display coordinatates in centimeters (are given in meters in the yaml files) (used in MovementWindow to display data)
    _jointPosConvFact = 180/np.pi # conversion from radiants to degrees (used in MovementWindow to display data)
    trigger_worker = pyqtSignal(int) 

    def __init__(self):
        QtCore.QObject.__init__(self)
        self.ROS = RosManager()
        self.DialogMovementWindow = QtWidgets.QDialog()
        self.DialogRobotWindow = QtWidgets.QDialog()
        self.FMRRMainWindow = QtWidgets.QMainWindow() # FMRRMainWindow is an instance of the class QtWidgets.QMainWindow. Not to be confuse with the namof the file FMRRMainWindow.py
        self.ROS.get_logger().info("FMRR cell node started.")

        # Set up Movement Worker
        self.worker_thread = QThread()
        self.MovementWorker = MovementActionWorker(self.ROS)
        self.MovementWorker.moveToThread(self.worker_thread)
        
    def initializeVariables(self):
        self.isPaused = False
        # self.FIRST_TIME = True
        self.JogOn = False
        self.NumberExecMovements = 0
        self.TotalTrainingTime = 0
        self.ActualTrainingTime = 0
        self.execution_time_percentage = 0
        self.iPhase = 0
        self._counter_request_homing_procedure = 0 # counter to avoid multiple request of homing procedure
        self.trigger_worker.connect(self.MovementWorker.start_movementFCT)# type: ignore
        self.MovementWorker.finished.connect(self.on_worker_finished)
        self.MovementWorker.progress.connect(self.on_worker_progress)
        self.worker_thread.start()
        self.MovementWorker.clearFCT()
                
    def updateWindowTimerCallback(self):
        self._update_windows_timer = QTimer()                                           # creo l'oggetto
        self._update_windows_timer.timeout.connect( lambda: self.updateFMRRWindow() )                    # lo collego ad una callback
        self._update_windows_timer.timeout.connect( lambda: self.uiRobotWindow.updateRobotWindow( self.DialogRobotWindow ) ) # lo collego ad una seconda  callback
        self._update_windows_timer.start(self._update_windows_period)

    def updateTrainingTimer(self):
        self._update_TrainingTimer = QTimer()                                           # creo l'oggetto
        self._update_TrainingTimer.timeout.connect(lambda: self.update_TrainingParameters())
        self._update_TrainingTimer.start(self._update_TrainingTime)
        self.ROS.get_logger().info("Update Training TImer Started (event loop in the main thread)")

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
        self.sendTrajectoryFCT()
        self.updateTrainingTimer()
        self.ModalityActualValue = self.Modalities[0]
        print(f'------------------------------------------{self.spinBoxSpeedOvr[0].value()}')
        self.startMovementFCT(self.spinBoxSpeedOvr[0].value()) # type: ignore
        self.pushButton_PAUSEtrainig.enablePushButton(1)
        self.pushButton_STOPtrainig.enablePushButton(1)
    
    def clbk_PAUSEtrainig(self):
        _translate = QtCore.QCoreApplication.translate
        if self.pushButton_PAUSEtrainig.State:
            self.pushButton_PAUSEtrainig.State = 0    
            self.pushButton_PAUSEtrainig.setText( _translate("FMRRMainWindow", "RESUME training") )
            self.isPaused = True
            print( 'The robot movement was successfully stopped: %s' % self.isPaused )
            
        else:
            self.pushButton_PAUSEtrainig.State = 1    
            self.pushButton_PAUSEtrainig.setText( _translate("FMRRMainWindow", "PAUSE training") )
            self.isPaused = False
            print( 'The robot movement was successfully resumed: %s' % self.isPaused )

    def clbk_STOPtrainig(self):   
        self.MovementWorker.stopFCT() 
        for iProgBar in self.progressBarPhases:
            iProgBar = 0
        self.pushButton_PAUSEtrainig.enablePushButton(0)
        self.pushButton_STOPtrainig.enablePushButton(0)
        self._update_TrainingTimer.stop()
              
    def update_TrainingParameters(self):
        if not self.isPaused:
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
        else:
            self.ROS.get_logger().info("Training is paused, skipping update.")
            return
        
    def on_worker_finished(self):
        self.iPhase += 1
        self.movement_completed = True
    
    def on_worker_progress(self,value):
        self.execution_time_percentage = int(value)  # Get the progress percentage from the worker

    def clbk_spinBox_MaxVel(self):
        speed_ovr_Value = self.spinBox_MaxVel.value()
        speed_ovr_msg = Int16()
        speed_ovr_msg.data = speed_ovr_Value
        self.ROS.pub_speed_ovr.publish(speed_ovr_msg)

    def start_homing_procedure(self):
        self.homing_done = False
        switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        if switch_client.wait_for_service(timeout_sec=5.0):
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = []
            switch_req.deactivate_controllers = [self.MovementWorker.controller_name]
            switch_req.strictness = SwitchController.Request.STRICT
            self.ROS.get_logger().info(f'Deactivating controller {self.MovementWorker.controller_name}...')
            future = switch_client.call_async(switch_req)
            rclpy.spin_until_future_complete(self, future)
            # time.sleep(0.3)
            self.set_homing_mode_all_dofs()
        else:
            self.ROS.get_logger().error('Failed to deactivate controller before homing.')

    def set_homing_mode_all_dofs(self):
        self.remaining_homing_dofs = set(JOINT_NAMES)
        self._set_mode_for_next_dof(6, after_all_done=self.activate_controller_after_homing)

    def activate_controller_after_homing(self):
        time.sleep(0.5)
        switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        if switch_client.wait_for_service(timeout_sec=5.0):
            switch_req = SwitchController.Request()
            switch_req.activate_controllers = [self.MovementWorker.controller_name]
            switch_req.deactivate_controllers = []
            switch_req.strictness = SwitchController.Request.STRICT
            self.ROS.get_logger().info(f'Activating controller {self.MovementWorker.controller_name}...')
            future = switch_client.call_async(switch_req)
            rclpy.spin_until_future_complete(self, future)
            self.set_cyclic_mode_all_dofs()
        else:
            self.ROS.get_logger().error('Failed to activate controller after configure.')

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
            self.ROS.get_logger().info(f"Setting mode {mode_value} for {dof}...")
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            # time.sleep(0.2)
            self._set_mode_for_next_dof(mode_value, after_all_done)
        else:
            self.ROS.get_logger().error(f"Service unavailable to set mode {mode_value} for {dof}")
            self._set_mode_for_next_dof(mode_value, after_all_done)

    def confirm_homing_completed(self):
        self.ROS.get_logger().info('✅ Homing procedure completed successfully.')
        self.homing_done = True
           
######################## MAIN FUNCTIONS ###########################
########
######## setupUi_MainWindow and retranslateUi_MainWindow start the FMRR windox and comnnects callbacks to widgets               
########
######## start_ROS_functions: start ROS node and subscribers                
########
###################################################################
    def setupUi_MainWindow(self):
        Ui_FMRRMainWindow.setupUi(self, self.FMRRMainWindow)
        self.homing_done = False
    
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
    
     
    def startPauseService(self):
        self.ROS.get_logger().info("Pause service is not implemented yet.")

    def startMovementFCT(self, default_speed_override=100):
        self.movement_completed = False
        self._home_status = GoalStatus.STATUS_UNKNOWN
        self._home_goal_sent = False
        self.trigger_worker.emit(default_speed_override)  # Trigger the worker to start the movement
        
        
    def clbk_BtnGOtoZero(self):
        self.ROS.get_logger().info("Going to zero position...")
        home_goal = FollowJointTrajectory.Goal()
        home_goal.trajectory.joint_names = JOINT_NAMES

        home_init_point = JointTrajectoryPoint()
        home_init_point.positions = self.ROS.RobotJointPosition
        home_init_point.velocities = [0.0] * len(JOINT_NAMES)
        home_init_point.accelerations = [0.0] * len(JOINT_NAMES)
        home_init_point.effort = []
        home_init_point.time_from_start = Duration(sec=0, nanosec=0)
        home_goal.trajectory.points.append(home_init_point)  # type: ignore

        home_point = JointTrajectoryPoint()
        home_point.positions = [0.0] * len(JOINT_NAMES)
        home_point.velocities = [0.0] * len(JOINT_NAMES)
        home_point.accelerations = [0.0] * len(JOINT_NAMES)
        home_point.effort = []
        home_point.time_from_start = Duration(sec=3, nanosec=0)
        home_goal.trajectory.points.append(home_point) # type: ignore
        
        home_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        home_future = self.MovementWorker._clientFollowCartTraj.send_goal_async(home_goal)
        #home_future.add_done_callback(self.home_cancel_callback)
        return True


    def sendTrajectoryFCT(self):       
        TrjYamlData = self.uiMovementWindow.TrjYamlData
        self.CartesianPositions = TrjYamlData.get("cart_trj3").get("cart_positions")
        self.TimeFromStart = TrjYamlData.get("cart_trj3").get("time_from_start")
        self.MovementWorker.setFCT(self.CartesianPositions, self.TimeFromStart)  # type: ignore    


    

        

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
    ui.updateWindowTimerCallback()
    ui.initializeVariables()
    ui.FMRRMainWindow.show()
    sys.exit(app.exec_())
    

if __name__ == "__main__":
    main()
    rclpy.shutdown()
