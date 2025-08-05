# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\RobotWindow.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from json import load
import os
import sys
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox
from .RobotWindow import Ui_RobotWindow
import yaml
from yaml.loader import SafeLoader
import numpy as np
from copy import deepcopy
#import FMRRMainProgram as FMRRMain

#MC Classes/methods
from . import MC_Tools

#ROS
from builtin_interfaces.msg import Duration
#import rospkg
from geometry_msgs.msg import Point
#from StringIO import StringIO
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ros2node.api import get_node_names


class FMRR_Ui_RobotWindow(Ui_RobotWindow):
    def __init__(self) -> None:
        super().__init__()

##############################################################################################################
#####                                                                                                    #####  
#####                                        MOVE THE ROBOT TO DEFINED POSITION                          ##### 
#####                                                 callbacks                                          #####
#####                                                                                                    #####
##############################################################################################################
    ###### Callback of buttons to Joint Approach with trajectory controller (TODO)
    def clbk_JointApproach(self, JointNr):
        if self.ui_FMRRMainWindow.ROS.current_controller != self.ui_FMRRMainWindow.ROS.trajectory_controller_name:
            self.ui_FMRRMainWindow._ros_node.get_logger().info(f"Switching to position mode of operation and loading {self.ui_FMRRMainWindow.ROS.trajectory_controller_name}")
            if not self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(8, self.ui_FMRRMainWindow.ROS.trajectory_controller_name):
                QMessageBox.warning(self.DialogRobotWindow, "Warning", "Failed to switch to trajectory controller. Please check the controller configuration.")
                return
        ActualRobotConfiguration  = deepcopy( self.ui_FMRRMainWindow.ROS.HandlePosition )
        print('The current joint configuration is: %s' % ActualRobotConfiguration)
        NewRobotConfiguration = ActualRobotConfiguration
        JointTargetPosition = (float(self.doubleSpin_Joint1_Value.value()), float(self.doubleSpin_Joint2_Value.value()), float(self.doubleSpin_Joint3_Value.value()))
        if JointNr == 3:
            NewRobotConfiguration = JointTargetPosition
        else:
            NewRobotConfiguration[JointNr] = JointTargetPosition[JointNr]
        print(f'The new RobotConfiguration is: {NewRobotConfiguration}')
        # _TimeTolerance = Duration(sec= 5, nanosec=int(1e-9))
        # self.ui_FMRRMainWindow.clbk_ApproachPoint(NewRobotConfiguration)
        
    def clbk_StartMoveRobotManually(self):
        self.pushButton_StartMoveRobotManually.enablePushButton(0)
        self.pushButton_StopMoveRobotManually.enablePushButton(1)
        self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(9, self.ui_FMRRMainWindow.ROS.admittance_controller)
        
    def clbk_StopMoveRobotManually(self):
        self.pushButton_StartMoveRobotManually.enablePushButton(1)
        self.pushButton_StopMoveRobotManually.enablePushButton(0)
        self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(8, self.ui_FMRRMainWindow.ROS.trajectory_controller_name)
        pass

    def clbk_BtnAbsoluteHoming(self):
        
        file_path = '/tmp/absolute_homing_performed'
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
            except Exception as e:
                print(f"Failed to remove {file_path}: {e}")
        file_path = '/tmp/relative_homing_performed'
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
            except Exception as e:
                print(f"Failed to remove {file_path}: {e}")
            
        #attiva manual guidance 
        self.clbk_StartMoveRobotManually()
        
        # Create a Dialogn Button to ask the user if he wants to do an absolute homing
        MyString = "Acknowledge when the robot is in the home position"
        question = QMessageBox(QMessageBox.Warning, "Absolute Homing", MyString, QMessageBox.Apply)
        question.button(QMessageBox.Apply).setText("Acknowledge")
        
        decision = question.exec_()
        
        self.clbk_StopMoveRobotManually()
        
        # Create an empty file under /tmp
        with open('/tmp/absolute_homing_performed', 'w') as f:
            f.write("homing performed")
            f.close()
    
    def clbk_BtnRelativeHoming(self):
        print("Relative Homing performed")
        file_path = '/tmp/absolute_homing_performed'
        if os.path.exists(file_path):
            try:
                print("Removing file: ", file_path)
                os.remove(file_path)
            except Exception as e:
                print(f"Failed to remove {file_path}: {e}")
        file_path = '/tmp/relative_homing_performed'
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
            except Exception as e:
                print(f"Failed to remove {file_path}: {e}")
        print(f"Files removed {not os.path.exists(file_path)}")
        # Call homing service
        self.ui_FMRRMainWindow.ROS.start_homing_procedure()
        # Create an empty file under /tmp
        with open('/tmp/relative_homing_performed', 'w') as f:
            f.write("homing performed")
            f.close()
    
    def clbk_BtnLoadJointPosition(self):
        FMRR_RootPath = self.ui_FMRRMainWindow.FMRR_Paths['Root']
        JoinDataPath = self.ui_FMRRMainWindow.FMRR_Paths['Joint_Configuration']
        filename = QtWidgets.QFileDialog.getOpenFileName(None, "Load Joint Conf", JoinDataPath, "*.yaml")
     
        if bool(filename[0]):
            JointTargetPosition = yaml.load(open(filename [0]), Loader=SafeLoader)
            JointData = JointTargetPosition.get("a_movement_definition").get("begin_joint_config") [0]
            self.doubleSpin_Joint1_Value.setValue(JointData[0]) 
            self.doubleSpin_Joint2_Value.setValue(JointData[1]) 
            self.doubleSpin_Joint3_Value.setValue(JointData[2]) 
            
##############################################################################################################
#####                                                                                                    #####  
#####                                            MOVEMENT PARAMETERS                                     ##### 
#####                                                 callbacks                                          #####
#####                                                                                                    #####
##############################################################################################################            

    def clbk_BtnGOtoTraining(self):
        if self.ui_FMRRMainWindow.ROS.current_controller != self.ui_FMRRMainWindow.ROS.trajectory_controller_name:
            self.ui_FMRRMainWindow._ros_node.get_logger().info(f"Switching to position mode of operation and loading {self.ui_FMRRMainWindow.ROS.trajectory_controller_name}")
            if not self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(8, self.ui_FMRRMainWindow.ROS.trajectory_controller_name):
                QMessageBox.warning(self.DialogRobotWindow, "Warning", "Failed to switch to trajectory controller. Please check the controller configuration.")
                return
        self.DialogFMRRMainWindow.show()
        self.DialogRobotWindow.hide()
             
    def clbk_BtnGotoMovement(self):
        if self.ui_FMRRMainWindow.ROS.current_controller != self.ui_FMRRMainWindow.ROS.trajectory_controller_name:
            self.ui_FMRRMainWindow._ros_node.get_logger().info(f"Switching to position mode of operation and loading {self.ui_FMRRMainWindow.ROS.trajectory_controller_name}")
            if not self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(8, self.ui_FMRRMainWindow.ROS.trajectory_controller_name):
                QMessageBox.warning(self.DialogRobotWindow, "Warning", "Failed to switch to trajectory controller. Please check the controller configuration.")
                return
        if self.ui_FMRRMainWindow.ROS.are_motors_on:
            QMessageBox.warning(self.DialogRobotWindow, "Warning", "Motors are still active. Please turn them off before exiting this window!")
            return
        self.ui_FMRRMainWindow.DialogMovementWindow.show()
        self.DialogRobotWindow.hide()

######
    def updateRobotWindow(self, RobotWindow):
#       Joint and tool postions are converted and displayed 
#       conversion factors (fromm RAD to Degree and from meters tocentimeteres are declared at the beginning od the FMRRMainProgram class    
#       Joints lcd 
        _jointPosConvFact = self.ui_FMRRMainWindow._jointPosConvFact #
        try:
            RobotJointPosition = self.ui_FMRRMainWindow.ROS.RobotJointPosition
        except:
            return
#        print('ActualRobotJointPositon:')
#        print(RobotJointPosition)
        RobotJointPosition = [Ji_position * _jointPosConvFact for Ji_position in RobotJointPosition]
        
        ##       Handle lcd 
        _toolPosCovFact = self.ui_FMRRMainWindow._toolPosCovFact
        try:
            HandlePosition = self.ui_FMRRMainWindow.ROS.HandlePosition
        except:
            return
        # print('HandlePosition:')
        # print(HandlePosition)
        HandlePosition[:] = [i_position * _toolPosCovFact for i_position in HandlePosition]

        CurrentDisplayedHandlePos = [self.lcdNumber_X, self.lcdNumber_Y, self.lcdNumber_Z]
        for iCoord in range(0,3):
            CurrentDisplayedHandlePos[iCoord].display( int(HandlePosition[iCoord]) )         

        ##############################################################################################
        #####                                                                                    #####
        #####                               SET GOAL FRAME                                       #####
        #####                                                                                    #####
        #####                                                                                    #####
        #######################à######################################################################
        file_path = '/tmp/absolute_homing_performed'
        if os.path.exists(file_path):
            self.lineEdit_MoveRobotPosition_GoalPosition.setText( "Goal Position (ABSOLUTE FRAME)" ) # type: ignore
            self.frame_GoalPosition.setStyleSheet("background-color: rgb(0, 190, 0);")
            self.frame_GoalPosition.setEnabled(True)
            self.pushButton_LoadJointPosition.setEnabled(True)
            self.pushButton_SaveStartPosition.setEnabled(True)
        else:    
            file_path = '/tmp/relative_homing_performed'
            if os.path.exists(file_path):
                self.lineEdit_MoveRobotPosition_GoalPosition.setText( "Goal Position (RELATIVE FRAME)" ) # type: ignore
                self.frame_GoalPosition.setStyleSheet("Background: light blue")
                self.frame_GoalPosition.setEnabled(True)
                self.pushButton_LoadJointPosition.setEnabled(False)
                self.pushButton_SaveStartPosition.setEnabled(False)
            else:
                self.lineEdit_MoveRobotPosition_GoalPosition.setText( "Goal Position (!!! UNDEFINED FRAME!!!)" ) # type: ignore
                self.frame_GoalPosition.setEnabled(False)

        self.frame_JOG.setEnabled(self.ui_FMRRMainWindow.ROS.enable_jog_buttons)
        self.frame_ManualGuide.setEnabled(self.ui_FMRRMainWindow.ROS.enable_manual_guidance)
        self.frame_Homing.setEnabled(self.ui_FMRRMainWindow.ROS.enable_zeroing)
        self.frame_GoalPosition.setEnabled(self.ui_FMRRMainWindow.ROS.enable_ptp)
        self.frame_GoalPosition.setEnabled(self.ui_FMRRMainWindow.ROS.enable_ptp)
        self.pushButton_ResetFaults.setEnabled(self.ui_FMRRMainWindow.ROS.manual_reset_faults)

        self.pushButton_StartMotors.setEnabled(not self.ui_FMRRMainWindow.ROS.are_motors_on)
        self.pushButton_StopMotors.setEnabled(self.ui_FMRRMainWindow.ROS.are_motors_on)

        if self.ui_FMRRMainWindow.ROS.is_in_fault_state:
            self.frame_FaultDetected.setStyleSheet("background-color: red; border-radius: 10px;")
        else:
            self.frame_FaultDetected.setStyleSheet("background-color: green; border-radius: 10px;")
    
              
    def setupUi_RobotWindow(self, RobotWindow):
        Ui_RobotWindow.setupUi(self, RobotWindow)
                        
    def retranslateUi_RobotWindow(self, RobotWindow):
        
        _translate = QtCore.QCoreApplication.translate
        RobotWindow.setWindowTitle(_translate("RobotWindow", "Movement Window"))
        
############################      Modify LCDs
######
      
        LcdWidgets  = [ self.lcdNumber_X, self.lcdNumber_Y, self.lcdNumber_Z]  #, self.lcdNumber_J4, self.lcdNumber_J5, self.lcdNumber_J6,

        for iWidget in LcdWidgets:
            iWidget.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
            iWidget.setDigitCount(4)
        
       
#        LcdWidgetsSartEndposition = [ self.lcdNumber_StartPos_X, self.lcdNumber_StartPos_Y , self.lcdNumber_StartPos_Z ,
#                                   self.lcdNumber_EndPos_X, self.lcdNumber_EndPos_Y, self.lcdNumber_EndPos_Z ]
#        
#        for iWidgetES in LcdWidgetsSartEndposition:
#           iWidgetES.setDigitCount(6)


#####
##############################################################################################

        
        ##############################################################################################
        #####                                                                                    #####  
        #####                               ENABLE/DISABLE BUTTONS                               ##### 
        #####                               (In MAIN add this line ...                           #####
        #####                  "QPushButton.enablePushButton = MC_Tools.enablePushButton"        #####
        #####                                                                                    #####
        #####                                                                                    #####
        ##############################################################################################        
        
# ENABLE
        self.pushButton_LoadJointPosition.enablePushButton(1)

        self.pushButton_Xminus.enablePushButton(1)
        self.pushButton_Yminus.enablePushButton(1)
        self.pushButton_Zminus.enablePushButton(1)

        self.pushButton_Xplus.enablePushButton(1) 
        self.pushButton_Yplus.enablePushButton(1)
        self.pushButton_Zplus.enablePushButton(1)
        
        self.pushButton_GOtoTraining.enablePushButton(1)
        self.pushButton_GOtoMovement.enablePushButton(1)
        self.pushButton_StartMoveRobotManually.enablePushButton(1)
        self.pushButton_StartMotors.setEnabled(1)
        self.pushButton_ResetFaults.enablePushButton(1)
            
# DISABLE        
        self.pushButton_StopMoveRobotManually.enablePushButton(0)
        self.pushButton_StopMotors.setEnabled(0)
        
        self.pushButton_Approach_Joint1.enablePushButton(1)
        self.pushButton_Approach_Joint2.enablePushButton(1)
        self.pushButton_Approach_Joint3.enablePushButton(1)
        # self.pushButton_Approach_Joint4.enablePushButton(0)
        # self.pushButton_Approach_Joint5.enablePushButton(0)
        # self.pushButton_Approach_Joint6.enablePushButton(0)
        self.pushButton_ApproachAllJoint.enablePushButton(1)
        
        self.pushButton_SaveStartPosition.enablePushButton(1)

        ##############################################################################################
        #####                                                                                    #####  
        #####                         MOVE THE ROBOT TO DEFINED POSITION                         ##### 
        #####                                      Connections                                   #####
        #####                                                                                    #####
        ##############################################################################################
        self.pushButton_AbsHoming.clicked.connect(self.clbk_BtnAbsoluteHoming)
        self.pushButton_RelativeHoming.clicked.connect(self.clbk_BtnRelativeHoming)
        self.pushButton_StartMoveRobotManually.clicked.connect(self.clbk_StartMoveRobotManually)
        self.pushButton_StopMoveRobotManually.clicked.connect(self.clbk_StopMoveRobotManually)
        self.pushButton_StartMotors.clicked.connect(self.ui_FMRRMainWindow.clbk_StartMotors)
        self.pushButton_StopMotors.clicked.connect(self.ui_FMRRMainWindow.clbk_StopMotors)
        self.pushButton_LoadJointPosition.clicked.connect(self.clbk_BtnLoadJointPosition)

        self.radioButton_EnableHoming.setCheckable(True)
        self.radioButton_EnableHoming.clicked.connect(self.ui_FMRRMainWindow.ROS.zeroing_enable)

        self.pushButton_JOG.setCheckable(True)
        self.pushButton_JOG.toggled.connect(self.ui_FMRRMainWindow.ROS.jog_enable)

        self.pushButton_ManualGuide.setCheckable(True)
        self.pushButton_ManualGuide.toggled.connect(self.ui_FMRRMainWindow.ROS.manual_guidance_enable)
        
        self.pushButton_Xminus.pressed.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(-1, 0))
        self.pushButton_Xplus.pressed.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(1, 0))
        self.pushButton_Yminus.pressed.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(-1, 1))
        self.pushButton_Yplus.pressed.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(1, 1))
        self.pushButton_Zminus.pressed.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(-1, 2))
        self.pushButton_Zplus.pressed.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(1, 2))
        # Quando rilasci il bottone, interrompi il movimento
        self.pushButton_Xminus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 0))
        self.pushButton_Xplus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 0))
        self.pushButton_Yminus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 1))
        self.pushButton_Yplus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 1))
        self.pushButton_Zminus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 2))
        self.pushButton_Zplus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 2))

        self.comboBox_ResetFaults.currentIndexChanged.connect(self.ui_FMRRMainWindow.ROS.reset_mode_changed)
        self.pushButton_ResetFaults.clicked.connect(self.ui_FMRRMainWindow.clbk_BtnResetFaults)
        
        self.pushButton_Approach_Joint1.clicked.connect( lambda: self.clbk_JointApproach(0) )
        self.pushButton_Approach_Joint2.clicked.connect( lambda: self.clbk_JointApproach(1) )
        self.pushButton_Approach_Joint3.clicked.connect( lambda: self.clbk_JointApproach(2) )
        self.pushButton_ApproachAllJoint.clicked.connect( lambda: self.clbk_JointApproach(3) )
        
        ##############################################################################################
        #####                                                                                    #####  
        #####                                   GENERAL BUTTONS                                  ##### 
        #####                                     Connections                                    #####
        #####                                                                                    #####
        ##############################################################################################

        self.pushButton_GOtoTraining.clicked.connect(lambda: self.clbk_BtnGOtoTraining())
        self.pushButton_GOtoMovement.clicked.connect(lambda: self.clbk_BtnGotoMovement())
        RobotWindow.adjustSize()

def main():
    #QPushButton.enablePushButton = MC_Tools.enablePushButton #Adding a new method defined in MC_Tools file    
    app = QtWidgets.QApplication(sys.argv)
    RobotWindow = QtWidgets.QDialog()
    ui = FMRR_Ui_RobotWindow()
    ui.setupUi(ui,RobotWindow)
    ui.retranslateUi_RobotWindow(ui,RobotWindow)
    ui.GetCurrentPositions(ui)
    RobotWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
