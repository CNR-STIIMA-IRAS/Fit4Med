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
from PyQt5.QtWidgets import QMessageBox, QWidget
from .RobotWindow import Ui_RobotWindow
import yaml
from yaml.loader import SafeLoader
import numpy as np
from scipy import interpolate
from copy import deepcopy
#import FMRRMainProgram as FMRRMain

#MC Classes/methods
from . import MC_Tools

#ROS
from builtin_interfaces.msg import Duration
#import rospkg
from geometry_msgs.msg import Point
#from StringIO import StringIO


class FMRR_Ui_RobotWindow(Ui_RobotWindow):
    def __init__(self) -> None:
        super().__init__()

  
 
##############################################################################################################
#####                                                                                                    #####  
#####                                        MOVE THE ROBOT TO DEFINED POSITION                          ##### 
#####                                                 callbacks                                          #####
#####                                                                                                    #####
##############################################################################################################
    
    ###### Callback of buttons to MOVE (TRANSLATIONS and ROTATIONS)      
    def clbk_BtnPlusMinusCoordinate(self, SignIncrement, CoordinateNr): # NCoordinate = 0,1,2 --> x,y,z or 0,1,2,3,4,5 -->J1,J2,J3 ..,J6.
        if self.ui_FMRRMainWindow.AnswerPauseService:
            self.ui_FMRRMainWindow.AnswerPauseService = self.ui_FMRRMainWindow.pauseService(False)
            print( 'The robot movement was resumed: %s' % self.ui_FMRRMainWindow.AnswerPauseService )        

        self.ui_FMRRMainWindow.JogOn = True
        
        if CoordinateNr < 3:
            Increment = self.spinBox_MoveRobotPosition.value()
            NewHandlePosition = self.ui_FMRRMainWindow.RobotJointPosition #[0.0, 0.0, 0.0]  # initialization of  position = [x,y,z,q1,q2,q3,q4] coorinates are RELATIVE!!!
            print('The current handle postions are: %s' % NewHandlePosition)

            self.ui_FMRRMainWindow.clearFCT()
            _TimeFromStart = Duration(sec=0, nanosec=0)          
            self.ui_FMRRMainWindow.add_pointFCT(NewHandlePosition,_TimeFromStart)

            NewHandlePosition [CoordinateNr] = self.ui_FMRRMainWindow.RobotJointPosition[CoordinateNr] + SignIncrement * float(Increment) /100.0 # conversion from meters to cm
            print('The new handle postions are: %s' % NewHandlePosition)
            _TimeFromStart = Duration(sec=1, nanosec=0)          
            self.ui_FMRRMainWindow.add_pointFCT(NewHandlePosition,_TimeFromStart)
            self.ui_FMRRMainWindow.startMovementFCT()
        else:
            print("coordinate number must be between 1 and 3 for this platform")
            # Increment =  self.spinBox_MoveRobotRotationalVelocity.value()
            # CoordinateNr -= 3 # minus x,y,z coordinates
            # ActualRobotConfiguration  = self.ui_FMRRMainWindow.RobotJointPosition
            # print('The current joint configuration is: %s' % ActualRobotConfiguration)
            # NewRobotConfiguration = ActualRobotConfiguration
            # NewRobotConfiguration[CoordinateNr] += SignIncrement * float(Increment)*np.pi/180.0# conversion from degrees to radiants
            # print('The new RobotConfiguration is: %s' % NewRobotConfiguration)
            # _TimeFromStart = Duration(1.0) # duration in s    
            # _TimeTolerance = Duration(0.1)       
            # self.ui_FMRRMainWindow.clearFJT()              
            # self.ui_FMRRMainWindow.add_pointFJT(NewRobotConfiguration, _TimeFromStart, _TimeTolerance)
            # self.ui_FMRRMainWindow.startMovementFJT()

        self.ui_FMRRMainWindow.JogOn = False
   
#        ToRobotNewPositions = self.FromRobotPositions
#        ToRobotNewPositions[CoordinateNr] = self.FromRobotPositions[CoordinateNr] + SignIncrement * Increment
#        self.CurrentPositions[CoordinateNr].display(ToRobotNewPositions[CoordinateNr]) 
#        self.setNewPositions(ToRobotNewPositions)
    

    # def clbk_BtnEnableApproach(self):
    #     _translate = QtCore.QCoreApplication.translate
    #     if self.pushButton_EnableApproach.State:

    #        self.pushButton_EnableApproach.setText(_translate("RobotWindow", "Disable approach"))
    #        self.pushButton_EnableApproach.State = 0

    #        self.pushButton_Approach_Joint1.enablePushButton(1)
    #        self.pushButton_Approach_Joint2.enablePushButton(1)
    #        self.pushButton_Approach_Joint3.enablePushButton(1)
    #     #    self.pushButton_Approach_Joint4.enablePushButton(1)
    #     #    self.pushButton_Approach_Joint5.enablePushButton(1)
    #     #    self.pushButton_Approach_Joint6.enablePushButton(1)
    #        self.pushButton_ApproachAllJoint.enablePushButton(1)

    #     else:
    #        self.pushButton_EnableApproach.setText(_translate("RobotWindow", "Enable approach"))
    #        self.pushButton_EnableApproach.State = 1
           
    #        self.pushButton_Approach_Joint1.enablePushButton(0)
    #        self.pushButton_Approach_Joint2.enablePushButton(0)
    #        self.pushButton_Approach_Joint3.enablePushButton(0)
    #     #    self.pushButton_Approach_Joint4.enablePushButton(0)
    #     #    self.pushButton_Approach_Joint5.enablePushButton(0)
    #     #    self.pushButton_Approach_Joint6.enablePushButton(0)
    #        self.pushButton_ApproachAllJoint.enablePushButton(0)
           
    def clbk_JointApproach(self, JointNr):
        ActualRobotConfiguration  = self.ui_FMRRMainWindow.RobotJointPosition
        print('The current joint configuration is: %s' % ActualRobotConfiguration)
        NewRobotConfiguration = ActualRobotConfiguration
        JointTargetPosition = (self.doubleSpin_Joint1_Value, self.doubleSpin_Joint2_Value, self.doubleSpin_Joint3_Value) #, self.doubleSpin_Joint4_Value, self.doubleSpin_Joint5_Value, self.doubleSpin_Joint6_Value)
        if JointNr == 3:
            NewRobotConfiguration = JointTargetPosition
        else:
            NewRobotConfiguration[JointNr] = JointTargetPosition[JointNr]
        print('The new RobotConfiguration is: %s' % NewRobotConfiguration)
        _TimeFromStart = Duration(sec=5, nanosec=0) # duration in s            
        _TimeTolerance = Duration(sec= 5, nanosec=int(1e-9))       
        self.ui_FMRRMainWindow.clearFCT()              
        self.ui_FMRRMainWindow.add_pointFCT(NewRobotConfiguration, _TimeFromStart, _TimeTolerance)
        self.ui_FMRRMainWindow.startMovementFCT()
        
    def clbk_StartMoveRobotManually(self):
        
        self.pushButton_StartMoveRobotManually.enablePushButton(0)
        self.pushButton_StopMoveRobotManually.enablePushButton(1)
        #attiva manual guidance
        pass
    
    def clbk_StopMoveRobotManually(self):
        self.pushButton_StartMoveRobotManually.enablePushButton(1)
        self.pushButton_StopMoveRobotManually.enablePushButton(0)
        
        #attiva position control
    
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
        # chiamare homing
        
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
        self.DialogFMRRMainWindow.show()
        self.DialogRobotWindow.hide()
        
        
    def clbk_BtnGotoMovement(self):
        self.ui_FMRRMainWindow.DialogMovementWindow.show()
        self.DialogRobotWindow.hide()
        
        
######
    def updateRobotWindow(self, RobotWindow):
#       Joint and tool postions are converted and displayed 
#       conversion factors (fromm RAD to Degree and from meters tocentimeteres are declared at the beginning od the FMRRMainProgram class    

#       Joints lcd 
        _jointPosConvFact = self.ui_FMRRMainWindow._jointPosConvFact #
        try:
            RobotJointPosition = self.ui_FMRRMainWindow.RobotJointPosition
        except:
            return
#        print('ActualRobotJointPositon:')
#        print(RobotJointPosition)
        RobotJointPosition = [Ji_position * _jointPosConvFact for Ji_position in RobotJointPosition]
        
        ##       Handle lcd 
        _toolPosCovFact = self.ui_FMRRMainWindow._toolPosCovFact
        try:
            HandlePosition = self.ui_FMRRMainWindow.HandlePosition
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
                self.frame_GoalPosition.setStyleSheet("background-color: rgb(0, 0, 190);")
                self.frame_GoalPosition.setEnabled(True)
                self.pushButton_LoadJointPosition.setEnabled(False)
                self.pushButton_SaveStartPosition.setEnabled(False)
            else:
                self.lineEdit_MoveRobotPosition_GoalPosition.setText( "Goal Position (!!! UNDEFINED FRAME!!!)" ) # type: ignore
                self.frame_GoalPosition.setEnabled(False)
             
          
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
            
# DISABLE        
        self.pushButton_StopMoveRobotManually.enablePushButton(0)
        
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
        self.pushButton_AbsHoming.clicked.connect(lambda: self.clbk_BtnAbsoluteHoming())
        self.pushButton_RelativeHoming.clicked.connect(lambda: self.clbk_BtnRelativeHoming())
        self.pushButton_StartMoveRobotManually.clicked.connect(lambda: self.clbk_StartMoveRobotManually())
        self.pushButton_StopMoveRobotManually.clicked.connect(lambda: self.clbk_StopMoveRobotManually())

        self.pushButton_LoadJointPosition.clicked.connect(lambda: self.clbk_BtnLoadJointPosition())

        self.pushButton_Xminus.clicked.connect(lambda: self.clbk_BtnPlusMinusCoordinate(-1 ,0) )
        self.pushButton_Yminus.clicked.connect(lambda: self.clbk_BtnPlusMinusCoordinate(-1, 1) )
        self.pushButton_Zminus.clicked.connect(lambda: self.clbk_BtnPlusMinusCoordinate(-1, 2) )

        self.pushButton_Xplus.clicked.connect(lambda: self.clbk_BtnPlusMinusCoordinate(1 ,0) ) 
        self.pushButton_Yplus.clicked.connect(lambda: self.clbk_BtnPlusMinusCoordinate(1, 1) )
        self.pushButton_Zplus.clicked.connect(lambda: self.clbk_BtnPlusMinusCoordinate(1, 2) )
        
        self.pushButton_Approach_Joint1.clicked.connect( lambda: self.clbk_JointApproach(0) )
        self.pushButton_Approach_Joint2.clicked.connect( lambda: self.clbk_JointApproach(1) )
        self.pushButton_Approach_Joint3.clicked.connect( lambda: self.clbk_JointApproach(2) )
        # self.pushButton_Approach_Joint4.clicked.connect( lambda: self.clbk_JointApproach(3) )
        # self.pushButton_Approach_Joint5.clicked.connect( lambda: self.clbk_JointApproach(4) )
        # self.pushButton_Approach_Joint6.clicked.connect( lambda: self.clbk_JointApproach(5) )
        self.pushButton_ApproachAllJoint.clicked.connect( lambda: self.clbk_JointApproach(6) )
        
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
    QPushButton.enablePushButton = MC_Tools.enablePushButton #Adding a new method defined in MC_Tools file    
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
