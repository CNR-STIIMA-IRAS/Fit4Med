import os
import sys
import time
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox
from RobotWindow import Ui_RobotWindow
import yaml
from yaml.loader import SafeLoader
from copy import deepcopy


class FMRR_Ui_RobotWindow(Ui_RobotWindow):
    def __init__(self) -> None:
        super().__init__()

    ###### Callback of buttons to Joint Approach with trajectory controller (TODO)
    def clbk_JointApproach(self, JointNr):
        ActualRobotConfiguration  = deepcopy( self.ui_FMRRMainWindow.ROS.HandlePosition)
        print('The current joint configuration is: %s' % ActualRobotConfiguration)
        NewRobotConfiguration = ActualRobotConfiguration
        JointTargetPosition = (float(self.doubleSpin_Joint1_Value.value()), float(self.doubleSpin_Joint2_Value.value()), float(self.doubleSpin_Joint3_Value.value()))
        if any(abs(JointTargetPosition[idx]) > 0.5 for idx in range(len(self.ui_FMRRMainWindow.ROS._joint_names))):
            QMessageBox.warning(self.DialogRobotWindow, "Warning", f"Joint target position is out of range. Please set a value between -0.5 and 0.5.")
            return
        if JointNr == 3:
            NewRobotConfiguration = JointTargetPosition
        else:
            NewRobotConfiguration[JointNr] = JointTargetPosition[JointNr]
        print(f'The new RobotConfiguration is: {NewRobotConfiguration}')
        target_time = max(abs(NewRobotConfiguration[i]) for i in range(len(self.ui_FMRRMainWindow.ROS._joint_names)))/0.1
        print(f"Moving to the new position in {target_time} seconds")
        if not self.ui_FMRRMainWindow.ROS.are_motors_on:
            QMessageBox.warning(self.DialogRobotWindow, "Warning", "Motors are OFF. Please turn them ON before moving the robot.")
            return
        self.ui_FMRRMainWindow.clbk_ApproachPoint(NewRobotConfiguration, target_time)

    def clbk_StartMoveRobotManually(self):
        self.pushButton_StartMoveRobotManually.enablePushButton(0)
        self.pushButton_StopMoveRobotManually.enablePushButton(1)
        self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(9, self.ui_FMRRMainWindow.ROS.admittance_controller)
        
    def clbk_StopMoveRobotManually(self):
        self.pushButton_StartMoveRobotManually.enablePushButton(1)
        self.pushButton_StopMoveRobotManually.enablePushButton(0)
        self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(8, self.ui_FMRRMainWindow.ROS.trajectory_controller_name)
        pass
    
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
        self.ui_FMRRMainWindow.ROS.perform_homing()
        # Create an empty file under /tmp
        with open('/tmp/relative_homing_performed', 'w') as f:
            f.write("homing performed")
            f.close()       

    def clbk_BtnGOtoTraining(self):
        if self.ui_FMRRMainWindow.ROS.current_controller != self.ui_FMRRMainWindow.ROS.trajectory_controller_name:
            print(f"Switching to position mode of operation and loading {self.ui_FMRRMainWindow.ROS.trajectory_controller_name}")
            if not self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(8, self.ui_FMRRMainWindow.ROS.trajectory_controller_name):
                QMessageBox.warning(self.DialogRobotWindow, "Warning", "Failed to switch to trajectory controller. Please check the controller configuration.")
                return
        self.DialogFMRRMainWindow.show()
        self.DialogRobotWindow.hide()
             
    def clbk_BtnGotoMovement(self):
        if self.ui_FMRRMainWindow.ROS.current_controller != self.ui_FMRRMainWindow.ROS.trajectory_controller_name:
            print(f"Switching to position mode of operation and loading {self.ui_FMRRMainWindow.ROS.trajectory_controller_name}")
            if not self.ui_FMRRMainWindow.ROS.controller_and_op_mode_switch(8, self.ui_FMRRMainWindow.ROS.trajectory_controller_name):
                QMessageBox.warning(self.DialogRobotWindow, "Warning", "Failed to switch to trajectory controller. Please check the controller configuration.")
                return
        if self.ui_FMRRMainWindow.ROS.are_motors_on:
            QMessageBox.warning(self.DialogRobotWindow, "Warning", "Motors are still active. Please turn them off before exiting this window!")
            return
        self.ui_FMRRMainWindow.DialogMovementWindow.show()
        self.DialogRobotWindow.hide()

    ##############################################################################################
    #####                                                                                    #####
    #####                               UPDATE FUNCTION                                      #####
    #####                                                                                    #####
    #######################à######################################################################  
    def updateRobotWindow(self, RobotWindow: QtWidgets.QDialog):
        if self.ui_FMRRMainWindow.ROS_active:
    #       Joint and tool postions are converted and displayed 
    #       conversion factors (fromm RAD to Degree and from meters to centimeteres are declared at the beginning od the FMRRMainProgram class    
    #       Joints lcd 
            _jointPosConvFact = self.ui_FMRRMainWindow._jointPosConvFact #
            try:
                RobotJointPosition = self.ui_FMRRMainWindow.ROS.RobotJointPosition
            except:
                return
            RobotJointPosition = [Ji_position * _jointPosConvFact for Ji_position in RobotJointPosition]
            
            ##       Handle lcd 
            _toolPosCovFact = self.ui_FMRRMainWindow._toolPosCovFact
            try:
                HandlePosition = self.ui_FMRRMainWindow.ROS.HandlePosition
            except:
                return
            HandlePosition[:] = [i_position * _toolPosCovFact for i_position in HandlePosition]

            CurrentDisplayedHandlePos = [self.lcdNumber_X, self.lcdNumber_Y, self.lcdNumber_Z]
            for iCoord in range(0,3):
                CurrentDisplayedHandlePos[iCoord].display( int(HandlePosition[iCoord]) )         

            # Goal Frame settings
            file_path = '/tmp/relative_homing_performed'
            if os.path.exists(file_path):
                self.frame_GoalPosition.setEnabled(True)
            else:
                self.lineEdit_MoveRobotPosition_GoalPosition.setText( "Goal Position (!!! DEFINE A ZERO!!!)" ) # type: ignore
                self.frame_GoalPosition.setEnabled(False)

            self.frame_JOG.setEnabled(self.ui_FMRRMainWindow.ROS.enable_jog_buttons)
            self.frame_ManualGuide.setEnabled(self.ui_FMRRMainWindow.ROS.enable_manual_guidance)
            self.frame_Homing.setEnabled(self.ui_FMRRMainWindow.ROS.enable_zeroing)
            self.frame_GoalPosition.setEnabled(self.ui_FMRRMainWindow.ROS.enable_ptp)
            self.pushButton_ResetFaults.setEnabled(self.ui_FMRRMainWindow.ROS.manual_reset_faults)
            self.pushButton_StartMotors.setEnabled(not self.ui_FMRRMainWindow.ROS.are_motors_on)
            self.pushButton_StopMotors.setEnabled(self.ui_FMRRMainWindow.ROS.are_motors_on)

            if self.ui_FMRRMainWindow.ROS.is_in_fault_state:
                self.frame_FaultDetected.setStyleSheet("background-color: red; border-radius: 10px;")
            else:
                self.frame_FaultDetected.setStyleSheet("background-color: green; border-radius: 10px;")

    def disconnect_ROS_callbacks(self):
        try:
            self.pushButton_JOG.toggled.disconnect(self.ui_FMRRMainWindow.ROS.jog_enable)
            self.pushButton_ManualGuide.toggled.disconnect(self.ui_FMRRMainWindow.ROS.manual_guidance_enable)
            self.radioButton_EnableHoming.clicked.disconnect(self.ui_FMRRMainWindow.ROS.zeroing_enable)

            self.comboBox_ResetFaults.currentIndexChanged.disconnect(self.ui_FMRRMainWindow.ROS.reset_mode_changed)

            self.pushButton_Xminus.pressed.disconnect()
            self.pushButton_Xplus.pressed.disconnect()
            self.pushButton_Yminus.pressed.disconnect()
            self.pushButton_Yplus.pressed.disconnect()
            self.pushButton_Zminus.pressed.disconnect()
            self.pushButton_Zplus.pressed.disconnect()

            self.pushButton_Xminus.released.disconnect()
            self.pushButton_Xplus.released.disconnect()
            self.pushButton_Yminus.released.disconnect()
            self.pushButton_Yplus.released.disconnect()
            self.pushButton_Zminus.released.disconnect()
            self.pushButton_Zplus.released.disconnect()
            
            print("🔌 Signal-slot ROS disconnected")
        except Exception as e:
            print(f"⚠️ Error during disconnect_ROS_callbacks: {e}")

    def reconnect_ROS_callbacks(self):
        self.pushButton_Xminus.enablePushButton(1)
        self.pushButton_Yminus.enablePushButton(1)
        self.pushButton_Zminus.enablePushButton(1)
        self.pushButton_Xplus.enablePushButton(1)
        self.pushButton_Yplus.enablePushButton(1)
        self.pushButton_Zplus.enablePushButton(1)
        self.pushButton_GOtoTraining.enablePushButton(1)
        self.pushButton_GOtoMovement.enablePushButton(1)
        self.pushButton_StartMoveRobotManually.enablePushButton(1)
        self.pushButton_StartMotors.enablePushButton(1)
        self.pushButton_ResetFaults.enablePushButton(1)

        # DISABLE
        self.pushButton_StopMoveRobotManually.enablePushButton(0)
        self.pushButton_StopMotors.enablePushButton(0)
        self.pushButton_Approach_Joint1.enablePushButton(1)
        self.pushButton_Approach_Joint2.enablePushButton(1)
        self.pushButton_Approach_Joint3.enablePushButton(1)
        self.pushButton_ApproachAllJoint.enablePushButton(1)

        # CALLBACKS
        self.pushButton_RelativeHoming.clicked.connect(self.clbk_BtnRelativeHoming)
        self.pushButton_StartMoveRobotManually.clicked.connect(self.clbk_StartMoveRobotManually)
        self.pushButton_StopMoveRobotManually.clicked.connect(self.clbk_StopMoveRobotManually)
        self.pushButton_StartMotors.clicked.connect(self.ui_FMRRMainWindow.clbk_StartMotors)
        self.pushButton_StopMotors.clicked.connect(self.ui_FMRRMainWindow.clbk_StopMotors)
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
        self.pushButton_Xminus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 0))
        self.pushButton_Xplus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 0))
        self.pushButton_Yminus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 1))
        self.pushButton_Yplus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 1))
        self.pushButton_Zminus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 2))
        self.pushButton_Zplus.released.connect(lambda: self.ui_FMRRMainWindow.ROS.jog_command(0, 2))
        self.comboBox_ResetFaults.currentIndexChanged.connect(self.ui_FMRRMainWindow.ROS.reset_mode_changed)
        self.pushButton_ResetFaults.clicked.connect(self.ui_FMRRMainWindow.clbk_BtnResetFaults)
        self.pushButton_Approach_Joint1.clicked.connect(lambda: self.clbk_JointApproach(0))
        self.pushButton_Approach_Joint2.clicked.connect(lambda: self.clbk_JointApproach(1))
        self.pushButton_Approach_Joint3.clicked.connect(lambda: self.clbk_JointApproach(2))
        self.pushButton_ApproachAllJoint.clicked.connect(lambda: self.clbk_JointApproach(3))
        self.pushButton_GOtoTraining.clicked.connect(lambda: self.clbk_BtnGOtoTraining())
        self.pushButton_GOtoMovement.clicked.connect(lambda: self.clbk_BtnGotoMovement())

    def setupUi_RobotWindow(self, RobotWindow):
        Ui_RobotWindow.setupUi(self, RobotWindow)
                        
    def retranslateUi_RobotWindow(self, RobotWindow):
        _translate = QtCore.QCoreApplication.translate
        RobotWindow.setWindowTitle(_translate("RobotWindow", "Movement Window"))
        
        LcdWidgets  = [ self.lcdNumber_X, self.lcdNumber_Y, self.lcdNumber_Z]

        for iWidget in LcdWidgets:
            iWidget.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
            iWidget.setDigitCount(4)
        

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

        self.pushButton_Xminus.enablePushButton(1)
        self.pushButton_Yminus.enablePushButton(1)
        self.pushButton_Zminus.enablePushButton(1)

        self.pushButton_Xplus.enablePushButton(1) 
        self.pushButton_Yplus.enablePushButton(1)
        self.pushButton_Zplus.enablePushButton(1)
        
        self.pushButton_GOtoTraining.enablePushButton(1)
        self.pushButton_GOtoMovement.enablePushButton(1)
        self.pushButton_StartMoveRobotManually.enablePushButton(1)
        self.pushButton_StartMotors.enablePushButton(1)
        self.pushButton_ResetFaults.enablePushButton(1)
            
# DISABLE        
        self.pushButton_StopMoveRobotManually.enablePushButton(0)
        self.pushButton_StopMotors.enablePushButton(0)
        
        self.pushButton_Approach_Joint1.enablePushButton(1)
        self.pushButton_Approach_Joint2.enablePushButton(1)
        self.pushButton_Approach_Joint3.enablePushButton(1)
        # self.pushButton_Approach_Joint4.enablePushButton(0)
        # self.pushButton_Approach_Joint5.enablePushButton(0)
        # self.pushButton_Approach_Joint6.enablePushButton(0)
        self.pushButton_ApproachAllJoint.enablePushButton(1)

        ##############################################################################################
        #####                                                                                    #####  
        #####                         MOVE THE ROBOT TO DEFINED POSITION                         ##### 
        #####                                      Connections                                   #####
        #####                                                                                    #####
        ##############################################################################################
        self.pushButton_RelativeHoming.clicked.connect(self.clbk_BtnRelativeHoming)
        self.pushButton_StartMoveRobotManually.clicked.connect(self.clbk_StartMoveRobotManually)
        self.pushButton_StopMoveRobotManually.clicked.connect(self.clbk_StopMoveRobotManually)
        self.pushButton_StartMotors.clicked.connect(self.ui_FMRRMainWindow.clbk_StartMotors)
        self.pushButton_StopMotors.clicked.connect(self.ui_FMRRMainWindow.clbk_StopMotors)

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
