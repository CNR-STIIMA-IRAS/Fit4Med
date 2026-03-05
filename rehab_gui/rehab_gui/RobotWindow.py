import os
import sys
from PyQt5 import QtWidgets, QtWidgets
from PyQt5.QtWidgets import QMessageBox, QPushButton, QProgressBar, QWidget, QSizePolicy
from PyQt5.QtCore import QTimer, QObject, Qt, pyqtSignal, QThread
from ui.uiRobotWindow import Ui_RobotWindow
from ui.uiFMRRMainWindow import Ui_FMRRMainWindow
from RosCommunicationManager import RosCommunicationManager
from copy import deepcopy
import time

class Worker(QObject):
    
    def __init__(self, progress_dialog : QProgressBar):
        super().__init__()
        self.stop_thread : bool = False
        self.progress_dialog = progress_dialog
        
    def run(self):
        while not self.stop_thread:
            time.sleep(0.1)
            self.progress_dialog.setValue(self.progress_dialog.value()+1)
        
        self.progress_dialog.setValue(100)

class ProgressBarWorker(QObject):

    def __init__(self, progress_bar : QProgressBar):
        super().__init__()
        self.progress_dialog = progress_bar
        self.progress_dialog.setVisible(True)
        self.progress_dialog.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.progress_dialog.setValue(0)
        self.progress_dialog.hide()

        self.worker_thread = QThread()
        self.worker = Worker(self.progress_dialog)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.run) 

    def start(self):
        self.worker.stop_thread = False
        self.worker_thread.start()
        self.progress_dialog.show()

    def stop(self):
        self.worker.stop_thread = True
        self.worker_thread.quit()
        self.worker_thread.wait()
        self.progress_dialog.close()

class RobotWindow(QtWidgets.QDialog):
    _progres_dialog_finished = pyqtSignal(bool)
    _update_window_period = 100
    ROS : RosCommunicationManager = None #type: ignore

    def __init__(self, main_app) -> None:
        super().__init__()
        self.ui = Ui_RobotWindow()
        self.ui.setupUi(self)
        self.main_app = main_app
        self.ui.comboBox_MOO.addItems(['', 'Zeroing', 'JOG', 'Manual Guidance', 'PTP'])
        self.ui.progressBar_RelativeHoming.setVisible(False)

    def handleButtonCallbackFailure(self, pb : QPushButton, callback, error_msg : str) -> None:
        previous_state = pb.isChecked()  # Save the previous state
        try:
            # Call the boolean function
            success = callback()
            if not success:
                QMessageBox.warning(self, "Error", error_msg)
            # If successful, toggle the button's checked state
            pb.setChecked(not previous_state)
        except Exception as e:
            # Restore previous state if there was an error
            pb.setChecked(previous_state)
            QMessageBox.warning(self, "Exception", str(e))

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS = ROS
        self.parent_timer = parent_timer
        self.parent_timer.timeout.connect(self.updateWindow)

        # Collega i segnali di cambiamento a funzioni di callback
        self.ui.comboBox_MOO.currentIndexChanged.connect(self.onBehaviourOptionChanged)
        self.ui.comboBox_MOO.activated.connect(self.onBehaviourActivation) # type: ignore
        
        self.ui.pushButton_RelativeHoming.clicked.connect(lambda :self.handleButtonCallbackFailure(self.ui.pushButton_RelativeHoming, self.relativeHoming, "Homing failed...")) # type: ignore
        self.ui.pushButton_RelativeHoming.setStyleSheet("")

        self.ui.pushButton_MoveRobotManually.setCheckable(True)
        self.ui.pushButton_MoveRobotManually.toggled.connect(self.moveRobotManually) # type: ignore
        self.ui.pushButton_MoveRobotManually.setStyleSheet("")
        self.ui.pushButton_MoveRobotManually.setText("Activate Manual Guidance")

        self.ui.pushButton_JOG.setCheckable(True)
        self.ui.pushButton_JOG.toggled.connect(self.moveJOG)
        self.ui.pushButton_JOG.setStyleSheet("")
        self.ui.pushButton_JOG.setText("Activate JOG")
        
        self.ui.pushButton_Xminus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = -1))
        self.ui.pushButton_Xminus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = 0))
        self.ui.pushButton_Xplus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = 1))
        self.ui.pushButton_Xplus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=0, direction = 0))

        self.ui.pushButton_Yminus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = -1))
        self.ui.pushButton_Yminus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = 0))
        self.ui.pushButton_Yplus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = 1))
        self.ui.pushButton_Yplus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=1, direction = 0))

        self.ui.pushButton_Zminus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = -1))
        self.ui.pushButton_Zminus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = 0))
        self.ui.pushButton_Zplus.pressed.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = 1))
        self.ui.pushButton_Zplus.released.connect(lambda : self.ROS.toogleJoggingBehaviour(axis=2, direction = 0))

        self.ui.pushButton_ApproachAllJoint.setCheckable(True)
        self.ui.pushButton_ApproachAllJoint.toggled.connect(self.goTo)

        self.ui.pushButton_SensorBias.setCheckable(False)
        self.ui.pushButton_SensorBias.clicked.connect(lambda: self.ROS.SonarBias())

    def onBehaviourOptionChanged(self, index):
        # Handle the change in MOO option here
        selected_option = self.ui.comboBox_MOO.itemText(index)
        print(f"MOO option changed to: {selected_option}")
    
    def onBehaviourActivation(self, index):
        print(f"MOO option activated to: {index}")
        try:
            if index == 1:  # Zeroing
                self.ROS.enableControllerBehaviour("Homing")
            elif index == 2:  # JOG
                self.ROS.setManualMode(True)
                self.ROS.enableControllerBehaviour("Jogging")
            elif index == 3:  # Manual Guidance
                self.ROS.setManualMode(True)
                self.ROS.enableControllerBehaviour("ManualGuidance")
            elif index == 4:  # PTP
                self.ROS.setManualMode(False)
                ActualRobotConfiguration = deepcopy( self.ROS.getHandleFeedbackPosition())
                self.ui.doubleSpin_Joint1_Value.setValue(ActualRobotConfiguration[0])
                self.ui.doubleSpin_Joint2_Value.setValue(ActualRobotConfiguration[1])
                self.ui.doubleSpin_Joint3_Value.setValue(ActualRobotConfiguration[2])
                self.ROS.enableControllerBehaviour("PTP")
        except Exception as e:
            print(f'Exception: {e}')
        
    ###### Callback of buttons to Joint Approach with trajectory controller (TODO)
    def goTo(self, toggled) -> None:
        print(f'toogled the goTo event with toggled value {toggled}')
        if toggled == 0:
            self.ROS.stopAnyMovement()
            return
        
        self.ROS.setManualMode(True)
        time.sleep(0.5)

        ActualRobotConfiguration  = deepcopy( self.ROS.getHandleFeedbackPosition())
        NewRobotConfiguration = ActualRobotConfiguration
        JointTargetPosition = (float(self.ui.doubleSpin_Joint1_Value.value()), float(self.ui.doubleSpin_Joint2_Value.value()), float(self.ui.doubleSpin_Joint3_Value.value()))
        if any(abs(JointTargetPosition[idx]) > 0.5 for idx in range(len(self.ROS.getJointNames()))):
            QMessageBox.warning(self, "Warning", f"Joint target position is out of range. Please set a value between -0.5 and 0.5.")
            return
        NewRobotConfiguration = JointTargetPosition
        
        target_time = max(abs(NewRobotConfiguration[i]) for i in range(len(self.ROS.getJointNames())))/0.1
        if target_time < 1.0:
            target_time = 2.0
        
        if self.ROS.turnOnMotors():
            print(f'Go To {NewRobotConfiguration} from {self.ROS.getHandleFeedbackPosition()}')
            self.ROS.sendPTPTrajectory(NewRobotConfiguration, target_time)
        else:
            QMessageBox.warning(self, "Warning", "Failed in switching on the motors")
        
    def moveRobotManually(self, activate):
        if activate:
            if not self.ROS.turnOnMotors():
                self.ui.pushButton_MoveRobotManually.setChecked(False)
        else:
            self.ROS.turnOffMotors()
    
    def moveJOG(self, activate):
        if activate:
            if not self.ROS.turnOnMotors():
                self.ui.pushButton_JOG.setChecked(False)
        else:
            self.ROS.turnOffMotors()

    def relativeHoming(self) -> bool:
        
        ##########
        pd = ProgressBarWorker(self.ui.progressBar_RelativeHoming)
        pd.start()
        if not os.path.exists(os.path.join("/","tmp")):
            # Create the directory
            os.makedirs(os.path.join("/","tmp"))

        file_path = os.path.join("/", "tmp", "absolute_homing_performed")
        if os.path.exists(file_path):
            try:
                print("Removing file: ", file_path)
                os.remove(file_path)
            except Exception as e:
                print(f"Failed to remove {file_path}: {e}")
                pd.stop()
                return False
        file_path = os.path.join("/", "tmp", "relative_homing_performed")
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
            except Exception as e:
                print(f"Failed to remove {file_path}: {e}")
                pd.stop()
                return False
        print(f"Files removed {not os.path.exists(file_path)}")
        # Call homing service
        if not self.ROS.performHoming():
            pd.stop()
            return False
        # Create an empty file under /tmp
        with open(os.path.join("/", "tmp", "relative_homing_performed"), 'w') as f:
            f.write("homing performed")
            f.close()

        pd.stop()
        return True

    ##############################################################################################
    #####                                                                                    #####
    #####                               UPDATE FUNCTION                                      #####
    #####                                                                                    #####
    #######################à######################################################################  
    def updateWindow(self):
        
        self.enableRelativeHomingButton(self.ROS.isHomingEnabled() and self.ui.comboBox_MOO.currentIndex() == 1)
        self.enableJogButton(self.ROS.isJoggingBehaviourEnabled() and self.ui.comboBox_MOO.currentIndex() == 2)
        self.enableManualGuidanceButton(self.ROS.isManualGuidanceBehaviourEnabled() and self.ui.comboBox_MOO.currentIndex() == 3)
        self.enablePTPFrame(self.ROS.isPTPEnabled() and self.ui.comboBox_MOO.currentIndex() == 4)

        if self.main_app.ui.tabWidget.currentIndex() == 0 and\
            self.ROS.areMotorsOn() and self.ROS.getTrajectoryCompleted():
            self.ui.pushButton_ApproachAllJoint.setChecked(False)
            self.ROS.setTrajectoryCompleted(False)
            self.ROS.turnOffMotors()

    def enableRelativeHomingButton(self, activate: bool):
        self.ui.frame_ReativeHoming.setEnabled(activate)
        self.ui.pushButton_RelativeHoming.setEnabled(activate)
        if activate: 
            self.ui.pushButton_RelativeHoming.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
        else:
            self.ui.pushButton_RelativeHoming.setStyleSheet("")

    def enableJogButton(self, activate: bool):
        self.ui.frame_JOG.setEnabled(activate)
        self.ui.pushButton_JOG.setEnabled(activate)
        self.ui.pushButton_Xminus.setEnabled(activate)
        self.ui.pushButton_Xplus.setEnabled(activate)
        self.ui.pushButton_Yminus.setEnabled(activate)
        self.ui.pushButton_Yplus.setEnabled(activate)
        self.ui.pushButton_Zminus.setEnabled(activate)
        self.ui.pushButton_Zplus.setEnabled(activate)
        if activate: 
            if self.ROS.areMotorsOn():
                self.ui.pushButton_JOG.setStyleSheet("background-color: rgb(255, 69, 0); color: black;")
                self.ui.pushButton_JOG.setText("Deactivate Jog")
                self.ui.pushButton_Xminus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Xplus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Yminus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Yplus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Zminus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Zplus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
            else:
                self.ui.pushButton_JOG.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
                self.ui.pushButton_JOG.setText("Activate Jog")
                self.ui.pushButton_Xminus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Xplus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Yminus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Yplus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Zminus.setStyleSheet("background-color: rgb(255,255,224); color: black;")
                self.ui.pushButton_Zplus.setStyleSheet("background-color: rgb(255,255,224); color: black;")

        else:
            self.ui.pushButton_JOG.setStyleSheet("")
            self.ui.pushButton_Xminus.setStyleSheet("")
            self.ui.pushButton_Xplus.setStyleSheet("")
            self.ui.pushButton_Yminus.setStyleSheet("")
            self.ui.pushButton_Yplus.setStyleSheet("")
            self.ui.pushButton_Zminus.setStyleSheet("")
            self.ui.pushButton_Zplus.setStyleSheet("")


    def enableManualGuidanceButton(self, activate: bool):
        self.ui.frame_MoveRobotManually.setEnabled(activate)
        self.ui.pushButton_MoveRobotManually.setEnabled(activate)
        if activate: 
            if self.ROS.areMotorsOn():
                self.ui.pushButton_MoveRobotManually.setStyleSheet("background-color: rgb(255, 69, 0); color: black;")
                self.ui.pushButton_MoveRobotManually.setText("Deactivate Manual Guidance")
            else:
                self.ui.pushButton_MoveRobotManually.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
                self.ui.pushButton_MoveRobotManually.setText("Activate Manual Guidance")
        else:
            self.ui.pushButton_MoveRobotManually.setStyleSheet("")

    def enablePTPFrame(self, activate: bool):
        self.ui.frame_GoalPosition.setEnabled(activate)
        if activate:
            if self.ROS.areMotorsOn():
                self.ui.pushButton_ApproachAllJoint.setStyleSheet("background-color: rgb(255, 69, 0); color: black;")
                self.ui.pushButton_ApproachAllJoint.setText("Interrupt")
            else:
                self.ui.pushButton_ApproachAllJoint.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
                self.ui.pushButton_ApproachAllJoint.setText("GoTo")
        else:
            self.ui.pushButton_ApproachAllJoint.setStyleSheet("")
            self.ui.pushButton_ApproachAllJoint.setText("GoTo")

def main():
    app = QtWidgets.QApplication(sys.argv)
    ui = RobotWindow(None)
    ui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

