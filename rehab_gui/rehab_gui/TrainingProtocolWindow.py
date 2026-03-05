import yaml
import time
import numpy as np
from yaml.loader import SafeLoader

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QProgressBar, QSpinBox, QLCDNumber, QComboBox, QMessageBox, QWidget, QButtonGroup
from PyQt5.QtCore import QTimer, pyqtSignal


from rich.traceback import install
install(show_locals=True)

from ui.uiTrainingProtocolWindow import Ui_TrainingProtocolWindow
from RosCommunicationManager import RosCommunicationManager
from copy import deepcopy

#########################################################################
##
##
##
##
##
#########################################################################
class TrainingProtocolWindow(QtWidgets.QDialog):
    _update_window_period = 100
    turn_off_motors_signal = pyqtSignal(int)
    ROS : RosCommunicationManager = None #type: ignore

    def __init__(self, ui_main) -> None:
        super().__init__()
        self.ui = Ui_TrainingProtocolWindow()
        self.ui.setupUi(self)
        self.ui_main = ui_main
        
        # self.FIRST_TIME = True
        self.Training_ON = False
        self.NumberExecMovements = 0
        self.TotalTrainingTime = 0
        self.ActualTrainingTime = 0
        self.execution_time_percentage = 0
        self.single_movement_time = 0

        self._counter_request_homing_procedure = 0 # counter to avoid multiple request of homing procedure
        self.ProtocolData = None

        #   GENERAL
        self.progressBarPhases = [QProgressBar() for _ in range(20)] 
        self.spinBoxSpeedOvr =  [self.ui.spinBoxPercPhase01, self.ui.spinBoxPercPhase02, self.ui.spinBoxPercPhase03, self.ui.spinBoxPercPhase04,
                                 self.ui.spinBoxPercPhase05, self.ui.spinBoxPercPhase06, self.ui.spinBoxPercPhase07, self.ui.spinBoxPercPhase08,
                                 self.ui.spinBoxPercPhase09, self.ui.spinBoxPercPhase10, self.ui.spinBoxPercPhase11, self.ui.spinBoxPercPhase12,
                                 self.ui.spinBoxPercPhase13, self.ui.spinBoxPercPhase14, self.ui.spinBoxPercPhase15, self.ui.spinBoxPercPhase16,
                                 self.ui.spinBoxPercPhase17, self.ui.spinBoxPercPhase18, self.ui.spinBoxPercPhase19, self.ui.spinBoxPercPhase20]
        self.spinBoxDuration =  [ self.ui.spinBoxDurationPhase01, self.ui.spinBoxDurationPhase02, self.ui.spinBoxDurationPhase03, self.ui.spinBoxDurationPhase04,
                                 self.ui.spinBoxDurationPhase05, self.ui.spinBoxDurationPhase06, self.ui.spinBoxDurationPhase07, self.ui.spinBoxDurationPhase08,
                                 self.ui.spinBoxDurationPhase09, self.ui.spinBoxDurationPhase10, self.ui.spinBoxDurationPhase11, self.ui.spinBoxDurationPhase12,
                                 self.ui.spinBoxDurationPhase13, self.ui.spinBoxDurationPhase14, self.ui.spinBoxDurationPhase15, self.ui.spinBoxDurationPhase16,
                                 self.ui.spinBoxDurationPhase17, self.ui.spinBoxDurationPhase18, self.ui.spinBoxDurationPhase19, self.ui.spinBoxDurationPhase20]
        self.lcdNumberPhases =  [QLCDNumber() for _ in range(20)] 
        self.comboBoxPhases =  [QComboBox() for _ in range(20)] 
        for widget in self.findChildren(QWidget):
            NameStr = widget.objectName()
            lenstr= len(NameStr)
            if NameStr[lenstr-7:lenstr-2] == 'Phase':
                WidgetItem = int(NameStr[-2:])-1            
                if isinstance(widget, QtWidgets.QProgressBar):
                    self.progressBarPhases[WidgetItem] = widget
                    self.progressBarPhases[WidgetItem].setValue(0) # type: ignore
                if isinstance(widget, QtWidgets.QLCDNumber):
                    self.lcdNumberPhases[WidgetItem] = widget
                if isinstance(widget, QtWidgets.QComboBox):
                    self.comboBoxPhases[WidgetItem] = widget
        for WidgetItem in range(0,20):
            self.lcdNumberPhases[WidgetItem].setSegmentStyle(QtWidgets.QLCDNumber.Flat) # type: ignore

        for WidgetItem in range(0,20):
            self.spinBoxDuration[WidgetItem].setValue(60) # type: ignore

        #   ENABLE Buttons      
        #   DISABLE Buttons
        self.ui.pushButton_STARTtrainig.setCheckable(True)
        self.ui.pushButton_STARTtrainig.setEnabled(False)
        self.ui.pushButton_STARTtrainig.setStyleSheet("")
        self.ui.pushButton_PauseTrainig.setEnabled(False)
        self.ui.pushButton_PauseTrainig.setStyleSheet("")
        self.ui.pushButton_ResumeTraining.setEnabled(False)
        self.ui.pushButton_ResumeTraining.setStyleSheet("")
        #self.ui.pushButton_StopMotors.setEnabled(False)
        #   CALLBACKS 
        #    Buttons
        self.ui.pushButton_LoadCreateProtocol.clicked.connect(self.clbk_LoadCreateProtocol)
        self.ui.pushButton_SaveProtocol.clicked.connect(self.clbk_SaveProtocol)
        self.ui.pushButton_STARTtrainig.toggled.connect(self.startStopTraining)
        self.ui.pushButton_PauseTrainig.pressed.connect(self.clbk_PauseTrainig)
        self.ui.pushButton_ResumeTraining.pressed.connect(self.clbk_ResumeTrainig)
        #    CAHNGES
        self.ui.lcdNumberExerciseTotalTime.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        self.ui.lcdNumber_MovementCOUNT.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        
        #   RADIO BUTTONS - Mode Selection (user controlled via callback)
        self.modeButtonGroup = QButtonGroup()
        self.modeButtonGroup.addButton(self.ui.radioButton_RehabMode, 0)
        self.modeButtonGroup.addButton(self.ui.radioButton_EEGMode, 1)
        self.ui.radioButton_RehabMode.toggled.connect(self.clbk_ModeChanged)
        self.ui.radioButton_EEGMode.toggled.connect(self.clbk_ModeChanged)
        self.EEGModeEnabled = False
        
        #   RADIO BUTTONS - Side Selection (program controlled, no callback)
        self.sideButtonGroup = QButtonGroup()
        self.sideButtonGroup.addButton(self.ui.radioButton_SideLeft, 0)
        self.sideButtonGroup.addButton(self.ui.radioButton_SideRight, 1)
        
        #   RADIO BUTTONS - Type of Exercise Selection (program controlled, no callback)
        self.typeOfExerciseButtonGroup = QButtonGroup()
        self.typeOfExerciseButtonGroup.addButton(self.ui.radioButton_TypeOfExercise_Reaching, 0)
        self.typeOfExerciseButtonGroup.addButton(self.ui.radioButton_TypeOfExercise_HandtoMouth, 1)

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS = ROS
        self.parent_timer = parent_timer
        self.parent_timer.timeout.connect(self.updateWindow)
    
    def updateWindow(self):
        if self.ui_main.movement_loaded and not self.Training_ON:
            self.ui.pushButton_LoadCreateProtocol.setEnabled(True)
        else:
            self.ui.pushButton_LoadCreateProtocol.setEnabled(False)

        if self.ui_main.movement_loaded and self.ProtocolData is not None:
            self.ui.pushButton_STARTtrainig.setEnabled(True)
            self.ui.pushButton_STARTtrainig.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
        else:
            self.ui.pushButton_STARTtrainig.setEnabled(False)
            self.ui.pushButton_STARTtrainig.setStyleSheet("")

        # Enable Save button only if a protocol has been loaded
        if self.ProtocolData is not None:
            self.ui.pushButton_SaveProtocol.setEnabled(True)
        else:
            self.ui.pushButton_SaveProtocol.setEnabled(False)

        if self.ui_main.rehabMovementWindow.SideOfMovement == 1: #Left
            self.ui.radioButton_SideLeft.blockSignals(True)
            self.ui.radioButton_SideRight.blockSignals(True)
            self.ui.radioButton_SideLeft.setChecked(True)
            self.ui.radioButton_SideLeft.blockSignals(False)
            self.ui.radioButton_SideRight.blockSignals(False)
        elif self.ui_main.rehabMovementWindow.SideOfMovement == 2: #Right
            self.ui.radioButton_SideLeft.blockSignals(True)
            self.ui.radioButton_SideRight.blockSignals(True)
            self.ui.radioButton_SideRight.setChecked(True)
            self.ui.radioButton_SideLeft.blockSignals(False)
            self.ui.radioButton_SideRight.blockSignals(False)
        else:
            self.ui.radioButton_SideLeft.blockSignals(True)
            self.ui.radioButton_SideRight.blockSignals(True)
            self.ui.radioButton_SideLeft.setChecked(False)
            self.ui.radioButton_SideRight.setChecked(False)
            self.ui.radioButton_SideLeft.blockSignals(False)
            self.ui.radioButton_SideRight.blockSignals(False)
        
        if self.ui_main.rehabMovementWindow.TypeOfMovement == 1: # Reaching 
            self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(True)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(True)
            self.ui.radioButton_TypeOfExercise_Reaching.setChecked(True)
            self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(False)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(False)

            # Set Mode of Operation to 2 to activate proximity sensor
            self.ROS.setModeOfOperation(2)

        elif self.ui_main.rehabMovementWindow.TypeOfMovement == 2: # HtMM
            self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(True)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(True)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.setChecked(True)
            self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(False)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(False)

            # Set Mode of Operation to 1 to activate switch sensor
            self.ROS.setModeOfOperation(1)

        else:
            self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(True)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(True)
            self.ui.radioButton_TypeOfExercise_Reaching.setChecked(False)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.setChecked(False)
            self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(False)
            self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(False)

            self.ROS.setModeOfOperation(0) # Default mode, no sensor activated

        self.ui.lcdNumber_MovementCOUNT.display( self.NumberExecMovements ) 
        self.ui.lcdNumberExerciseTotalTime.display( np.floor((self.TotalTrainingTime - self.ActualTrainingTime)/60) )

        if self.Training_ON:
            _iPhase = self.ROS.getExerciseRepetitionCounter()
            if _iPhase <= 19:
                self.progressBarPhases[_iPhase].setValue(self.ROS.getExecutionTimePercentage())
                if self.ROS.getExerciseRepetitionCounter() > 0:
                    self.progressBarPhases[_iPhase - 1].setValue(100) # type: ignore
                if self.ROS.getExerciseCompleted():
                    self.ROS.setExerciseCompleted(False)
                    self.ModalityActualValue = self.Modalities[_iPhase] # change here the modality 
                    self.NumberExecMovements += 1
                    print(f'Number of movements: {self.NumberExecMovements} - Ovr: {self.spinBoxSpeedOvr[_iPhase].value()}')
                    self.spinBoxSpeedOvr[_iPhase].enabled = False
            else:
                self.progressBarPhases[19].setValue(100)
                self.stopTrainig()

    def clbk_ModeChanged(self):
        """Callback for mode selection radio buttons (RehabMode / EEGMode)"""
        if self.ui.radioButton_RehabMode.isChecked():
            print("Rehab Mode selected")
            self.ui.radioButton_EEGMode.setChecked(False)
            self.EEGModeEnabled = False
        elif self.ui.radioButton_EEGMode.isChecked():
            print("EEG Mode selected")
            self.ui.radioButton_RehabMode.setChecked(False)
            self.EEGModeEnabled = True

    def clbk_LoadCreateProtocol(self):
        filename = QtWidgets.QFileDialog.getOpenFileName(None, "Load Protocol", self.ui_main.FMRR_Paths['Protocols'] , "*.yaml")
        if bool(filename[0]):
            # load data  
            self.ProtocolData = yaml.load(open(filename [0]), Loader=SafeLoader) # type: ignore
            # get values       
            self.PhaseIsEnabled = self.ProtocolData["Phases"].get('PhaseIsEnabled')[0]
            self.NrEnabledPhases = sum( self.PhaseIsEnabled ) #Sistemare se non si usa
            self.TotalTrainingTime =  self.ui_main.PhaseDuration * self.NrEnabledPhases
            self.Modalities = self.ProtocolData["Phases"].get('Modalities')[0]
            self.Percentage = self.ProtocolData["Phases"].get('Percentage')[0]
            # Load durations if available (for backward compatibility with old protocols)
            duration_data = self.ProtocolData["Phases"].get('Duration')
            self.Durations = duration_data[0] if duration_data else None
            self.ui.lcdNumber_SinglePhaseDuration.display(np.floor(self.ui_main.PhaseDuration))
            self.ui.lcdNumberExerciseTotalTime.display( np.floor(self.TotalTrainingTime/60) )
            self.ui.lcdNumber_MaxVel.display(np.floor(self.ui_main.Vmax))
            
            for iPhase in range(20):
                self.lcdNumberPhases[iPhase].setNumDigits(3) # type: ignore
                iPhaseVel = int( float(self.Percentage[iPhase]) /100 * self.ui_main.Vmax )
                self.lcdNumberPhases[iPhase].display( iPhaseVel ) # type: ignore
                self.spinBoxSpeedOvr[iPhase].setValue(self.Percentage[iPhase]) # type: ignore
                # Load duration if available in YAML
                if self.Durations is not None:
                    self.spinBoxDuration[iPhase].setValue(self.Durations[iPhase]) # type: ignore
                    
            for iProgressBar in self.progressBarPhases:
                iProgressBar.setValue(0) # type: ignore
                
    def clbk_SaveProtocol(self):
        filename = QtWidgets.QFileDialog.getSaveFileName(None, "Save Protocol", self.ui_main.FMRR_Paths['Protocols'], "*.yaml")
        if bool(filename[0]):
            # Gather data from GUI
            speed_percentages = [sp.value() for sp in self.spinBoxSpeedOvr]
            durations = [sp.value() for sp in self.spinBoxDuration]
            
            # Create protocol structure (inverse of clbk_LoadCreateProtocol)
            protocol_data = {
                'Phases': {
                    'PhaseIsEnabled': [self.PhaseIsEnabled],
                    'Percentage': [speed_percentages],
                    'Duration': [durations],
                    'Modalities': [self.Modalities]
                },
                'V_max': [self.ui_main.Vmax],
                'PhaseDuration': [self.ui_main.PhaseDuration]
            }
            
            # Save to YAML file
            with open(filename[0], 'w') as file:
                yaml.dump(protocol_data, file, default_flow_style=None)
            
    def startStopTraining(self, start: bool):
        if start:
            self.ui.pushButton_PauseTrainig.setEnabled(True)
            self.ui.pushButton_STARTtrainig.setStyleSheet("background-color: rgb(255, 69, 0); color: black;")
            self.ui.pushButton_STARTtrainig.setText("STOP TRAINING")
            if not self.startTrainig():
                self.ui.pushButton_STARTtrainig.setChecked(False)
        else:
            self.stopTrainig()
            self.ui.pushButton_STARTtrainig.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
            self.ui.pushButton_STARTtrainig.setText("START TRAINING")
            self.ui.pushButton_PauseTrainig.setEnabled(False)
            self.ui.pushButton_ResumeTraining.setEnabled(False)

    def startTrainig(self) -> bool:
        if not self.ROS.isRosCommunicationActive() or self.ROS.isInFaultState():
            QMessageBox.warning(self, "Warning", "No communication is active or the robot is in FAULT state.")
            return False
        
        HandlePosition = self.ROS.getHandleFeedbackPosition()
        if any([abs(HandlePosition[idx]) > 3e-3 for idx in range(3)]):
            print(f"[Movement Program] Handle position is not zero, detected states: [{HandlePosition[0]}, {HandlePosition[1]}, {HandlePosition[2]}]")
            QMessageBox.warning(self, "Warning", "Handle position is not zero, please set it to zero before going to training")
            return False

        self.ROS.setManualMode(False)
        self.ROS.enableControllerBehaviour("FCT")
        self.Training_ON = True
        self.ActualTrainingTime = 0
        self.sendExercise()
        self.ModalityActualValue = self.Modalities[0]

        for speedSpinBox in self.spinBoxSpeedOvr:
            speedSpinBox.setEnabled(False)
        for durationSpinBox in self.spinBoxDuration:
            durationSpinBox.setEnabled(False)

        return True
        
    def clbk_PauseTrainig(self):
        self.ROS.triggerSoftMovementStart(amplitude=0.0, time_constant=0.2, target='speed_ovr')
        self.ui.pushButton_ResumeTraining.setEnabled(True)
    
    def clbk_ResumeTrainig(self):
        self.ROS.triggerSoftMovementStop()
        self.ui.pushButton_PauseTrainig.setEnabled(True)

    def stopTrainig(self):
        self.Training_ON = False
        self.ROS.stopAnyMovement()
        self.ROS.triggerSoftMovementStop()
        for iProgBar in self.progressBarPhases:
            iProgBar.setValue(0) 
        for speedSpinBox in self.spinBoxSpeedOvr:
            speedSpinBox.setEnabled(True)
        for durationSpinBox in self.spinBoxDuration:
            durationSpinBox.setEnabled(True)
        self.ProtocolData = None

    def sendExercise(self):
        if self.ROS.isRosCommunicationActive():
            TrjYamlData = self.ui_main.rehabMovementWindow.TrjYamlData
            self.CartesianPositions = TrjYamlData.get("cart_trj3").get("cart_positions")
            self.TimeFromStart = TrjYamlData.get("cart_trj3").get("time_from_start")
            if not self.ROS.turnOnMotors():
                return
            self.ROS.setExercise(self.CartesianPositions, self.TimeFromStart, [ sp.value() for sp in self.spinBoxSpeedOvr], [ sp.value() for sp in self.spinBoxDuration], self.EEGModeEnabled)
        else:
            QMessageBox.warning(self, "Warning", "Check the state of the driver - No communication is active.")
            
        
def main(args=None):
    pass

if __name__ == "__main__":
    main()