# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from GuiRosTasks import Call, gui_task
from YamlSupport import (read_yaml, validate_movement, validate_protocol,
                         atomic_save_yaml, gui_transaction, check_spin_value,
                         number, YamlDataError, report_yaml_error)

import os
import yaml
import time
import numpy as np
try:
    from yaml import CSafeLoader as SafeLoader
except ImportError:
    from yaml import SafeLoader

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QProgressBar, QSpinBox, QLCDNumber, QComboBox, QMessageBox, QWidget, QButtonGroup, QFileDialog, QApplication
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

# rich tracebacks are installed once by session_log (terminal + gui_errors.log).

from ui.uiTrainingProtocolWindow import Ui_TrainingProtocolWindow
from RehabilitationMovementWindow import ExerciseType
from RosCommunicationManager import RosCommunicationManager
from copy import deepcopy

# Who suspended the exercise.
SUSPENDED_BY_ROBOT = 'robot'  # the robot side aborted a repetition (e.g. tracking error)
SUSPENDED_BY_GUI = 'gui'      # the GUI got no progress from the robot for too long

# control_msgs/FollowJointTrajectory result codes -> (state label, explanation)
_ROBOT_SUSPENSION_REASONS = {
    -4: ('TRACKING ERROR', 'Tracking error: the robot deviated from the trajectory beyond the allowed tolerance.'),
    -5: ('TRACKING ERROR', 'Tracking error: the robot did not reach the end of the movement within the allowed tolerance.'),
    -1: ('ROBOT ERROR', 'The robot controller rejected the movement (invalid goal).'),
    -2: ('ROBOT ERROR', 'The robot controller rejected the movement (invalid joints).'),
    -3: ('ROBOT ERROR', 'The robot controller rejected the movement (old header timestamp).'),
}

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
        self._stop_pending = False
        self.Training_ON = False
        self.NumberExecMovements = 0
        self._near_zero_triggered = True
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
        self._min_speed_ovr = 10
        for speed_spin_box in self.spinBoxSpeedOvr:
            speed_spin_box.setMinimum(self._min_speed_ovr)
            if speed_spin_box.value() < self._min_speed_ovr:
                speed_spin_box.setValue(self._min_speed_ovr)
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
            self.spinBoxDuration[WidgetItem].valueChanged.connect(self.clbk_DurationChanged)

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
        self.ui.lcdNumberExerciseTotalTime.setDigitCount(4)
        self.ui.lcdNumber_MovementCOUNT.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
        
        #   RADIO BUTTONS - Mode Selection (user controlled via callback)
        # Two independent toggle buttons: "Save" and "EEG".
        # Neither selected  → default Rehab mode
        # Save selected     → ros2 bag recording on start/stop
        # EEG selected      → EEG sync pulse at movement start
        # Clicking an already-selected button deselects it (non-exclusive group).
        self.modeButtonGroup = QButtonGroup()
        self.modeButtonGroup.setExclusive(False)  # allow deselection by re-click
        self.modeButtonGroup.addButton(self.ui.radioButton_RehabMode, 0)
        self.modeButtonGroup.addButton(self.ui.radioButton_EEGMode, 1)
        self.ui.radioButton_RehabMode.setText("Save")
        self.ui.radioButton_RehabMode.toggled.connect(self.clbk_ModeChanged)
        self.ui.radioButton_EEGMode.toggled.connect(self.clbk_ModeChanged)
        self.EEGModeEnabled = False
        self.SaveModeEnabled = False
        self._bag_recording_active: bool = False
        
        #   RADIO BUTTONS - Side Selection (program controlled, no callback)
        self.sideButtonGroup = QButtonGroup()
        self.sideButtonGroup.addButton(self.ui.radioButton_SideLeft, 0)
        self.sideButtonGroup.addButton(self.ui.radioButton_SideRight, 1)
        
        #   RADIO BUTTONS - Type of Exercise Selection (program controlled, no callback)
        self.typeOfExerciseButtonGroup = QButtonGroup()
        self.typeOfExerciseButtonGroup.addButton(self.ui.radioButton_TypeOfExercise_Reaching, 0)
        self.typeOfExerciseButtonGroup.addButton(self.ui.radioButton_TypeOfExercise_HandtoMouth, 1)

        # Cache for updateWindow to avoid redundant widget updates
        self._last_load_enabled = None
        self._last_start_state = None
        self._last_save_enabled = None
        self._last_side = None
        self._last_type = None
        self._last_movement_count = None
        self._last_total_time_display = None
        self._last_movement_name_text = None

        self._iPhase_0 = 0

        # GUI-side suspension: the robot reports progress at least every ~1 s
        # while moving (phase durations <= 100 s). No change for
        # _NO_PROGRESS_TIMEOUT_S (and no PAUSE) means the robot side stopped
        # talking or moving; then the GUI stops the movement itself. Longer
        # than the robot's retry window for its own notifications (10 s).
        self._NO_PROGRESS_TIMEOUT_S = 15.0
        self._exec_pct_prev: int = -1
        self._exec_pct_changed_at: float = time.monotonic()
        self._training_paused: bool = False
        self._suspension_dialog = None

    def _get_pending_phase_durations(self) -> list:
        return [sp.value() for idx, sp in enumerate(self.spinBoxDuration) if idx >= self._iPhase_0]

    def _get_pending_phase_percentages(self) -> list:
        return [sp.value() for idx, sp in enumerate(self.spinBoxSpeedOvr) if idx >= self._iPhase_0]

    def _update_total_training_time_display(self, force: bool = False) -> None:
        pending_durations = self._get_pending_phase_durations()
        self.TotalTrainingTime = sum(pending_durations)
        total_time_display = np.floor(self.TotalTrainingTime) / 60  # convert seconds to minutes
        if force or total_time_display != self._last_total_time_display:
            self._last_total_time_display = total_time_display
            self.ui.lcdNumberExerciseTotalTime.display(total_time_display)

    def _set_training_buttons_idle(self) -> None:
        # Keep button visuals and logical state aligned without triggering startStopTraining(False).
        self.ui.pushButton_STARTtrainig.blockSignals(True)
        self.ui.pushButton_STARTtrainig.setChecked(False)
        self.ui.pushButton_STARTtrainig.blockSignals(False)
        self.ui.pushButton_STARTtrainig.setStyleSheet("background-color: rgb(85, 255, 127); color: black;")
        self.ui.pushButton_STARTtrainig.setText("START TRAINING")
        self.ui.pushButton_PauseTrainig.setEnabled(False)
        self.ui.pushButton_ResumeTraining.setEnabled(False)

    def _reset_progress_watch(self) -> None:
        self._exec_pct_prev = -1
        self._exec_pct_changed_at = time.monotonic()

    def _handle_exercise_suspension(self, i_phase: int, suspended_by: str, info=None) -> None:
        """Stop the training, keep i_phase for the resume, tell the operator why.

        suspended_by == SUSPENDED_BY_ROBOT: the robot already stopped the motion
        (aborted repetition); only the motors are switched off, asynchronously.
        suspended_by == SUSPENDED_BY_GUI: nothing heard from the robot, which may
        still be moving: the movement is stopped first (stop + motors off).
        Never a blocking ROS call here (it would freeze the whole GUI).
        """
        phase = i_phase + 1  # as shown to the operator
        if suspended_by == SUSPENDED_BY_ROBOT:
            code = info['error_code'] if info else None
            label, reason = _ROBOT_SUSPENSION_REASONS.get(
                code, ('ROBOT ERROR', f'The robot controller aborted the movement (code {code}).'))
            if info is None:
                label, reason = 'ROBOT STOP', 'The movement was stopped on the robot side.'
            elif info.get('message'):
                # Often the most useful part (joint, error size, "controller
                # deactivated", ...): not only in the details.
                reason += f"\nRobot controller: {info['message']}"
        else:
            label = 'NO PROGRESS'
            reason = info['reason']
        print(f"[TrainingProtocol] Suspension at phase {phase} by {suspended_by}: {reason} {info or ''}")
        self.Training_ON = False
        self._near_zero_triggered = False
        self._training_paused = False
        self._reset_progress_watch()
        self._iPhase_0 = i_phase
        self._update_total_training_time_display(force=True)
        self._set_training_buttons_idle()
        self.ROS.setExerciseSuspended(False)
        self.ROS.setMovementStopped(False)
        self.ROS.setExerciseInSuspension(True, label)  # orange warning in MotorsWindow
        self._stop_bag_recording()                      # stop bag if Save mode was active
        if suspended_by == SUSPENDED_BY_GUI:
            # Non-blocking: runs on its own worker, then switches the motors off.
            if not self.ROS.requestStopAnyMovement():
                self.ROS.turnOffMotorsAsync()
        else:
            self.ROS.turnOffMotorsAsync()               # non-blocking motor stop
        self._show_suspension_dialog(
            phase, label, reason,
            (f"Suspended by: {suspended_by}\nPhase: {phase}\n"
             + ''.join(f"{key}: {value}\n" for key, value in (info or {}).items())))

    def _show_suspension_dialog(self, phase: int, label: str, reason: str, details: str) -> None:
        # Not exec_(): a nested event loop would keep running the GUI (and this
        # window's updates) underneath. The operator closes it when ready.
        if self._suspension_dialog is not None:
            self._suspension_dialog.close()
        box = QMessageBox(QMessageBox.Warning, f"Training suspended - {label.lower()}",
                          f"Training suspended at phase {phase}.\n\n{reason}\n\n"
                          f"Press START TRAINING to resume from phase {phase}.",
                          QMessageBox.Ok, self)
        box.setDetailedText(details)
        box.setWindowModality(Qt.NonModal)
        box.setAttribute(Qt.WA_DeleteOnClose)
        box.finished.connect(lambda _result: setattr(self, '_suspension_dialog', None))
        box.show()
        self._suspension_dialog = box

    def connect(self, ROS: RosCommunicationManager, parent_timer: QTimer):
        self.ROS = ROS
        self.parent_timer = parent_timer
        self.ROS.stopCompleted.connect(self._onStopCompleted)
    
    def updateWindow(self):
        if self._stop_pending:
            return
        movement_name_text = ""
        if self.ui_main.movement_loaded:
            movement_name_text = self.ui_main.rehabMovementWindow.ui.lineEdit_MovementName.text().strip()
        if movement_name_text != self._last_movement_name_text:
            self._last_movement_name_text = movement_name_text
            self.ui.lineEdit.setText(movement_name_text)

        load_enabled = self.ui_main.movement_loaded and not self.Training_ON
        if load_enabled != self._last_load_enabled:
            self._last_load_enabled = load_enabled
            self.ui.pushButton_LoadCreateProtocol.setEnabled(load_enabled)

        mode_set = self.ROS.isModeSet()
        start_state = (bool(self.ui_main.movement_loaded), self.ProtocolData is not None, mode_set)
        if start_state != self._last_start_state:
            self._last_start_state = start_state
            if start_state[0] and start_state[1] and start_state[2]:
                self.ui.pushButton_STARTtrainig.setEnabled(True)
                self.ui.pushButton_STARTtrainig.setStyleSheet("background-color: rgb(255, 69, 0); color: black;" if self.Training_ON else "background-color: rgb(85, 255, 127); color: black;")
            else:
                self.ui.pushButton_STARTtrainig.setEnabled(False)
                self.ui.pushButton_STARTtrainig.setStyleSheet("")

        save_enabled = self.ProtocolData is not None
        if save_enabled != self._last_save_enabled:
            self._last_save_enabled = save_enabled
            self.ui.pushButton_SaveProtocol.setEnabled(save_enabled)

        side = self.ui_main.rehabMovementWindow.SideOfMovement
        if side != self._last_side:
            self._last_side = side
            if side == 1:
                self.ui.radioButton_SideLeft.blockSignals(True)
                self.ui.radioButton_SideRight.blockSignals(True)
                self.ui.radioButton_SideLeft.setChecked(True)
                self.ui.radioButton_SideLeft.blockSignals(False)
                self.ui.radioButton_SideRight.blockSignals(False)
            elif side == 2:
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

        movement_type: ExerciseType = self.ui_main.rehabMovementWindow.TypeOfMovement
        if movement_type != self._last_type:
            self._last_type = movement_type
            if movement_type == ExerciseType.REACHING:
                self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(True)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(True)
                self.ui.radioButton_TypeOfExercise_Reaching.setChecked(True)
                self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(False)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(False)
            elif movement_type == ExerciseType.HAND_TO_MOUTH:
                self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(True)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(True)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.setChecked(True)
                self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(False)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(False)
            else:
                self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(True)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(True)
                self.ui.radioButton_TypeOfExercise_Reaching.setChecked(False)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.setChecked(False)
                self.ui.radioButton_TypeOfExercise_Reaching.blockSignals(False)
                self.ui.radioButton_TypeOfExercise_HandtoMouth.blockSignals(False)

        if self.NumberExecMovements != self._last_movement_count:
            self._last_movement_count = self.NumberExecMovements
            self.ui.lcdNumber_MovementCOUNT.display(self.NumberExecMovements)

        if self.Training_ON:
            _iPhase = self.ROS.getExerciseRepetitionCounter() + self._iPhase_0
            if _iPhase <= 19:
                # --- Suspension reported by the robot (aborted repetition) ---
                if self.ROS.getExerciseSuspended():
                    self._handle_exercise_suspension(_iPhase, SUSPENDED_BY_ROBOT,
                                                     self.ROS.getExerciseSuspensionInfo())
                    return
                # A stop not requested by this GUI (ours clears Training_ON first).
                if self.ROS.getMovementStopped():
                    self._handle_exercise_suspension(_iPhase, SUSPENDED_BY_ROBOT, None)
                    return

                # --- Suspension decided by the GUI: no progress from the robot ---
                _pct = self.ROS.getExecutionTimePercentage()
                _now = time.monotonic()
                if _pct != self._exec_pct_prev or self._training_paused:
                    self._exec_pct_changed_at = _now
                elif _now - self._exec_pct_changed_at > self._NO_PROGRESS_TIMEOUT_S:
                    # Also at 0%: a robot side that never reports is caught too
                    # (a phase leaves 0% within ~1 s once accepted, <= 5 s).
                    self._handle_exercise_suspension(_iPhase, SUSPENDED_BY_GUI, {
                        'reason': (f'No progress received from the robot for '
                                   f'{_now - self._exec_pct_changed_at:.0f} s (stuck at {_pct}%). '
                                   'The GUI stopped the movement.'),
                        'progress_pct': _pct,
                    })
                    return
                self._exec_pct_prev = _pct

                self.progressBarPhases[_iPhase].setValue(_pct)
                if self.ROS.getExerciseRepetitionCounter() > 0:
                    self.progressBarPhases[_iPhase - 1].setValue(100) # type: ignore
                if self.ROS.getExerciseCompleted():
                    self.ROS.setExerciseCompleted(False)
                    self.ModalityActualValue = self.Modalities[_iPhase] # change here the modality
                    self._exec_pct_changed_at = time.monotonic()  # a new phase starts
                # set movemnt count lcd number
                _handle_pos = self.ROS.getHandleFeedbackPosition()
                _is_near_zero = self.ROS.isRosCommunicationActive() and all(abs(p) < 0.01 for p in _handle_pos)
                if _is_near_zero and not self._near_zero_triggered:
                    self._near_zero_triggered = True
                    self.NumberExecMovements += 1
                    print(f'Number of movements: {self.NumberExecMovements} - Ovr: {self.spinBoxSpeedOvr[_iPhase].value()}')
                elif not _is_near_zero:
                    if self._near_zero_triggered and self.EEGModeEnabled:
                        self.ROS.eegSync(self.NumberExecMovements)
                    self._near_zero_triggered = False
            else:
                self.progressBarPhases[19].setValue(100)
                self.stopTrainig()
        if not self.ROS.isRosCommunicationActive():
            # Properly uncheck/reset the button every time so it's never left
            # showing "STOP TRAINING" after an emergency disconnect.
            if self.Training_ON or self.ui.pushButton_STARTtrainig.isChecked():
                self.stopTrainig()
            # Invalidate the start_state cache so the button re-enables as soon
            # as the connection is restored (without needing movement state change).
            self._last_start_state = None
            self.ui.pushButton_STARTtrainig.setEnabled(False)
            self.ui.pushButton_PauseTrainig.setEnabled(False)
            self.ui.pushButton_ResumeTraining.setEnabled(False)

    def clbk_ModeChanged(self):
        """Callback for mode/save selection buttons.

        Both buttons are independent and can be active simultaneously:
        - Save: ros2 bag recording starts/stops with training
        - EEG:  EEG sync pulse sent at each movement start
        Clicking an already-selected button deselects it.
        """
        sender = self.sender()
        if sender == self.ui.radioButton_RehabMode:
            self.SaveModeEnabled = self.ui.radioButton_RehabMode.isChecked()
            print("Save Mode " + ("selected" if self.SaveModeEnabled else "deselected"))
        elif sender == self.ui.radioButton_EEGMode:
            self.EEGModeEnabled = self.ui.radioButton_EEGMode.isChecked()
            print("EEG Mode " + ("selected" if self.EEGModeEnabled else "deselected"))

    def clbk_DurationChanged(self, _value: int):
        self._update_total_training_time_display()

    def clbk_LoadCreateProtocol(self):
        if self.Training_ON or self._stop_pending or self.ROS.isCommandBusy():
            QMessageBox.warning(self, "Protocollo", "Terminare l'operazione in corso prima di caricare un protocollo.")
            return
        dlg = QFileDialog(None, "Load Protocol", self.ui_main.FMRR_Paths['Protocols'], "*.yaml")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        dlg.setFileMode(QFileDialog.ExistingFile)
        screen = QApplication.primaryScreen().availableGeometry()
        dlg.setMaximumSize(screen.width(), screen.height())
        dlg.resize(min(int(screen.width() * 0.9), 1100), min(int(screen.height() * 0.85), 750))
        if not dlg.exec_():
            return
        selected = dlg.selectedFiles()
        if not selected or not selected[0]:
            return
        filename = [selected[0]]

        try:
            candidate = read_yaml(filename[0], validate_protocol)
            self._applyProtocol(candidate)
        except Exception as exc:
            report_yaml_error(self, filename[0], exc)

    def _applyProtocol(self, candidate):
        validate_protocol(candidate)
        phases = candidate['Phases']
        flags, modes = list(phases['PhaseIsEnabled'][0]), list(phases['Modalities'][0])
        percentages = [int(x) for x in phases['Percentage'][0]]
        # Old files without Duration get explicit defaults, never previous values.
        durations = [int(x) for x in phases['Duration'][0]] if 'Duration' in phases else [60] * 20
        effective = [max(self._min_speed_ovr, p) for p in percentages]
        vmax = number(getattr(self.ui_main, 'Vmax', None), 'velocità del movimento caricato', minimum=0)
        phase_duration = number(getattr(self.ui_main, 'PhaseDuration', None), 'durata del movimento caricato', minimum=0)
        for i in range(20):
            check_spin_value(self.spinBoxSpeedOvr[i], effective[i], 'Percentage[{}]'.format(i))
            check_spin_value(self.spinBoxDuration[i], durations[i], 'Duration[{}]'.format(i))
        displays = [int(p / 100 * vmax) for p in effective]
        # Normalize the in-memory protocol to the values actually displayed.
        normalized = deepcopy(candidate)
        normalized['Phases']['Percentage'] = [effective]
        normalized['Phases']['Duration'] = [durations]
        widgets = [(w, 'value', 'setValue') for w in self.spinBoxSpeedOvr + self.spinBoxDuration + self.progressBarPhases]
        widgets += [(w, 'value', 'display') for w in self.lcdNumberPhases]
        widgets += [(w, 'value', 'display') for w in (self.ui.lcdNumber_SinglePhaseDuration, self.ui.lcdNumber_MaxVel, self.ui.lcdNumberExerciseTotalTime)]
        names = ['ProtocolData', 'PhaseIsEnabled', 'NrEnabledPhases', 'Modalities', 'Percentage', 'Durations',
                 'TotalTrainingTime', '_iPhase_0', '_last_total_time_display', '_last_start_state', '_last_load_enabled']
        with gui_transaction([(self, names)], widgets):
            self.ProtocolData = normalized
            self.PhaseIsEnabled, self.NrEnabledPhases = flags, sum(flags)
            self.Modalities, self.Percentage, self.Durations = modes, effective, durations
            self._iPhase_0 = 0
            self.TotalTrainingTime = sum(durations)
            self._last_total_time_display = np.floor(self.TotalTrainingTime) / 60
            self._last_start_state = self._last_load_enabled = None
            self.ui.lcdNumber_SinglePhaseDuration.display(np.floor(phase_duration))
            self.ui.lcdNumber_MaxVel.display(np.floor(vmax))
            for i in range(20):
                self.lcdNumberPhases[i].display(displays[i])
                self.spinBoxSpeedOvr[i].setValue(effective[i])
                self.spinBoxDuration[i].setValue(durations[i])
                self.progressBarPhases[i].setValue(0)
            self.ui.lcdNumberExerciseTotalTime.display(self._last_total_time_display)

    def clbk_SaveProtocol(self):
        dlg = QFileDialog(None, "Save Protocol", self.ui_main.FMRR_Paths['Protocols'], "*.yaml")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        dlg.setAcceptMode(QFileDialog.AcceptSave)
        screen = QApplication.primaryScreen().availableGeometry()
        dlg.setMaximumSize(screen.width(), screen.height())
        dlg.resize(min(int(screen.width() * 0.9), 1100), min(int(screen.height() * 0.85), 750))
        if not dlg.exec_():
            return
        selected = dlg.selectedFiles()
        if not selected or not selected[0]:
            return
        filename = [selected[0]]
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
            try:
                atomic_save_yaml(filename[0], protocol_data, validate_protocol)
            except Exception as exc:
                report_yaml_error(self, filename[0], exc)
            
    def startStopTraining(self, start: bool):
        if self._stop_pending:
            return
        if start:
            self._startTrainingTask()
        else:
            self.stopTrainig()

    @gui_task
    def _startTrainingTask(self):
        self.ui_main.syncExerciseTypeToPLC(force=True)
        if not (yield from self.startTrainig.__wrapped__(self)):
            self.Training_ON = False
            self._set_training_buttons_idle()
            return
        self.ui.pushButton_STARTtrainig.setStyleSheet('background-color: rgb(255, 69, 0); color: black;')
        self.ui.pushButton_STARTtrainig.setText('STOP TRAINING')
        self.ui.pushButton_PauseTrainig.setEnabled(True)


    @gui_task
    def startTrainig(self) -> bool:
        if not self.ROS.isRosCommunicationActive() or self.ROS.isInFaultState():
            QMessageBox.warning(self, 'Warning', 'No communication is active or the robot is in FAULT state.')
            return False
        HandlePosition = self.ROS.getHandleFeedbackPosition()
        if any([abs(HandlePosition[idx]) > 0.003 for idx in range(3)]):
            print(f'[Movement Program] Handle position is not zero, detected states: [{HandlePosition[0]}, {HandlePosition[1]}, {HandlePosition[2]}]')
            QMessageBox.warning(self, 'Warning', 'Handle position is not zero, please set it to zero before going to training')
            return False
        self.ROS.setManualMode(False)
        if not (yield Call(self.ROS.enableControllerBehaviour, 'FCT')):
            return False
        self.ROS.setExerciseSuspended(False)
        self.ROS.setMovementStopped(False)
        self.ROS.setExerciseInSuspension(False)
        self._near_zero_triggered = True
        self._training_paused = False
        self._reset_progress_watch()
        self.ActualTrainingTime = 0
        if not (yield from self.sendExercise.__wrapped__(self)):
            return False
        self.Training_ON = True
        self.ModalityActualValue = self.Modalities[0]
        for speedSpinBox in self.spinBoxSpeedOvr:
            speedSpinBox.setEnabled(False)
        for durationSpinBox in self.spinBoxDuration:
            durationSpinBox.setEnabled(False)
        self.NumberExecMovements = 0
        if self.SaveModeEnabled:
            self._start_bag_recording()
        return True
        
    @gui_task
    def clbk_PauseTrainig(self):
        self._training_paused = True
        yield Call(self.ROS.triggerSoftMovementStart, amplitude=0.0, time_constant=0.2, target='speed_ovr')
        self.ui.pushButton_ResumeTraining.setEnabled(True)
    
    @gui_task
    def clbk_ResumeTrainig(self):
        self._training_paused = False
        self._reset_progress_watch()
        yield Call(self.ROS.triggerSoftMovementStop)
        self.ui.pushButton_PauseTrainig.setEnabled(True)

    def stopTrainig(self):
        if self._stop_pending:
            return
        self._stop_bag_recording()
        self.Training_ON = False
        self._near_zero_triggered = False
        self.ROS.setExerciseInSuspension(False)
        # A stopped or completed protocol starts again from phase 1: only a
        # suspension (tracking error, _handle_exercise_suspension) resumes
        # from the interrupted phase.
        self._iPhase_0 = 0
        self._update_total_training_time_display(force=True)
        self._stop_pending = True
        self.ui.pushButton_STARTtrainig.setText("STOPPING...")
        self.ui.pushButton_STARTtrainig.setEnabled(False)
        self.ui.pushButton_PauseTrainig.setEnabled(False)
        self.ui.pushButton_ResumeTraining.setEnabled(False)
        if not self.ROS.requestStopAnyMovement(soft_stop=True):
            self._onStopCompleted(False)

    def _onStopCompleted(self, ok):
        if not self._stop_pending:
            return
        self._stop_pending = False
        self._set_training_buttons_idle()
        self._last_start_state = None
        for bar in self.progressBarPhases:
            bar.setValue(0)
        for spin in self.spinBoxSpeedOvr + self.spinBoxDuration:
            spin.setEnabled(True)
        if not ok:
            QMessageBox.warning(self, "Stop", "Arresto non confermato. Controllare stato robot e log ROS.")

        # self.ProtocolData = None

    def _start_bag_recording(self) -> None:
        """Request the bag_recorder_node on the Linux PC to start recording."""
        self.ROS.startBagRecording()
        self._bag_recording_active = True
        print('[BagRecord] Start request sent.')

    def _stop_bag_recording(self) -> None:
        """Request the bag_recorder_node on the Linux PC to stop recording."""
        if not self._bag_recording_active:
            return
        self._bag_recording_active = False
        self.ROS.stopBagRecording()
        print('[BagRecord] Stop request sent.')

    @gui_task
    def sendExercise(self):
        if not self.ROS.isRosCommunicationActive():
            QMessageBox.warning(self, 'Warning', 'Check the state of the driver - No communication is active.')
            return
        TrjYamlData = self.ui_main.rehabMovementWindow.TrjYamlData
        self.CartesianPositions = TrjYamlData.get('cart_trj3').get('cart_positions')
        self.TimeFromStart = TrjYamlData.get('cart_trj3').get('time_from_start')
        if not (yield Call(self.ROS.turnOnMotors)):
            QMessageBox.warning(self, 'Warning', 'Failed in switching on the motors')
            return
        pending_percentages = self._get_pending_phase_percentages()
        pending_durations = self._get_pending_phase_durations()
        sent = (yield Call(self.ROS.setExercise, self.CartesianPositions, self.TimeFromStart, pending_percentages, pending_durations, self.EEGModeEnabled))
        if not sent:
            # The motors were switched on above for this exercise: do not leave
            # them on, holding position, with nothing to execute.
            yield Call(self.ROS.turnOffMotors)
            QMessageBox.warning(self, 'Warning',
                                'The exercise could not be sent to the robot: motors switched off.\n'
                                'Check the robot state and the logs, then try again.')
            return False
        self.TotalTrainingTime = sum(pending_durations)
        self._update_total_training_time_display(force=True)
        return True
            
        
def main(args=None):
    pass

if __name__ == "__main__":
    main()
