"""Which protocol phases a START sends after suspensions, stops and completions.
Run: QT_QPA_PLATFORM=offscreen python -m unittest discover -s tests -v
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import types
import unittest
from unittest.mock import Mock, patch

# ROS transport stubbed: these tests must never reach a robot.
if 'roslibpy' not in sys.modules:
    roslibpy = types.ModuleType('roslibpy')
    roslibpy.Ros = object
    roslibpy.Service = Mock()
    roslibpy.ServiceRequest = lambda request=None: request or {}
    sys.modules['roslibpy'] = roslibpy

from PyQt5.QtWidgets import QApplication
import TrainingProtocolWindow as tpw
from RehabilitationMovementWindow import ExerciseType

APP = QApplication.instance() or QApplication([])

PHASES = 20
DURATION_S = 60


def protocol():
    return {'Phases': {'PhaseIsEnabled': [[1] * PHASES], 'Modalities': [[0] * PHASES],
                       'Percentage': [[50] * PHASES], 'Duration': [[DURATION_S] * PHASES]}}


class TrainingResumeTest(unittest.TestCase):
    def setUp(self):
        movement = types.SimpleNamespace(
            ui=types.SimpleNamespace(lineEdit_MovementName=Mock(text=Mock(return_value='mov'))),
            SideOfMovement=1, TypeOfMovement=ExerciseType.REACHING,
            TrjYamlData={'cart_trj3': {'cart_positions': [[0, 0, 0]], 'time_from_start': [[0.1]]}})
        self.ui_main = types.SimpleNamespace(movement_loaded=True, rehabMovementWindow=movement,
                                             Vmax=10.0, PhaseDuration=4.0,
                                             FMRR_Paths={'Protocols': '.'})
        self.window = tpw.TrainingProtocolWindow(self.ui_main)
        self.ros = Mock()
        self.ros.isRosCommunicationActive.return_value = True
        self.ros.isModeSet.return_value = True
        self.ros.getExerciseSuspended.return_value = False
        self.ros.getMovementStopped.return_value = False
        self.ros.getExerciseCompleted.return_value = False
        self.ros.getExecutionTimePercentage.return_value = 10
        self.ros.getHandleFeedbackPosition.return_value = [0.1, 0.1, 0.1]
        self.ros.getExerciseRepetitionCounter.return_value = 0
        self.ros.requestStopAnyMovement.return_value = True
        self.window.connect(self.ros, Mock())
        patch.object(tpw.QMessageBox, 'warning').start()
        self.addCleanup(patch.stopall)
        self.window._applyProtocol(protocol())

    def start(self):
        """Run the sending part of START; return how many phases were sent."""
        task = tpw.TrainingProtocolWindow.sendExercise.__wrapped__(self.window)
        next(task)                       # turnOnMotors
        call = task.send(True)           # setExercise(positions, times, ovrs, durations, eeg)
        with self.assertRaises(StopIteration):
            task.send(True)
        self.window.Training_ON = True
        self.ros.getExerciseRepetitionCounter.return_value = 0
        return len(call.args[3])

    def suspend_at(self, repetitions_done):
        """Tracking error reported by ROS after some repetitions of this run."""
        self.ros.getExerciseRepetitionCounter.return_value = repetitions_done
        self.ros.getExerciseSuspended.return_value = True
        self.ros.getExerciseSuspensionInfo.return_value = {
            'success': False, 'action_status': 6, 'error_code': -4,
            'message': 'path tolerance violated', 'movement_kind': 'exercise'}
        self.window.updateWindow()
        self.ros.getExerciseSuspended.return_value = False
        self.assertFalse(self.window.Training_ON)

    def complete(self):
        """The ROS side reports every sent repetition as done."""
        self.ros.getExerciseRepetitionCounter.return_value = PHASES - self.window._iPhase_0
        self.window.updateWindow()
        self.window._onStopCompleted(True)
        self.assertFalse(self.window.Training_ON)

    def stop(self):
        self.window.stopTrainig()
        self.window._onStopCompleted(True)

    def total_minutes(self):
        return self.window.ui.lcdNumberExerciseTotalTime.value()

    def test_suspension_resumes_from_the_interrupted_phase(self):
        self.assertEqual(self.start(), PHASES)
        self.suspend_at(4)
        self.assertEqual(self.total_minutes(), (PHASES - 4) * DURATION_S / 60)
        self.assertEqual(self.start(), PHASES - 4)

    def test_protocol_completed_after_a_resume_restarts_from_phase_1(self):
        self.start()
        self.suspend_at(4)
        self.start()
        self.complete()
        self.assertEqual(self.total_minutes(), PHASES * DURATION_S / 60)
        self.assertEqual(self.start(), PHASES)  # was PHASES - 4

    def test_stop_after_a_resume_restarts_from_phase_1(self):
        self.start()
        self.suspend_at(4)
        self.start()
        self.stop()
        self.assertEqual(self.start(), PHASES)  # was PHASES - 4

    def test_two_suspensions_add_up(self):
        self.start()
        self.suspend_at(4)
        self.start()
        self.suspend_at(3)  # 3 more phases done after the resume
        self.assertEqual(self.start(), PHASES - 7)

    def test_protocol_without_suspension_restarts_from_phase_1(self):
        self.start()
        self.complete()
        self.assertEqual(self.start(), PHASES)

    def test_ros_loss_after_a_resume_restarts_from_phase_1(self):
        self.start()
        self.suspend_at(4)
        self.start()
        self.ros.isRosCommunicationActive.return_value = False
        self.window.updateWindow()
        self.window._onStopCompleted(False)
        self.ros.isRosCommunicationActive.return_value = True
        self.assertEqual(self.start(), PHASES)


if __name__ == '__main__':
    unittest.main()
