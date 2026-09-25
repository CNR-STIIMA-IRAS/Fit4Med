"""Suspensions reported by the robot vs decided by the GUI, and the result services.
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
from sync_ros_events import SyncRosManager
from RosCommunicationManager import RosCommunicationManager

APP = QApplication.instance() or QApplication([])

TRACKING_ERROR = {'success': False, 'action_status': 6, 'error_code': -4,
                  'message': 'Repetition 2: path tolerance violated on joint_y', 'movement_kind': 'exercise'}


class ResultServicesTest(unittest.TestCase):
    """The GUI side of /rehab_gui/*: tecnobody_msgs/TrajectoryResult, answered with `accepted`."""

    def setUp(self):
        self.ros = types.SimpleNamespace(result_info=SyncRosManager.result_info,
                                         exercise_suspended=False, exercise_suspension=None,
                                         trajectory_completed=False, trajectory_result=None,
                                         repetition_cnt=0, exercise_completed=False)

    def test_suspension_keeps_the_reason(self):
        response = {}
        self.assertTrue(SyncRosManager.on_exercise_suspended(self.ros, dict(TRACKING_ERROR), response))
        self.assertEqual(response, {'accepted': True})
        self.assertTrue(self.ros.exercise_suspended)
        self.assertEqual(self.ros.exercise_suspension, TRACKING_ERROR)

    def test_malformed_suspension_is_still_a_suspension(self):
        response = {}
        SyncRosManager.on_exercise_suspended(self.ros, {'error_code': 'not a number'}, response)
        self.assertTrue(self.ros.exercise_suspended)
        self.assertEqual(self.ros.exercise_suspension['error_code'], 999)
        self.assertEqual(response, {'accepted': True})

    def test_failed_trajectory_still_completes(self):
        response = {}
        result = {'success': False, 'action_status': 6, 'error_code': -5,
                  'message': 'goal tolerance', 'movement_kind': 'go_to_start'}
        SyncRosManager.on_trajectory_finished(self.ros, result, response)
        self.assertTrue(self.ros.trajectory_completed)  # the windows switch the motors off on it
        self.assertEqual(self.ros.trajectory_result['movement_kind'], 'go_to_start')
        self.assertEqual(response, {'accepted': True})

    def fake_sender(self):
        clients = {name: Mock(call=Mock(return_value={'success': True}))
                   for name in ('reset_speed_over_client', 'set_trajectory_client',
                                'set_go_to_start_trajectory_client', 'set_rehab_exercise_client',
                                'set_eeg_exercise_client')}
        ros = types.SimpleNamespace(RobotJointPosition=[0.0, 0.0, 0.0], trajectory_completed=True,
                                    trajectory_result={'success': False}, execution_time_percentage=100,
                                    exercise_suspended=True, exercise_suspension={'error_code': -4},
                                    repetition_cnt=5, exercise_completed=True, **clients)
        ros._forget_last_trajectory = lambda: SyncRosManager._forget_last_trajectory(ros)
        return ros

    def test_a_new_trajectory_forgets_the_previous_completion(self):
        """A 'completed' left by a stopped trajectory must not end the next one."""
        for send in (SyncRosManager.send_ptp_trajectory, SyncRosManager.send_go_to_start_ptp_trajectory):
            ros = self.fake_sender()
            self.assertTrue(send(ros, [0.1, 0.0, 0.0], 2.0))
            self.assertFalse(ros.trajectory_completed)
            self.assertIsNone(ros.trajectory_result)

    def test_a_new_exercise_starts_from_zero_progress(self):
        ros = self.fake_sender()
        with patch.object(sys.modules['roslibpy'], 'Message', dict, create=True):
            self.assertTrue(SyncRosManager.set_exercise(ros, [[0, 0, 0]], [[0.1]], [50], [60], False))
        self.assertEqual(ros.execution_time_percentage, 0)
        self.assertFalse(ros.exercise_suspended)
        self.assertIsNone(ros.exercise_suspension)

    def test_clearing_the_flag_clears_the_cause(self):
        manager = types.SimpleNamespace(rOk=lambda: True, ROS=types.SimpleNamespace(
            exercise_suspended=True, exercise_suspension=dict(TRACKING_ERROR)))
        RosCommunicationManager.setExerciseSuspended(manager, False)
        self.assertFalse(manager.ROS.exercise_suspended)
        self.assertIsNone(manager.ROS.exercise_suspension)

    def test_repetition_finished_is_counted_and_accepted(self):
        response = {}
        SyncRosManager.on_exercise_finished(self.ros, {'success': True}, response)
        self.assertEqual(self.ros.repetition_cnt, 1)
        self.assertEqual(response, {'accepted': True})


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class SuspensionKindsTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        patch.object(tpw.time, 'monotonic', self.clock).start()
        self.addCleanup(patch.stopall)
        movement = types.SimpleNamespace(
            ui=types.SimpleNamespace(lineEdit_MovementName=Mock(text=Mock(return_value='mov'))),
            SideOfMovement=1, TypeOfMovement=ExerciseType.REACHING)
        ui_main = types.SimpleNamespace(movement_loaded=True, rehabMovementWindow=movement,
                                        Vmax=10.0, PhaseDuration=4.0, FMRR_Paths={'Protocols': '.'})
        self.window = tpw.TrainingProtocolWindow(ui_main)
        self.ros = Mock()
        self.ros.isRosCommunicationActive.return_value = True
        self.ros.isModeSet.return_value = True
        self.ros.getExerciseSuspended.return_value = False
        self.ros.getMovementStopped.return_value = False
        self.ros.getExerciseCompleted.return_value = False
        self.ros.getExecutionTimePercentage.return_value = 30
        self.ros.getHandleFeedbackPosition.return_value = [0.1, 0.1, 0.1]
        self.ros.getExerciseRepetitionCounter.return_value = 2
        self.ros.requestStopAnyMovement.return_value = True
        self.window.connect(self.ros, Mock())
        self.window._applyProtocol({'Phases': {'PhaseIsEnabled': [[1] * 20], 'Modalities': [[0] * 20],
                                               'Percentage': [[50] * 20], 'Duration': [[60] * 20]}})
        self.window.Training_ON = True
        self.window._reset_progress_watch()
        self.addCleanup(lambda: self.window._suspension_dialog and self.window._suspension_dialog.close())

    def tick(self, seconds=0.1):
        self.clock.now += seconds
        self.window.updateWindow()

    def dialog_text(self):
        self.assertIsNotNone(self.window._suspension_dialog)
        return self.window._suspension_dialog.text()

    def test_tracking_error_from_the_robot(self):
        self.ros.getExerciseSuspended.return_value = True
        self.ros.getExerciseSuspensionInfo.return_value = dict(TRACKING_ERROR)
        self.tick()
        self.assertFalse(self.window.Training_ON)
        self.assertEqual(self.window._iPhase_0, 2)  # resume from the third phase
        self.ros.setExerciseInSuspension.assert_called_with(True, 'TRACKING ERROR')
        self.ros.turnOffMotorsAsync.assert_called_once()
        self.ros.requestStopAnyMovement.assert_not_called()  # the robot is already stopped
        self.assertIn('phase 3', self.dialog_text())
        self.assertIn('deviated from the trajectory', self.dialog_text())
        self.assertIn('path tolerance violated on joint_y', self.window._suspension_dialog.detailedText())

    def test_unknown_robot_code_is_reported_with_its_value(self):
        self.ros.getExerciseSuspended.return_value = True
        self.ros.getExerciseSuspensionInfo.return_value = dict(TRACKING_ERROR, error_code=999)
        self.tick()
        self.ros.setExerciseInSuspension.assert_called_with(True, 'ROBOT ERROR')
        self.assertIn('code 999', self.dialog_text())
        # The controller's own words are in the main text, not only in the details.
        self.assertIn('Robot controller: Repetition 2: path tolerance violated on joint_y', self.dialog_text())

    def test_stop_on_the_robot_side(self):
        self.ros.getMovementStopped.return_value = True
        self.tick()
        self.ros.setExerciseInSuspension.assert_called_with(True, 'ROBOT STOP')
        self.ros.turnOffMotorsAsync.assert_called_once()

    def test_no_progress_is_a_gui_suspension_that_stops_the_movement(self):
        self.tick()
        for _ in range(14):
            self.tick(1.0)
        self.assertTrue(self.window.Training_ON)  # 14 s: still waiting
        self.tick(1.5)
        self.assertFalse(self.window.Training_ON)
        self.assertEqual(self.window._iPhase_0, 2)
        self.ros.setExerciseInSuspension.assert_called_with(True, 'NO PROGRESS')
        self.ros.requestStopAnyMovement.assert_called_once()  # the robot may still be moving
        self.ros.turnOffMotorsAsync.assert_not_called()       # done by the stop sequence
        self.assertIn('No progress received from the robot', self.dialog_text())

    def test_no_progress_at_all_is_caught_too(self):
        """A robot side that never sends a single progress update (stuck at 0 %)."""
        self.ros.getExecutionTimePercentage.return_value = 0
        self.tick()
        self.tick(14.0)
        self.assertTrue(self.window.Training_ON)
        self.tick(1.5)
        self.assertFalse(self.window.Training_ON)
        self.ros.setExerciseInSuspension.assert_called_with(True, 'NO PROGRESS')
        self.assertIn('stuck at 0%', self.dialog_text())

    def test_no_progress_falls_back_to_motors_off_if_the_stop_is_refused(self):
        self.ros.requestStopAnyMovement.return_value = False
        self.tick()
        self.tick(16.0)
        self.ros.turnOffMotorsAsync.assert_called_once()

    def test_progress_changes_keep_the_training_running(self):
        for pct in range(31, 60):
            self.ros.getExecutionTimePercentage.return_value = pct
            self.tick(1.0)
        self.assertTrue(self.window.Training_ON)
        self.ros.requestStopAnyMovement.assert_not_called()

    def test_pause_does_not_count_as_no_progress(self):
        self.tick()
        self.window._training_paused = True
        self.tick(60.0)
        self.assertTrue(self.window.Training_ON)
        self.window._training_paused = False  # RESUME: the wait starts again
        self.window._reset_progress_watch()
        self.tick(10.0)
        self.assertTrue(self.window.Training_ON)


if __name__ == '__main__':
    unittest.main()
