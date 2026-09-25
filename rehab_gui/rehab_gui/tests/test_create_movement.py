"""CREATE movement must report errors, never raise out of the Qt slot.
Run: QT_QPA_PLATFORM=offscreen python -m unittest discover -s tests -v
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import sys
import tempfile
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
import RehabilitationMovementWindow as rmw

APP = QApplication.instance() or QApplication([])


def movement(points):
    """Minimal valid movement file content with the given cart positions."""
    n = len(points)
    return {
        'a_movement_definition': {
            'type': [2], 'side': [1], 'vel_profile': [2], 'max_velocity': [10.0], 'total_time': [2.0],
            'begin_config': [[0.0, 0.0, 0.0]], 'end_config': [[0.1, 0.1, 0.1]],
            'begin_joint_config': [[0.0, 0.0, 0.0]],
        },
        'cart_trj3': {
            'cart_positions': points,
            'time_from_start': [[0.1 * (i + 1)] for i in range(n)],
        },
    }


class CreateMovementTest(unittest.TestCase):
    def setUp(self):
        self.training = types.SimpleNamespace(Training_ON=False, _stop_pending=False)
        self.main_app = types.SimpleNamespace(movement_loaded=0, FMRR_Paths={'Movements': '.'},
                                              trainingProtocolWindow=self.training)
        self.window = rmw.RehabilitationMovementWindow(self.main_app)
        self.window.ROS = Mock(isCommandBusy=Mock(return_value=False))
        self.window.End_HandlePosition = [0.10, 0.05, 0.08]
        self.window.ui.doubleSpinBox_MoveTime.setValue(2.0)
        self.warning = patch.object(rmw.QMessageBox, 'warning').start()
        self.critical = patch.object(rmw.QMessageBox, 'critical').start()
        self.report = patch.object(rmw, 'report_yaml_error').start()
        # No real file dialogs: cancel the "save as" of the created movement.
        self.save_as = patch.object(rmw.QtWidgets.QFileDialog, 'getSaveFileName', return_value=('', '')).start()
        self.addCleanup(patch.stopall)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)

    def select(self, reaching=False, hand_to_mouth=False):
        ui = self.window.ui
        # The radios sit in an exclusive group (Hand to Mouth checked by
        # default): clicks cannot clear both, but loading a movement file with
        # type 0 does (_applyMovement). Reproduce that state.
        ui.buttonGroup.setExclusive(False)
        for radio in (ui.radioButton_TypeOfExercise_Reaching, ui.radioButton_TypeOfExercise_HandtoMouth):
            radio.setAutoExclusive(False)
        ui.radioButton_TypeOfExercise_Reaching.setChecked(reaching)
        ui.radioButton_TypeOfExercise_HandtoMouth.setChecked(hand_to_mouth)
        ui.radioButton_SideLeft.setChecked(True)

    def test_no_exercise_type_is_a_warning_not_a_crash(self):
        self.select()
        self.window.clbk_BtnCreateMovementData()  # used to raise UnboundLocalError
        self.warning.assert_called_once()
        self.assertFalse(hasattr(self.window, 'TrjYamlData'))

    def load_active_movement(self):
        """An active movement loaded from file, as before a CREATE."""
        points = [[0.01 * i, 0.01 * i, 0.01 * i] for i in range(12)]
        self.window._applyMovement(movement(points), 'loaded_one')
        self.assertTrue(self.main_app.movement_loaded)
        return self.window.TrjYamlData

    def test_reaching_still_creates_the_movement(self):
        self.select(reaching=True)
        self.window.clbk_BtnCreateMovementData()
        self.critical.assert_not_called()
        self.report.assert_not_called()
        self.assertIn('cart_trj3', self.window._created_movement['TrjYamlData'])

    def test_unsaved_creation_does_not_replace_the_active_movement(self):
        active = self.load_active_movement()
        self.select(reaching=True)
        self.window.ui.radioButton_SideRight.setChecked(True)
        self.window.clbk_BtnCreateMovementData()  # "save as" cancelled
        self.assertIs(self.window.TrjYamlData, active)
        self.assertEqual(self.window.TypeOfMovement, rmw.ExerciseType.HAND_TO_MOUTH)
        self.assertEqual(self.window.SideOfMovement, 1)
        self.assertEqual(self.main_app.Vmax, 10.0)
        self.assertEqual(self.window.ui.lineEdit_MovementName.text(), 'loaded_one')
        self.assertIsNotNone(self.window._created_movement)

    def test_saved_creation_becomes_the_active_movement(self):
        self.load_active_movement()
        self.select(reaching=True)
        self.save_as.return_value = (str(self.tmp / 'new_reaching.yaml'), '')
        self.window.clbk_BtnCreateMovementData()
        self.assertTrue((self.tmp / 'new_reaching.yaml').exists())
        self.assertEqual(self.window.TypeOfMovement, rmw.ExerciseType.REACHING)
        self.assertEqual(self.window.TrjYamlData['a_movement_definition']['type'], [1])
        self.assertEqual(self.main_app.PhaseDuration, 4.0)
        self.assertTrue(self.main_app.movement_loaded)
        self.assertEqual(self.window.ui.lineEdit_MovementName.text(), 'new_reaching')
        self.assertIsNone(self.window._created_movement)

    def test_save_button_later_commits_the_created_movement(self):
        active = self.load_active_movement()
        self.select(reaching=True)
        self.window.clbk_BtnCreateMovementData()  # cancelled
        self.assertIs(self.window.TrjYamlData, active)
        self.save_as.return_value = (str(self.tmp / 'later.yaml'), '')
        self.window.clbk_BtnSAVEMovement()
        self.assertEqual(self.window.TypeOfMovement, rmw.ExerciseType.REACHING)
        self.assertIsNot(self.window.TrjYamlData, active)
        self.assertIsNone(self.window._created_movement)

    def test_loading_discards_an_unsaved_creation(self):
        self.select(reaching=True)
        self.window.clbk_BtnCreateMovementData()
        self.load_active_movement()
        self.assertIsNone(self.window._created_movement)

    def test_create_is_refused_during_training(self):
        self.training.Training_ON = True
        self.select(reaching=True)
        self.window.clbk_BtnCreateMovementData()
        self.warning.assert_called_once()
        self.assertIsNone(self.window._created_movement)

    def test_failed_creation_leaves_the_active_movement_untouched(self):
        active = self.load_active_movement()
        self.select(hand_to_mouth=True)
        self.window.ui.radioButton_SideRight.setChecked(True)
        flat_x = [[0.0, 0.01 * i, 0.02 * i] for i in range(12)]
        with patch.object(rmw, 'open_file', return_value='source.yaml'), \
             patch.object(rmw, 'read_yaml', return_value=movement(flat_x)):
            self.window.clbk_BtnCreateMovementData()
        self.critical.assert_called_once()
        self.assertIs(self.window.TrjYamlData, active)
        self.assertEqual(self.window.SideOfMovement, 1)
        self.assertIsNone(self.window._created_movement)

    def test_hand_to_mouth_source_without_displacement_is_reported(self):
        self.select(hand_to_mouth=True)
        flat_x = [[0.0, 0.01 * i, 0.02 * i] for i in range(12)]  # never moves along x
        with patch.object(rmw, 'open_file', return_value='source.yaml'), \
             patch.object(rmw, 'read_yaml', return_value=movement(flat_x)):
            self.window.clbk_BtnCreateMovementData()  # used to raise ZeroDivisionError
        self.critical.assert_called_once()
        self.assertIn('no displacement', self.critical.call_args[0][2])

    def test_hand_to_mouth_interpolation_failure_is_reported(self):
        self.select(hand_to_mouth=True)
        # Repeated points: zero-length steps break the cubic interpolation.
        points = [[0.0, 0.0, 0.0]] * 3 + [[0.01 * i, 0.01 * i, 0.01 * i] for i in range(1, 10)]
        with patch.object(rmw, 'open_file', return_value='source.yaml'), \
             patch.object(rmw, 'read_yaml', return_value=movement(points)):
            self.window.clbk_BtnCreateMovementData()  # used to raise ValueError
        self.critical.assert_called_once()


if __name__ == '__main__':
    unittest.main()
