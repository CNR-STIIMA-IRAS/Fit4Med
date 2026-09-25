"""GoTo (PTP) timing and failure handling in RobotWindow.
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
import RobotWindow as rw

APP = QApplication.instance() or QApplication([])


class GoToTest(unittest.TestCase):
    def setUp(self):
        self.window = rw.RobotWindow(types.SimpleNamespace())
        self.window.ROS = Mock()
        self.window.ui.pushButton_ApproachAllJoint.setCheckable(True)
        self.window.ui.pushButton_ApproachAllJoint.setChecked(True)
        self.warning = patch.object(rw.QMessageBox, 'warning').start()
        self.addCleanup(patch.stopall)

    def run_goto(self, target, current, motors_on=True):
        """Drive the GoTo task; return the Calls it yields after the delay."""
        ui = self.window.ui
        for spin, value in zip((ui.doubleSpin_Joint1_Value, ui.doubleSpin_Joint2_Value,
                                ui.doubleSpin_Joint3_Value), target):
            spin.setValue(value)
        self.window.ROS.getRobotJointPosition.return_value = current
        task = rw.RobotWindow._goTo_afterDelay.__wrapped__(self.window)
        calls = []
        answer = None
        try:
            call = next(task)  # the 0.5 s delay
            while True:
                call = task.send(answer)
                calls.append(call)
                answer = motors_on if call.function is self.window.ROS.turnOnMotors else True
        except StopIteration:
            pass
        return calls

    def sent_time(self, calls):
        ptp = [c for c in calls if c.function is self.window.ROS.sendPTPTrajectory]
        self.assertEqual(len(ptp), 1)
        return ptp[0].args[1]

    def test_duration_uses_the_distance_from_the_current_position(self):
        # Was 2.0 s (distance of the target from zero): 0.225 m/s.
        calls = self.run_goto((0.05, 0.0, 0.1), [-0.4, 0.0, 0.1])
        self.assertAlmostEqual(self.sent_time(calls), 4.5)

    def test_duration_uses_the_path_length_not_the_largest_axis(self):
        # Was 1.2 s: 0.208 m diagonal at 0.17 m/s.
        calls = self.run_goto((0.12, 0.12, 0.12), [0.0, 0.0, 0.0])
        self.assertAlmostEqual(self.sent_time(calls), 0.12 * 3 ** 0.5 / rw.PTP_MAX_SPEED)

    def test_short_moves_take_the_minimum_time(self):
        calls = self.run_goto((0.45, 0.0, 0.0), [0.40, 0.0, 0.0])
        self.assertEqual(self.sent_time(calls), rw.PTP_MIN_TIME_S)

    def test_out_of_range_target_releases_the_button(self):
        calls = self.run_goto((0.6, 0.0, 0.0), [0.0, 0.0, 0.0])
        self.assertEqual(calls, [])  # not even the motors are switched on
        self.warning.assert_called_once()
        self.assertFalse(self.window.ui.pushButton_ApproachAllJoint.isChecked())

    def test_motors_failure_releases_the_button(self):
        calls = self.run_goto((0.1, 0.0, 0.0), [0.0, 0.0, 0.0], motors_on=False)
        self.assertEqual([c.function for c in calls], [self.window.ROS.turnOnMotors])
        self.warning.assert_called_once()
        self.assertFalse(self.window.ui.pushButton_ApproachAllJoint.isChecked())

    def check_aborted_with_motors_off(self, calls):
        self.assertEqual([c.function for c in calls],
                         [self.window.ROS.turnOnMotors, self.window.ROS.turnOffMotors])
        self.warning.assert_called_once()
        self.assertFalse(self.window.ui.pushButton_ApproachAllJoint.isChecked())

    def test_invalid_current_position_switches_the_motors_off(self):
        self.check_aborted_with_motors_off(self.run_goto((0.1, 0.0, 0.0), [float('nan'), 0.0, 0.0]))

    def test_incomplete_current_position_switches_the_motors_off(self):
        self.check_aborted_with_motors_off(self.run_goto((0.1, 0.0, 0.0), [0.0, 0.0]))


if __name__ == '__main__':
    unittest.main()
