"""A failed send after switching the motors on must switch them off again.
Run: QT_QPA_PLATFORM=offscreen python -m unittest discover -s tests -v
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import time
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

from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QApplication
from GuiRosTasks import GuiTask
import TrainingProtocolWindow as tpw
import RehabilitationMovementWindow as rmw

APP = QApplication.instance() or QApplication([])


def drive(task, answers):
    """Run a GUI-task generator, answering each Call; return (functions called, result).

    answers maps a Call's function to the value it returns (default True).
    """
    called, value = [], None
    try:
        call = next(task)
        while True:
            called.append(call.function)
            call = task.send(answers.get(call.function, True))
    except StopIteration as done:
        value = done.value
    return called, value


class TrainingSendTest(unittest.TestCase):
    def setUp(self):
        movement = types.SimpleNamespace(
            TrjYamlData={'cart_trj3': {'cart_positions': [[0, 0, 0]], 'time_from_start': [[0.1]]}})
        self.window = tpw.TrainingProtocolWindow(types.SimpleNamespace(rehabMovementWindow=movement))
        self.ros = Mock()
        self.ros.isRosCommunicationActive.return_value = True
        self.window.ROS = self.ros
        self.warning = patch.object(tpw.QMessageBox, 'warning').start()
        self.addCleanup(patch.stopall)

    def send(self, **answers):
        return drive(tpw.TrainingProtocolWindow.sendExercise.__wrapped__(self.window),
                     {getattr(self.ros, name): value for name, value in answers.items()})

    def test_successful_send_leaves_the_motors_on(self):
        called, ok = self.send()
        self.assertTrue(ok)
        self.assertEqual(called, [self.ros.turnOnMotors, self.ros.setExercise])
        self.warning.assert_not_called()

    def test_failed_send_switches_the_motors_off(self):
        called, ok = self.send(setExercise=False)
        self.assertFalse(ok)
        self.assertEqual(called, [self.ros.turnOnMotors, self.ros.setExercise, self.ros.turnOffMotors])
        self.warning.assert_called_once()

    def test_motors_not_switching_on_is_reported(self):
        called, ok = self.send(turnOnMotors=False)
        self.assertFalse(ok)
        self.assertEqual(called, [self.ros.turnOnMotors])
        self.warning.assert_called_once()

    def test_stop_during_the_send_adds_nothing(self):
        """STOP cancels the task: the stop sequence switches the motors off, not us."""
        parent = QObject()
        task = GuiTask(tpw.TrainingProtocolWindow.sendExercise.__wrapped__(self.window), parent)

        def send_cancelled_by_stop(*args):
            task.cancel.set()  # what requestStopAnyMovement does meanwhile
            return False
        self.ros.setExercise.side_effect = send_cancelled_by_stop
        finished = []
        task.finished.connect(lambda: finished.append(True))
        task.start()
        deadline = time.monotonic() + 5
        while not finished and time.monotonic() < deadline:
            APP.processEvents()
            time.sleep(0.005)
        self.assertTrue(finished)
        self.ros.turnOffMotors.assert_not_called()
        self.warning.assert_not_called()


class GoToStartSendTest(unittest.TestCase):
    def setUp(self):
        self.window = rmw.RehabilitationMovementWindow(types.SimpleNamespace(movement_loaded=0))
        self.ros = Mock()
        self.ros.getCurrentControllerName.return_value = 'go_to_start_controller'
        self.ros.getGoToStartControllerName.return_value = 'go_to_start_controller'
        self.window.ROS = self.ros
        self.warning = patch.object(rmw.QMessageBox, 'warning').start()
        self.addCleanup(patch.stopall)

    def send(self, **answers):
        return drive(rmw.RehabilitationMovementWindow._goToStartPosition_afterDelay.__wrapped__(self.window),
                     {getattr(self.ros, name): value for name, value in answers.items()})

    def test_successful_send_leaves_the_motors_on(self):
        called, _ = self.send()
        self.assertEqual(called, [self.ros.turnOnMotors, self.ros.sendGoToStartPTPTrajectory])
        self.warning.assert_not_called()

    def test_failed_send_switches_the_motors_off_and_restores_the_controller(self):
        called, _ = self.send(sendGoToStartPTPTrajectory=False)
        self.assertEqual(called, [self.ros.turnOnMotors, self.ros.sendGoToStartPTPTrajectory,
                                  self.ros.turnOffMotors, self.ros.enableControllerBehaviour])
        self.warning.assert_called_once()

    def test_motors_not_switching_on_sends_nothing(self):
        called, _ = self.send(turnOnMotors=False)
        self.assertEqual(called, [self.ros.turnOnMotors])
        self.warning.assert_called_once()


if __name__ == '__main__':
    unittest.main()
