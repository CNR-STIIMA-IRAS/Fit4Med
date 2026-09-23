"""Qt event-loop tests with simulated ROS, no robot/server required.
Run: QT_QPA_PLATFORM=offscreen python -m unittest discover -s tests -v
"""
import ast
from collections import UserDict
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import threading
import time
import types
import unittest
from unittest.mock import Mock
# ROS transport deliberately stubbed: these tests must never reach a robot.
roslibpy = types.ModuleType('roslibpy')
roslibpy.Ros = object
roslibpy.Service = Mock()
roslibpy.ServiceRequest = lambda request=None: request or {}
sys.modules['roslibpy'] = roslibpy
from PyQt5.QtCore import QCoreApplication, QThread, QTimer
from PyQt5.QtWidgets import QApplication
from GuiRosTasks import Call
from RosCommunicationManager import RosCommunicationManager
from sync_ros_events import RosilibpyServiceHandler, SyncRosManager, command_context

app = QApplication.instance() or QApplication([])
def spin_until(predicate, seconds=3):
    deadline = time.monotonic() + seconds
    while not predicate():
        app.processEvents()
        time.sleep(.001)
        if time.monotonic() > deadline:
            raise AssertionError('Qt operation did not finish')
    app.processEvents()

class AsyncTests(unittest.TestCase):
    def manager(self):
        m = RosCommunicationManager(['x'], 1, 'unused', 9090, None)
        m.ros_client = types.SimpleNamespace(is_connected=True, close=Mock())
        m.ROS = Mock()
        return m

    def test_gui_heartbeat_and_continuation_thread(self):
        m = self.manager(); events=[]; ticks=[]
        timer = QTimer(); timer.timeout.connect(lambda: ticks.append(1)); timer.start(2)
        def blocking():
            self.assertNotEqual(QThread.currentThread(), app.thread())
            time.sleep(.04)
            return 7
        def sequence():
            self.assertEqual(QThread.currentThread(), app.thread())
            result = yield Call(blocking)
            self.assertEqual(QThread.currentThread(), app.thread())
            events.append(result)
            result = yield Call(lambda: 8)
            events.append(result)
        self.assertTrue(m.startGuiTask(sequence()))
        self.assertFalse(m.startGuiTask(sequence()))
        spin_until(lambda: not m.isCommandBusy())
        timer.stop()
        self.assertEqual(events, [7,8]); self.assertGreater(len(ticks), 2)

    def test_stop_order_duplicate_and_cancel(self):
        m=self.manager(); events=[]; results=[]
        def pending():
            time.sleep(.04)
        def sequence():
            yield Call(pending)
            events.append('MUST NOT RUN')
        def stop():
            events.append('stop'); time.sleep(.02); return True
        m.ROS.stop_movement.side_effect=stop
        m.ROS.soft_movement_stop_client.call.side_effect=lambda: events.append('soft') or {'success':True}
        m.stopCompleted.connect(results.append)
        m.startGuiTask(sequence())
        self.assertTrue(m.requestStopAnyMovement(soft_stop=True))
        self.assertTrue(m.requestStopAnyMovement(soft_stop=True))
        spin_until(lambda: not m.isCommandBusy())
        self.assertEqual(events,['stop','soft']); self.assertEqual(results,[True])

    def test_stop_failure_reported(self):
        m=self.manager(); results=[]
        m.ROS.stop_movement.return_value=False
        m.stopCompleted.connect(results.append)
        m.requestStopAnyMovement()
        spin_until(lambda: not m.isCommandBusy())
        self.assertEqual(results,[False])

    def test_jog_release_preserves_motor_state(self):
        m=self.manager(); m.ROS.manual_reset_faults=True
        m.ROS.soft_movement_stop_client.call.return_value=UserDict({'success':True})
        m.requestStopAnyMovement(jog_axis=0)
        spin_until(lambda: not m.isCommandBusy())
        m.ROS.stop_movement.assert_not_called()
        m.ROS.turn_off_motors.assert_not_called()
        m.ROS.soft_movement_stop_client.call.assert_called_once()

    def test_shutdown_defers_cleanup_until_command_finishes(self):
        m=self.manager(); events=[]; results=[]
        def pending():
            time.sleep(.03); events.append('request returned')
        def sequence():
            yield Call(pending)
            events.append('MUST NOT RUN')
        def cleanup(): events.append('cleanup'); return True
        m._shutdown_blocking=cleanup
        m.shutdownFinished.connect(results.append)
        m.startGuiTask(sequence()); m.requestShutdown()
        spin_until(lambda: bool(results))
        self.assertEqual(events,['request returned','cleanup'])
        self.assertEqual(results,[True])

    def test_wrapper_no_stale_success(self):
        h=RosilibpyServiceHandler(Mock(), '/service', 'type')
        h.service_client=Mock()
        h.service_client.call.side_effect=[{'success':True},TimeoutError('simulated')]
        self.assertEqual(h.call(),{'success':True})
        self.assertIsNone(h.call()); self.assertIsNone(h.response)

    def test_cancel_prevents_new_motion_service_but_allows_motor_off(self):
        flag=threading.Event(); flag.set(); command_context.cancel=flag
        try:
            h=RosilibpyServiceHandler(Mock(), '/start_motion', 'type'); h.service_client=Mock()
            self.assertIsNone(h.call()); h.service_client.call.assert_not_called()
            h.namespace='/ethercat_checker/stop_motors'
            h.call(); h.service_client.call.assert_called_once()
        finally:
            command_context.cancel=None

    def test_mode_only_switch_and_failed_response(self):
        s=SyncRosManager.__new__(SyncRosManager)
        s._joint_names=['x']; s.current_controller_name='ctrl'
        s.coe_drive_states=types.SimpleNamespace(modes_of_operation=['MODE_CYCLIC_SYNC_POSITION'])
        s.switch_controller=Mock()
        def change(mode):
            s.coe_drive_states.modes_of_operation=['MODE_CYCLIC_SYNC_VELOCITY']; return True
        s.set_mode_of_operation=Mock(side_effect=change)
        self.assertTrue(s.controller_and_op_mode_switch(9,'ctrl'))
        s.switch_controller.assert_not_called()
        s.mode_of_op_client=Mock()
        self.assertTrue(SyncRosManager.set_mode_of_operation(s,9))
        s.mode_of_op_client.call.assert_not_called()
        s.mode_of_op_client.call.return_value=None
        self.assertFalse(SyncRosManager.set_mode_of_operation(s,8))

    def test_async_callbacks_are_per_request(self):
        h=RosilibpyServiceHandler(Mock(), '/service', 'type'); h.service_client=Mock()
        a=Mock(); b=Mock()
        h.call_async(on_done_callback=a); first=h.service_client.call.call_args.kwargs['callback']
        h.call_async(on_done_callback=b); second=h.service_client.call.call_args.kwargs['callback']
        first(1); second(2)
        a.assert_called_once_with(1); b.assert_called_once_with(2)

    def test_ros_userdict_controller_response(self):
        s=SyncRosManager.__new__(SyncRosManager)
        s._joint_names=['x']; s.current_controller_name='old'
        s.coe_drive_states=types.SimpleNamespace(modes_of_operation=['MODE_CYCLIC_SYNC_POSITION'])
        s.switch_controller=Mock(return_value=UserDict({'ok':True}))
        self.assertTrue(s.controller_and_op_mode_switch(8,'new'))

    def test_training_stop_slot_integration(self):
        source=Path(__file__).resolve().parents[1] / 'TrainingProtocolWindow.py'
        cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('stopTrainig','_onStopCompleted')]
        harness=ast.ClassDef(name='Harness',bases=[],keywords=[],body=methods,decorator_list=[])
        module=ast.fix_missing_locations(ast.Module(body=[harness],type_ignores=[]))
        scope={'QMessageBox':Mock()}; exec(compile(module,str(source),'exec'),scope)
        view=scope['Harness'](); m=self.manager(); view.ROS=m
        view._stop_pending=False; view.Training_ON=True
        view.ui=Mock(); view._stop_bag_recording=Mock(); view._set_training_buttons_idle=Mock()
        view.progressBarPhases=[Mock()]; view.spinBoxSpeedOvr=[Mock()]; view.spinBoxDuration=[Mock()]
        m.ROS.stop_movement.return_value=True
        m.ROS.soft_movement_stop_client.call.return_value=UserDict({'success':True})
        m.stopCompleted.connect(view._onStopCompleted)
        view.stopTrainig()
        self.assertTrue(view._stop_pending)
        view._set_training_buttons_idle.assert_not_called()
        view.stopTrainig()  # repeat timer/button event must not reset stop flags
        spin_until(lambda: not m.isCommandBusy())
        self.assertFalse(view._stop_pending)
        view._set_training_buttons_idle.assert_called_once()
        m.ROS.stop_movement.assert_called_once()

    def test_python38_syntax_all_files(self):
        for p in Path(__file__).resolve().parents[1].glob('*.py'):
            ast.parse(p.read_text(), feature_version=(3,8))

if __name__=='__main__': unittest.main()
