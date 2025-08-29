import sys
from PyQt5.QtCore import pyqtSignal, QObject
import time

# mathematics
import numpy as np
from scipy.interpolate import CubicSpline

#MC Classes/methods

#ROS
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ethercat_controller_msgs.action import SetModesOfOperationAction

from action_msgs.msg import GoalStatus

def get_status_string(status):
    status_dict = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED'
    }
    return status_dict.get(status, 'UNKNOWN')

class FollowJointTrajectoryActionManager(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)
    stopped = pyqtSignal()

    def __init__(self, ros_node : Node, joint_names : list, controller_name='joint_trajectory_controller'):
        super().__init__() # type: ignore
        self.node = ros_node
        self._init_time_s = 0
        self._is_paused = False
        self._pause_start_time = None
        self._paused_duration = 0.0
        self._joint_names = joint_names
        self.action_server_timer = self.node.create_timer(0.5, self._check_client_ready)

    def _check_client_ready(self):
        if not hasattr(self, 'client'):
            self.client = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        if self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info('Follow Joint Trajectory Action server is available.')
            self.action_server_timer.cancel()
            self.clear()
        else:
            self.node.get_logger().info('Waiting for Follow Joint Trajectory Action server...', throttle_duration_sec=5.0)
            
    def clear(self):
        self.goalFCT = FollowJointTrajectory.Goal()
        self.goalFCT.trajectory.joint_names = self._joint_names

    def set(self, cartesian_positions, time_from_start,**kwargs):
        # Called only at the beginning
        self.clear()
        if type(time_from_start[0]) is not Duration: 
            self._total_time_s = (time_from_start[-1][0] - time_from_start[0][0])
        else:
            self._total_time_s = time_from_start[-1].sec - time_from_start[0].sec + (time_from_start[-1].nanosec - time_from_start[0].nanosec) * 1e-9
        if self._total_time_s <= 0:
            self._total_time_s = 1.0
        _numSample = len(time_from_start)
        t_spl = np.zeros(_numSample)
        self._time_from_start_duration = [0] * _numSample
        self._point_velocities = [0.0] * _numSample
        self._point_accelerations= [0.0] * _numSample
        for iPoint in range(0, _numSample):
            if type(time_from_start[iPoint]) is not Duration:
                _time_from_start_s = time_from_start[iPoint][0]
            else:
                _time_from_start_s = time_from_start[iPoint].sec + time_from_start[iPoint].nanosec * 1e-9
            t_spl[iPoint] = float(_time_from_start_s)
        # Interpolate trajectory and calculate velocities and accelerations
        P = np.array(cartesian_positions)
        # Set bc_type so that velocity is zero at start and end
        bc_type = ((1, 0.0), (1, 0.0))  # Only first derivative (velocity) = 0 at both ends
        splines = [CubicSpline(t_spl, P[:, i], bc_type=bc_type) for i in range(3)]
        for iPoint in range(_numSample):
            t_iPoint = t_spl[iPoint]
            pos = np.array([s(t_iPoint) for s in splines])
            vel = np.array([s.derivative(1)(t_iPoint) for s in splines])
            acc = np.array([s.derivative(2)(t_iPoint) for s in splines])
            self._time_from_start_duration[iPoint] = Duration(sec=int(t_iPoint),nanosec=int((t_iPoint - int(t_iPoint)) * 1e9))
            self._point_velocities[iPoint] = vel
            self.addPoint(pos, vel, acc, t_iPoint, **kwargs)

    def setSpeedOverride(self, speed_ovr : int = 100):
        self.speed_scale = speed_ovr / 100.0
        vel_scaled = [v * self.speed_scale for v in self._point_velocities]
        acc_scaled = [a * self.speed_scale*self.speed_scale for a in self._point_velocities]
        for iPoint in range(len(self.goalFCT.trajectory.points)):
            _d = self._time_from_start_duration[iPoint]
            _time_from_start_s = (_d.sec + (_d.nanosec / 1e9)) / self.speed_scale

            self.goalFCT.trajectory.points[iPoint].time_from_start = Duration(
                sec=int(_time_from_start_s),
                nanosec=int((_time_from_start_s - int(_time_from_start_s)) * 1e9)
            )
            self.goalFCT.trajectory.points[iPoint].velocities = vel_scaled[iPoint]
            self.goalFCT.trajectory.points[iPoint].accelerations = acc_scaled[iPoint]
            if iPoint==0 or iPoint==len(self.goalFCT.trajectory.points):
                if vel_scaled[iPoint].any() != 0:
                    for dof in range(len(vel_scaled[iPoint])):
                        if abs(vel_scaled[iPoint][dof]) < 1e-3:
                            vel_scaled[iPoint][dof] = 0.0
                        else: 
                            print(f"Warning: Velocity at point {iPoint} is not zero, but it should be. Vel: {vel_scaled[iPoint]}")
                if acc_scaled[iPoint].any() != 0:
                    for dof in range(len(acc_scaled[iPoint])):
                        if abs(vel_scaled[iPoint][dof]) < 1e-3:
                            acc_scaled[iPoint][dof] = 0.0
                        else: 
                            print(f"Warning: Acceleration at point {iPoint} is not zero, but it should be. Acc: {vel_scaled[iPoint]}")

    def addPoint(self, positions, velocities, accelerations, time, **kwargs):
        point=JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.effort = [] * len(positions)
        point.time_from_start = time
        self.goalFCT.trajectory.points.append(point)
        
    def startMovement(self, default_speed_ovr):
    # Called at the beginning of each movement
        self.setSpeedOverride(default_speed_ovr)
        self._init_time_s = time.time()
        self._paused_duration = 0.0
        self._last_actual_time_pct = 0.0
        self._pause_start_time = None
        self.goalFCT.trajectory.header.stamp = self.node.get_clock().now().to_msg()
        self._send_goal_future = self.client.send_goal_async(self.goalFCT)
        self._send_goal_future.add_done_callback(self.on_goal_accepted)

    def is_paused(self, trigger_pause):
        if trigger_pause:
            if not self._is_paused:
                self._pause_start_time = time.time()
            self._is_paused = True
            self.node.get_logger().info("⏸️ Paused movement")
        else:
            if self._is_paused and self._pause_start_time is not None:
                pause_duration = time.time() - self._pause_start_time
                self._paused_duration += pause_duration
                self.node.get_logger().info(f"▶️ Resumed movement after {pause_duration:.2f}s pause")
                self._pause_start_time = None
            self._is_paused = False

    def on_goal_accepted(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().info('Goal rejected!!')
            return
        self.node.get_logger().info('Goal accepted')
        self._get_goal_result_future = self._goal_handle.get_result_async()
        self._get_goal_result_future.add_done_callback(self.on_done)
        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self.timer = self.node.create_timer(0.02, lambda: self.check_status())

    def on_done(self, future):
        try:
            self.finished.emit()
            self.timer.cancel()
            self._goal_handle = None
        except Exception as e:
            self.node.get_logger().error(f'Exception in on_done: {e}')

    def check_status(self):
        if self._is_paused:
            self.progress.emit(self._last_actual_time_pct)
            return

        old_status = self._goal_status
        self._goal_status = self._goal_handle.status
        if old_status != self._goal_status:
            old_status_string = get_status_string(old_status)
            status_string = get_status_string(self._goal_status)
            self.node.get_logger().info(f'Action transition from {old_status_string} to {status_string}')

        actual_time = time.time()
        effective_time = actual_time - self._init_time_s - self._paused_duration
        total_scaled_time = self._total_time_s / self.speed_scale
        actual_time_from_start_percentage = (effective_time / total_scaled_time) * 100
        actual_time_from_start_percentage = min(actual_time_from_start_percentage, 100.0)

        self.progress.emit(int(actual_time_from_start_percentage))
        self._last_actual_time_pct = int(actual_time_from_start_percentage)

    def on_cancelled(self, future):
        if future.result() is not None:
            self.node.get_logger().info(f'Goal cancelled: {future.result()}')
        else:
            self.node.get_logger().info('Goal cancelled without result')
        self.clear()
        self.stopped.emit()
        self.goal_handle = None
    
    def stop(self):
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_cancelled)
        else:
            self.node.get_logger().info('goal handle has not been created yet')


class ModesOfOperationActionManager(QObject):
    finished    = pyqtSignal()
    stopped    = pyqtSignal()
    def __init__(self, ros_node : Node):
        super().__init__() # type: ignore
        self.node = ros_node
        self.goal_handle = None
        self.client = ActionClient(ros_node, SetModesOfOperationAction, '/state_controller/async_set_modes_of_operation')
        if not self.client.wait_for_server(2):
            raise ValueError('Set Modes of Operation Action server not available, exiting...')
    
    def switch_mode_of_operation(self, dof : str, new_mode: str):
        self.goal = SetModesOfOperationAction.Goal()
        self.goal.dof_name = dof
        self.goal.mode_of_operation = new_mode
        self.moo_future = self.client.send_goal_async(self.goal)
        self.moo_future.add_done_callback(self.on_goal_accepted)

    def on_goal_accepted(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().info('Goal rejected!!')
            return
        self.node.get_logger().info('Goal accepted')
        self._get_moo_goal_result_future = self.goal_handle.get_result_async()
        self._get_moo_goal_result_future.add_done_callback(self.on_done)

    def on_done(self, future):
        self.finished.emit()
        self.goal_handle = None

    def on_cancelled(self, future):
        if future.result() is not None:
            self.node.get_logger().info(f'Goal cancelled: {future.result().message}')
        else:
            self.node.get_logger().info('Goal cancelled without result')
        self.stopped.emit()
        self.goal_handle = None

    def stop(self):
        if hasattr(self, 'goal_handle') and self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_cancelled)
            self.goal = None
    

class ASyncRosManager(QObject):

    def __init__(self, ros_node, joint_names, controller_name='joint_trajectory_controller'):
        super(ASyncRosManager, self).__init__()
        
        self._fct = FollowJointTrajectoryActionManager(ros_node, joint_names, controller_name)
        self._moo = ModesOfOperationActionManager(ros_node)
        
    def moo(self) -> ModesOfOperationActionManager:
        return self._moo
    
    def fct(self) -> FollowJointTrajectoryActionManager:
        return self._fct

    def stop(self):
        self._fct.stop()
        self._moo.stop()
