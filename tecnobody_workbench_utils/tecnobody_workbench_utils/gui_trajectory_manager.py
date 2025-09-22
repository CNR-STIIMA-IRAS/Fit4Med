import sys
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
from rclpy.client import Client
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ethercat_controller_msgs.action import SetModesOfOperationAction
from tecnobody_msgs.srv import SetTrajectory, StartMovement, MovementProgress
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger

from rich.traceback import install
install(show_locals=True)

class FollowJointTrajectoryActionManager(Node):

    def __init__(self, controller_name='joint_trajectory_controller'):
        super().__init__("fct_manager_node") # type: ignore
        self._init_time_s = 0
        self._is_paused = False
        self._pause_start_time = None
        self._paused_duration = 0.0
        self._joint_names = ['joint_x', 'joint_y', 'joint_z']
        self.set_trajectory_server = self.create_service(
            SetTrajectory,
            "/tecnobody_workbench_utils/set_trajectory",
            self.set
        )
        self.start_movement_server = self.create_service(
            StartMovement,
            "/tecnobody_workbench_utils/start_movement",
            self.startMovement
        )
        self.stop_movement_server = self.create_service(
            Trigger,
            "/tecnobody_workbench_utils/stop_movement",
            self.stop
        )
        self.client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        if self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('Follow Joint Trajectory Action server is available.')
            
    def clear(self):
        self.goalFCT = FollowJointTrajectory.Goal()
        self.goalFCT.trajectory.joint_names = self._joint_names
    
    def set(self, request, response):
        cartesian_positions = [r.point for r in request.cartesian_positions]
        time_from_start = [r.time_from_start for r in request.cartesian_positions]
        # Called only at the beginning
        self.clear()
        self._total_time_s = (time_from_start[-1] - time_from_start[0])
        if self._total_time_s <= 0:
            self._total_time_s = 1.0
        _numSample = len(time_from_start)
        t_spl = np.zeros(_numSample)
        self._time_from_start_duration = [0] * _numSample
        self._point_velocities = [0.0] * _numSample
        self._point_accelerations= [0.0] * _numSample
        for iPoint in range(0, _numSample):
            _time_from_start_s = time_from_start[iPoint]
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
            self.addPoint(pos, vel, acc, t_iPoint)
        self.go_home_mode = False
        response.success = True
        return response
    
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
                            self.get_logger().info(f"Warning: Velocity at point {iPoint} is not zero, but it should be. Vel: {vel_scaled[iPoint]}")
                if acc_scaled[iPoint].any() != 0:
                    for dof in range(len(acc_scaled[iPoint])):
                        if abs(vel_scaled[iPoint][dof]) < 1e-3:
                            acc_scaled[iPoint][dof] = 0.0
                        else:
                            self.get_logger().info(f"Warning: Acceleration at point {iPoint} is not zero, but it should be. Acc: {vel_scaled[iPoint]}")
    
    def addPoint(self, positions, velocities, accelerations, time):
        point=JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.effort = [] * len(positions)
        point.time_from_start = time
        self.goalFCT.trajectory.points.append(point)
    
    def startMovement(self, request, response):
    # Called at the beginning of each movement
        default_speed_ovr = request.speed_ovr
        self.setSpeedOverride(default_speed_ovr)
        self._init_time_s = time.time()
        self._paused_duration = 0.0
        self._last_actual_time_pct = 0.0
        self._pause_start_time = None
        self.goalFCT.trajectory.header.stamp = self.get_clock().now().to_msg()
        self._send_goal_future = self.client.send_goal_async(self.goalFCT)
        self._send_goal_future.add_done_callback(self.on_goal_accepted)
        self.go_home_mode = False
        response.success = True
        return response
    
    # def is_paused(self, trigger_pause):
    #     if trigger_pause:
    #         if not self._is_paused:
    #             self._pause_start_time = time.time()
    #         self._is_paused = True
    #         self.get_logger().info("⏸️ Paused movement")
    #     else:
    #         if self._is_paused and self._pause_start_time is not None:
    #             pause_duration = time.time() - self._pause_start_time
    #             self._paused_duration += pause_duration
    #             self.get_logger().info(f"▶️ Resumed movement after {pause_duration:.2f}s pause")
    #             self._pause_start_time = None
    #         self._is_paused = False
    
    def on_goal_accepted(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected!!')
            return
        self.get_logger().info('Goal accepted!!!!!!')
        self._get_goal_result_future = self._goal_handle.get_result_async()
        self._get_goal_result_future.add_done_callback(self.on_done)
        self._goal_status = GoalStatus.STATUS_UNKNOWN
        self.timer = self.create_timer(0.02, lambda: self.check_status())
    
    def on_done(self, future):
        try:
            client : Client = self.create_client(Trigger, '/rehab_gui/fct_finished')
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Trajectory DONE server is not available.')
            req = Trigger.Request()               
            client.call_async(req)
            self.get_logger().info('Trajectory DONE sent to GUI')                
            self._goal_handle = None
            if hasattr(self, 'timer'):
                self.timer.cancel()
        except Exception as e:
            self.get_logger().info(f'Exception in on_done: {e}')
    
    def check_status(self):
        if self._is_paused:
            self.progress.emit(self._last_actual_time_pct)
            return
    
        old_status = self._goal_status
        self._goal_status = self._goal_handle.status
        # if old_status != self._goal_status:
        #     old_status_string = get_status_string(old_status)
        #     status_string = get_status_string(self._goal_status)
        #     self.get_logger().info(f'Action transition from {old_status_string} to {status_string}')
    
        actual_time = time.time()
        effective_time = actual_time - self._init_time_s - self._paused_duration
        total_scaled_time = self._total_time_s / self.speed_scale
        actual_time_from_start_percentage = (effective_time / total_scaled_time) * 100
        actual_time_from_start_percentage = min(actual_time_from_start_percentage, 100.0)
    
        client = self.create_client(MovementProgress, "/rehab_gui/fct_progress")

        req = MovementProgress.Request()
        req.progress = actual_time_from_start_percentage
        client.call_async(req)
        self._last_actual_time_pct = int(actual_time_from_start_percentage)
    
    def on_cancelled(self, future):
        if future.result() is not None:
            self.get_logger().info(f'Goal cancelled: {future.result()}')
        else:
            self.get_logger().info('Goal cancelled without result')
        self.clear()
        self.goal_handle = None
    
    def stop(self, request, response):
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_cancelled)
            response.success = True
        else:
            self.get_logger().info('goal handle has not been created yet')
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    import os
    os.sched_setaffinity(0, {6})

    node = FollowJointTrajectoryActionManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            time.sleep(0.01)
            
        node.get_logger().info('^^^^^^^^^^^^^^^^^ Shutting down...')
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()