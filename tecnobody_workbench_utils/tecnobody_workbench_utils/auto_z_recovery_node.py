# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Automatic Z-axis recovery node.

When the recovery launch starts this node:
  1. Waits for eth_checker and filter_commands_node services.
  2. Calls /ethercat_checker/start_motors.
  3. Jogs Z+ by RECOVERY_DISTANCE_M at RECOVERY_VELOCITY_MPS.
  4. Calls /tecnobody_workbench_utils/soft_movement_stop.
  5. Calls /ethercat_checker/stop_motors.
  6. Exits.
"""

import enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tecnobody_msgs.srv import SoftMovement

RECOVERY_DISTANCE_M  = 0.05   # 5 cm
RECOVERY_VELOCITY_MPS = 0.01  # 1 cm/s
JOG_TIME_CONSTANT_S  = 0.3
JOINT_Z_NAME         = 'joint_z'
SETTLE_S             = 3.0    # time after services are up before starting


class _S(enum.Enum):
    WAITING        = 0  # waiting for services + first joint state
    SETTLING       = 1  # countdown before commanding motors
    STARTING_MOTORS = 2  # called start_motors, awaiting response
    JOGGING        = 3  # moving Z+, monitoring position
    STOPPING_JOG   = 4  # called soft_stop + stop_motors, awaiting responses
    DONE           = 5


class AutoZRecoveryNode(Node):

    def __init__(self):
        super().__init__('auto_z_recovery_node')

        self._state = _S.WAITING
        self._z_start = None
        self._z_current = None
        self._settle_start = None
        self._pending_start  = None   # Future for start_motors
        self._pending_jog_stop = None  # Future for soft_movement_stop
        self._pending_stop   = None   # Future for stop_motors

        self._start_motors_cli = self.create_client(
            Trigger, '/ethercat_checker/start_motors')
        self._stop_motors_cli = self.create_client(
            Trigger, '/ethercat_checker/stop_motors')
        self._jog_start_cli = self.create_client(
            SoftMovement, '/tecnobody_workbench_utils/soft_movement_start')
        self._jog_stop_cli = self.create_client(
            Trigger, '/tecnobody_workbench_utils/soft_movement_stop')

        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        self._ctrl_timer = self.create_timer(0.1, self._loop)
        self.get_logger().info(
            f'AutoZRecoveryNode started — target: Z+ '
            f'{RECOVERY_DISTANCE_M*100:.0f} cm at '
            f'{RECOVERY_VELOCITY_MPS*100:.0f} cm/s')

    # ------------------------------------------------------------------ #

    def _js_cb(self, msg: JointState):
        if JOINT_Z_NAME in msg.name:
            self._z_current = msg.position[msg.name.index(JOINT_Z_NAME)]

    def _services_ready(self) -> bool:
        return (self._start_motors_cli.service_is_ready()
                and self._stop_motors_cli.service_is_ready()
                and self._jog_start_cli.service_is_ready()
                and self._jog_stop_cli.service_is_ready())

    # ------------------------------------------------------------------ #

    def _loop(self):
        s = self._state

        if s == _S.WAITING:
            if self._services_ready() and self._z_current is not None:
                self._settle_start = self.get_clock().now()
                self.get_logger().info(
                    f'All services ready. Waiting {SETTLE_S:.0f} s for drives to settle...')
                self._state = _S.SETTLING

        elif s == _S.SETTLING:
            elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
            if elapsed >= SETTLE_S:
                self.get_logger().info('Settling done — calling start_motors...')
                self._pending_start = self._start_motors_cli.call_async(Trigger.Request())
                self._state = _S.STARTING_MOTORS

        elif s == _S.STARTING_MOTORS:
            if not self._pending_start.done():
                return
            result = self._pending_start.result()
            if not result.success:
                self.get_logger().error(
                    f'start_motors failed: {result.message}. Aborting recovery.')
                self._state = _S.DONE
                self.create_timer(0.5, self._shutdown)
                return
            self.get_logger().info(f'Motors on: {result.message}')
            self._z_start = self._z_current
            self.get_logger().info(
                f'Starting Z+ jog from z={self._z_start:.4f} m')
            req = SoftMovement.Request()
            req.data.target = 'jog'
            req.data.amplitude = float(RECOVERY_VELOCITY_MPS)
            req.data.time_constant = float(JOG_TIME_CONSTANT_S)
            req.data.jog_joint_idx = 2  # joint_z
            self._jog_start_cli.call_async(req)
            self._state = _S.JOGGING

        elif s == _S.JOGGING:
            distance = self._z_current - self._z_start
            if distance >= RECOVERY_DISTANCE_M:
                self.get_logger().info(
                    f'Moved {distance*100:.1f} cm — stopping jog and motors...')
                self._pending_jog_stop = self._jog_stop_cli.call_async(Trigger.Request())
                self._pending_stop = self._stop_motors_cli.call_async(Trigger.Request())
                self._state = _S.STOPPING_JOG

        elif s == _S.STOPPING_JOG:
            if not self._pending_jog_stop.done() or not self._pending_stop.done():
                return
            self.get_logger().info('Recovery complete — jog stopped, motors off.')
            self._state = _S.DONE
            self.create_timer(0.5, self._shutdown)

    # ------------------------------------------------------------------ #

    def _shutdown(self):
        self.get_logger().info('AutoZRecoveryNode shutting down.')
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AutoZRecoveryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
