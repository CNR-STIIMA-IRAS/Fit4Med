# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Automatic Z-axis recovery node.

When the recovery launch starts this node waits for filter_commands_node's
soft_movement_start service to become available, then jogs Z+ by
RECOVERY_DISTANCE_M meters at RECOVERY_VELOCITY_MPS and exits.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tecnobody_msgs.srv import SoftMovement

RECOVERY_DISTANCE_M = 0.05   # 5 cm
RECOVERY_VELOCITY_MPS = 0.01  # 1 cm/s → 5 s to travel 5 cm
JOG_TIME_CONSTANT_S = 0.3    # ramp duration
JOINT_Z_NAME = 'joint_z'
SETTLE_S = 3.0               # time after service is up before starting jog


class AutoZRecoveryNode(Node):

    def __init__(self):
        super().__init__('auto_z_recovery_node')

        self._z_start = None
        self._z_current = None
        self._jog_started = False
        self._done = False
        self._settle_start = None

        self._start_cli = self.create_client(
            SoftMovement, '/tecnobody_workbench_utils/soft_movement_start')
        self._stop_cli = self.create_client(
            Trigger, '/tecnobody_workbench_utils/soft_movement_stop')

        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        self._ctrl_timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f'AutoZRecoveryNode started — waiting for soft_movement_start service '
            f'(recover {RECOVERY_DISTANCE_M*100:.0f} cm at '
            f'{RECOVERY_VELOCITY_MPS*100:.0f} cm/s)...')

    # ------------------------------------------------------------------ #

    def _js_cb(self, msg: JointState):
        if JOINT_Z_NAME in msg.name:
            idx = msg.name.index(JOINT_Z_NAME)
            self._z_current = msg.position[idx]

    def _control_loop(self):
        if self._done:
            return

        # Wait for filter_commands_node service and at least one joint state
        if not self._start_cli.service_is_ready():
            return
        if self._z_current is None:
            return

        # Start settle countdown once service is up
        if self._settle_start is None:
            self._settle_start = self.get_clock().now()
            self.get_logger().info(
                f'Services ready. Waiting {SETTLE_S:.0f} s for drives to settle...')
            return

        elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
        if elapsed < SETTLE_S:
            return

        # Record start position and send jog command (once)
        if not self._jog_started:
            self._z_start = self._z_current
            self.get_logger().info(
                f'Starting auto Z+ jog from z={self._z_start:.4f} m, '
                f'target: +{RECOVERY_DISTANCE_M*100:.0f} cm')
            self._send_jog_start()
            self._jog_started = True
            return

        # Monitor progress
        distance = self._z_current - self._z_start
        if distance >= RECOVERY_DISTANCE_M:
            self.get_logger().info(
                f'Recovery complete: moved {distance*100:.1f} cm. Stopping jog.')
            self._send_jog_stop()
            self._done = True
            # Let the timer fire one more tick then shut down
            self.create_timer(1.0, self._shutdown)

    def _send_jog_start(self):
        req = SoftMovement.Request()
        req.data.target = 'jog'
        req.data.amplitude = float(RECOVERY_VELOCITY_MPS)
        req.data.time_constant = float(JOG_TIME_CONSTANT_S)
        req.data.jog_joint_idx = 2  # joint_z
        self._start_cli.call_async(req)

    def _send_jog_stop(self):
        if self._stop_cli.service_is_ready():
            self._stop_cli.call_async(Trigger.Request())

    def _shutdown(self):
        self.get_logger().info('AutoZRecoveryNode done — shutting down.')
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
