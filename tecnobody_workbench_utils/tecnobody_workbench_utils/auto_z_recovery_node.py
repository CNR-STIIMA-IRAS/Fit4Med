# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Automatic Z-axis recovery node.

When the recovery launch starts this node:
  1. Waits for all services + first /joint_states.
  2. Sends switch_mode_of_operation(CSV=9) for all joints, then spins until
     get_drive_states confirms every joint is in MODE_CYCLIC_SYNC_VELOCITY.
  3. Calls /ethercat_checker/start_motors, then spins until drives_on == True.
  4. Jogs Z+ by RECOVERY_DISTANCE_M at RECOVERY_VELOCITY_MPS via filter_commands_node.
  5. Calls soft_movement_stop + stop_motors, then exits.
"""

import enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tecnobody_msgs.srv import SoftMovement
from ethercat_controller_msgs.srv import SwitchDriveModeOfOperation, GetDriveStates

RECOVERY_DISTANCE_M   = 0.05   # 5 cm
RECOVERY_VELOCITY_MPS = 0.01   # 1 cm/s
JOG_TIME_CONSTANT_S   = 0.3
JOINT_NAMES           = ['joint_x', 'joint_y', 'joint_z']
JOINT_Z_NAME          = 'joint_z'
SETTLE_S              = 3.0
CSV_MODE_STR          = 'MODE_CYCLIC_SYNC_VELOCITY'
CSV_MODE              = 9
VERIFY_TIMEOUT_S      = 15.0   # max time to wait for mode / motors-on confirmation


class _S(enum.Enum):
    WAITING          = 0
    SETTLING         = 1
    SETTING_MODE     = 2   # sent mode requests, now polling until confirmed
    STARTING_MOTORS  = 3   # sent start_motors, now polling until drives_on
    JOGGING          = 4
    STOPPING_JOG     = 5
    DONE             = 6


class AutoZRecoveryNode(Node):

    def __init__(self):
        super().__init__('auto_z_recovery_node')

        self._state          = _S.WAITING
        self._z_start        = None
        self._z_current      = None
        self._settle_start   = None
        self._verify_start   = None   # timeout reference for verification loops
        self._poll_future    = None   # current get_drive_states future
        self._pending_start  = None   # start_motors future
        self._pending_jog_stop  = None
        self._pending_stop   = None

        self._set_mode_cli = self.create_client(
            SwitchDriveModeOfOperation,
            '/state_controller/switch_mode_of_operation')
        self._get_states_cli = self.create_client(
            GetDriveStates,
            '/ethercat_checker/get_drive_states')
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
        return (self._set_mode_cli.service_is_ready()
                and self._get_states_cli.service_is_ready()
                and self._start_motors_cli.service_is_ready()
                and self._stop_motors_cli.service_is_ready()
                and self._jog_start_cli.service_is_ready()
                and self._jog_stop_cli.service_is_ready())

    def _abort(self, reason: str):
        self.get_logger().error(f'Aborting recovery: {reason}')
        self._state = _S.DONE
        self.create_timer(0.5, self._shutdown)

    def _verify_timed_out(self) -> bool:
        if self._verify_start is None:
            return False
        elapsed = (self.get_clock().now() - self._verify_start).nanoseconds / 1e9
        return elapsed > VERIFY_TIMEOUT_S

    def _send_mode_requests(self):
        for joint in JOINT_NAMES:
            req = SwitchDriveModeOfOperation.Request()
            req.dof_name = joint
            req.mode_of_operation = CSV_MODE
            self._set_mode_cli.call_async(req)

    def _poll_drive_states(self):
        self._poll_future = self._get_states_cli.call_async(GetDriveStates.Request())

    # ------------------------------------------------------------------ #

    def _loop(self):
        s = self._state

        # ── 1. Wait for all services and first joint state ──────────────
        if s == _S.WAITING:
            if self._services_ready() and self._z_current is not None:
                self._settle_start = self.get_clock().now()
                self.get_logger().info(
                    f'All services ready. Waiting {SETTLE_S:.0f} s for drives to settle...')
                self._state = _S.SETTLING

        # ── 2. Settle countdown ─────────────────────────────────────────
        elif s == _S.SETTLING:
            elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
            if elapsed >= SETTLE_S:
                self.get_logger().info(
                    f'Setting mode of operation to {CSV_MODE_STR} on all joints...')
                self._send_mode_requests()
                self._verify_start = self.get_clock().now()
                self._poll_drive_states()
                self._state = _S.SETTING_MODE

        # ── 3. Spin until all joints report CSV mode ────────────────────
        elif s == _S.SETTING_MODE:
            if self._poll_future is None or not self._poll_future.done():
                return
            if self._verify_timed_out():
                self._abort(f'Timeout waiting for {CSV_MODE_STR}')
                return

            modes = self._poll_future.result().states.modes_of_operation
            self.get_logger().debug(f'  modes: {list(modes)}')
            if all(m == CSV_MODE_STR for m in modes):
                self.get_logger().info(
                    f'All joints confirmed in {CSV_MODE_STR}. Calling start_motors...')
                self._pending_start = self._start_motors_cli.call_async(Trigger.Request())
                self._verify_start = self.get_clock().now()
                self._poll_future = None
                self._state = _S.STARTING_MOTORS
            else:
                # Re-send mode requests and poll again
                self._send_mode_requests()
                self._poll_drive_states()

        # ── 4. Spin until drives_on == True ─────────────────────────────
        elif s == _S.STARTING_MOTORS:
            # First wait for start_motors response
            if self._pending_start is not None and not self._pending_start.done():
                return
            if self._pending_start is not None and self._pending_start.done():
                result = self._pending_start.result()
                if not result.success:
                    self._abort(f'start_motors failed: {result.message}')
                    return
                self.get_logger().info(f'start_motors: {result.message}')
                self._pending_start = None
                # Start polling drives_on
                self._poll_drive_states()
                return

            # Poll until drives_on
            if self._poll_future is None or not self._poll_future.done():
                return
            if self._verify_timed_out():
                self._abort('Timeout waiting for drives_on')
                return

            drives_on = self._poll_future.result().states.drives_on
            if drives_on:
                self.get_logger().info('Motors on — starting Z+ jog.')
                self._z_start = self._z_current
                req = SoftMovement.Request()
                req.data.target = 'jog'
                req.data.amplitude = float(RECOVERY_VELOCITY_MPS)
                req.data.time_constant = float(JOG_TIME_CONSTANT_S)
                req.data.jog_joint_idx = 2  # joint_z
                self._jog_start_cli.call_async(req)
                self._poll_future = None
                self._state = _S.JOGGING
            else:
                self._poll_drive_states()

        # ── 5. Monitor Z position ────────────────────────────────────────
        elif s == _S.JOGGING:
            distance = self._z_current - self._z_start
            if distance >= RECOVERY_DISTANCE_M:
                self.get_logger().info(
                    f'Moved {distance*100:.1f} cm — stopping.')
                self._pending_jog_stop = self._jog_stop_cli.call_async(Trigger.Request())
                self._pending_stop = self._stop_motors_cli.call_async(Trigger.Request())
                self._state = _S.STOPPING_JOG

        # ── 6. Wait for stop acks ────────────────────────────────────────
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
