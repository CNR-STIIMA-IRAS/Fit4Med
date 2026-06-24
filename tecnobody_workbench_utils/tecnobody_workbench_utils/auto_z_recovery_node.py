# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Automatic Z-axis recovery node.

Self-healing state machine: never aborts. Runs until the full precondition
is met (no faults, all joints in CSV mode, drives_on=True), then jogs Z+
by RECOVERY_DISTANCE_M and stops.

State flow:
  WAITING → SETTLING → CHECKING ←──────────────────────────────────┐
                            │ fault_present                         │
                            ├──────────────→ RESETTING_FAULTS ─────┤ (retry delay)
                            │ mode != CSV                           │
                            ├──────────────→ SETTING_MODE ──────────┤
                            │ not drives_on                         │
                            ├──────────────→ STARTING_MOTORS ───────┘
                            │ all ok
                            └──────────────→ JOGGING → STOPPING_JOG → DONE
"""

import enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tecnobody_msgs.srv import SoftMovement
from tecnobody_msgs.msg import PlcController
from ethercat_controller_msgs.srv import SwitchDriveModeOfOperation, GetDriveStates

RECOVERY_DISTANCE_M   = 0.05   # 5 cm
RECOVERY_VELOCITY_MPS = 0.01   # 1 cm/s
JOG_TIME_CONSTANT_S   = 0.3
JOINT_NAMES           = ['joint_x', 'joint_y', 'joint_z']
JOINT_Z_NAME          = 'joint_z'
SETTLE_S              = 3.0
CSV_MODE_STR          = 'MODE_CYCLIC_SYNC_VELOCITY'
CSV_MODE              = 9
RETRY_DELAY_S         = 1.5   # pause between check cycles
LOG_PERIOD_S          = 3.0   # max rate for repeating status messages


class _S(enum.Enum):
    WAITING          = 0
    SETTLING         = 1
    CHECKING         = 2   # read hardware state → route to fix or jog
    RESETTING_FAULTS = 3   # call reset_fault, then RETRY_DELAY → CHECKING
    SETTING_MODE     = 4   # send mode switch requests, then RETRY_DELAY → CHECKING
    STARTING_MOTORS  = 5   # call start_motors, then RETRY_DELAY → CHECKING
    RETRY_DELAY      = 6   # fixed pause before returning to CHECKING
    JOGGING          = 7
    STOPPING_JOG     = 8
    DONE             = 9


class AutoZRecoveryNode(Node):

    def __init__(self):
        super().__init__('auto_z_recovery_node')

        self._state         = _S.WAITING
        self._z_start       = None
        self._z_current     = None
        self._settle_start  = None
        self._delay_start   = None   # timestamp for RETRY_DELAY
        self._active_future = None   # single in-flight service future
        self._last_log      = {}     # state → last log time, to avoid spam

        self._pending_jog_stop = None
        self._pending_stop     = None

        plc_qos = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self._plc_pub = self.create_publisher(
            PlcController, '/PLC_controller/plc_commands', plc_qos)

        self._get_states_cli = self.create_client(
            GetDriveStates, '/ethercat_checker/get_drive_states')
        self._reset_fault_cli = self.create_client(
            Trigger, '/state_controller/reset_fault')
        self._set_mode_cli = self.create_client(
            SwitchDriveModeOfOperation,
            '/state_controller/switch_mode_of_operation')
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
            f'AutoZRecoveryNode — target Z+ {RECOVERY_DISTANCE_M*100:.0f} cm '
            f'at {RECOVERY_VELOCITY_MPS*100:.0f} cm/s. '
            f'Will retry until preconditions are met.')

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _js_cb(self, msg: JointState):
        if JOINT_Z_NAME in msg.name:
            self._z_current = msg.position[msg.name.index(JOINT_Z_NAME)]

    def _services_ready(self) -> bool:
        return (self._get_states_cli.service_is_ready()
                and self._reset_fault_cli.service_is_ready()
                and self._set_mode_cli.service_is_ready()
                and self._start_motors_cli.service_is_ready()
                and self._stop_motors_cli.service_is_ready()
                and self._jog_start_cli.service_is_ready()
                and self._jog_stop_cli.service_is_ready())

    def _throttled_log(self, key: str, msg: str):
        """Log msg at most once every LOG_PERIOD_S seconds for the given key."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log.get(key, 0.0) >= LOG_PERIOD_S:
            self.get_logger().info(msg)
            self._last_log[key] = now

    def _go_check(self):
        """Transition to CHECKING with a fresh poll."""
        self._active_future = self._get_states_cli.call_async(GetDriveStates.Request())
        self._state = _S.CHECKING

    def _go_delay_then_check(self):
        """Brief pause, then back to CHECKING."""
        self._delay_start = self.get_clock().now()
        self._state = _S.RETRY_DELAY

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #

    def _loop(self):
        s = self._state

        # ── 1. Wait for all services + first joint state ─────────────────
        if s == _S.WAITING:
            if self._services_ready() and self._z_current is not None:
                self._settle_start = self.get_clock().now()
                self.get_logger().info(
                    f'All services ready. Settling for {SETTLE_S:.0f} s...')
                self._state = _S.SETTLING
            else:
                self._throttled_log('waiting', 'Waiting for services and joint states...')

        # ── 2. Settle countdown ──────────────────────────────────────────
        elif s == _S.SETTLING:
            elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
            if elapsed >= SETTLE_S:
                self.get_logger().info('Settle done — entering check loop.')
                self._go_check()

        # ── 3. Central check: read hardware, route to correct fix ────────
        elif s == _S.CHECKING:
            if not self._active_future.done():
                return

            states = self._active_future.result().states
            self._active_future = None

            fault   = states.fault_present
            drives  = states.drives_on
            modes   = list(states.modes_of_operation)
            dstates = list(states.drive_states)
            all_csv = all(m == CSV_MODE_STR for m in modes)

            self.get_logger().debug(
                f'CHECK  fault={fault}  drives_on={drives}  '
                f'modes={modes}  states={dstates}')

            if fault:
                self.get_logger().warning(
                    f'Fault present (states={dstates}) — resetting faults.')
                self._active_future = self._reset_fault_cli.call_async(Trigger.Request())
                self._state = _S.RESETTING_FAULTS

            elif not all_csv:
                self.get_logger().info(
                    f'Modes not CSV yet ({modes}) — sending mode switch.')
                self._send_csv_mode()
                self._go_delay_then_check()

            elif not drives:
                self.get_logger().info(
                    f'Drives not on (states={dstates}) — enabling manual mode, calling start_motors.')
                self._publish_plc('PLC_node/manual_mode', 1)
                self._active_future = self._start_motors_cli.call_async(Trigger.Request())
                self._state = _S.STARTING_MOTORS

            else:
                # All preconditions met
                self.get_logger().info(
                    f'All OK (modes={modes}, drives_on={drives}). Starting Z+ jog.')
                self._start_jog()

        # ── 4. Wait for reset_fault, then re-check ───────────────────────
        elif s == _S.RESETTING_FAULTS:
            if not self._active_future.done():
                return
            msg = self._active_future.result().message
            self.get_logger().info(f'reset_fault response: {msg}')
            self._active_future = None
            self._go_delay_then_check()

        # ── 5. Wait for start_motors, log result, then re-check ──────────
        elif s == _S.STARTING_MOTORS:
            if not self._active_future.done():
                return
            result = self._active_future.result()
            if result.success:
                self.get_logger().info(f'start_motors OK: {result.message}')
            else:
                self.get_logger().warning(
                    f'start_motors failed: {result.message} — will retry.')
            self._active_future = None
            self._go_delay_then_check()

        # ── 6. Fixed pause before returning to CHECKING ──────────────────
        elif s == _S.RETRY_DELAY:
            elapsed = (self.get_clock().now() - self._delay_start).nanoseconds / 1e9
            if elapsed >= RETRY_DELAY_S:
                self._go_check()

        # ── 7. Jog Z+ until RECOVERY_DISTANCE_M ─────────────────────────
        elif s == _S.JOGGING:
            distance = self._z_current - self._z_start
            self._throttled_log('jogging', f'Jogging Z+: {distance*100:.1f} cm / '
                                f'{RECOVERY_DISTANCE_M*100:.0f} cm')
            if distance >= RECOVERY_DISTANCE_M:
                self.get_logger().info(
                    f'Moved {distance*100:.1f} cm — stopping jog and motors.')
                self._pending_jog_stop = self._jog_stop_cli.call_async(Trigger.Request())
                self._pending_stop = self._stop_motors_cli.call_async(Trigger.Request())
                self._state = _S.STOPPING_JOG

        # ── 8. Wait for both stop acks ───────────────────────────────────
        elif s == _S.STOPPING_JOG:
            if not self._pending_jog_stop.done() or not self._pending_stop.done():
                return
            self._publish_plc('PLC_node/manual_mode', 0)
            self.get_logger().info('Recovery complete — jog stopped, motors off, manual mode cleared.')
            self._state = _S.DONE
            self.create_timer(0.5, self._shutdown)

    # ------------------------------------------------------------------ #

    def _publish_plc(self, interface_name: str, value: int):
        msg = PlcController()
        msg.interface_names = [interface_name]
        msg.values = [value]
        self._plc_pub.publish(msg)

    def _send_csv_mode(self):
        for joint in JOINT_NAMES:
            req = SwitchDriveModeOfOperation.Request()
            req.dof_name = joint
            req.mode_of_operation = CSV_MODE
            self._set_mode_cli.call_async(req)

    def _start_jog(self):
        self._z_start = self._z_current
        req = SoftMovement.Request()
        req.data.target = 'jog'
        req.data.amplitude = float(RECOVERY_VELOCITY_MPS)
        req.data.time_constant = float(JOG_TIME_CONSTANT_S)
        req.data.jog_joint_idx = 2  # joint_z
        self._jog_start_cli.call_async(req)
        self._state = _S.JOGGING

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
