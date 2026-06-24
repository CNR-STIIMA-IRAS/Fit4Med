# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Automatic Z-axis recovery node.

Replicates what plc_manager + rehab_gui do together during z_recovery:

1. Publishes PLC_node/z_recovery=0 so the PLC safety relay bypasses the
   z_limit interlock, allowing the estop key to be turned even with the
   limit switch active.  (plc_manager did this to the OLD ros2_control_node;
   we must repeat it to the NEW one that just started in the recovery env.)
2. Waits until drives exit STATE_SWITCH_ON_DISABLED  ← key has been turned,
   drives now have power.
3. Self-healing check loop (never aborts):
     fault_present  →  reset_fault
     mode != CSV    →  switch_mode_of_operation
     not drives_on  →  start_motors
     all OK         →  jog Z+ by RECOVERY_DISTANCE_M
4. After jogging: stops jog + stops motors.

State flow:
  WAITING → SETTLING → INIT_RECOVERY → WAIT_FOR_POWER →
  CHECKING ←──────────────────────────────────────────┐
       │ fault                                         │
       ├──→ RESETTING_FAULTS ──→ RETRY_DELAY ──────────┤
       │ mode != CSV                                    │
       ├──→ SETTING_MODE ──→ RETRY_DELAY ───────────────┤
       │ not drives_on                                  │
       ├──→ STARTING_MOTORS ──→ RETRY_DELAY ────────────┘
       │ all OK
       └──→ JOGGING → STOPPING_JOG → DONE
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
DISABLED_STR          = 'STATE_SWITCH_ON_DISABLED'
RETRY_DELAY_S         = 1.5
LOG_PERIOD_S          = 3.0


class _S(enum.Enum):
    WAITING          = 0
    SETTLING         = 1
    INIT_RECOVERY    = 2   # publish z_recovery=0, then wait for power
    WAIT_FOR_POWER   = 3   # poll until drives leave STATE_SWITCH_ON_DISABLED
    CHECKING         = 4   # read hardware → route to correct fix
    RESETTING_FAULTS = 5
    SETTING_MODE     = 6
    STARTING_MOTORS  = 7
    RETRY_DELAY      = 8
    JOGGING          = 9
    STOPPING_JOG     = 10
    DONE             = 11


class AutoZRecoveryNode(Node):

    def __init__(self):
        super().__init__('auto_z_recovery_node')

        self._state         = _S.WAITING
        self._z_start       = None
        self._z_current     = None
        self._settle_start  = None
        self._delay_start   = None
        self._active_future = None
        self._last_log      = {}

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
            f'AutoZRecoveryNode started — target Z+ '
            f'{RECOVERY_DISTANCE_M*100:.0f} cm at {RECOVERY_VELOCITY_MPS*100:.0f} cm/s')

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

    def _throttled_log(self, key: str, msg: str, level: str = 'info'):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_log.get(key, 0.0) >= LOG_PERIOD_S:
            getattr(self.get_logger(), level)(msg)
            self._last_log[key] = now

    def _publish_plc(self, interface_name: str, value: int):
        msg = PlcController()
        msg.interface_names = [interface_name]
        msg.values = [value]
        self._plc_pub.publish(msg)

    def _go_check(self):
        self._active_future = self._get_states_cli.call_async(GetDriveStates.Request())
        self._state = _S.CHECKING

    def _go_delay_then_check(self):
        self._delay_start = self.get_clock().now()
        self._state = _S.RETRY_DELAY

    def _send_csv_mode(self):
        for joint in JOINT_NAMES:
            req = SwitchDriveModeOfOperation.Request()
            req.dof_name = joint
            req.mode_of_operation = CSV_MODE
            self._set_mode_cli.call_async(req)

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
                    f'All services ready. Settling {SETTLE_S:.0f} s...')
                self._state = _S.SETTLING
            else:
                self._throttled_log('waiting', 'Waiting for services and /joint_states...')

        # ── 2. Settle countdown ──────────────────────────────────────────
        elif s == _S.SETTLING:
            elapsed = (self.get_clock().now() - self._settle_start).nanoseconds / 1e9
            if elapsed >= SETTLE_S:
                self._state = _S.INIT_RECOVERY

        # ── 3. Publish z_recovery=0 to bypass z_limit in safety relay ───
        #       (plc_manager set it on the OLD ros2_control_node;
        #        the NEW node in the recovery env needs it again)
        elif s == _S.INIT_RECOVERY:
            self.get_logger().info(
                'Publishing PLC_node/z_recovery=0 — bypassing z_limit safety interlock.')
            self._publish_plc('PLC_node/z_recovery', 0)
            self.get_logger().info(
                'Waiting for operator to turn the recovery key to clear the emergency...')
            self._active_future = self._get_states_cli.call_async(GetDriveStates.Request())
            self._state = _S.WAIT_FOR_POWER

        # ── 4. Spin until drives leave STATE_SWITCH_ON_DISABLED ──────────
        #       (that transition means the key was turned, drives have power)
        elif s == _S.WAIT_FOR_POWER:
            if not self._active_future.done():
                return

            dstates = list(self._active_future.result().states.drive_states)
            self._active_future = None

            if all(d == DISABLED_STR for d in dstates):
                self._throttled_log(
                    'power',
                    f'Emergency still active (states={dstates}). '
                    'Turn the recovery key to proceed...')
                # Re-publish z_recovery=0 each poll cycle so it is never lost
                self._publish_plc('PLC_node/z_recovery', 0)
                self._active_future = self._get_states_cli.call_async(GetDriveStates.Request())
            else:
                self.get_logger().info(
                    f'Emergency cleared — drives have power (states={dstates}).')
                self._go_check()

        # ── 5. Central check: read hardware, route to correct fix ────────
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
                    f'Fault detected (states={dstates}) — resetting.')
                self._active_future = self._reset_fault_cli.call_async(Trigger.Request())
                self._state = _S.RESETTING_FAULTS

            elif not all_csv:
                self.get_logger().info(
                    f'Setting CSV mode on all joints (current={modes}).')
                self._send_csv_mode()
                self._go_delay_then_check()

            elif not drives:
                self.get_logger().info(
                    f'Starting motors (states={dstates}).')
                self._active_future = self._start_motors_cli.call_async(Trigger.Request())
                self._state = _S.STARTING_MOTORS

            else:
                self.get_logger().info(
                    'All preconditions met — starting Z+ jog.')
                self._z_start = self._z_current
                req = SoftMovement.Request()
                req.data.target = 'jog'
                req.data.amplitude = float(RECOVERY_VELOCITY_MPS)
                req.data.time_constant = float(JOG_TIME_CONSTANT_S)
                req.data.jog_joint_idx = 2  # joint_z
                self._jog_start_cli.call_async(req)
                self._state = _S.JOGGING

        # ── 6. Wait for reset_fault → retry ──────────────────────────────
        elif s == _S.RESETTING_FAULTS:
            if not self._active_future.done():
                return
            self.get_logger().info(
                f'reset_fault: {self._active_future.result().message}')
            self._active_future = None
            self._go_delay_then_check()

        # ── 7. Wait for start_motors → retry regardless of result ────────
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

        # ── 8. Fixed pause before returning to CHECKING ──────────────────
        elif s == _S.RETRY_DELAY:
            elapsed = (self.get_clock().now() - self._delay_start).nanoseconds / 1e9
            if elapsed >= RETRY_DELAY_S:
                self._go_check()

        # ── 9. Jog Z+ until RECOVERY_DISTANCE_M ─────────────────────────
        elif s == _S.JOGGING:
            distance = self._z_current - self._z_start
            self._throttled_log(
                'jogging',
                f'Jogging Z+: {distance*100:.1f} / {RECOVERY_DISTANCE_M*100:.0f} cm')
            if distance >= RECOVERY_DISTANCE_M:
                self.get_logger().info(
                    f'Moved {distance*100:.1f} cm — stopping.')
                self._pending_jog_stop = self._jog_stop_cli.call_async(Trigger.Request())
                self._pending_stop = self._stop_motors_cli.call_async(Trigger.Request())
                self._state = _S.STOPPING_JOG

        # ── 10. Wait for both stop acks ──────────────────────────────────
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
