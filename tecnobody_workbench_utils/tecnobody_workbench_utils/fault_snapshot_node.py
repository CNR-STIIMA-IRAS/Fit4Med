# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
"""Keep the last seconds of joint states and save them when a drive faults.

Run 0093: joint_y stopped with a drive fault (AL006, overload), but the drive
torque before the fault was unknown: ec_link_monitor samples it every 10 s.
This node keeps /joint_states (position, velocity, effort; effort is the drive
torque actual value 0x6077, per mille of the rated torque) in a ring buffer of
pre_fault_s seconds. When a drive enters FAULT (from
/ethercat_checker/drive_states), it records post_fault_s more seconds and
writes everything to fault_snapshot_<time>.csv in ROS_LOG_DIR, i.e. the folder
of the current start, which ends up in the session zip.

The CSV starts with '#' lines: fault time, drives in fault, the drive state
changes in the window, and the peak |effort| per joint in the last 2 s before
the fault. Then one row per /joint_states sample, t_rel_s relative to the
fault.

It also writes torque_stats.txt in the same folder: the torque of each axis
per movement, in % of the rated torque (the unit of the crash protection
threshold P1-57 in delta*.yaml), to tune that threshold. A movement is one
goal of the trajectory controllers (from their action status topics): a
repetition of an exercise or a return to start; consecutive repetitions are
also summed up as one EXERCISE, and all the movements of the start as START
TOTAL (at exit). See torque_stats.py.
"""

import collections
import datetime
import math
import os
import threading
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from ethercat_controller_msgs.msg import Cia402DriveStates
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from tecnobody_workbench_utils.cpu_affinity import pin_to_housekeeping_cpus
from tecnobody_workbench_utils.torque_stats import (
    crash_threshold_pct, FILE_HEADER, MovementStats)

FALLBACK_DIR = os.path.expanduser('~/.ros/fit4med_diagnostics')
PEAK_WINDOW_S = 2.0
# A fault that flaps must not fill the disk.
MAX_SNAPSHOTS_PER_RUN = 20


# Trajectory controllers whose goals are the movements of torque_stats.txt.
MOVEMENT_CONTROLLERS = {
    'joint_trajectory_controller': 'repetition',
    'go_to_start_controller': 'return to start',
}
EXERCISE_CONTROLLER = 'joint_trajectory_controller'
OPEN_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                 GoalStatus.STATUS_CANCELING)
STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _clock_text(t: float) -> str:
    return datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]


def _faulted(states) -> list[int]:
    return [i for i, state in enumerate(states) if 'FAULT' in state]


class FaultSnapshotNode(Node):

    def __init__(self, out_dir: str):
        super().__init__('fault_snapshot_node')
        self.pre_fault_s = float(self.declare_parameter('pre_fault_s', 20.0).value)
        self.post_fault_s = float(self.declare_parameter('post_fault_s', 2.0).value)
        self.out_dir = out_dir

        self._lock = threading.Lock()
        # (stamp_s, names, positions, velocities, efforts)
        self._samples: collections.deque = collections.deque()
        # (stamp_s, drive states, status words): changes only
        self._events: collections.deque = collections.deque(maxlen=500)
        self._last_drive_key = None
        self._fault_active = False
        self._pending: dict | None = None
        self._written = 0

        # Torque statistics per movement (torque_stats.txt).
        # "over N%" uses the crash protection threshold P1-57 of the drives,
        # read from the installed delta400x.yaml, so it follows any tuning;
        # the parameter only if that cannot be read.
        self.stats_threshold_pct = float(
            self.declare_parameter('stats_threshold_pct', 100.0).value)
        try:
            from ament_index_python.packages import get_package_share_directory
            drive_config = os.path.join(
                get_package_share_directory('tecnobody_workbench'), 'config', 'delta400x.yaml')
            threshold = crash_threshold_pct(drive_config)
            if threshold:
                self.stats_threshold_pct = threshold
        except Exception:  # noqa: BLE001
            pass
        self.exercise_gap_s = float(self.declare_parameter('exercise_gap_s', 10.0).value)
        self._open_movements: dict[bytes, tuple[str, MovementStats]] = {}
        self._seen_goals: set[bytes] = set()
        self._exercise: MovementStats | None = None
        self._exercise_count = 0
        self._exercise_first = 0
        self._repetitions = 0
        self._last_repetition_end: float | None = None
        self._total: MovementStats | None = None
        self._stats_path = os.path.join(self.out_dir, 'torque_stats.txt')
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for controller in MOVEMENT_CONTROLLERS:
            self.create_subscription(
                GoalStatusArray,
                f'/{controller}/follow_joint_trajectory/_action/status',
                lambda msg, c=controller: self._on_goal_status(c, msg),
                status_qos)

        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            Cia402DriveStates, '/ethercat_checker/drive_states', self._on_drive_states, 10)
        self.create_timer(0.2, self._write_if_due)
        self.get_logger().info(
            f'Keeping {self.pre_fault_s:g} s of /joint_states; '
            f'on a drive fault saving them (+{self.post_fault_s:g} s) to {self.out_dir}')

    def _on_joint_states(self, msg: JointState) -> None:
        t = _stamp_s(msg.header.stamp) or time.time()
        n = len(msg.name)

        def padded(values) -> list[float]:
            # JointState allows empty position/velocity/effort arrays.
            values = list(values)
            return values if len(values) == n else [math.nan] * n

        sample = (t, list(msg.name), padded(msg.position), padded(msg.velocity),
                  padded(msg.effort))
        for _controller, movement in self._open_movements.values():
            movement.add_sample(t, sample[1], sample[4], self.stats_threshold_pct)
        keep_s = self.pre_fault_s + self.post_fault_s + 1.0
        with self._lock:
            self._samples.append(sample)
            while self._samples and self._samples[0][0] < t - keep_s:
                self._samples.popleft()

    def _on_goal_status(self, controller: str, msg: GoalStatusArray) -> None:
        now = time.time()
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            if status.status in OPEN_STATUSES:
                if goal_id not in self._seen_goals:
                    self._seen_goals.add(goal_id)
                    self._start_movement(controller, goal_id, now)
            elif goal_id in self._open_movements:
                self._end_movement(
                    goal_id, now, STATUS_TEXT.get(status.status, f'status {status.status}'))
            else:
                # Finished before this node saw it running (e.g. started late).
                self._seen_goals.add(goal_id)

    def _start_movement(self, controller: str, goal_id: bytes, now: float) -> None:
        if controller == EXERCISE_CONTROLLER:
            if (self._exercise is not None and self._last_repetition_end is not None
                    and now - self._last_repetition_end > self.exercise_gap_s):
                self._close_exercise()
            self._repetitions += 1
            label = f'repetition {self._repetitions}'
        else:
            self._close_exercise()
            label = MOVEMENT_CONTROLLERS[controller]
        self._open_movements[goal_id] = (controller, MovementStats(label, now))

    def _end_movement(self, goal_id: bytes, now: float, result: str) -> None:
        controller, movement = self._open_movements.pop(goal_id)
        movement.end_t = max(movement.end_t, now)
        movement.result = result
        self._write_stats(movement)
        if self._total is None:
            self._total = MovementStats('START TOTAL', movement.start_t)
        self._total.merge(movement)
        if controller == EXERCISE_CONTROLLER:
            if self._exercise is None:
                self._exercise_count += 1
                self._exercise = MovementStats(
                    f'== EXERCISE {self._exercise_count}', movement.start_t)
                self._exercise_first = self._repetitions
            self._exercise.merge(movement)
            self._last_repetition_end = movement.end_t

    def _close_exercise(self) -> None:
        if self._exercise is None:
            self._repetitions = 0
            return
        first, last = self._exercise_first, self._repetitions
        self._exercise.label += (
            f' (repetition {first})' if first == last else f' (repetitions {first}-{last})')
        self._write_stats(self._exercise)
        self._exercise = None
        self._repetitions = 0
        self._last_repetition_end = None

    def _write_stats(self, movement: MovementStats) -> None:
        text = movement.report(self.stats_threshold_pct)
        self.get_logger().info('Torque ' + text.replace('\n', ' |'))
        try:
            os.makedirs(self.out_dir, exist_ok=True)
            new = not os.path.exists(self._stats_path)
            with open(self._stats_path, 'a', encoding='utf-8') as out:
                if new:
                    out.write(FILE_HEADER)
                out.write(text + '\n')
        except OSError as exc:
            self.get_logger().error(f'Torque stats not saved: {exc}')

    def flush_stats(self) -> None:
        """At exit: close the open movements, the exercise and the total."""
        now = time.time()
        for goal_id in list(self._open_movements):
            self._end_movement(goal_id, now, 'still running at exit')
        self._close_exercise()
        if self._total is not None:
            self._write_stats(self._total)
            self._total = None

    def _on_drive_states(self, msg: Cia402DriveStates) -> None:
        t = _stamp_s(msg.header.stamp) or time.time()
        states = list(msg.drive_states)
        words = [int(w) for w in msg.status_words]
        key = (tuple(states), tuple(words))
        with self._lock:
            if key != self._last_drive_key:
                self._last_drive_key = key
                self._events.append((t, list(msg.dof_names), states, words))
        faulted = _faulted(states)
        fault_now = bool(faulted) or msg.fault_present
        if fault_now and not self._fault_active and self._pending is None:
            names = [msg.dof_names[i] for i in faulted] or ['(fault_present)']
            self._pending = {
                't_fault': t,
                'dofs': names,
                'states': dict(zip(msg.dof_names, states)),
                'words': dict(zip(msg.dof_names, words)),
                'due': time.monotonic() + self.post_fault_s,
            }
            self.get_logger().warning(
                f'Drive fault on {", ".join(names)}: saving the joint states '
                f'of the last {self.pre_fault_s:g} s in {self.post_fault_s:g} s')
        self._fault_active = fault_now

    def _write_if_due(self, force: bool = False) -> None:
        pending = self._pending
        if pending is None or (not force and time.monotonic() < pending['due']):
            return
        self._pending = None
        if self._written >= MAX_SNAPSHOTS_PER_RUN:
            self.get_logger().warning('Fault snapshot not saved: limit per run reached')
            return
        try:
            path = self._write(pending)
        except OSError as exc:
            self.get_logger().error(f'Fault snapshot not saved: {exc}')
            return
        self._written += 1
        self.get_logger().warning(f'Fault snapshot saved: {path}')

    def _write(self, pending: dict) -> str:
        t_fault = pending['t_fault']
        start = t_fault - self.pre_fault_s
        with self._lock:
            samples = [s for s in self._samples if s[0] >= start]
            # The changes in the window, plus the last state before it.
            before = [e for e in self._events if e[0] < start][-1:]
            events = before + [e for e in self._events if e[0] >= start]

        joints = samples[-1][1] if samples else []
        peaks = {}
        for t, names, _pos, _vel, eff in samples:
            if t_fault - PEAK_WINDOW_S <= t <= t_fault:
                for name, value in zip(names, eff):
                    peak = abs(peaks.get(name, (0.0, 0.0))[0])
                    if not math.isnan(value) and abs(value) >= peak:
                        peaks[name] = (value, t - t_fault)

        os.makedirs(self.out_dir, exist_ok=True)
        stamp = datetime.datetime.fromtimestamp(t_fault).strftime('%H%M%S_%f')[:-3]
        path = os.path.join(self.out_dir, f'fault_snapshot_{stamp}.csv')
        with open(path, 'w', encoding='ascii') as out:
            out.write(f'# fault at {_clock_text(t_fault)} (stamp {t_fault:.3f}) '
                      f'on: {", ".join(pending["dofs"])}\n')
            out.write('# drive states at the fault: ' + ', '.join(
                f'{dof}={state} (0x{pending["words"].get(dof, 0):04X})'
                for dof, state in pending['states'].items()) + '\n')
            out.write('# effort = drive torque actual (0x6077), per mille of rated torque\n')
            out.write(f'# peak |effort| in the {PEAK_WINDOW_S:g} s before the fault: ' + ', '.join(
                f'{name}={value:.0f} at {dt:+.2f} s'
                for name, (value, dt) in sorted(peaks.items())) + '\n')
            out.write('# drive state changes (t_rel_s: dof=state status_word):\n')
            for t, dofs, states, words in events:
                out.write(f'#   {t - t_fault:+8.3f} ' + ' '.join(
                    f'{dof}={state}(0x{word:04X})'
                    for dof, state, word in zip(dofs, states, words)) + '\n')
            columns = ['t_rel_s', 'clock']
            for name in joints:
                columns += [f'{name}_pos', f'{name}_vel', f'{name}_effort']
            out.write(','.join(columns) + '\n')
            for t, names, pos, vel, eff in samples:
                values = dict(zip(names, zip(pos, vel, eff)))
                row = [f'{t - t_fault:.3f}', _clock_text(t)]
                for name in joints:
                    p, v, e = values.get(name, (math.nan,) * 3)
                    row += [f'{p:.6f}', f'{v:.6f}', f'{e:.0f}' if not math.isnan(e) else 'nan']
                out.write(','.join(row) + '\n')

        if peaks:
            self.get_logger().warning(
                'Peak |effort| (per mille of rated) in the 2 s before the fault: ' + ', '.join(
                    f'{name}={value:.0f} at {dt:+.2f} s'
                    for name, (value, dt) in sorted(peaks.items())))
        return path

    def flush(self) -> None:
        """Save a pending snapshot now (shutdown before post_fault_s ran out)."""
        self._write_if_due(force=True)


def main(args=None):
    pin_to_housekeeping_cpus('fault_snapshot_node')
    rclpy.init(args=args)
    node = FaultSnapshotNode(os.environ.get('ROS_LOG_DIR') or FALLBACK_DIR)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.flush()
        except Exception as exc:  # noqa: BLE001
            print(f'[fault_snapshot_node] snapshot not saved at exit: {exc}', flush=True)
        try:
            node.flush_stats()
        except Exception as exc:  # noqa: BLE001
            print(f'[fault_snapshot_node] torque stats not saved at exit: {exc}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
