# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import os
import subprocess
import threading
from typing import Callable

from tecnobody_workbench_utils.fsm import PendingTransition
from tecnobody_workbench_utils.utils import (
    find_matching_pids,
    make_platform_controller_readiness_monitor,
    make_recovery_controller_status_monitor,
    stop_launch_environment,
)

from plc_manager.plc_types import State


class EnvironmentManager:
    PLATFORM_LAUNCHER = "run_platform_control.launch.py"
    RECOVERY_LAUNCHER = "run_z_recovery_control.launch.py"
    ROSBRIDGE_LAUNCHER = "run_rosbridge.launch.py"

    def __init__(
        self,
        node,
        service_group,
        timer_group,
    ) -> None:
        self.node = node
        self.startup_cleanup_action: Callable[[], None] | None = None
        self.node.declare_parameter('environment_process_poll_period', 0.5)
        self._process_cache_lock = threading.Lock()
        self._launcher_present = {
            self.PLATFORM_LAUNCHER: False,
            self.RECOVERY_LAUNCHER: False,
            self.ROSBRIDGE_LAUNCHER: False,
        }
        self._refresh_process_cache()
        self._process_timer = self.node.create_timer(
            float(
                self.node.get_parameter(
                    'environment_process_poll_period'
                ).value
            ),
            self._refresh_process_cache,
            callback_group=timer_group,
        )
        self.platform_controller_readiness_monitor = (
            make_platform_controller_readiness_monitor(
                self.node,
                callback_group=service_group,
                logger=self.node.get_logger(),
            )
        )
        self.recovery_controller_status_monitor = (
            make_recovery_controller_status_monitor(
                self.node,
                callback_group=service_group,
                logger=self.node.get_logger(),
            )
        )

    def destroy(self) -> None:
        self._process_timer.cancel()
        self.node.destroy_timer(self._process_timer)
        self.platform_controller_readiness_monitor.destroy()
        self.recovery_controller_status_monitor.destroy()

    def _refresh_process_cache(self) -> None:
        launcher_present = {
            launcher: bool(find_matching_pids(launcher))
            for launcher in self._launcher_present
        }
        with self._process_cache_lock:
            self._launcher_present = launcher_present

    def _launchers_present(self, launcher_names: list[str]) -> bool:
        with self._process_cache_lock:
            return all(
                self._launcher_present.get(launcher_name, False)
                for launcher_name in launcher_names
            )

    def _launchers_stopped(self, launcher_names: list[str]) -> bool:
        with self._process_cache_lock:
            return all(
                not self._launcher_present.get(launcher_name, False)
                for launcher_name in launcher_names
            )

    def check_env_running_stopped(self) -> bool:
        return self._launchers_stopped([
            self.PLATFORM_LAUNCHER,
            self.ROSBRIDGE_LAUNCHER,
        ])

    def check_env_running_recovery_stopped(self) -> bool:
        return self._launchers_stopped([
            self.RECOVERY_LAUNCHER,
            self.ROSBRIDGE_LAUNCHER,
        ])

    def check_env_running(self, check_controller_status: bool = True) -> bool:
        if not self._launchers_present([
            self.PLATFORM_LAUNCHER,
            self.ROSBRIDGE_LAUNCHER,
        ]):
            return False

        if check_controller_status:
            return self.platform_controller_readiness_monitor.ok()

        return True

    def check_env_running_recovery(
        self,
        check_controller_status: bool = True,
    ) -> bool:
        if not self._launchers_present([
            self.RECOVERY_LAUNCHER,
            self.ROSBRIDGE_LAUNCHER,
        ]):
            return False

        if check_controller_status:
            return self.recovery_controller_status_monitor.ok()

        return True

    def check_expected_processes(
        self,
        state: State,
        pending: PendingTransition | None,
    ) -> bool:
        if pending is not None:
            return True

        if state == State.RUNNING:
            return self.check_env_running(False)
        if state == State.RUNNING_RECOVERY:
            return self.check_env_running_recovery(False)
        if state in (State.IDLE, State.ESTOP):
            return self.check_env_running_stopped()
        if state in (State.IDLE_RECOVERY, State.ESTOP_RECOVERY, State.RECOVERED):
            return self.check_env_running_recovery_stopped()

        return False

    def bringup_env(self) -> None:
        self.startup_cleanup_action = self.kill_env
        self.platform_controller_readiness_monitor.reset()

        subprocess.Popen(
            [" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh"],
            shell=True,
            executable="/bin/bash"
        )
        subprocess.Popen(
            [" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_bridge.sh"],
            shell=True,
            executable="/bin/bash"
        )

    def bringup_recovery_env(self) -> None:
        self.startup_cleanup_action = self.kill_recovery_env
        self.recovery_controller_status_monitor.reset()

        subprocess.Popen(
            [" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env_z_recovery.sh"],
            shell=True,
            executable="/bin/bash",
            env={**os.environ, 'AUTO_RECOVER': '1'}
        )
        subprocess.Popen(
            [" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_bridge.sh"],
            shell=True,
            executable="/bin/bash"
        )

    def handle_idle_stop(self) -> None:
        cleanup_action = self.startup_cleanup_action
        self.startup_cleanup_action = None

        if cleanup_action is not None:
            cleanup_action()
            return

    def kill_recovery_env(self) -> None:
        self.startup_cleanup_action = None
        self.recovery_controller_status_monitor.reset()

        stop_launch_environment(
            "run_z_recovery_control.launch.py",
            "run_z_recovery_control.launch"
        )

        stop_launch_environment(
            "ros2 launch tecnobody_workbench run_rosbridge.launch.py",
            "run_rosbridge.launch",
            9090
        )

    def kill_env(self) -> None:
        self.startup_cleanup_action = None
        self.platform_controller_readiness_monitor.reset()

        stop_launch_environment(
            "run_platform_control.launch.py",
            "run_platform_control.launch"
        )

        stop_launch_environment(
            "ros2 launch tecnobody_workbench run_rosbridge.launch.py",
            "run_rosbridge.launch",
            9090
        )
