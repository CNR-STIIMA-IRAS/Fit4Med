# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

import os
import subprocess
from typing import Callable

from tecnobody_workbench_utils.fsm import PendingTransition
from tecnobody_workbench_utils.utils import (
    check_env,
    check_env_stopped,
    make_platform_controller_readiness_monitor,
    make_recovery_controller_status_monitor,
    stop_launch_environment,
)

from plc_manager.plc_types import Event, State


class EnvironmentManager:
    def __init__(
        self,
        node,
        service_group,
        publish_command: Callable[[str, int], None],
    ) -> None:
        self.node = node
        self.publish_command = publish_command
        self.startup_cleanup_action: Callable[[], None] | None = None
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
        self.platform_controller_readiness_monitor.destroy()
        self.recovery_controller_status_monitor.destroy()

    def check_env_running_stopped(self) -> bool:
        if check_env_stopped([
            "run_platform_control.launch.py",
            "run_rosbridge.launch.py"
        ]):
            return True

        return False

    def check_env_running_recovery_stopped(self) -> bool:
        if check_env_stopped([
            "run_z_recovery_control.launch.py",
            "run_rosbridge.launch.py"
        ]):
            return True

        return False

    def check_env_running(self, check_controller_status: bool = True) -> bool:
        if not check_env([
            "run_platform_control.launch.py",
            "run_rosbridge.launch.py"
        ]):
            return False

        if check_controller_status:
            return self.platform_controller_readiness_monitor.ok()

        return True

    def check_env_running_recovery(
        self,
        check_controller_status: bool = True,
    ) -> bool:
        if not check_env([
            "run_z_recovery_control.launch.py",
            "run_rosbridge.launch.py"
        ]):
            return False

        if check_controller_status:
            return self.recovery_controller_status_monitor.ok()

        return True

    def bringup_env(self) -> None:
        self.startup_cleanup_action = self.kill_env
        self.platform_controller_readiness_monitor.reset()
        self.publish_command('PLC_node/z_recovery', 1)
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
        self.publish_command('PLC_node/z_recovery', 0)
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

    def reset_sw_estop(self) -> None:
        self.publish_command('PLC_node/estop', 1)
        self.publish_command('PLC_node/manual_mode', 0)

    def handle_idle_stop(self) -> None:
        cleanup_action = self.startup_cleanup_action
        self.startup_cleanup_action = None

        if cleanup_action is not None:
            cleanup_action()
            return

        self.reset_sw_estop()

    def kill_recovery_env(self) -> None:
        self.startup_cleanup_action = None
        self.recovery_controller_status_monitor.reset()
        self.publish_command('PLC_node/brake_disable', 0)
        self.publish_command('PLC_node/z_recovery', 1)

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
        self.publish_command('PLC_node/brake_disable', 0)
        self.publish_command('PLC_node/estop', 1)

        stop_launch_environment(
            "run_platform_control.launch.py",
            "run_platform_control.launch"
        )

        stop_launch_environment(
            "ros2 launch tecnobody_workbench run_rosbridge.launch.py",
            "run_rosbridge.launch",
            9090
        )

    def cleanup_timed_out_transition(
        self,
        pending: PendingTransition | None,
    ) -> None:
        if pending is None:
            return

        if pending.event == Event.START:
            if pending.source == State.IDLE:
                self.kill_env()
            elif pending.source == State.IDLE_RECOVERY:
                self.kill_recovery_env()
        elif pending.event == Event.STOP or pending.event == Event.FAIL:
            if pending.source == State.RUNNING:
                self.kill_env()
            elif pending.source == State.RUNNING_RECOVERY:
                self.kill_recovery_env()
