# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from typing import Any

from tecnobody_msgs.msg import PlcController


PLC_COMMAND_INTERFACE_NAMES = [
    'PLC_node/mode_of_operation',
    'PLC_node/power_cutoff',
    'PLC_node/sonar_teach',
    'PLC_node/s_output.4',
    'PLC_node/estop',
    'PLC_node/manual_mode',
    'PLC_node/force_sensors_pwr',
    'PLC_node/brake_disable',
    'PLC_node/eeg_sync',
    'PLC_node/z_recovery',
]


class PlcCommandPublisher:
    def __init__(self, command_publisher: Any, logger: Any) -> None:
        self.command_publisher = command_publisher
        self.logger = logger
        self.plc_outputs = PlcController()
        self.plc_outputs.values = [0] * 10
        self.plc_outputs.interface_names = list(PLC_COMMAND_INTERFACE_NAMES)

    def publish_bringup_commands(self) -> None:
        self.publish_command('PLC_node/estop', 1)
        self.publish_command('PLC_node/z_recovery', 1)
        self.publish_command('PLC_node/force_sensors_pwr', 1)

    def publish_command(self, name: str, value: int) -> None:
        if name in self.plc_outputs.interface_names:  # type: ignore
            idx = self.plc_outputs.interface_names.index(name)  # type: ignore
            self.plc_outputs.values[idx] = value  # type: ignore
            self.command_publisher.publish(self.plc_outputs)
            self.logger.info(f"Published PLC command: {self.plc_outputs.values}")  # type: ignore
        else:
            self.logger.warn(  # type: ignore
                f"Interface name '{name}' not found in command message."
            )
