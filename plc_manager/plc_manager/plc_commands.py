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

    def _publish_command(self, name: str, value: int) -> None:
        if name in self.plc_outputs.interface_names:  # type: ignore
            idx = self.plc_outputs.interface_names.index(name)  # type: ignore
            self.plc_outputs.values[idx] = value  # type: ignore
            self.command_publisher.publish(self.plc_outputs)
            self.logger.info(f"Published PLC command: {self.plc_outputs.values}")  # type: ignore
        else:
            self.logger.warn(  # type: ignore
                f"Interface name '{name}' not found in command message."
            )

    def set_automatic_mode(self) -> None:
        self._publish_command('PLC_node/manual_mode', 0)

    def set_manual_mode(self) -> None:
        self._publish_command('PLC_node/manual_mode', 1)

    def brake_enable(self) -> None:
        self._publish_command('PLC_node/brake_disable', 0)

    def brake_disable(self) -> None:
        self._publish_command('PLC_node/brake_disable', 1)

    def wire_endstroke_to_emergency_chain(self) -> None:
        """
        Wire the end-stroke limit switch to the emergency chain.
        This is done by setting the 'z_recovery' command to 1, which
        enables the end-stroke switch to trigger an emergency stop.
        """
        self._publish_command('PLC_node/z_recovery', 1)

    def detach_endstroke_from_emergency_chain(self) -> None:
        """
        Detach the end-stroke limit switch from the emergency chain.
        This is done by setting the 'z_recovery' command to 0, which
        disables the end-stroke switch from triggering an emergency stop.
        """
        self._publish_command('PLC_node/z_recovery', 0)

    def raise_sw_estop(self) -> None:
        """
        SW Emergency stop
            =>  Open the Emergency stop chain, the next iteration
                will see the estop_value = EMERGENCY and trigger the STOP event
        """
        self._publish_command('PLC_node/estop', 0)

    def clear_sw_estop(self) -> None:
        self._publish_command('PLC_node/estop', 1)

    def power_force_sensors(self) -> None:
        self._publish_command('PLC_node/force_sensors_pwr', 1)

    def cut_power_force_sensors(self) -> None:
        self._publish_command('PLC_node/force_sensors_pwr', 0)

    def publish_bringup_commands(self) -> None:
        self.clear_sw_estop()
        self.wire_endstroke_to_emergency_chain()
        self.power_force_sensors()

    def disable_eeg_sync(self) -> None:
        self._publish_command('PLC_node/eeg_sync', 0)

    def enable_eeg_sync(self) -> None:
        self._publish_command('PLC_node/eeg_sync', 1)
