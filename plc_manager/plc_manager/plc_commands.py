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
        self._last_published_values: list[int] | None = None

    def _publish_command(self, name: str, value: int, force_print: bool = False) -> None:
        if name in self.plc_outputs.interface_names:  # type: ignore
            idx = self.plc_outputs.interface_names.index(name)  # type: ignore
            self.plc_outputs.values[idx] = value  # type: ignore
            current_values = list(self.plc_outputs.values)  # type: ignore

            self.command_publisher.publish(self.plc_outputs)

            if not force_print and self._last_published_values == current_values:
                return
            
            self._last_published_values = current_values

            command_values = [
                f"{interface_name.removeprefix('PLC_node/')}: {command_value}"
                for interface_name, command_value in zip(self.plc_outputs.interface_names, current_values)  # type: ignore
            ]
            self.logger.info(f"PLC command: {command_values}")  # type: ignore
        else:
            self.logger.warn(  # type: ignore
                f"Interface name '{name}' not found in command message.",
                throttle_duration_sec=5.0
            )

    def set_automatic_mode(self) -> None:
        self._publish_command('PLC_node/manual_mode', 0)

    def set_manual_mode(self) -> None:
        self._publish_command('PLC_node/manual_mode', 1)

    def close_brake(self) -> None:
        self._publish_command('PLC_node/brake_disable', 0)

    def open_brake(self) -> None:
        self._publish_command('PLC_node/brake_disable', 1)

    def wire_endstroke_to_emergency_chain(self) -> None:
        """
        Wire the end-stroke limit switch to the emergency chain.
        This is done by setting the 'z_recovery' command to 1, which
        enables the end-stroke switch to trigger an emergency stop.
        """
        self._publish_command('PLC_node/z_recovery', 1, force_print=True)

    def detach_endstroke_from_emergency_chain(self) -> None:
        """
        Detach the end-stroke limit switch from the emergency chain.
        This is done by setting the 'z_recovery' command to 0, which
        disables the end-stroke switch from triggering an emergency stop.
        """
        self._publish_command('PLC_node/z_recovery', 0, force_print=True)

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
