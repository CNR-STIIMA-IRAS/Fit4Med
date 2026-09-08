# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from typing import Any
import threading

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
        self._lock = threading.RLock()
        self.plc_outputs = PlcController()
        # Unknown until the GUI supplies a value; never invent a startup reset.
        self.plc_outputs.interface_names = [
            name for name in PLC_COMMAND_INTERFACE_NAMES
            if name != 'PLC_node/eeg_sync'
        ]
        self.plc_outputs.values = [0] * len(self.plc_outputs.interface_names)
        self._last_published_values: list[int] | None = None

    def _copy_outputs(self) -> PlcController:
        message = PlcController()
        message.interface_names = list(self.plc_outputs.interface_names)
        message.values = list(self.plc_outputs.values)
        return message

    def receive_gui_eeg_sync(self, message: PlcController) -> None:
        """Remember and forward only the GUI's EEG byte, including identical retries."""
        if list(message.interface_names) != ['PLC_node/eeg_sync'] or len(message.values) != 1:
            self.logger.warning('Ignoring malformed GUI EEG command.')
            return
        value = int(message.values[0])
        if not 0 <= value <= 255:
            self.logger.warning('Ignoring GUI EEG command outside uint8 range.')
            return
        with self._lock:
            outputs = self._copy_outputs()
            names = list(outputs.interface_names)
            values = list(outputs.values)
            if 'PLC_node/eeg_sync' not in names:
                index = PLC_COMMAND_INTERFACE_NAMES.index('PLC_node/eeg_sync')
                names.insert(index, 'PLC_node/eeg_sync')
                values.insert(index, value)
            else:
                values[names.index('PLC_node/eeg_sync')] = value
            outputs.interface_names = names
            outputs.values = values
            # Replace the message so status readers also see a consistent snapshot.
            self.plc_outputs = outputs
            command = PlcController()
            command.interface_names = ['PLC_node/eeg_sync']
            command.values = [value]
            # Use the same publisher and lock as snapshots: no older snapshot
            # can be published after this GUI update.
            self.command_publisher.publish(command)

    def _publish_command(self, name: str, value: int, force_print: bool = False) -> None:
        if name == 'PLC_node/eeg_sync':
            raise ValueError('EEG sync may only be changed through the GUI input.')
        with self._lock:
            if name not in self.plc_outputs.interface_names:
                self.logger.warn(
                    f"Interface name '{name}' not found in command message.",
                    throttle_duration_sec=5.0,
                )
                return
            outputs = self._copy_outputs()
            idx = outputs.interface_names.index(name)
            outputs.values[idx] = value
            self.plc_outputs = outputs
            current_values = list(outputs.values)
            self.command_publisher.publish(outputs)
            if not force_print and self._last_published_values == current_values:
                return
            self._last_published_values = current_values
            command_values = [
                f"{interface_name.removeprefix('PLC_node/')}: {command_value}"
                for interface_name, command_value in zip(outputs.interface_names, current_values)
            ]
            self.logger.info(f"PLC command: {command_values}")

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
