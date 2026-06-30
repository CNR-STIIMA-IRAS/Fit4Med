# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
 
import os
import json
import types
import time
from enum import Enum, auto


# Must be BEFORE importing rclpy (sets logging format globally)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.client import Client
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tecnobody_msgs.msg import PlcController, PlcStates
from ethercat_msgs.srv import GetSlaveStates
from std_srvs.srv import Trigger
import sys
import subprocess
import threading
import signal
from enum import Enum
from typing import Any, Callable

from tecnobody_workbench_utils.utils import (
    check_env, check_env_stopped,
    make_platform_controller_readiness_monitor,
    make_recovery_controller_status_monitor,
    stop_launch_environment
)
from tecnobody_workbench_utils.utils import bcolors as bc

from tecnobody_workbench_utils.fsm import (
    StateMachine, InvalidTransition, GuardFailed,
    TransitionTimeout, PendingTransition
)
from plc_manager.udp_client import UdpClient

class EStopState(Enum):
    OK = 1
    EMERGENCY = 0

class EthercatCheckState(Enum):
    IDLE = 0
    PENDING = 1
    READY = 2
    FAILED = 3


class State(Enum):
    IDLE = auto()
    IDLE_RECOVERY = auto()
    ESTOP = auto()
    RUNNING = auto()
    RUNNING_RECOVERY = auto()
    RECOVERED = auto()
    ERROR = auto()

class Event(Enum):
    SWITCH_MODE = auto()
    START = auto()
    STOP = auto()
    FAIL = auto()
    NONE = auto()


class PLCControllerInterface(Node):

    def __init__(self, target_ip: str):

        super().__init__('plc_manager')

        # ========== Callback Groups ==========
        # Three separate groups prevent callback blocking:
        self.plc_group = MutuallyExclusiveCallbackGroup()       # PLC state updates
        self.timer_group = MutuallyExclusiveCallbackGroup()     # Low-rate status timers
        self.service_group = MutuallyExclusiveCallbackGroup()   # EtherCAT service clients

        # ========== PLC State Subscription ==========
        # Receives PlcStates from plc_controller at max rate
        # Includes estop, all inputs, and interface names
        self.state_subscriber_callback_running : bool = False
        self.state_subscriber : Subscription = self.create_subscription( #type: ignore
            PlcStates,
            '/PLC_controller/plc_states',
            self.state_callback,
            10,
            callback_group=self.plc_group, 

        )
        
        # ========== PLC Command Publisher ==========
        # RELIABLE QoS: guarantees all commands reach PLC even if temporarily unavailable
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.command_publisher = self.create_publisher( #type: ignore
            PlcController,
            '/PLC_controller/plc_commands',
            qos,
        )

        # ========== PLC State Variables ==========
        self.interface_names : list[str] = []
        self.state_values : list[int] = []
        self.command_values : list[int] = []
        self.launch_status : list[bool] = []
        self.sw_estop_cached : EStopState = EStopState.OK   # Cached E-stop value for transition detection
        self.estop_cached : EStopState = EStopState.EMERGENCY   # Cached E-stop value for transition detection
        self.lock = threading.Lock()                            # Protects state_callback() concurrent access
        
        # ========== Command Message Structure ==========
        # Pre-allocate PlcController message with 10 outputs (matches plc_controller.yaml)
        self.plc_outputs = PlcController()
        self.plc_outputs.values  = [0] * 10
        self.plc_outputs.interface_names = [
            'PLC_node/mode_of_operation',       # [0] unused
            'PLC_node/power_cutoff',            # [1] unused
            'PLC_node/sonar_teach',             # [2] ultrasonic sensor calibration
            'PLC_node/s_output.4',              # [3] unused
            'PLC_node/estop',                   # [4] main E-stop signal
            'PLC_node/manual_mode',             # [5] unused
            'PLC_node/force_sensors_pwr',       # [6] F/T sensor power control
            'PLC_node/brake_disable',           # [7] Brake Disable (0=enabled, 1=disabled)
            'PLC_node/eeg_sync',                # [8] EEG sync pulse
            'PLC_node/z_recovery'               # [9] Z-axis limit switch recovery flag
        ]

        # ========== Launcher Health Monitoring ==========
        self.processes_status_cached = False
        self._startup_cleanup_action: Callable[[], None] | None = None
        self.platform_controller_readiness_monitor = make_platform_controller_readiness_monitor(
            self,
            callback_group=self.service_group,
            logger=self.get_logger(),
        )

        self.recovery_controller_status_monitor = make_recovery_controller_status_monitor(
            self,
            callback_group=self.service_group,
            logger=self.get_logger(),
        )


        # ========== UDP Status Client ==========#  
        # Coordinates with rehab_gui for emergency stop and status
        self.client = UdpClient(target_ip, 5005)
        self.send_running_cnt = 0
        self.send_running_dec = 10  # Send status every 10*50ms = 500ms at 50 Hz
        self.shutdown_requested = False

        # ========== Signal Handlers ==========
        signal.signal(signal.SIGINT, self.signal_handler) #type: ignore
        signal.signal(signal.SIGTERM, self.signal_handler) #type: ignore

        # ========== EtherCAT PLC Slave Identifier ==========
        # Substring matched against slave_names returned by GetSlaveStates services.
        # Default matches the configured ros2_control name for the SICK GetC100 PLC.
        self.declare_parameter('plc_slave_identifier', 'sickPLC')
        self.declare_parameter(
            'ethercat_startup_check_service',
            '/ethercat_slaves_status_check/check',
        )
        self.declare_parameter(
            'ethercat_slave_state_services',
            ['/tecnobody/get_slave_states_master1'],
        )
        self.declare_parameter('ethercat_state_poll_period', 1.0)
        self.declare_parameter('ethercat_state_stale_after', 5.0)
        self.declare_parameter('ethercat_state_response_timeout', 3.0)

        self.z_limit_still_active = False

        self.ethercat_startup_check_service_name = (
            self._get_ethercat_startup_check_service_name()
        )
        self.ethercat_slaves_status_check_client : Client = self.create_client(Trigger, #type: ignore
            self.ethercat_startup_check_service_name,
            callback_group=self.service_group,
        )
        self.ethercat_startup_check_state = EthercatCheckState.IDLE
        self.ethercat_startup_check_requested = True

        self.ethercat_check_state = EthercatCheckState.IDLE
        self._ethercat_cache_lock = threading.Lock()
        self._ethercat_slave_names: list[str] = []
        self._ethercat_slave_states: list[str] = []
        self._ethercat_last_update_monotonic: float | None = None
        self._ethercat_last_error: str | None = None
        self._ethercat_pending_requests: dict[Any, str] = {}
        self._ethercat_poll_started_at: float | None = None
        self._ethercat_poll_slave_names: list[str] = []
        self._ethercat_poll_slave_states: list[str] = []
        self._ethercat_poll_errors: list[str] = []
        self._ethercat_state_stale_after = float(
            self.get_parameter('ethercat_state_stale_after').value #type: ignore
        )
        self._ethercat_state_response_timeout = float(
            self.get_parameter('ethercat_state_response_timeout').value #type: ignore
        )
        self._ethercat_slave_state_clients = {
            service_name: self.create_client( #type: ignore
                GetSlaveStates,
                service_name,
                callback_group=self.service_group,
            )
            for service_name in self._get_ethercat_slave_state_service_names()
        }
        self.ethercat_state_timer = self.create_timer(
            float(self.get_parameter('ethercat_state_poll_period').value), #type: ignore
            self.poll_ethercat_slave_states,
            callback_group=self.timer_group,
            autostart=False,
        )
        self.start_environment_requested = True

        self.fsm : StateMachine[State, Event] = StateMachine[State,Event](State.IDLE, self.get_logger()) #type: ignore
        self.fsm.add_transition(
            Event.SWITCH_MODE,
            State.IDLE, 
            State.IDLE_RECOVERY, 
            action=lambda: self.publish_command('PLC_node/z_recovery', 0),
            msg="🔑 Change mode (IDLE=>IDLE_RECOVERY)",
        )
        self.fsm.add_transition(
            Event.SWITCH_MODE,
            State.RECOVERED, 
            State.IDLE, 
            msg="🔑 Change mode (RECOVERED=>IDLE)",
        )
        self.fsm.add_transition(
            Event.START,
            State.IDLE, 
            State.RUNNING, 
            msg="🔑 KEY TURN DETECTED: E-stop 0=>1 (IDLE=>RUNNING)",
            guard=self._ready_to_start, 
            action=self._bringup_env,
            success_check=self.check_env_running,
            max_steps=5000,
            failure_destination=State.ERROR,
        )
        self.fsm.add_transition(
            Event.START,
            State.IDLE_RECOVERY, 
            State.RUNNING_RECOVERY, 
            msg="🔑 KEY TURN DETECTED: E-stop 0=>1 (IDLE_RECOVERY=>RUNNING_RECOVERY)",
            guard=self._ready_to_start, 
            action=self._bringup_recovery_env,
            success_check=self.check_env_running_recovery,
            max_steps=5000,
            failure_destination=State.ERROR
        )
        
        self.fsm.add_transition(
            Event.STOP,
            State.RUNNING_RECOVERY,
            State.RECOVERED,
            msg="🛑 EMERGENCY STOP REQUESTED (RUNNING_RECOVERY=>RECOVERED)",
            action=self._kill_recovery_env,
            success_check=self.check_env_running_recovery_stopped,
            max_steps=5000,
            failure_destination=State.ERROR
        )

        self.fsm.add_transition(
            Event.STOP,
            State.RUNNING, 
            State.IDLE, 
            msg="🛑 EMERGENCY STOP REQUESTED (RUNNING=>IDLE)",
            action=self._kill_env,
            success_check=self.check_env_running_stopped,
            max_steps=5000,
            failure_destination=State.ERROR
        )

        self.fsm.add_transition(
            Event.FAIL,
            State.RUNNING, 
            State.ERROR, 
            msg="� SYSTEM FAILURE (RUNNING=>ERROR)",
            action=self._kill_env,
            success_check=self.check_env_running_stopped,
            max_steps=5000,
            failure_destination=State.ERROR
        )
        
        self.fsm.add_transition(Event.FAIL,
                State.RUNNING_RECOVERY, 
                State.ERROR, 
                msg=" SYSTEM FAILURE (RUNNING_RECOVERY=>ERROR)",
                action=self._kill_recovery_env,
                success_check=self.check_env_running_recovery_stopped,
                max_steps=5000,
                failure_destination=State.ERROR)

        self.fsm.add_transition(Event.STOP,
                State.IDLE,
                State.IDLE,
                msg="🛑 EMERGENCY STOP REQUESTED (IDLE=>IDLE)",
                action=self._handle_idle_stop,
                success_check=self.check_env_running_stopped,
                max_steps=5000,
                failure_destination=State.ERROR)
        
        self.fsm.add_transition(Event.STOP,
                State.IDLE_RECOVERY,
                State.IDLE_RECOVERY,
                msg="🛑 EMERGENCY STOP REQUESTED (IDLE_RECOVERY=>IDLE_RECOVERY)",
                action=self._handle_idle_stop,
                success_check=self.check_env_running_recovery_stopped,
                max_steps=5000,
                failure_destination=State.ERROR)

        self.bringup_done = False
    
    def cleanup(self) -> None:
        """Perform graceful cleanup and resource deallocation.
        
        Called before node destruction to:
        1. Ask the GUI to stop its ROS communication
        2. Cancel launcher health check timer
        3. Close UDP socket
        4. Destroy node resources
        
        Returns:
            None. Leaves node in destroyed state.
        """
        self.get_logger().info("Cleanup: Cancelling timers and destroying node.") #type: ignore
        self.client.close()
        self.platform_controller_readiness_monitor.destroy()
        self.recovery_controller_status_monitor.destroy()
        self.destroy_node()
    
    def _ros_gui_disconnected(self) -> bool:
        """
        Handle ROS GUI disconnection event.
        """
        msg = self.client.last_received_message
        self.get_logger().info(f"Last received message: {msg}")  # Debug print statement
        if msg is None or msg == b"ROS_DISCONNECTED":
            return True
        
        return False

    def _ros_gui_connection_failed(self) -> bool:
        return self.client.last_received_message == b"ROS_CONNECTION_FAILED"

    def _ethercat_slaves_status_ok(self) -> bool:
        return self.ethercat_startup_check_state == EthercatCheckState.READY

    def _ready_to_start(self) -> bool:
        return self._ros_gui_disconnected() and self._ethercat_slaves_status_ok()

    def _get_ethercat_startup_check_service_name(self) -> str:
        service_name = str(
            self.get_parameter('ethercat_startup_check_service').value #type: ignore
        ).strip()
        return service_name or '/ethercat_slaves_status_check/check'

    def _get_ethercat_slave_state_service_names(self) -> list[str]:
        raw_service_names = self.get_parameter(
            'ethercat_slave_state_services'
        ).value #type: ignore

        if raw_service_names is None:
            service_names = []
        elif isinstance(raw_service_names, str):
            service_names = [
                name.strip()
                for name in raw_service_names.split(',')
                if name.strip()
            ]
        else:
            service_names = [
                str(name).strip()
                for name in raw_service_names
                if str(name).strip()
            ]

        return service_names or ['/tecnobody/get_slave_states_master1']

    def _plc_slave_identifier(self) -> str:
        return str(self.get_parameter('plc_slave_identifier').value) #type: ignore

    def _ethercat_cache_stale_locked(self) -> bool:
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            return False

        if self._ethercat_last_update_monotonic is None:
            return True

        return (
            time.monotonic() - self._ethercat_last_update_monotonic
            > self._ethercat_state_stale_after
        )

    def _find_plc_slave_locked(self) -> tuple[str | None, str | None]:
        plc_identifier = self._plc_slave_identifier()
        for slave_name, slave_state in zip(
            self._ethercat_slave_names,
            self._ethercat_slave_states,
        ):
            if plc_identifier in slave_name:
                return slave_name, slave_state

        return None, None

    def _visible_ethercat_slave_pairs_locked(self) -> list[tuple[str, str]]:
        return list(zip(self._ethercat_slave_names, self._ethercat_slave_states))

    def _refresh_ethercat_check_state_locked(self) -> None:
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            self.ethercat_check_state = EthercatCheckState.READY
            self._ethercat_last_error = (
                'Skipped EtherCAT health check '
                '(PLC_MANAGER_SKIP_ETHERCAT=1).'
            )
            return

        if not self._ethercat_slave_names:
            self.ethercat_check_state = EthercatCheckState.FAILED
            self._ethercat_last_error = 'EtherCAT GetSlaveStates returned no slaves.'
            return

        if len(self._ethercat_slave_names) != len(self._ethercat_slave_states):
            self.ethercat_check_state = EthercatCheckState.FAILED
            self._ethercat_last_error = (
                'EtherCAT GetSlaveStates returned mismatched slave names/states.'
            )
            return

        non_operational_slaves = [
            f'{slave_name} => {slave_state}'
            for slave_name, slave_state in zip(
                self._ethercat_slave_names,
                self._ethercat_slave_states,
            )
            if slave_state != 'OP'
        ]

        if not non_operational_slaves:
            self.ethercat_check_state = EthercatCheckState.READY
            if self._ethercat_poll_errors:
                self._ethercat_last_error = '; '.join(self._ethercat_poll_errors)
            else:
                self._ethercat_last_error = None
            return

        self.ethercat_check_state = EthercatCheckState.FAILED
        self._ethercat_last_error = (
            'EtherCAT slaves are not operational: '
            + '; '.join(non_operational_slaves)
        )

    def _mark_ethercat_check_failed(self, message: str) -> None:
        with self._ethercat_cache_lock:
            self.ethercat_check_state = EthercatCheckState.FAILED
            self._ethercat_last_error = message

        self.get_logger().warning(message, throttle_duration_sec=5.0) #type: ignore

    def _check_ethercat_request_timeout(self) -> None:
        with self._ethercat_cache_lock:
            if not self._ethercat_pending_requests:
                return

            if self._ethercat_poll_started_at is None:
                return

            if (
                time.monotonic() - self._ethercat_poll_started_at
                <= self._ethercat_state_response_timeout
            ):
                return

            timed_out_services = ', '.join(
                self._ethercat_pending_requests.values()
            )
            self._ethercat_pending_requests.clear()
            self._ethercat_poll_started_at = None
            self.ethercat_check_state = EthercatCheckState.FAILED
            self._ethercat_last_error = (
                'Timed out waiting for EtherCAT state services: '
                f'{timed_out_services}'
            )

        message = self._ethercat_last_error or 'EtherCAT state request timed out.'
        self.get_logger().warning(message, throttle_duration_sec=5.0) #type: ignore

    def poll_ethercat_slave_states(self) -> None:
        
        if os.environ.get('PLC_MANAGER_SKIP_ETHERCAT', '0') == '1':
            with self._ethercat_cache_lock:
                self._ethercat_last_update_monotonic = time.monotonic()
                self._ethercat_slave_names = []
                self._ethercat_slave_states = []
                self._refresh_ethercat_check_state_locked()
            return

        if self.fsm.state not in (State.RUNNING, State.RUNNING_RECOVERY):
            with self._ethercat_cache_lock:
                self._ethercat_pending_requests.clear()
                self._ethercat_poll_started_at = None
                self._ethercat_slave_names = []
                self._ethercat_slave_states = []
                self._ethercat_last_update_monotonic = None
                self.ethercat_check_state = EthercatCheckState.IDLE
                self._ethercat_last_error = (
                    'Runtime EtherCAT telemetry waits for START.'
                )
            return

        with self._ethercat_cache_lock:
            if self._ethercat_pending_requests:
                should_check_timeout = True
            else:
                should_check_timeout = False

        if should_check_timeout:
            self._check_ethercat_request_timeout()
            return

        ready_clients = [
            (service_name, client)
            for service_name, client in self._ethercat_slave_state_clients.items()
            if client.service_is_ready() #type: ignore
        ]

        if not ready_clients:
            service_names = ', '.join(self._ethercat_slave_state_clients.keys())
            self._mark_ethercat_check_failed(
                'No configured EtherCAT GetSlaveStates services are ready: '
                f'{service_names}'
            )
            return

        with self._ethercat_cache_lock:
            self.ethercat_check_state = EthercatCheckState.PENDING
            self._ethercat_poll_started_at = time.monotonic()
            self._ethercat_poll_slave_names = []
            self._ethercat_poll_slave_states = []
            self._ethercat_poll_errors = []

        started_requests = 0
        for service_name, client in ready_clients:
            try:
                future = client.call_async(GetSlaveStates.Request()) #type: ignore
            except Exception as exception:  # noqa: BLE001
                with self._ethercat_cache_lock:
                    self._ethercat_poll_errors.append(
                        f'{service_name}: call failed ({exception})'
                    )
                continue

            with self._ethercat_cache_lock:
                self._ethercat_pending_requests[future] = service_name

            future.add_done_callback(self._on_ethercat_slave_states_response) #type: ignore
            started_requests += 1

        if started_requests == 0:
            with self._ethercat_cache_lock:
                errors = '; '.join(self._ethercat_poll_errors)
            self._mark_ethercat_check_failed(
                errors or 'No EtherCAT GetSlaveStates request could be started.'
            )

    def _on_ethercat_slave_states_response(self, future: Any) -> None:
        with self._ethercat_cache_lock:
            service_name = self._ethercat_pending_requests.pop(future, None)

        if service_name is None:
            return

        try:
            response = future.result()
            slave_names = [str(name) for name in response.slave_names]
            slave_states = [str(state) for state in response.slave_states]
        except Exception as exception:  # noqa: BLE001
            slave_names = []
            slave_states = []
            error = f'{service_name}: call failed ({exception})'
        else:
            error = None

        with self._ethercat_cache_lock:
            if error is not None:
                self._ethercat_poll_errors.append(error)
            else:
                self._ethercat_poll_slave_names.extend(slave_names)
                self._ethercat_poll_slave_states.extend(slave_states)

            if self._ethercat_pending_requests:
                return

            self._finalize_ethercat_poll_locked()

    def _finalize_ethercat_poll_locked(self) -> None:
        self._ethercat_poll_started_at = None

        if not self._ethercat_poll_slave_names:
            self.ethercat_check_state = EthercatCheckState.FAILED
            self._ethercat_last_error = (
                '; '.join(self._ethercat_poll_errors)
                or 'EtherCAT GetSlaveStates returned no slaves.'
            )
            return

        self._ethercat_slave_names = list(self._ethercat_poll_slave_names)
        self._ethercat_slave_states = list(self._ethercat_poll_slave_states)
        self._ethercat_last_update_monotonic = time.monotonic()
        self._refresh_ethercat_check_state_locked()

    def _ethercat_status_payload(self) -> dict[str, object]:
        with self._ethercat_cache_lock:
            last_update_age = (
                None
                if self._ethercat_last_update_monotonic is None
                else max(0.0, time.monotonic() - self._ethercat_last_update_monotonic)
            )
            plc_name, plc_state = self._find_plc_slave_locked()
            stale = self._ethercat_cache_stale_locked()
            visible_pairs = self._visible_ethercat_slave_pairs_locked()

            return {
                "ok": (
                    self.ethercat_check_state == EthercatCheckState.READY
                    and not stale
                ),
                "state": self.ethercat_check_state.name,
                "stale": stale,
                "last_update_age_s": (
                    None if last_update_age is None else round(last_update_age, 3)
                ),
                "plc_slave_identifier": self._plc_slave_identifier(),
                "plc_slave": (
                    None
                    if plc_name is None
                    else {"name": plc_name, "state": plc_state}
                ),
                "service_names": list(self._ethercat_slave_state_clients.keys()),
                "slaves": [
                    {"name": slave_name, "state": slave_state}
                    for slave_name, slave_state in visible_pairs
                ],
                "last_error": self._ethercat_last_error,
            }
    
    def publish_bringup_commands(self) -> None:
        """Publish initial startup commands to enable PLC outputs.
        
        Called once during system bringup to:
        1. Release E-stop (set to 1 = normal operation)
        2. Enable force/torque sensor power
        3. Enable Z recovery flag to add vertical switch axis limit to safety chain
        These commands prepare the PLC for motion control.
        
        Returns:
            None. Publishes commands via self.publish_command()
        """
        self.publish_command('PLC_node/estop', 1)
        self.publish_command('PLC_node/z_recovery', 1)
        self.publish_command('PLC_node/force_sensors_pwr', 1)

    def _get_z_limit_switch_state(self) -> bool:
        z_limit_active = False
        for _i in range(len(self.interface_names)):
            if self.interface_names[_i] == "z_limit_switch":
                z_limit_active = (self.state_values[_i] == 1)
                break
        return z_limit_active


    def _bringup_env(self) -> None:
        self._startup_cleanup_action = self._kill_env
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


    def _bringup_recovery_env(self) -> None:
        self._startup_cleanup_action = self._kill_recovery_env
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


    def _reset_sw_estop(self) -> None:
        self.publish_command('PLC_node/estop', 1)
        self.publish_command('PLC_node/manual_mode', 0)

    def _handle_idle_stop(self) -> None:
        cleanup_action = self._startup_cleanup_action
        self._startup_cleanup_action = None

        if cleanup_action is not None:
            cleanup_action()
            return

        self._reset_sw_estop()

    def _detect_estop_event(
        self,
        estop_value: EStopState,
        sw_estop_value: EStopState,
    ) -> Event:
        if (
            self.estop_cached == EStopState.EMERGENCY
            and estop_value == EStopState.OK
            and sw_estop_value == EStopState.OK
        ):
            return Event.START

        if self.estop_cached == EStopState.OK and estop_value == EStopState.OK:
            if self.sw_estop_cached != sw_estop_value:
                if sw_estop_value == EStopState.EMERGENCY:
                    self.publish_command('PLC_node/estop', 0)           # Emergency stop (deactivate)
                    self.publish_command('PLC_node/manual_mode', 0)     # Return to automatic
                    self.get_logger().info(bc.FAIL + 'Raised a SW ESTOP!' + bc.ENDC) #type: ignore
                self.sw_estop_cached = sw_estop_value
            return Event.NONE

        if self.estop_cached == EStopState.OK and estop_value == EStopState.EMERGENCY:
            return Event.STOP

        return Event.NONE


    def _state_callback_estop(self, estop_value: EStopState, 
                              z_limit_active: bool, 
                              sw_estop_value: EStopState) -> None:
        
        ESTOP : EStopState = estop_value
        SW_ESTOP: EStopState = sw_estop_value
       
        
        event = self._detect_estop_event(ESTOP, SW_ESTOP)
        if self.fsm.pending is None:
            
            try:
                if event != Event.NONE:
                    self.fsm.trigger(event) #type: ignore 
                    self.estop_cached = ESTOP
                    self.sw_estop_cached = SW_ESTOP
            except InvalidTransition as e:
                self.get_logger().error(f"Invalid transition: {e}") #type: ignore
            except GuardFailed as e:
                self.get_logger().error(f"Guard condition failed: {e}") #type: ignore
                if event == Event.START:
                    self._startup_cleanup_action = None
                    self.publish_command('PLC_node/estop', 0)
                    self.publish_command('PLC_node/manual_mode', 0)
                    self.estop_cached = ESTOP
                    self.sw_estop_cached = SW_ESTOP
            
        else:
            pending = self.fsm.pending
            try:
                if event == Event.STOP and pending.event == Event.START:
                    self.get_logger().warning( #type: ignore
                        "Emergency stop requested while START transition is pending. "
                        "Cancelling startup and triggering STOP."
                    )
                    self.fsm.cancel_pending()
                    self.fsm.trigger(Event.STOP) #type: ignore
                    self.estop_cached = ESTOP
                    self.sw_estop_cached = SW_ESTOP
                    return

                status = self.fsm.step() #type: ignore
            except InvalidTransition as e:
                self.get_logger().error(f"Invalid transition: {e}") #type: ignore
            except GuardFailed as e:
                self.get_logger().error(f"Guard condition failed: {e}") #type: ignore
            except TransitionTimeout as e:
                self.get_logger().error(f"Timeout transition: {e}") #type: ignore
                self._cleanup_timed_out_transition(pending)
            
    def _cleanup_timed_out_transition(self, pending: PendingTransition | None) -> None:
        if pending is None:
            return

        if pending.event == Event.START:
            if pending.source == State.IDLE:
                self._kill_env()
            elif pending.source == State.IDLE_RECOVERY:
                self._kill_recovery_env()
        elif pending.event == Event.STOP or pending.event == Event.FAIL:
            if pending.source == State.RUNNING:
                self._kill_env()
            elif pending.source == State.RUNNING_RECOVERY:
                self._kill_recovery_env()

    def state_callback(self, msg: PlcStates) -> None:
        # ========== Acquire lock to prevent concurrent execution ==========
        if not self.bringup_done:
            return
        
        if not self.lock.acquire(blocking=False):
            self.get_logger().warn("state_callback already in execution, ignore.") #type: ignore
            return
        
        try:
    
            # ========== Extract PLC state from message ==========
            self.interface_names = list(msg.interface_names) #type: ignore
            self.state_values = list(msg.values) #type: ignore
            
            # ========== DEFINE WHICH STATE MACHINE IS
            # Gate Z-axis recovery on the dedicated PLC input 'z_limit_switch'
            # (value 1 = limit active). Only a genuine Z-limit event enters the
            # recovery procedure; all other emergencies are a normal stop.
            z_limit_active = self._get_z_limit_switch_state()
            
            # ========== First Call of the Callback ========== 
            if self.fsm.pending is None:
                if self.fsm.state == State.IDLE and z_limit_active:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'Raised a Z-LIMIT event! Set IDLE_RECOVERY state' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.fsm.trigger(Event.SWITCH_MODE)
                
                if self.fsm.state == State.RECOVERED and not z_limit_active:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'Recovered the Z-LIMIT event' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.fsm.trigger(Event.SWITCH_MODE)
            
                if self.fsm.state == State.IDLE:    
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'Waiting for ros controllers to start - TURN THE KEY to START!' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )

                elif self.fsm.state == State.IDLE_RECOVERY:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'RECOVERY MODE - Waiting for ros controllers to start - TURN THE KEY to START!' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )

                if self.fsm.state in (State.RUNNING, State.RUNNING_RECOVERY) and self._ros_gui_connection_failed():
                    self.client.clear_last_message()
                    self.fsm.trigger(Event.FAIL)

            # ========== Process Status info ==========
            processes_status = \
                self.check_env_running(False) if self.fsm.state == State.RUNNING else\
                self.check_env_running_recovery(False) if self.fsm.state == State.RUNNING_RECOVERY else\
                self.check_env_running_stopped() if self.fsm.state == State.IDLE else\
                self.check_env_running_recovery_stopped() \
                    if self.fsm.state == State.RECOVERED or self.fsm.state == State.IDLE_RECOVERY else\
                False
            if not processes_status:
                self.get_logger().error( #type: ignore
                    bc.FAIL + 'The state of the processes is not matching the expected state!' + bc.ENDC,
                    throttle_duration_sec=5.0
                )

            sw_estop : EStopState = EStopState.OK
            if (
                (self.fsm.state == State.RUNNING or self.fsm.state == State.RUNNING_RECOVERY) and
                (self.processes_status_cached and not processes_status)
            ):
                sw_estop = EStopState.EMERGENCY

            if self.fsm.state == State.ERROR:
                sw_estop = EStopState.EMERGENCY
                        
            self.processes_status_cached = processes_status

            # ========== Search for E-stop signal in state message ==========
            for int_idx in range(len(self.interface_names)):
                if self.interface_names[int_idx] == "estop":
                    self._state_callback_estop(
                        EStopState.OK if self.state_values[int_idx] == 1 else EStopState.EMERGENCY, #type: ignore
                        z_limit_active, 
                        sw_estop
                    )
        except InvalidTransition as e:
            self.get_logger().error(f"Invalid transition: {e}") #type: ignore
        except GuardFailed as e:
            self.get_logger().error(f"Guard condition failed: {e}") #type: ignore
        except Exception as e:
            self.get_logger().error(f"Exception in state_callback: {e}") #type: ignore
        finally:
            # ========== FSM States info ==========
            self._notify_gui(self._fsm_status_payload())
            self.lock.release()

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

    def check_env_running_recovery(self, check_controller_status: bool = True) -> bool:

        if not check_env([
            "run_z_recovery_control.launch.py",
            "run_rosbridge.launch.py"
        ]):
            return False

        if check_controller_status:
            return self.recovery_controller_status_monitor.ok()

        return True

    def _kill_recovery_env(self) -> None:
        self._startup_cleanup_action = None
        self.recovery_controller_status_monitor.reset()
        self.publish_command('PLC_node/brake_disable', 0)  # Ensure brakes enabled
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

    def _kill_env(self) -> None:
        self._startup_cleanup_action = None
        self.platform_controller_readiness_monitor.reset()
        self.publish_command('PLC_node/brake_disable', 0)  # Ensure brakes enabled
        self.publish_command('PLC_node/estop', 1) # Reset SW E-stop to normal operation if it was triggered

        stop_launch_environment(
            "run_platform_control.launch.py",
            "run_platform_control.launch"
        )
        

        stop_launch_environment(
                "ros2 launch tecnobody_workbench run_rosbridge.launch.py",
                "run_rosbridge.launch", 
                9090
        )

    def _fsm_status_payload(self) -> bytes:
        pending = self.fsm.pending
        payload: dict[str, object] = {
            "schema": "fit4med.plc_fsm_status.v1",
            "state": self.fsm.state.name,
            "pending": None,
            "ethercat": self._ethercat_status_payload(),
            "plc_outputs": {
                "interface_names": list(self.plc_outputs.interface_names), #type: ignore
                "values": [int(value) for value in self.plc_outputs.values], #type: ignore
            },
            "plc_inputs": {
                "interface_names": list(self.interface_names), #type: ignore
                "values": [int(value) for value in self.state_values], #type: ignore
            },
        }

        if pending is not None:
            payload["pending"] = {
                "event": pending.event.name,
                "source": pending.source.name,
                "target": pending.transition.destination.name,
                "steps": pending.steps,
            }

        return json.dumps(payload, separators=(",", ":")).encode()

    def _notify_gui(self, udp_msg: bytes) -> None:
        try:
            self.client.send(udp_msg)
        except Exception as e:
            self.get_logger().warn(f"Error communicating with GUI: {e}") #type: ignore


    def publish_command(self, name: str, value: int) -> None:
        """Publish single PLC command by interface name.
        
        Helper function to update and publish a specific PLC command value.
        Updates the plc_outputs, finds the interface by name, updates its value,
        and publishes the entire 8-element command vector.
        
        Example:
            publish_command('PLC_node/estop', 1)        # Release E-stop
            publish_command('PLC_node/force_sensors_pwr', 1)  # Enable F/T power
        
        Args:
            name (str): Interface identifier (must match one in plc_outputs.interface_names)
            value (int): Command value (typically 0 or 1)
        
        Returns:
            None. Publishes PlcController message if name found.
        
        Log Output:
            - INFO: "Published command message: [0, 0, 0, 0, 1, 0, 0, 0]"
            - WARN: "Interface name '{name}' not found in command message" if not found
        """
        if name in self.plc_outputs.interface_names: #type: ignore
            # Find index of interface by name
            idx = self.plc_outputs.interface_names.index(name) #type: ignore
            # Update value
            self.plc_outputs.values[idx] = value #type: ignore
            # Publish entire command vector
            self.command_publisher.publish(self.plc_outputs)
            self.get_logger().info(f"Published PLC command: {self.plc_outputs.values}") #type: ignore
        else:
            self.get_logger().warn( #type: ignore
                f"Interface name '{name}' not found in command message."
            )


    def signal_handler(self, sig: int, frame: types.FrameType) -> None:
        """Handle SIGINT and SIGTERM for graceful shutdown.
        
        Called when Ctrl+C pressed or SIGTERM received. Sets shutdown_requested
        flag for main() loop to detect and break gracefully.
        
        Args:
            sig: Signal number (SIGINT or SIGTERM)
            frame: Current stack frame
        
        Returns:
            None. Sets shutdown_requested flag.
        """
        self.get_logger().info("Signal received. Setting shutdown_requested flag.") #type: ignore
        self.shutdown_requested = True


def main(args=None): #type: ignore
    """Entry point for the PLC manager state machine node.
    
    Initializes PLCControllerInterface node, and runs
    the multi-threaded executor loop that processes PLC state changes.
    
    Execution Flow:
        1. Parse command-line arguments (target_ip for UDP GUI coordination)
        2. Initialize ROS 2 with custom signal handling
        3. Set CPU affinity to core 2 for deterministic scheduling
        4. Create PLCControllerInterface node
        5. Create MultiThreadedExecutor with 2 threads
        6. Run executor spin loop until:
           - shutdown_requested flag set by signal handler
           - bringup_done flag set after first successful launch
           - rclpy.ok() returns False (system shutdown)
        7. Graceful cleanup and ROS 2 shutdown
    
    Bringup Sequence (first iteration):
        1. Check if EtherCAT PLC slave active through /ethercat_slaves_status_check/check
        2. Wait for subscribers to command topic (> 0 subscribers)
        3. When both ready: publish_bringup_commands (E-stop=1, F/T power=1)
        4. Set bringup_done=True to prevent repeated bringup
    
    Args:
        args (list, optional): Command-line arguments (default: None)
            Expected: [script_name, target_ip, ...]
            If not provided: defaults to "127.0.0.0" (localhost)
    
    Returns:
        None. Blocks until shutdown.
    """
    # Initialize ROS 2 without automatic signal handling
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO) #type: ignore
    
    # ========== Parse arguments ==========
    target_ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.0"

    # ========== Create node instance ==========
    node = PLCControllerInterface(target_ip)
    
    # ========== Create multi-threaded executor ==========
    # Thread 1: PLC state subscription callback (state_callback)
    # Thread 2: low-rate EtherCAT timer and service response callbacks
    mt_executor = MultiThreadedExecutor(num_threads=2)
    mt_executor.add_node(node)

    # ========== Set CPU affinity to core 2 ==========
    # Ensures deterministic scheduling for control tasks
    import os
    os.sched_setaffinity(0, {7})
    
    # ========== Bringup flag (execute once at startup) ==========
    try:
        node.bringup_done = False
        while rclpy.ok():
            mt_executor.spin_once(timeout_sec=0.1)

            if node.ethercat_startup_check_requested:

                if not node.ethercat_slaves_status_check_client.wait_for_service(timeout_sec=5.0):
                    node.get_logger().error( #type: ignore
                        "EtherCAT status check service not available. "
                        f"Ensure {node.ethercat_startup_check_service_name} "
                        "is running."
                    )
                    continue

                future = node.ethercat_slaves_status_check_client.call_async(Trigger.Request()) #type: ignore
                node.ethercat_startup_check_state = EthercatCheckState.PENDING

                mt_executor.spin_until_future_complete(
                    future, #type: ignore
                    timeout_sec=5.0,
                )

                if future.done() and future.result() is not None: #type: ignore
                    startup_check_result = future.result() #type: ignore
                    node.ethercat_startup_check_state = (
                        EthercatCheckState.READY
                        if startup_check_result.success
                        else EthercatCheckState.FAILED
                    )
                    if not startup_check_result.success:
                        node.get_logger().warning( #type: ignore
                            "EtherCAT startup check failed: "
                            f"{startup_check_result.message}"
                        )
                else:
                    node.ethercat_startup_check_state = EthercatCheckState.FAILED
                    node.get_logger().warning( #type: ignore
                        "EtherCAT startup check timed out."
                    )

                node.ethercat_startup_check_requested = not (
                    node.ethercat_startup_check_state == EthercatCheckState.READY
                )

            # ========== Initial bringup sequence (once only) ==========
            if not node.bringup_done \
            and node._ethercat_slaves_status_ok() \
            and node.command_publisher.get_subscription_count() > 0:
                node.publish_bringup_commands()
                node.bringup_done = True
                node.ethercat_state_timer.reset()  # Start EtherCAT polling timer after bringup
                
            # ========== Check for graceful shutdown request ==========
            if node.shutdown_requested:
                node.get_logger().warning('Received graceful shutdown request!') #type: ignore
                break

    except Exception as e:
        node.get_logger().error(f"Unexpected exception in main loop: {e}") #type: ignore
    
    # ========== Graceful cleanup ==========
    try:
        node.cleanup()
        rclpy.try_shutdown() #type: ignore
    except Exception as e:
        print(f"Error during shutdown: {e}")


if __name__ == '__main__':
    main()
