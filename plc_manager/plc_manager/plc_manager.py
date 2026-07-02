# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
 
import os
import time
import types

from typing import Tuple

# Must be BEFORE importing rclpy (sets logging format globally)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tecnobody_msgs.msg import PlcController, PlcStates
import sys
import threading
import signal

from tecnobody_workbench_utils.utils import bcolors as bc

from tecnobody_workbench_utils.fsm import (
    InvalidTransition, GuardFailed,
    TransitionTimeout,
)
from plc_manager.environment_manager import EnvironmentManager
from plc_manager.ethercat_monitor import EthercatMonitor
from plc_manager.fsm_config import build_plc_fsm
from plc_manager.gui_status import GuiStatusPublisher
from plc_manager.plc_commands import PlcCommandPublisher
from plc_manager.plc_types import EStopState, Event, State
from plc_manager.udp_client import UdpClient


class FailedSafeShutdown(Exception):
    pass


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
        self.last_estop_monotonic: float | None = None
        self.lock = threading.Lock()                            # Protects state_callback() concurrent access
        
        self.plc_commands = PlcCommandPublisher(
            self.command_publisher,
            self.get_logger(),
        )

        # ========== Launcher Health Monitoring ==========
        self.processes_status_cached = False
        self.environment = EnvironmentManager(
            self,
            service_group=self.service_group,
            timer_group=self.timer_group,
        )


        # ========== UDP Status Client ==========#  
        # Coordinates with rehab_gui for emergency stop and status
        self.client = UdpClient(target_ip, 5005)
        self.gui_status = GuiStatusPublisher(self.client, self.get_logger())
        self.send_running_cnt = 0
        self.send_running_dec = 10  # Send status every 10*50ms = 500ms at 50 Hz
        self.shutdown_requested = False
        self.shutdown_stop_requested = False
        self.shutdown_estop_closed = False

        # ========== Signal Handlers ==========
        signal.signal(signal.SIGINT, self.signal_handler) #type: ignore
        signal.signal(signal.SIGTERM, self.signal_handler) #type: ignore

        self.z_limit_still_active = False
        self.ethercat = EthercatMonitor(
            self,
            service_group=self.service_group,
            timer_group=self.timer_group,
            fsm_state_getter=lambda: self.fsm.state,
        )
        self.start_environment_requested = True
        self.declare_parameter('estop_rearm_holdoff_sec', 1.0)
        self.fsm = build_plc_fsm(self)
        self.bringup_done = False
    
    def cleanup(self) -> None:
        self.get_logger().info("Cleanup: destroying node resources.") #type: ignore
        self.client.close()
        self.environment.destroy()
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
        return self.ethercat.startup_status_ok()

    def _estop_rearm_delay_elapsed(self) -> bool:
        if self.last_estop_monotonic is None:
            return True

        holdoff_sec = float(
            self.get_parameter('estop_rearm_holdoff_sec').value
        )
        elapsed_sec = time.monotonic() - self.last_estop_monotonic
        if elapsed_sec >= holdoff_sec:
            return True

        self.get_logger().warning(  # type: ignore
            (
                "START requested before software re-arm holdoff "
                f"elapsed ({elapsed_sec:.2f}/{holdoff_sec:.2f} s)."
            ),
            throttle_duration_sec=1.0,
        )
        return False

    def _get_plc_input_bool(
        self,
        interface_name: str,
        default: bool = False,
    ) -> bool:
        if interface_name not in self.interface_names:
            return default

        interface_index = self.interface_names.index(interface_name)
        return bool(self.state_values[interface_index])

    def _get_z_limit_switch_state(self) -> bool:
        return self._get_plc_input_bool("z_limit_switch")

    def _detect_estop_event(
        self,
        estop_value: EStopState,
        sw_estop_value: EStopState,
    ) -> Tuple[Event,str]:

        if self.estop_cached == EStopState.OK and estop_value == EStopState.EMERGENCY:
            self.last_estop_monotonic = time.monotonic()
            return (Event.STOP, "!!! Emergency stop triggered !!! - Stop Drivers and Controllers")

        if estop_value == EStopState.OK and self.sw_estop_cached != sw_estop_value:
            if sw_estop_value == EStopState.EMERGENCY:
                self.plc_commands.raise_sw_estop()
                self.plc_commands.set_automatic_mode()
                self.get_logger().info(bc.FAIL + 'Raised a SW ESTOP!' + bc.ENDC)    # type: ignore
            return (Event.NONE, "SW ESTOP changed - No FSM event needed")

        if (
            self.estop_cached == EStopState.EMERGENCY
            and estop_value == EStopState.OK
            and sw_estop_value == EStopState.OK
            and self.fsm.state in (State.IDLE, State.IDLE_RECOVERY)
        ):
            return (Event.START, "Emergency chain is closed - Start Drivers and Controllers Bringup")

        if self.estop_cached == EStopState.EMERGENCY and estop_value == EStopState.OK:
            return (Event.NONE, "Emergency chain is closed - emergency latch stays active")

        if self.estop_cached == EStopState.OK and estop_value == EStopState.OK:
            return (Event.NONE, "The Emergency stop has been reset, No SW ESTOP Present, and cached ESTOP already OK - No action needed")

        return (Event.NONE, f"[ESTOP={estop_value}, SW_ESTOP={sw_estop_value}] - No action needed")

    def _state_callback_estop(self, estop_value: EStopState, 
                              z_limit_active: bool, 
                              sw_estop_value: EStopState) -> None:
        
        ESTOP : EStopState = estop_value
        SW_ESTOP: EStopState = sw_estop_value
        
        event, msg = self._detect_estop_event(ESTOP, SW_ESTOP)
        try:
            if self.fsm.pending is None:
                if event != Event.NONE:
                    self.fsm.trigger(event, msg) #type: ignore

            else:
                pending = self.fsm.pending
                if event == Event.STOP and pending.event == Event.START:
                    self.get_logger().warning( #type: ignore
                        "Emergency stop requested while START transition is pending. "
                        "Cancelling startup and triggering STOP."
                    )
                    self.fsm.cancel_pending()
                    self.fsm.trigger(Event.STOP, "Emergency stop requested while START transition is pending.")
                    return

                status = self.fsm.step() #type: ignore
        except InvalidTransition as e:
            self.get_logger().error(f"Invalid transition: {e}") #type: ignore
        except GuardFailed as e:
            self.get_logger().error(f"Guard condition failed: {e}") #type: ignore
            if event == Event.START:
                self.environment.startup_cleanup_action = None
                self.plc_commands.raise_sw_estop()
                self.plc_commands.set_automatic_mode()
        except TransitionTimeout as e:
            self.get_logger().error(f"Timeout transition: {e}") #type: ignore
        finally:
            self.estop_cached = ESTOP
            self.sw_estop_cached = SW_ESTOP

    def force_shutdown_stop(self) -> None:
        if self.shutdown_stop_requested:
            return

        self.shutdown_stop_requested = True
        self.get_logger().warning( #type: ignore
            "Shutdown requested: opening PLC safety chain."
        )
        self.plc_commands.raise_sw_estop()
        self.plc_commands.close_brake()
        self.plc_commands.set_automatic_mode()
        self.plc_commands.wire_endstroke_to_emergency_chain()

    def shutdown_safe_to_exit(self) -> bool:
        return (
            self.fsm.pending is None
            and self.environment.startup_cleanup_action is None
            and self.fsm.state in (
                State.IDLE,
                State.IDLE_RECOVERY,
                State.ESTOP,
                State.ESTOP_RECOVERY,
                State.RECOVERED,
            )
        )

    def close_shutdown_safety_chain(self) -> None:
        if self.shutdown_estop_closed:
            return

        self.shutdown_estop_closed = True
        self.get_logger().warning( #type: ignore
            "Shutdown complete: closing PLC safety chain before exit."
        )
        self.plc_commands.clear_sw_estop()

    def _uses_recovery_cleanup_context(self) -> bool:
        recovery_states = (
            State.IDLE_RECOVERY,
            State.ESTOP_RECOVERY,
            State.RUNNING_RECOVERY,
            State.RECOVERED,
            State.ERROR_RECOVERY,
        )
        pending = self.fsm.pending
        if pending is not None:
            return (
                pending.source in recovery_states
                or pending.transition.destination in recovery_states
                or pending.transition.failure_destination in recovery_states
            )

        return self.fsm.state in recovery_states

    def force_environment_cleanup(self) -> None:
        recovery_context = self._uses_recovery_cleanup_context()
        pending = self.fsm.pending

        if pending is not None:
            failure_destination = pending.transition.failure_destination
            self.fsm.cancel_pending()
            if failure_destination is not None:
                self.fsm.state = failure_destination
        elif self.fsm.state == State.RUNNING:
            self.fsm.state = State.ERROR
        elif self.fsm.state == State.RUNNING_RECOVERY:
            self.fsm.state = State.ERROR_RECOVERY

        cleanup_action = self.environment.startup_cleanup_action
        if cleanup_action is not None:
            cleanup_action()
        elif recovery_context:
            self.environment.kill_recovery_env()
        else:
            self.environment.kill_env()

    def forced_environment_cleanup_complete(self) -> bool:
        return (
            self.environment.check_env_running_stopped()
            and self.environment.check_env_running_recovery_stopped()
        )

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
                if self.fsm.state in (State.IDLE, State.ESTOP) and z_limit_active:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + f'Z-LIMIT Reached! Set {"IDLE_RECOVERY" if self.fsm.state==State.IDLE else "ESTOP_RECOVERY"} state' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.fsm.trigger(Event.SWITCH_MODE, "Z-LIMIT Reached - Entering Recovery Mode")
                
                if self.fsm.state in (State.RECOVERED, State.ESTOP_RECOVERY) and not z_limit_active:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'Z-LIMIT Recovered! Set IDLE state' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.fsm.trigger(Event.SWITCH_MODE, "Z-LIMIT Recovered - Entering IDLE Mode")
            
                if self.fsm.state == State.IDLE:    
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'Waiting for ros controllers to start - TURN THE KEY to START!' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.plc_commands.set_automatic_mode()
                    self.plc_commands.clear_sw_estop()
                    self.plc_commands.close_brake()

                elif self.fsm.state == State.ESTOP:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'Emergency latch active - supervised restart required!' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.plc_commands.set_automatic_mode()
                    self.plc_commands.clear_sw_estop()
                    self.plc_commands.close_brake()

                elif self.fsm.state == State.IDLE_RECOVERY:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'RECOVERY MODE - Waiting for ros controllers to start - TURN THE KEY to START!' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.plc_commands.set_automatic_mode()
                    self.plc_commands.clear_sw_estop()
                    self.plc_commands.close_brake()

                elif self.fsm.state == State.ESTOP_RECOVERY:
                    self.get_logger().info( #type: ignore
                        bc.MAGENTA + 'RECOVERY MODE - Emergency latch active - supervised restart required!' + bc.ENDC,
                        throttle_duration_sec=5.0
                    )
                    self.plc_commands.set_automatic_mode()
                    self.plc_commands.clear_sw_estop()
                    self.plc_commands.close_brake()

                if self.fsm.state in (State.RUNNING, State.RUNNING_RECOVERY)\
                and self._ros_gui_connection_failed():
                    self.client.clear_last_message()
                    self.fsm.trigger(Event.FAIL, "ROS GUI connection failed")

            # ========== Process Status info ==========
            processes_status = self.environment.check_expected_processes(
                self.fsm.state,
                self.fsm.pending,
            )
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

            if self.fsm.state in (State.ERROR, State.ERROR_RECOVERY):
                sw_estop = EStopState.EMERGENCY
                        
            self.processes_status_cached = processes_status

            # ========== Search for E-stop signal in state message ==========
            if "estop" in self.interface_names:
                estop_index = self.interface_names.index("estop")
                estop_value = EStopState.OK if self.state_values[estop_index] == 1 else EStopState.EMERGENCY
                self._state_callback_estop(
                    estop_value,
                    z_limit_active,
                    sw_estop
                )
            else:
                self.get_logger().error( #type: ignore
                    bc.FAIL + 'No estop signal in the PLC state message! Escalate the shutdown' + bc.ENDC,
                    throttle_duration_sec=5.0
                )
                self.force_shutdown_stop()
                self.force_environment_cleanup()
                self.signal_handler(sig=signal.SIGTERM, frame=sys._getframe()) #type: ignore

        except InvalidTransition as e:
            self.get_logger().error(f"Invalid transition: {e}") #type: ignore
        except GuardFailed as e:
            self.get_logger().error(f"Guard condition failed: {e}") #type: ignore
        except Exception as e:
            self.get_logger().error(f"Exception in state_callback: {e}") #type: ignore
        finally:
            # ========== FSM States info ==========
            self.gui_status.notify(self._fsm_status_payload())
            self.lock.release()

    def _fsm_status_payload(self) -> bytes:
        return self.gui_status.fsm_status_payload(
            self.fsm,
            self.ethercat.status_payload(),
            self.plc_commands.plc_outputs,
            self.interface_names,
            self.state_values,
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

            if node.shutdown_requested:
                node.get_logger().warning( #type: ignore
                    'Received graceful shutdown request!',
                    throttle_duration_sec=5.0,
                )
                if node.shutdown_safe_to_exit():
                    node.close_shutdown_safety_chain()
                    break
                else:
                    node.force_shutdown_stop()
                    raise FailedSafeShutdown("Shutdown not safe to exit. Forced an SW ESTOP and cleanup actions.")

            if not node.ethercat.run_startup_check(mt_executor):
                continue

            # ========== Initial bringup sequence (once only) ==========
            if not node.bringup_done \
            and node._ethercat_slaves_status_ok() \
            and node.command_publisher.get_subscription_count() > 0:
                node.plc_commands.publish_bringup_commands()
                node.bringup_done = True
                node.ethercat.start_runtime_polling()
    except FailedSafeShutdown as e:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected exception in main loop: {e}") #type: ignore
        node.force_shutdown_stop()
    finally:
        watchdog = time.monotonic() + 5.0  # 5 seconds timeout for cleanup
        while not node.shutdown_safe_to_exit() and time.monotonic() < watchdog:
            mt_executor.spin_once(timeout_sec=0.1)
        if not node.shutdown_safe_to_exit():
            node.get_logger().error(  # type: ignore
                "Shutdown watchdog expired before a safe FSM state. "
                "Forcing launch-environment cleanup."
            )
            node.force_environment_cleanup()
            cleanup_watchdog = time.monotonic() + 5.0
            while (
                not node.forced_environment_cleanup_complete()
                and time.monotonic() < cleanup_watchdog
            ):
                mt_executor.spin_once(timeout_sec=0.1)

        if node.shutdown_safe_to_exit():
            node.close_shutdown_safety_chain()
        else:
            node.get_logger().error(  # type: ignore
                "Shutdown not verified safe; leaving PLC SW ESTOP open."
            )
    # ========== Graceful cleanup ==========
    try:
        node.cleanup()
        rclpy.try_shutdown() #type: ignore
    except Exception as e:
        print(f"Error during shutdown: {e}")


if __name__ == '__main__':
    main()
