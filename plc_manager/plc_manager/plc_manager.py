# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
 
import os
import types


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
    TransitionTimeout, PendingTransition
)
from plc_manager.environment_manager import EnvironmentManager
from plc_manager.ethercat_monitor import EthercatMonitor
from plc_manager.fsm_config import build_plc_fsm
from plc_manager.gui_status import GuiStatusPublisher
from plc_manager.plc_commands import PlcCommandPublisher
from plc_manager.plc_types import EStopState, EthercatCheckState, Event, State
from plc_manager.udp_client import UdpClient


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
        
        self.plc_commands = PlcCommandPublisher(
            self.command_publisher,
            self.get_logger(),
        )

        # ========== Launcher Health Monitoring ==========
        self.processes_status_cached = False
        self.environment = EnvironmentManager(
            self,
            service_group=self.service_group,
            publish_command=self.publish_command,
        )


        # ========== UDP Status Client ==========#  
        # Coordinates with rehab_gui for emergency stop and status
        self.client = UdpClient(target_ip, 5005)
        self.gui_status = GuiStatusPublisher(self.client, self.get_logger())
        self.send_running_cnt = 0
        self.send_running_dec = 10  # Send status every 10*50ms = 500ms at 50 Hz
        self.shutdown_requested = False

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
        self.fsm = build_plc_fsm(self)
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

    @property
    def _startup_cleanup_action(self):  # type: ignore
        return self.environment.startup_cleanup_action

    @_startup_cleanup_action.setter
    def _startup_cleanup_action(self, value) -> None:  # type: ignore
        self.environment.startup_cleanup_action = value

    @property
    def platform_controller_readiness_monitor(self):  # type: ignore
        return self.environment.platform_controller_readiness_monitor

    @property
    def recovery_controller_status_monitor(self):  # type: ignore
        return self.environment.recovery_controller_status_monitor

    @property
    def plc_outputs(self):  # type: ignore
        return self.plc_commands.plc_outputs

    @property
    def ethercat_startup_check_service_name(self) -> str:
        return self.ethercat.startup_check_service_name

    @property
    def ethercat_slaves_status_check_client(self):  # type: ignore
        return self.ethercat.startup_check_client

    @property
    def ethercat_startup_check_state(self) -> EthercatCheckState:
        return self.ethercat.startup_check_state

    @ethercat_startup_check_state.setter
    def ethercat_startup_check_state(self, value: EthercatCheckState) -> None:
        self.ethercat.startup_check_state = value

    @property
    def ethercat_startup_check_requested(self) -> bool:
        return self.ethercat.startup_check_requested

    @ethercat_startup_check_requested.setter
    def ethercat_startup_check_requested(self, value: bool) -> None:
        self.ethercat.startup_check_requested = value

    @property
    def ethercat_check_state(self) -> EthercatCheckState:
        return self.ethercat.check_state

    @ethercat_check_state.setter
    def ethercat_check_state(self, value: EthercatCheckState) -> None:
        self.ethercat.check_state = value

    @property
    def ethercat_state_timer(self):  # type: ignore
        return self.ethercat.state_timer

    def _ethercat_slaves_status_ok(self) -> bool:
        return self.ethercat.startup_status_ok()

    def _ready_to_start(self) -> bool:
        return self._ros_gui_disconnected() and self._ethercat_slaves_status_ok()

    def poll_ethercat_slave_states(self) -> None:
        self.ethercat.poll_slave_states()

    def _ethercat_status_payload(self) -> dict[str, object]:
        return self.ethercat.status_payload()
    
    def publish_bringup_commands(self) -> None:
        self.plc_commands.publish_bringup_commands()

    def _get_z_limit_switch_state(self) -> bool:
        z_limit_active = False
        for _i in range(len(self.interface_names)):
            if self.interface_names[_i] == "z_limit_switch":
                z_limit_active = (self.state_values[_i] == 1)
                break
        return z_limit_active


    def _bringup_env(self) -> None:
        self.environment.bringup_env()

    def _bringup_recovery_env(self) -> None:
        self.environment.bringup_recovery_env()

    def _reset_sw_estop(self) -> None:
        self.environment.reset_sw_estop()

    def _handle_idle_stop(self) -> None:
        self.environment.handle_idle_stop()

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
                    self.environment.startup_cleanup_action = None
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
        self.environment.cleanup_timed_out_transition(pending)

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
        return self.environment.check_env_running_stopped()

    def check_env_running_recovery_stopped(self) -> bool:
        return self.environment.check_env_running_recovery_stopped()

    def check_env_running(self, check_controller_status: bool = True) -> bool:
        return self.environment.check_env_running(check_controller_status)

    def check_env_running_recovery(self, check_controller_status: bool = True) -> bool:
        return self.environment.check_env_running_recovery(check_controller_status)

    def _kill_recovery_env(self) -> None:
        self.environment.kill_recovery_env()

    def _kill_env(self) -> None:
        self.environment.kill_env()

    def _fsm_status_payload(self) -> bytes:
        return self.gui_status.fsm_status_payload(
            self.fsm,
            self._ethercat_status_payload(),
            self.plc_outputs,
            self.interface_names,
            self.state_values,
        )

    def _notify_gui(self, udp_msg: bytes) -> None:
        self.gui_status.notify(udp_msg)


    def publish_command(self, name: str, value: int) -> None:
        self.plc_commands.publish_command(name, value)


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

            if not node.ethercat.run_startup_check(mt_executor):
                continue

            # ========== Initial bringup sequence (once only) ==========
            if not node.bringup_done \
            and node._ethercat_slaves_status_ok() \
            and node.command_publisher.get_subscription_count() > 0:
                node.publish_bringup_commands()
                node.bringup_done = True
                node.ethercat.start_runtime_polling()
                
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
