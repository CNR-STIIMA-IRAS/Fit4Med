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
from rclpy.client import Client
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tecnobody_msgs.msg import PlcController, PlcStates
from std_srvs.srv import Trigger
import sys
import subprocess
import threading
import signal
from enum import Enum

if __package__:
    from plc_manager.fsm import (
        StateMachine, State, Event, InvalidTransition, GuardFailed,
        TransitionTimeout, PendingTransition
    )
    from plc_manager.udp_client import UdpClient
    from plc_manager.utils import (
        check_env_running,
        check_env_running_recovery,
        check_env_running_stopped,
        check_env_running_recovery_stopped,
        stop_launch_environment
    )
    from plc_manager.utils import bcolors as bc
else:
    from fsm import (  # type: ignore
        StateMachine, State, Event, InvalidTransition, GuardFailed,
        TransitionTimeout, PendingTransition
    )
    from udp_client import UdpClient  # type: ignore
    from utils import (  # type: ignore
        check_env_running,
        check_env_running_recovery,
        check_env_running_stopped,
        check_env_running_recovery_stopped,
        stop_launch_environment
    )
    from utils import bcolors as bc  # type: ignore

class EStopState(Enum):
    OK = 1
    EMERGENCY = 0

class EthercatCheckState(Enum):
    IDLE = 0
    PENDING = 1
    READY = 2
    FAILED = 3

class PLCControllerInterface(Node):

    def __init__(self, target_ip: str):

        super().__init__('plc_manager')

        # ========== Callback Groups ==========
        # Three separate groups prevent callback blocking:
        self.plc_group = MutuallyExclusiveCallbackGroup()       # PLC state updates
        self.timer_group = MutuallyExclusiveCallbackGroup()     # Launcher health check
        self.service_group = MutuallyExclusiveCallbackGroup()   # Reserved for services

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
        self.command_msg = PlcController()
        self.command_msg.values  = [0] * 10
        self.command_msg.interface_names = [
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

        self.z_limit_still_active = False

        self.ethercat_slaves_status_check_client : Client = self.create_client(Trigger, #type: ignore
            '/ethercat_slaves_status_check/check', callback_group=self.service_group)

        self.ethercat_check_state = EthercatCheckState.IDLE
        self.ethercat_check_requested = True
        self.start_environment_requested = True

        self.fsm : StateMachine = StateMachine(State.IDLE, self.get_logger()) #type: ignore
        self.fsm.add_transition(Event.SWITCH_MODE,
                        State.IDLE, 
                        State.IDLE_RECOVERY, 
                        msg="🔑 Change mode (IDLE=>IDLE_RECOVERY)",
        )
        self.fsm.add_transition(Event.SWITCH_MODE,
                        State.RECOVERED, 
                        State.IDLE, 
                        msg="🔑 Change mode (RECOVERED=>IDLE)",
        )
        self.fsm.add_transition(Event.START,
                                State.IDLE, 
                                State.RUNNING, 
                                msg="🔑 KEY TURN DETECTED: E-stop 0=>1 (IDLE=>RUNNING)",
                                guard=self._ready_to_start, 
                                action=self._bringup_env,
                                success_check=check_env_running,
                                max_steps=100,
                                failure_destination=State.ERROR,
        )
        self.fsm.add_transition(Event.START,
                                State.IDLE_RECOVERY, 
                                State.RUNNING_RECOVERY, 
                                msg="🔑 KEY TURN DETECTED: E-stop 0=>1 (IDLE_RECOVERY=>RUNNING_RECOVERY)",
                                guard=self._ready_to_start, 
                                action=self._bringup_recovery_env,
                                success_check=check_env_running_recovery,
                                max_steps=100,
                                failure_destination=State.ERROR)
        
        self.fsm.add_transition(Event.STOP,
                        State.RUNNING_RECOVERY,
                        State.RECOVERED,
                        msg="🛑 EMERGENCY STOP REQUESTED (RUNNING_RECOVERY=>RECOVERED)",
                        action=self._kill_recovery_env,
                        success_check=check_env_running_recovery_stopped,
                        max_steps=100,
                        failure_destination=State.ERROR)

        self.fsm.add_transition(Event.STOP,
                State.RUNNING, 
                State.IDLE, 
                msg="🛑 EMERGENCY STOP REQUESTED (RUNNING=>IDLE)",
                action=self._kill_env,
                success_check=check_env_running_stopped,
                max_steps=100,
                failure_destination=State.ERROR)

        self.fsm.add_transition(Event.FAIL,
                State.RUNNING, 
                State.ERROR, 
                msg="� SYSTEM FAILURE (RUNNING=>ERROR)",
                action=self._kill_env,
                success_check=check_env_running_stopped,
                max_steps=100,
                failure_destination=State.ERROR)
        
        self.fsm.add_transition(Event.FAIL,
                State.RUNNING_RECOVERY, 
                State.ERROR, 
                msg=" SYSTEM FAILURE (RUNNING_RECOVERY=>ERROR)",
                action=self._kill_recovery_env,
                success_check=check_env_running_recovery_stopped,
                max_steps=100,
                failure_destination=State.ERROR)

        self.fsm.add_transition(Event.STOP,
                State.IDLE,
                State.IDLE,
                msg="🛑 EMERGENCY STOP REQUESTED (IDLE=>IDLE)",
                action=self._kill_env,
                success_check=check_env_running_stopped,
                max_steps=100,
                failure_destination=State.ERROR)
        
        self.fsm.add_transition(Event.STOP,
                State.IDLE_RECOVERY,
                State.IDLE_RECOVERY,
                msg="🛑 EMERGENCY STOP REQUESTED (IDLE_RECOVERY=>IDLE_RECOVERY)",
                action=self._kill_recovery_env,
                success_check=check_env_running_recovery_stopped,
                max_steps=100,
                failure_destination=State.ERROR)
    
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
        self.get_logger().info("Cleanup: Stopping GUI ROS communication.") #type: ignore
        self._notify_gui(b"STOP")

        self.get_logger().info("Cleanup: Cancelling timers and destroying node.") #type: ignore
        self.client.close()
        self.destroy_node()
    
    def _ros_gui_disconnected(self) -> bool:
        """Handle ROS GUI disconnection event.
        """
        msg = self.client.last_received_message
        if msg == b"ROS_DISCONNECTED":
            return True
        
        return False

    def _ros_gui_connection_failed(self) -> bool:
        return self.client.last_received_message == b"ROS_CONNECTION_FAILED"
    
    def _ethercat_slaves_status_ok(self) -> bool:
        return self.ethercat_check_state == EthercatCheckState.READY

    def _ready_to_start(self) -> bool:
        return self._ros_gui_disconnected() and self._ethercat_slaves_status_ok()
    
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

    def state_callback(self, msg: PlcStates) -> None:
        # ========== Acquire lock to prevent concurrent execution ==========
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
                check_env_running() if self.fsm.state == State.RUNNING else\
                check_env_running_recovery() if self.fsm.state == State.RUNNING_RECOVERY else\
                check_env_running_stopped() if self.fsm.state == State.IDLE else\
                check_env_running_recovery_stopped() \
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
            _msg : str = self.fsm.state.name
            self._notify_gui(_msg.encode())
            self.lock.release()

    def _kill_recovery_env(self) -> None:
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

    def _notify_gui(self, udp_msg: bytes) -> None:
        try:
            self.client.send(udp_msg)
        except Exception as e:
            self.get_logger().warn(f"Error communicating with GUI: {e}") #type: ignore


    def publish_command(self, name: str, value: int) -> None:
        """Publish single PLC command by interface name.
        
        Helper function to update and publish a specific PLC command value.
        Updates the command_msg, finds the interface by name, updates its value,
        and publishes the entire 8-element command vector.
        
        Example:
            publish_command('PLC_node/estop', 1)        # Release E-stop
            publish_command('PLC_node/force_sensors_pwr', 1)  # Enable F/T power
        
        Args:
            name (str): Interface identifier (must match one in command_msg.interface_names)
            value (int): Command value (typically 0 or 1)
        
        Returns:
            None. Publishes PlcController message if name found.
        
        Log Output:
            - INFO: "Published command message: [0, 0, 0, 0, 1, 0, 0, 0]"
            - WARN: "Interface name '{name}' not found in command message" if not found
        """
        if name in self.command_msg.interface_names: #type: ignore
            # Find index of interface by name
            idx = self.command_msg.interface_names.index(name) #type: ignore
            # Update value
            self.command_msg.values[idx] = value #type: ignore
            # Publish entire command vector
            self.command_publisher.publish(self.command_msg)
            self.get_logger().info(f"Published PLC command: {self.command_msg.values}") #type: ignore
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
        1. Check if EtherCAT PLC slave active (check_ethercat_plc_node)
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
    # Thread 2: Launcher health check timer (check_ros_launch_status)
    mt_executor = MultiThreadedExecutor(num_threads=2)
    mt_executor.add_node(node)

    # ========== Set CPU affinity to core 2 ==========
    # Ensures deterministic scheduling for control tasks
    import os
    os.sched_setaffinity(0, {7})
    
    # ========== Bringup flag (execute once at startup) ==========
    bringup_done = False
    
    try:
        while rclpy.ok():
            mt_executor.spin_once(timeout_sec=0.1)

            if node.ethercat_check_requested:

                if not node.ethercat_slaves_status_check_client.wait_for_service(timeout_sec=5.0):
                    node.get_logger().error( #type: ignore
                        "EtherCAT status check service not available. "
                        "Ensure ethercat_slaves_status_check is running."
                    )
                    continue
                    
                future = node.ethercat_slaves_status_check_client.call_async(Trigger.Request()) #type: ignore
                node.ethercat_check_state = EthercatCheckState.PENDING

                mt_executor.spin_until_future_complete(
                    future, #type: ignore
                    timeout_sec=5.0,
                )

                if future.done() and future.result() is not None: #type: ignore
                    node.ethercat_check_state = (
                        EthercatCheckState.READY
                        if future.result().success #type: ignore
                        else EthercatCheckState.FAILED
                    )
                else:
                    node.ethercat_check_state = EthercatCheckState.FAILED

                node.ethercat_check_requested = not (node.ethercat_check_state == EthercatCheckState.READY)

            # ========== Initial bringup sequence (once only) ==========
            if not bringup_done \
            and node.ethercat_check_state == EthercatCheckState.READY \
            and node.command_publisher.get_subscription_count() > 0:
                node.publish_bringup_commands()
                bringup_done = True
                
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
