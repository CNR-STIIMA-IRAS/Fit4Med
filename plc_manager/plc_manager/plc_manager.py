# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
 
"""PLC state machine manager for safety-certified rehabilitation platform control.

This module implements a critical state machine that coordinates the entire rehabilitation
system lifecycle through a safety-certified PLC connected to the E-stop chain. The node
manages transitions between two operational states:

IDLE State:
    - PLC waits for enable signal (reset key turn that enables software reset)
    - Safety sensors monitored; system inactive
    - Robot motion stack and its controllers not running
    
RUNNING State:
    - Control software (run_platform_control.launch.py) executing
    - All the default controllers are loaded
    - Real-time motion tracking and safety monitoring operational

State Transition Logic:
    KEY TURN (estop 0→1):
        [IDLE] → (check_ethercat_plc_node && subscribers active)
               → launch_ros2_env.sh (with the argument [--perform-homing] if is the first time)
               → [RUNNING]
    E-STOP PRESS (estop 1→0):
        [RUNNING] → send STOP to rehab_gui (wait 10 secs for STOPPED ack)
                  → kill run_platform_control.launch.py
                  → emergency stop PLC (E-stop 0→1)
                  → [IDLE]
    LAUNCHER CRASH:
        [RUNNING] → detect launcher not found in pgrep
                  → log warning
                  → set E-stop low to acknowledge crash
                  → resume monitoring for next key turn

Control Flow Detail:
    
    1. PLC Hardware Event (key turn: estop 0→1)
    2. state_callback() receives PlcStates message from plc_controller
    3. Transition detected: estop changed from 0 to 1
    4. check_ethercat_plc_node(): verify EtherCAT PLC slave is reported as OP
    5. If first time: launch with --perform-homing flag (motor calibration)
    6. If not first time: launch without homing (faster startup)
    7. Launch script runs: ros2 launch tecnobody_workbench run_platform_control.launch.py
    8. Controllers loaded: joint_trajectory_controller, F/T broadcaster, etc.
    9. Motion stack ready to receive exercise commands from rehab_gui
    
    When E-Stop pressed (estop 1→0):
    1. state_callback() detects transition
    2. Send UDP "STOP" message to rehab_gui port 5005
    3. Wait for "STOPPED" acknowledgment (10s timeout)
    4. Kill launcher process: ros2 launch ... run_platform_control.launch.py
    5. Publish emergency E-stop command to PLC (E-stop 0→1)
    6. Return to IDLE state

UDP Communication (GUI Status):
    - Sends b"STOP" when E-stop pressed (triggers GUI emergency halt)
    - Sends b"RUNNING" periodically (50 Hz / 10 = 5 Hz) to indicate system operational
    - Receives b"STOPPED" acknowledgment from GUI (confirms safe shutdown)
    - Used to coordinate graceful system-wide shutdown

Command Message Structure (PlcController):
    Interface Names (8 outputs to PLC):
        0: PLC_node/mode_of_operation    (unused, reserved)
        1: PLC_node/power_cutoff         (unused, reserved)
        2: PLC_node/sonar_teach          (ultrasonic sensor calibration, see sonar_teach.py)
        3: PLC_node/s_output.4           (unused, reserved)
        4: PLC_node/estop                (main emergency stop control)
        5: PLC_node/manual_mode          (unused, reserved)
        6: PLC_node/force_sensors_pwr    (enable/disable F/T sensor power)
        7: PLC_node/s_output.8           (unused, reserved)
    
    Values: [0, 0, 0, 0, 0, 0, 0, 0] initially, updated by publish_command()

EtherCAT Health Check:
    Queries ethercat_msgs/srv/GetSlaveStates from the ros2_control hardware component
    on demand from check_ethercat_plc_node(), with an internal rate limit.
    Expected PLC state: "sickPLC ... OP"
    If not found: system not ready to launch control stack

Performance:
    - State callback: <1 ms (non-blocking due to threading.Lock)
    - Launcher status check: 1 Hz (1 second timer)
    - UDP heartbeat: 50 Hz (every 20ms during RUNNING state)
    - Bringup latency: ~2s (from key turn to controllers ready)

Thread Safety:
    - state_callback() and publish_command() protected by threading.Lock
    - Prevents race conditions between state updates and command publishing
    - Non-blocking acquire fails gracefully if callback already executing

Attributes:
    plc_group (MutuallyExclusiveCallbackGroup): Serializes PLC state updates
    timer_group (MutuallyExclusiveCallbackGroup): Isolates launcher check timer
    service_group (MutuallyExclusiveCallbackGroup): EtherCAT state service client callbacks
    state_subscriber (Subscription): Receives PlcStates at max rate from plc_controller
    command_publisher (Publisher): Publishes PlcController commands with RELIABLE QoS
    interface_names (List[str]): 8 interface names for PLC command mapping
    state_values (List[int]): Current 8 input values from PLC state
    command_values (List[int]): 8 output command values to PLC
    ESTOP (int): Cached E-stop state (0=emergency, 1=normal operation)
    lock (threading.Lock): Protects state_callback() concurrent access
    client (UdpClient): UDP socket for GUI coordination
    ros_launched (bool): Flag indicating run_platform_control.launch.py active
    check_ros_status_timer (Timer): 1 Hz launcher health check
    FIRST_TIME (bool): Flag to apply homing on first launch
    shutdown_requested (bool): Flag for graceful node shutdown
"""

import time
import os

# Must be BEFORE importing rclpy (sets logging format globally)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

SKIP_ETHERCAT_CHECK = os.environ.get("PLC_MANAGER_SKIP_ETHERCAT", "0") == "1"

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ethercat_msgs.srv import GetSlaveStates
from tecnobody_msgs.msg import PlcController, PlcStates
import sys
import subprocess
import threading
import signal
import socket


class UdpClient:
    """Simple UDP client for status messaging to remote GUI.
    
    Implements bidirectional UDP communication for coordinating system-wide
    emergency stop and status reporting. Used to signal rehab_gui when E-stop
    is pressed and to receive acknowledgments of safe shutdown.
    
    Attributes:
        target_ip (str): IP address of GUI server (typically 127.0.0.1 or 10.2.16.43)
        target_port (int): UDP port of GUI server (5005)
        sock (socket): UDP socket for sending and receiving
        timeout (float): Receive timeout in seconds (default: 2.0)
    """

    def __init__(
        self,
        target_ip: str = "127.0.0.1",
        target_port: int = 5005,
        local_port: int = 0,
        timeout: float = 2.0
    ):
        """Initialize UDP client with target server address.
        
        Args:
            target_ip (str): IP address of GUI server
            target_port (int): UDP port of GUI server (default: 5005)
            local_port (int): Local port to bind (0 = auto-assign)
            timeout (float): Receive timeout in seconds (default: 2.0)
        """
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", local_port))
        self.sock.settimeout(timeout)

    def send(self, message: bytes) -> None:
        """Send UDP message to target server.
        
        Args:
            message (bytes): Message to send (e.g., b"STOP", b"RUNNING")
        """
        self.sock.sendto(message, (self.target_ip, self.target_port))

    def receive(self, bufsize: int = 4096) -> tuple[bytes, tuple]:
        """Receive UDP message from server (blocking until timeout).
        
        Args:
            bufsize (int): Maximum message size to receive (default: 4096)
        
        Returns:
            tuple: (data, sender_address) or (None, None) on timeout
        """
        try:
            data, addr = self.sock.recvfrom(bufsize)
            print(f"[UdpClient] Received message from {addr}: {data}")
            return data, addr
        except socket.timeout:
            return None, None

    def close(self) -> None:
        """Close UDP socket."""
        self.sock.close()


class bcolors:
    """ANSI color codes for terminal output.
    
    Provides colored logging for state transitions and critical events.
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class PLCControllerInterface(Node):
    """State machine node for PLC-coordinated system startup/shutdown.
    
    This node manages the complete lifecycle of the rehabilitation control system
    by monitoring E-stop state changes and orchestrating startup/shutdown of the
    motion control stack (run_platform_control.launch.py).
    
    The node implements a two-state machine:
    
    IDLE (E-stop=0):
        - No control software running
        - Safety sensors monitored by PLC only
        - Waiting for key turn or reset signal
    
    RUNNING (E-stop=1):
        - Motion control launcher executing
        - joint_trajectory_controller, F/T sensors active
        - System ready for exercise execution
    
    State transitions are triggered by E-stop signal changes detected via
    PlcStates subscription. The node ensures graceful transitions by:
    1. Waiting for GUI acknowledgment before shutdown
    2. Verifying EtherCAT slave readiness before startup
    3. Applying homing calibration on first launch
    4. Monitoring launcher health with 1 Hz status checks
    
    Attributes:
        plc_group (MutuallyExclusiveCallbackGroup): PLC state subscription callback group
        timer_group (MutuallyExclusiveCallbackGroup): Launcher check timer callback group
        service_group (MutuallyExclusiveCallbackGroup): EtherCAT state service client callbacks
        state_subscriber (Subscription): PlcStates from plc_controller
        command_publisher (Publisher): PlcController commands to plc_controller
        interface_names (List[str]): 8 PLC output interface identifiers
        state_values (List[int]): Current 8 input values from PLC
        command_values (List[int]): 8 output values to send to PLC
        ESTOP (int): Cached E-stop state for transition detection
        lock (threading.Lock): Protects concurrent access to state_callback()
        client (UdpClient): UDP client for GUI coordination (STOP/RUNNING)
        ros_launched (bool): True if run_platform_control.launch.py is running
        check_ros_status_timer (Timer): 1 Hz timer for launcher health check
        FIRST_TIME (bool): Flag to apply homing on initial startup
        shutdown_requested (bool): Flag for graceful node shutdown
    """

    def __init__(self, target_ip: str):
        """Initialize PLC controller interface node.
        
        Creates ROS 2 node infrastructure:
        - Subscription to PLC states (estop, inputs, etc.)
        - Publisher for PLC commands (estop, power, sonar_teach, etc.)
        - Timer for launcher health checks (1 Hz)
        - UDP client for GUI coordination
        
        Args:
            target_ip (str): IP address of rehab_gui server for UDP status messages
                (typically "127.0.0.1" for local testing or "10.2.16.43" for network)
        
        Side Effects:
            - Registers SIGINT and SIGTERM handlers
            - Sets CPU affinity to core 2 (deterministic scheduling)
            - Initializes thread lock for state protection
        """
        super().__init__('plc_manager')

        # ========== Callback Groups ==========
        # Three separate groups prevent callback blocking:
        self.plc_group = MutuallyExclusiveCallbackGroup()       # PLC state updates
        self.timer_group = MutuallyExclusiveCallbackGroup()     # Launcher health check
        self.service_group = MutuallyExclusiveCallbackGroup()   # Reserved for services

        # ========== PLC State Subscription ==========
        # Receives PlcStates from plc_controller at max rate
        # Includes estop, all inputs, and interface names
        self.state_subscriber_callback_running = False
        self.state_subscriber = self.create_subscription(
            PlcStates,
            '/PLC_controller/plc_states',
            self.state_callback,
            10,
            callback_group=self.plc_group
        )

        # ========== PLC Command Publisher ==========
        # RELIABLE QoS: guarantees all commands reach PLC even if temporarily unavailable
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.command_publisher = self.create_publisher(
            PlcController,
            '/PLC_controller/plc_commands',
            qos,
        )

        # ========== PLC State Variables ==========
        self.interface_names = []
        self.state_values = []
        self.command_values = []
        self.launch_status = []
        self.ESTOP = 0                          # Cached E-stop value for transition detection
        self.estop_initialized = False          # First PLC sample initializes ESTOP cache
        self.lock = threading.Lock()            # Protects state_callback() concurrent access
        self.command_lock = threading.Lock()    # Protects command_msg concurrent access
        self.shutdown_sequence_lock = threading.Lock()
        self.shutdown_worker = None
        self.platform_launch_process = None
        
        # ========== Command Message Structure ==========
        # Pre-allocate PlcController message with 10 outputs (matches plc_controller.yaml)
        self.command_msg = PlcController()
        self.command_msg.values = [0] * 10
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
        self.ros_launched = False
        self.ros_launched_prev = False
        self.check_ros_status_timer = self.create_timer(
            1.0,
            self.check_ros_launch_status,
            self.timer_group
        )
        self.FIRST_TIME = False

        # ========== UDP Status Client ==========
        # Coordinates with rehab_gui for emergency stop and status
        self.client = UdpClient(target_ip, 5005)
        self.send_running_cnt = 0
        self.send_running_dec = 10  # Send status every 10*50ms = 500ms at 50 Hz
        self.shutdown_requested = False

        # ========== EtherCAT State Service Client ==========
        self.declare_parameter('ethercat_slave_state_services', ['/tecnobody/get_slave_states_master0'])
        self.declare_parameter('ethercat_slave_names', ['delta1-0:0', 'delta2-0:1', 'delta3-0:2', 'AtiAxia90-0:3'])
        self.ethercat_slave_state_services : list[str] = list(
            self.get_parameter('ethercat_slave_state_services').value
        )
        self.ethercat_slave_names : list[str] = list(
            self.get_parameter('ethercat_slave_names').value
        )
        self.ethercat_state_clients : dict[str, rclpy.client.Client] = {}
        self.ethercat_state_inflight = {}
        self.ethercat_state_lock = threading.Lock()
        self.latest_ethercat_states = {}

        # ========== Signal Handlers ==========
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)


    def get_ethercat_state_service_names(self) -> list[str]:
        """Return configured GetSlaveStates service names."""
        service_names : list[str] = list(set(self.ethercat_slave_state_services))
        return service_names


    def get_ethercat_state_client(self, service_name: str):
        """Create or reuse a GetSlaveStates client for the given service name."""
        if service_name not in self.ethercat_state_clients:
            self.ethercat_state_clients[service_name] = self.create_client(
                GetSlaveStates,
                service_name,
                callback_group=self.service_group
            )
        return self.ethercat_state_clients[service_name]


    def poll_ethercat_slave_states(self) -> None:
        """Asynchronously refresh cached EtherCAT slave states, rate-limited."""
        if SKIP_ETHERCAT_CHECK:
            return

        service_names = self.get_ethercat_state_service_names()
        if not service_names:
            self.get_logger().warn(
                "No EtherCAT GetSlaveStates services configured.",
                throttle_duration_sec=5.0
            )
            return

        for service_name in service_names:
            future = self.ethercat_state_inflight.get(service_name)
            if future is not None and not future.done():
                continue

            client = self.get_ethercat_state_client(service_name)
            if not client.service_is_ready():
                continue

            future = client.call_async(GetSlaveStates.Request())
            self.ethercat_state_inflight[service_name] = future
            future.add_done_callback(
                lambda done_future, service_name=service_name:
                self.handle_ethercat_slave_states_response(service_name, done_future)
            )


    def handle_ethercat_slave_states_response(self, service_name: str, future) -> None:
        """Store one GetSlaveStates response in a cache readable by the state machine."""
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to query EtherCAT slave states from {service_name}: {exc}",
                throttle_duration_sec=5.0
            )
            return

        states = dict(zip(response.slave_names, response.slave_states))
        with self.ethercat_state_lock:
            self.latest_ethercat_states[service_name] = states


    def check_ethercat_plc_node(self) -> bool:
        """Verify EtherCAT PLC slave is active and operational.
        
        Uses the GetSlaveStates service exposed by the EtherCAT hardware component
        instead of invoking the ethercat CLI. Service requests are triggered from
        this check only and internally rate-limited; this method inspects the
        latest cached response.
        
        Returns:
            bool: True if PLC slave found in OP state, False otherwise
        
        Log Output:
            - INFO: "The PLC ethercat slave is active (=> sickPLC OP)"
            - Returns False if slave not found or not in OP state
        """
        if SKIP_ETHERCAT_CHECK:
            self.get_logger().info(
                "Skipping EtherCAT health check (PLC_MANAGER_SKIP_ETHERCAT=1)."
            )
            return True

        self.poll_ethercat_slave_states()

        matches = []
        with self.ethercat_state_lock:
            cached_states = dict(self.latest_ethercat_states)

        for service_name, states in cached_states.items():
            for slave_name, slave_state in states.items():
            if any(plc_name in slave_name for plc_name in self.ethercat_slave_names):
                    matches.append((service_name, slave_name, slave_state))
                    if slave_state == 'OP':
                        self.get_logger().info(
                            f'The PLC ethercat slave is active '
                            f'(=> {slave_name} {slave_state}, service {service_name})',
                            throttle_duration_sec=5.0
                        )
                        return True

        if matches:
            states_text = ', '.join(
                f'{slave_name}={slave_state} via {service_name}'
                for service_name, slave_name, slave_state in matches
            )
            self.get_logger().warn(
                f'EtherCAT PLC slave found but not operational: {states_text}',
                throttle_duration_sec=5.0
            )
        else:
            services_text = ', '.join(self.get_ethercat_state_service_names())
            self.get_logger().warn(
                f'Waiting for EtherCAT PLC state from GetSlaveStates service. '
                f'Configured services: {services_text or "none"}',
                throttle_duration_sec=5.0
            )
        return False


    def publish_bringup_commands(self) -> None:
        """Publish initial startup commands to enable PLC outputs.
        
        Called once during system bringup to:
        1. Release E-stop (set to 1 = normal operation)
        2. Enable force/torque sensor power
        
        These commands prepare the PLC for motion control.
        
        Returns:
            None. Publishes commands via self.publish_command()
        """
        self.publish_command('PLC_node/estop', 1)
        self.publish_command('PLC_node/force_sensors_pwr', 1)


    def cleanup(self) -> None:
        """Perform graceful cleanup and resource deallocation.
        
        Called before node destruction to:
        1. Cancel launcher health check timer
        2. Close UDP socket
        3. Destroy node resources
        
        Returns:
            None. Leaves node in destroyed state.
        """
        self.get_logger().info("Cleanup: Cancelling timers and destroying node.")
        if not self.check_ros_status_timer.is_canceled():
            self.check_ros_status_timer.cancel()
        self.client.close()
        self.destroy_node()


    def launch_control_stack(self) -> None:
        """Launch the ROS 2 control stack script in its own process group."""
        launch_script = "/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh"
        self.platform_launch_process = subprocess.Popen(
            launch_script,
            shell=True,
            executable="/bin/bash",
            start_new_session=True
        )


    def find_platform_launcher_pids(self) -> list[int]:
        """Return all PIDs matching the platform-control ros2 launch command."""
        try:
            output = subprocess.check_output(
                ["pgrep", "-f", "ros2 launch tecnobody_workbench run_platform_control.launch.py"],
                text=True
            )
        except subprocess.CalledProcessError:
            return []

        pids: list[int] = []
        for pid_text in output.splitlines():
            pid_text = pid_text.strip()
            if not pid_text:
                continue
            try:
                pids.append(int(pid_text))
            except ValueError:
                self.get_logger().warn(f"Ignoring unexpected PID from pgrep: {pid_text!r}")
        return pids


    def is_platform_launcher_running(self) -> bool:
        """Return True if the platform-control launcher appears to be running."""
        if self.platform_launch_process is not None and self.platform_launch_process.poll() is None:
            return True
        return bool(self.find_platform_launcher_pids())


    def terminate_platform_launcher(self) -> None:
        """Terminate any running platform-control launcher processes."""
        killed_any = False

        # Prefer terminating the process group that this node created. This also
        # catches child processes started by launch_ros2_env.sh if they stayed in
        # the same session/process group.
        proc = self.platform_launch_process
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                killed_any = True
            except ProcessLookupError:
                pass
            except Exception as exc:
                self.get_logger().warn(f"Could not terminate stored launcher process group: {exc}")

        # Fallback: pgrep may return more than one matching PID. Kill all of them.
        pids = self.find_platform_launcher_pids()
        for pid in pids:
            try:
                os.kill(pid, signal.SIGINT)
                killed_any = True
            except ProcessLookupError:
                pass
            except Exception as exc:
                self.get_logger().warn(f"Could not terminate launcher PID {pid}: {exc}")

        if killed_any:
            time.sleep(2)
            self.get_logger().info("✅ Launcher process terminated")
        else:
            self.get_logger().info("No platform_control.launch.py process found.")

        self.platform_launch_process = None
        self.ros_launched = False
        self.ros_launched_prev = False


    def wait_for_gui_stop_ack(self, timeout_sec: float = 10.0) -> bool:
        """Wait up to timeout_sec for rehab_gui to acknowledge STOP with STOPPED."""
        deadline = time.monotonic() + timeout_sec
        old_timeout = self.client.sock.gettimeout()

        try:
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False

                # Keep each recv short so the overall timeout is close to timeout_sec.
                self.client.sock.settimeout(min(0.2, remaining))
                data, _addr = self.client.receive()
                if data == b"STOPPED":
                    return True
        finally:
            self.client.sock.settimeout(old_timeout)


    def handle_estop_shutdown_sequence(self) -> None:
        """Run the long E-stop shutdown sequence outside state_callback()."""
        try:
            self.publish_command('PLC_node/brake_disable', 0)  # Ensure brakes enabled

            self.get_logger().info("📡 Sending STOP signal to rehab_gui...")
            self.client.send(b"STOP")

            if self.wait_for_gui_stop_ack(timeout_sec=10.0):
                self.get_logger().info("✅ GUI shutdown acknowledged")
            else:
                self.get_logger().warn(
                    "PLC Manager: No acknowledgment from FMRR GUI for STOP command.",
                    throttle_duration_sec=5.0
                )
        except Exception as exc:
            self.get_logger().warn(f"Error communicating with GUI: {exc}")

        self.get_logger().info("🔪 Killing run_platform_control.launch.py...")
        self.terminate_platform_launcher()


    def start_estop_shutdown_sequence(self) -> None:
        """Start one background E-stop shutdown worker if one is not already running."""
        with self.shutdown_sequence_lock:
            if self.shutdown_worker is not None and self.shutdown_worker.is_alive():
                self.get_logger().warn("E-stop shutdown sequence already running; ignoring duplicate request.")
                return

            self.shutdown_worker = threading.Thread(
                target=self.handle_estop_shutdown_sequence,
                name="plc_manager_estop_shutdown",
                daemon=True
            )
            self.shutdown_worker.start()


    def _handle_launcher_missing(self) -> bool:
        """Handle the platform launcher being absent, including crash transition logic."""
        self.ros_launched = False

        if self.ros_launched_prev:
            self.get_logger().warn(
                'Platform launcher disappeared unexpectedly. Calling SW ESTOP!',
                throttle_duration_sec=5.0
            )
            try:
                self.publish_command('PLC_node/estop', 0)           # Emergency stop
                self.publish_command('PLC_node/manual_mode', 0)     # Return to automatic
                time.sleep(0.05)
                self.publish_command('PLC_node/estop', 1)           # Re-enable normal operation
            except Exception as exc:
                self.get_logger().error(f"Failed to publish SW ESTOP after launcher crash: {exc}")

        self.ros_launched_prev = False
        return False


    def check_ros_launch_status(self) -> bool:
        """Monitor launcher health and manage control stack lifecycle (1 Hz timer)."""
        if self.shutdown_requested:
            return False

        launcher_name = "run_platform_control.launch.py"

        try:
            output = subprocess.check_output(
                ["pgrep", "-af", "ros2 launch"],
                text=True
            )
        except subprocess.CalledProcessError:
            # pgrep returns nonzero if no ros2 launch processes exist at all.
            self.get_logger().info(
                '[1;35mWaiting for ros controllers to start![0m',
                throttle_duration_sec=5.0
            )
            return self._handle_launcher_missing()

        for line in output.splitlines():
            if launcher_name in line:
                if not self.ros_launched:
                    self.get_logger().info(
                        '✅ Ros control launcher detected!',
                        throttle_duration_sec=20.0
                    )
                self.ros_launched = True
                self.ros_launched_prev = True
                return True

        self.get_logger().info(
            '[1;35mWaiting for ros controllers to start![0m',
            throttle_duration_sec=5.0
        )
        return self._handle_launcher_missing()


    def state_callback(self, msg: PlcStates) -> None:
        """Process PLC state updates and implement state machine logic."""
        if not self.lock.acquire(blocking=False):
            self.get_logger().warn("state_callback already in execution, ignore.")
            return

        try:
            self.interface_names = list(msg.interface_names)
            self.state_values = list(msg.values)

            for int_idx, interface_name in enumerate(self.interface_names):
                if interface_name != "estop":
                    continue

                current_estop = self.state_values[int_idx]

                # Initialize from the first observed PLC value so a node restart while
                # estop is already high does not look like a fresh 0→1 key turn.
                if not self.estop_initialized:
                    self.ESTOP = current_estop
                    self.estop_initialized = True
                    self.get_logger().info(
                        f"Initialized cached E-stop state from PLC: {self.ESTOP}"
                    )
                    return

                # ========== TRANSITION: 0→1 (KEY TURN: IDLE → RUNNING) ==========
                if current_estop == 1 and self.ESTOP == 0:
                    self.get_logger().info(
                        bcolors.OKBLUE +
                        "🔑 KEY TURN DETECTED: E-stop 0→1 (IDLE→RUNNING)" +
                        bcolors.ENDC
                    )

                    if not self.check_ethercat_plc_node():
                        self.get_logger().warn("EtherCAT PLC not ready, deferring launch")
                        break

                    if self.is_platform_launcher_running() or self.ros_launched:
                        self.get_logger().warn(
                            "Platform launcher already running; not launching a duplicate."
                        )
                        self.ros_launched = True
                        self.ros_launched_prev = True
                    elif not self.FIRST_TIME:
                        self.get_logger().info("🏠 First startup: launching without homing")
                        self.launch_control_stack()
                        self.FIRST_TIME = True
                    else:
                        self.get_logger().info("⚡ Restart: launching without homing")
                        self.launch_control_stack()

                # ========== TRANSITION: 1→0 (E-STOP PRESS: RUNNING → IDLE) ==========
                elif current_estop == 0 and self.ESTOP == 1:
                    self.get_logger().info(
                        bcolors.OKCYAN +
                        "🛑 E-STOP PRESSED: E-stop 1→0 (RUNNING→IDLE)" +
                        bcolors.ENDC
                    )
                    self.start_estop_shutdown_sequence()

                # ========== STABLE RUNNING STATE: Send periodic status ==========
                elif current_estop == 1 and self.ESTOP == 1:
                    try:
                        node_list = self.get_node_names()
                        if 'tecnobody_ethercat_checker_node' in node_list:
                            self.send_running_cnt += 1
                            if self.send_running_cnt % self.send_running_dec == 0:
                                self.client.send(b"RUNNING")
                    except Exception as exc:
                        self.get_logger().error(
                            f"Exception when sending message to UDP Server: {exc}"
                        )

                # current_estop == 0 and self.ESTOP == 0: stable IDLE, no action.
                self.ESTOP = current_estop
                break

        finally:
            self.lock.release()


    def publish_command(self, name: str, value: int) -> None:
        """Publish single PLC command by interface name in a thread-safe way."""
        with self.command_lock:
            if name not in self.command_msg.interface_names:
                self.get_logger().warn(
                    f"Interface name '{name}' not found in command message."
                )
                return

            idx = self.command_msg.interface_names.index(name)
            self.command_msg.values[idx] = value

            # Publish a copy so another thread cannot mutate the same message object
            # while the publisher is using it.
            msg = PlcController()
            msg.interface_names = list(self.command_msg.interface_names)
            msg.values = list(self.command_msg.values)
            self.command_publisher.publish(msg)
            self.get_logger().info(f"Published PLC command: {msg.values}")


    def signal_handler(self, sig, frame) -> None:
        """Handle SIGINT and SIGTERM for graceful shutdown.
        
        Called when Ctrl+C pressed or SIGTERM received. Sets shutdown_requested
        flag for main() loop to detect and break gracefully.
        
        Args:
            sig: Signal number (SIGINT or SIGTERM)
            frame: Current stack frame
        
        Returns:
            None. Sets shutdown_requested flag.
        """
        self.get_logger().info("Signal received. Setting shutdown_requested flag.")
        self.shutdown_requested = True


def main(args=None):
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
            If not provided: defaults to "127.0.0.1" (localhost)
    
    Returns:
        None. Blocks until shutdown.
    """
    # Initialize ROS 2 without automatic signal handling
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    
    # ========== Parse arguments ==========
    target_ip = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

    # ========== Create node instance ==========
    node = PLCControllerInterface(target_ip)
    
    # ========== Create multi-threaded executor ==========
    # Thread 1: PLC state subscription callback (state_callback)
    # Thread 2: Launcher health check timer (check_ros_launch_status)
    mt_executor = MultiThreadedExecutor(num_threads=2)
    mt_executor.add_node(node)

    # ========== Set CPU affinity ==========
    # Best-effort only: this can fail on systems with fewer CPUs, unsupported OSes,
    # or container/cgroup restrictions.
    try:
        os.sched_setaffinity(0, {7})
    except Exception as exc:
        node.get_logger().warn(f"Could not set CPU affinity: {exc}")
    
    # ========== Bringup flag (execute once at startup) ==========
    bringup_done = False
    
    try:
        while rclpy.ok():
            mt_executor.spin_once()

            # ========== Initial bringup sequence (once only) ==========
            if not bringup_done and node.check_ethercat_plc_node() and \
                    node.command_publisher.get_subscription_count() > 0:
                node.publish_bringup_commands()
                bringup_done = True
                
            # ========== Check for graceful shutdown request ==========
            if node.shutdown_requested:
                node.get_logger().warning('Received graceful shutdown request!')
                break

    except Exception as e:
        node.get_logger().error(f"Unexpected exception in main loop: {e}")
    
    # ========== Graceful cleanup ==========
    try:
        node.cleanup()
        rclpy.try_shutdown()
    except Exception as e:
        print(f"Error during shutdown: {e}")


if __name__ == '__main__':
    main()
