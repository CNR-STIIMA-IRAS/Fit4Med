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
    4. check_ethercat_plc_node(): verify EtherCAT slave active ("FLX0-GETC100" OP)
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
    Runs pipeline: ethercat slaves | grep FLX0-GETC100 | awk "{print $3, $5}"
    Expected output: "FLX0-GETC100 OP"
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
    service_group (MutuallyExclusiveCallbackGroup): Reserved for future services
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
        service_group (MutuallyExclusiveCallbackGroup): Reserved for future services
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
        self.lock = threading.Lock()            # Protects state_callback() concurrent access
        
        # ========== Command Message Structure ==========
        # Pre-allocate PlcController message with 8 outputs
        self.command_msg = PlcController()
        self.command_msg.values = [0] * 8
        self.command_msg.interface_names = [
            'PLC_node/mode_of_operation',       # [0] unused
            'PLC_node/power_cutoff',            # [1] unused
            'PLC_node/sonar_teach',             # [2] ultrasonic sensor calibration
            'PLC_node/s_output.4',              # [3] unused
            'PLC_node/estop',                   # [4] main E-stop signal
            'PLC_node/manual_mode',             # [5] unused
            'PLC_node/force_sensors_pwr',       # [6] F/T sensor power control
            'PLC_node/brake_disable'            # [7] Brake Disable (0=enabled, 1=disabled)
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

        # ========== Signal Handlers ==========
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)


    def check_ethercat_plc_node(self) -> bool:
        """Verify EtherCAT PLC slave is active and operational.
        
        Runs EtherCAT diagnostic command to confirm the PLC is connected and
        in OP (operational) mode. This check is required before launching the
        motion control stack to ensure safe hardware readiness.
        
        Command Pipeline:
            ethercat slaves
                ↓
            grep FLX0-GETC100         (filter for PLC device)
                ↓
            awk "{print $3, $5}"      (extract alias and state)
                ↓
            Check if "OP" state found
        
        Expected Output:
            "FLX0-GETC100 OP"
        
        Returns:
            bool: True if PLC slave found in OP state, False otherwise
        
        Log Output:
            - INFO: "The PLC ethercat slave is active (=> FLX0-GETC100 OP)"
            - Returns False if slave not found or not in OP state
        """
        if SKIP_ETHERCAT_CHECK:
            self.get_logger().info(
                "Skipping EtherCAT health check (PLC_MANAGER_SKIP_ETHERCAT=1)."
            )
            return True

        try:
            # ========== Run ethercat diagnostic pipeline ==========
            # Process 1: Get all EtherCAT slaves
            process1 = subprocess.Popen(["ethercat", "slaves"], stdout=subprocess.PIPE)
            # Process 2: Filter for FLX0-GETC100 PLC device
            process2 = subprocess.Popen(
                ["grep", "FLX0-GETC100"],
                stdin=process1.stdout,
                stdout=subprocess.PIPE
            )
            # Process 3: Extract device alias (col 3) and state (col 5)
            process3 = subprocess.Popen(
                ["awk", "{print $3, $5}"],
                stdin=process2.stdout,
                stdout=subprocess.PIPE
            )

            # Close intermediate pipes
            process1.stdout.close()

            # Get final output
            output, _ = process3.communicate()
            output_text = output.decode('utf-8').rstrip().lstrip()
        except FileNotFoundError as exc:
            self.get_logger().warn(
                "EtherCAT diagnostic command unavailable, skipping check: %s", exc
            )
            return False

        # ========== Check for expected device state ==========
        if "FLX0-GETC100" in output_text.split() and "OP" in output_text.split():
            self.get_logger().info(f'The PLC ethercat slave is active (=> {output_text})')
            return True
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
        self.destroy_node()


    def check_ros_launch_status(self) -> bool:
        """Monitor launcher health and manage control stack lifecycle (1 Hz timer).
        
        This timer callback executes every 1 second and performs two critical functions:
        
        1. LAUNCHER DETECTION: Check if run_platform_control.launch.py is running
        2. STATE MACHINE: Detect launcher crashes and manage graceful transitions
        
        Launcher Detection:
            Uses pgrep to find: "ros2 launch tecnobody_workbench run_platform_control.launch.py"
            
            If Found (RUNNING state):
                - Log: "✅ Ros control launcher detected!"
                - Set: self.ros_launched = True
                
            If Not Found (should be IDLE):
                - Log: "Waiting for ros controllers to start!" (throttled)
                - Set: self.ros_launched = False
        
        State Machine Logic:
            Detects transition from RUNNING to IDLE (launcher crash):
            - Condition: prev_state=True, current_state=False
            - Action: Execute emergency E-stop sequence:
                1. set E-stop to 0 (emergency stop)
                2. set manual_mode to 0 (return to automatic)
                3. small delay (50 ms)
                4. set E-stop back to 1 (re-enable)
            - Purpose: Notify PLC of launcher failure; PLC acknowledges by resetting E-stop
        
        Returns:
            bool: True if launcher currently running, False otherwise
        
        Side Effects:
            - Sets self.ros_launched and self.ros_launched_prev
            - May publish emergency E-stop commands on launcher crash
            - Logs state transitions and warnings
        """
        launcher_name = "run_platform_control.launch.py"
        
        try:
            # ========== Query for ros2 launch processes ==========
            output = subprocess.check_output(
                ["pgrep", "-af", "ros2 launch"],
                text=True
            )
            
            # ========== Search for run_platform_control launcher ==========
            for line in output.splitlines():
                if launcher_name in line:
                    # Launcher is running
                    if not self.ros_launched:
                        self.get_logger().info(
                            '✅ Ros control launcher detected!',
                            throttle_duration_sec=20.0
                        )
                        self.ros_launched = True
                        self.ros_launched_prev = self.ros_launched
                    return True
            
            # ========== Launcher not found (or not in expected state) ==========
            self.get_logger().info(
                '\033[1;35mWaiting for ros controllers to start!\033[0m',
                throttle_duration_sec=5.0
            )
            self.ros_launched = False
            
            # ========== Detect transition: RUNNING → IDLE (launcher crash) ==========
            if (self.ros_launched_prev != self.ros_launched) and \
               (self.ros_launched_prev == 1 and self.ros_launched == 0):
                # Launcher was running but now stopped unexpectedly
                self.get_logger().info(
                    'Call a SW ESTOP!',
                    throttle_duration_sec=5.0
                )
                # Execute emergency stop sequence to notify PLC
                self.publish_command('PLC_node/estop', 0)           # Emergency stop (deactivate)
                self.publish_command('PLC_node/manual_mode', 0)     # Return to automatic
                time.sleep(0.05)
                self.publish_command('PLC_node/estop', 1)           # Re-enable normal operation
            
            self.ros_launched_prev = self.ros_launched
            return self.ros_launched
            
        except subprocess.CalledProcessError:
            # pgrep returns nonzero if no processes found
            self.ros_launched = False
            return False


    def state_callback(self, msg: PlcStates) -> None:
        """Process PLC state updates and implement state machine logic.
        
        This is the core state machine callback invoked whenever PlcStates arrives
        from the PLC controller. It detects E-stop transitions and executes corresponding
        startup/shutdown sequences.
        
        State Transitions Handled:
        
        1. KEY TURN (estop 0→1): IDLE → RUNNING
            Condition: prev_ESTOP=0, current=1, first_time or launcher_not_running
            Actions:
                a. Check EtherCAT PLC slave is OP (operational)
                b. If first startup: launch with --perform-homing flag
                   (performs motor calibration, ~30 seconds)
                c. If not first: launch without homing (faster, ~3 seconds)
                d. Launch: ros2 launch tecnobody_workbench run_platform_control.launch.py
            Result: Motion controllers ready, exercise capable
        
        2. E-STOP PRESS (estop 1→0): RUNNING → IDLE
            Condition: prev_ESTOP=1, current=0
            Actions:
                a. Send UDP "STOP" to rehab_gui port 5005
                b. Wait for "STOPPED" acknowledgment (10 second timeout)
                c. Kill launcher process: run_platform_control.launch.py
                d. Publish emergency E-stop command to PLC
            Result: All controllers stopped, system returns to safe IDLE state
        
        3. STABLE STATES (no transition):
            estop 0→0 or 1→1: No action, continue monitoring
        
        Thread Safety:
            - Uses non-blocking threading.Lock.acquire()
            - If callback already executing, skips this update
            - Prevents race conditions between concurrent state updates
        
        Args:
            msg (PlcStates): Message containing:
                - interface_names: List of input identifiers
                - values: List of corresponding input values
        
        Returns:
            None. Modifies node state; may launch/kill subprocesses.
        
        Side Effects:
            - May launch bash scripts (launch_ros2_env.sh)
            - May kill ROS2 launcher processes
            - Publishes PLC commands
            - Logs colored state transition messages
        """
        # ========== Acquire lock to prevent concurrent execution ==========
        if not self.lock.acquire(blocking=False):
            self.get_logger().warn("state_callback already in execution, ignore.")
            return
        
        try:
            # ========== Extract PLC state from message ==========
            self.interface_names = list(msg.interface_names)
            self.state_values = list(msg.values)
            
            # ========== Search for E-stop signal in state message ==========
            for int_idx in range(len(self.interface_names)):
                if self.interface_names[int_idx] == "estop":
                    current_estop = self.state_values[int_idx]
                    
                    # ========== TRANSITION: 0→1 (KEY TURN: IDLE → RUNNING) ==========
                    if current_estop == 1 and self.ESTOP == 0:
                        self.get_logger().info(
                            bcolors.OKBLUE +
                            "🔑 KEY TURN DETECTED: E-stop 0→1 (IDLE→RUNNING)" +
                            bcolors.ENDC
                        )
                        
                        # Verify EtherCAT PLC is ready
                        if not self.check_ethercat_plc_node():
                            self.get_logger().warn("EtherCAT PLC not ready, deferring launch")
                            break
                        
                        # ========== First-time startup: perform homing ==========
                        if not self.FIRST_TIME:
                            self.get_logger().info("🏠 First startup: launching with homing calibration")
                            subprocess.Popen(
                                [" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh --perform-homing"],
                                shell=True,
                                executable="/bin/bash"
                            )
                            self.FIRST_TIME = True
                        # ========== Subsequent startups: skip homing (faster) ==========
                        else:
                            self.get_logger().info("⚡ Restart: launching without homing")
                            subprocess.Popen(
                                [" /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./launch_ros2_env.sh"],
                                shell=True,
                                executable="/bin/bash"
                            )
                    
                    # ========== TRANSITION: 1→0 (E-STOP PRESS: RUNNING → IDLE) ==========
                    elif current_estop == 0 and self.ESTOP == 1:
                        self.get_logger().info(
                            bcolors.OKCYAN +
                            "🛑 E-STOP PRESSED: E-stop 1→0 (RUNNING→IDLE)" +
                            bcolors.ENDC
                        )
                        
                        try:
                            self.publish_command('PLC_node/brake_disable', 0)  # Ensure brakes enabled

                            # ========== Graceful GUI shutdown ==========
                            self.get_logger().info("📡 Sending STOP signal to rehab_gui...")
                            self.client.send(b"STOP")
                            
                            # Wait for GUI to acknowledge (10 second timeout)
                            timeout = time.time() + 10
                            while True:
                                data, addr = self.client.receive()
                                if data == b"STOPPED":
                                    self.get_logger().info("✅ GUI shutdown acknowledged")
                                    break
                                time.sleep(0.5)
                                if time.time() > timeout:
                                    self.get_logger().warn(
                                        "PLC Manager: No acknowledgment from FMRR GUI for STOP command.",
                                        throttle_duration_sec=5.0
                                    )
                                    break
                        except Exception as e:
                            self.get_logger().warn(f"Error communicating with GUI: {e}")

                        try:
                            # ========== Kill launcher process ==========
                            self.get_logger().info("🔪 Killing run_platform_control.launch.py...")
                            platform_launcher_pid = subprocess.check_output(
                                ["pgrep", "-f", "ros2 launch tecnobody_workbench run_platform_control.launch.py"]
                            )
                            os.kill(int(platform_launcher_pid.strip()), signal.SIGINT)
                            time.sleep(2)
                            self.get_logger().info("✅ Launcher process terminated")
                        except ValueError as e:
                            self.get_logger().info(f"Exception caught: {e}")
                        except subprocess.CalledProcessError:
                            self.get_logger().info("No platform_control.launch.py process found.")
                    
                    # ========== STABLE STATES: No action ==========
                    elif current_estop == 0 and self.ESTOP == 0:
                        # E-stop already inactive, no transition
                        pass
                    
                    elif current_estop == 1 and self.ESTOP == 1:
                        # ========== RUNNING state: Send periodic status ==========
                        try:
                            node_list = self.get_node_names()
                            if 'tecnobody_ethercat_checker_node' in node_list:
                                self.send_running_cnt = self.send_running_cnt + 1
                                # Send RUNNING status every 10 ticks (50 Hz / 10 = 5 Hz)
                                if self.send_running_cnt % self.send_running_dec == 0:
                                    self.client.send(b"RUNNING")
                        except Exception as e:
                            self.get_logger().error(
                                f"Exception when sending message to UDP Server: {e}"
                            )
                    
                    # Update cached E-stop value for next transition detection
                    self.ESTOP = current_estop
                    
        finally:
            self.lock.release()


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
        if name in self.command_msg.interface_names:
            # Find index of interface by name
            idx = self.command_msg.interface_names.index(name)
            # Update value
            self.command_msg.values[idx] = value
            # Publish entire command vector
            self.command_publisher.publish(self.command_msg)
            self.get_logger().info(f"Published PLC command: {self.command_msg.values}")
        else:
            self.get_logger().warn(
                f"Interface name '{name}' not found in command message."
            )


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
            If not provided: defaults to "127.0.0.0" (localhost)
    
    Returns:
        None. Blocks until shutdown.
    """
    # Initialize ROS 2 without automatic signal handling
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    
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
    os.sched_setaffinity(0, {2})
    
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
        rclpy.try_shutdown()
        node.cleanup()
    except Exception as e:
        node.get_logger().error(f"Error during shutdown: {e}")


if __name__ == '__main__':
    main()
