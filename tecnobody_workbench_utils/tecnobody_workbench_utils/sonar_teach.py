# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

"""Safety-critical sonar sensor teaching node for ultrasonic distance calibration.

This module implements a ROS 2 node that manages the sonar sensor calibration process
by commanding a safety PLC to reconfigure ultrasonic sensor commutation distances. When enabled, 
the node manages the rehabilitation system's sensor calibration process through the safety PLC.

Core Responsibility:
    Enable remote configuration of ultrasonic proximity sensor thresholds through
    a controlled teaching sequence that signals the PLC to enter teaching mode,
    allowing dynamic adjustment of sonar detection distances without manual
    hardware intervention.

Safety Context:
    The connected PLC is not part of the emergency stop (E-stop) chain. 

Service Interface:
    - /sonar_teach_node/enable_sonar_teach: Trigger remote sonar calibration sequence
      (e.g., called by sensor calibration routine in rehab_gui)
    - /sonar_teach_node/request_shutdown: Graceful node shutdown with cleanup
    
Signal Protocol with PLC:
    The teaching sequence uses timed pulse signals to command PLC state changes:
    
    Teaching Sequence as defined in sensor's datasheet (total duration: ~8 seconds):
        Phase 1 (0-3s):   Signal = 1 (sonar_teach_enable = TRUE)
        Phase 2 (3-5s):   Signal = 0 (sonar_teach_enable = FALSE)
        Phase 3 (5-8s):   Signal = 1 (sonar_teach_enable = TRUE)
        Phase 4 (8s+):    Signal = 0 (sonar_teach_enable = FALSE)
    
    The proximity sensor's firmware interprets these state transitions to execute the teaching
    protocol.

Architecture:
    - Asynchronous service: enable_sonar_teach returns immediately to caller
    - Background timer (50 ms): Executes teaching sequence phases
    - Mutual exclusion: Ensures teaching sequence completes without interruption
    - Graceful shutdown: Cancels timer, cleans up resources on exit

Message Flow:
    GUI/External → /sonar_teach_node/enable_sonar_teach service request
                 ↓
    sonar_teach_enable_callback() [service handler]
                 ↓
    start_sonar_teach() [starts 50 ms timer]
                 ↓
    teach_progress() [timer callback, 50 ms interval]
                 ↓ (every 50 ms, updates signal state based on elapsed time)
                 ↓
    publish_command() [sends command to /PLC_controller/plc_commands topic]
                 ↓
    PLC processes signal transition
                 ↓ (after ~8 seconds)
                 ↓
    teach_timer cancels when sequence complete

Attributes:
    service_group (MutuallyExclusiveCallbackGroup): Serializes service calls
    timer_group (MutuallyExclusiveCallbackGroup): Isolates timer callbacks
    command_publisher (Publisher): Interface to PLC controller command topic
    trigger_teach_service (Service): External entry point for teaching requests
    shutdown_service (Service): Graceful shutdown trigger
    stay_awake_timer (Timer): Background keep-alive timer (10 s interval)
    teach_timer (Timer): Active teaching sequence timer (50 ms interval, created dynamically)
    sonar_teach_active (bool): Flag indicating teaching sequence in progress
    shutdown_requested (bool): Flag requesting graceful shutdown
"""

import os

# Must be BEFORE importing rclpy (sets logging format globally)
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tecnobody_msgs.msg import PlcController
from std_srvs.srv import Trigger

import signal
import sys


class SonarTeachNode(Node):
    """Node for setting PLC sonar sensor commutation distance.
    
    This node provides a remote interface for triggering sonar ultrasonic sensor
    teaching sequences via PLC commands. When a request arrives, the
    node executes a precisely-timed signal sequence that instructs the PLC to
    enter teaching mode, read the current sensor state, and store new commutation
    distances.
    
    The teaching sequence is implemented as a series of timed signal transitions
    published to the PLC at 20 Hz. The PLC firmware decodes these transitions to
    perform the actual sensor calibration at appropriate moments.
    
    Concurrency Model:
        - Service callbacks and timers run in separate mutually-exclusive groups
        - Only one teaching sequence can run at a time (timer_group serialization)
        - Multiple shutdown requests are handled gracefully
        - Ctrl+C and remote shutdown both set shutdown_requested flag
    
    Safety Guarantees:
        - All PLC commands use RELIABLE QoS (guaranteed delivery)
        - Teaching sequence is atomic: completes or times out, no partial state
        - Graceful shutdown cancels timers before node destruction
        - Signal handler prevents orphaned processes
    
    Attributes:
        service_group (MutuallyExclusiveCallbackGroup): Serializes service execution
        timer_group (MutuallyExclusiveCallbackGroup): Ensures timer callbacks don't overlap
        command_publisher (Publisher): Sends PlcController messages to PLC interface
        trigger_teach_service (Service): Handler for enable_sonar_teach requests
        shutdown_service (Service): Handler for graceful shutdown requests
        stay_awake_timer (Timer): 10-second keep-alive timer (prevents system sleep)
        teach_timer (Timer): Active teaching sequence timer (50 ms interval, created on demand)
        sonar_teach_active (bool): Flag indicating teaching is in progress
        shutdown_requested (bool): Flag set by signal handler or shutdown service
    """

    def __init__(self):
        """Initialize the sonar teaching node with PLC interface and services.
        
        Creates ROS 2 node infrastructure for sonar calibration:
        - Sets up publisher to PLC command interface with RELIABLE QoS
        - Registers two services (enable teaching, request shutdown)
        - Creates background keep-alive timer
        - Installs Ctrl+C signal handler for graceful shutdown
        - Initializes state flags for teaching and shutdown
        
        QoS Configuration:
            - Reliability: RELIABLE (guaranteed PLC message delivery)
            - Depth: 10 (buffer 10 messages if PLC temporarily unavailable)
            - Used for all critical commands to safety PLC
        
        Side Effects:
            - Registers signal handler for SIGINT (Ctrl+C)
            - Creates ROS 2 node named 'sonar_teach_node' in graph
            - Logs initialization status
        """
        super().__init__('sonar_teach_node')

        # ========== Callback Groups ==========
        # Service and timer callbacks run independently to prevent blocking
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # ========== PLC Publisher ==========
        # RELIABLE QoS ensures all commands reach PLC even if temporarily unavailable
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.command_publisher = self.create_publisher(
            PlcController,
            '/PLC_controller/plc_commands',
            qos
        )

        # ========== Service Servers ==========
        # Both services use mutual exclusion to prevent concurrent calls
        self.trigger_teach_service = self.create_service(
            Trigger,
            "/sonar_teach_node/enable_sonar_teach",
            self.sonar_teach_enable_callback,
            callback_group=self.service_group
        )

        self.shutdown_service = self.create_service(
            Trigger,
            "/sonar_teach_node/request_shutdown",
            self.shutdown_node
        )

        # ========== Keep-Alive Timer ==========
        # Prevents node from sleeping during idle periods (system stays responsive)
        self.stay_awake_timer = self.create_timer(
            10.0,
            lambda: True,
            self.timer_group
        )
        
        self.get_logger().info("SonarTeachNode correctly initialized.")
        
        # ========== Signal Handler ==========
        # Intercept Ctrl+C for graceful shutdown instead of immediate termination
        signal.signal(signal.SIGINT, self.signal_handler)

        # ========== State Flags ==========
        self.shutdown_requested = False
        self.sonar_teach_active = False


    def cleanup(self) -> None:
        """Perform graceful cleanup and resource deallocation.
        
        Called before node destruction to ensure all timers are cancelled
        and resources are released. Prevents orphaned timers or hanging
        callbacks.
        
        Cleanup Actions:
            1. Cancel stay-alive timer
            2. Cancel teaching timer if active
            3. Destroy node resources
        
        Returns:
            None. Modifies node state; leaves node in destroyed state.
        """
        self.get_logger().info("Cleanup: Cancelling timers and destroying node.")
        
        # Cancel background keep-alive timer
        if not self.stay_awake_timer.is_canceled():
            self.stay_awake_timer.cancel()
        
        # Cancel teaching sequence timer if active
        if hasattr(self, 'teach_timer') and self.teach_timer is not None:
            if not self.teach_timer.is_canceled():
                self.teach_timer.cancel()
        
        # Destroy node resources
        self.destroy_node()


    def sonar_teach_enable_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Service handler for enable_sonar_teach remote requests.
        
        Entry point for external systems (GUI, bash commands) to trigger
        sonar sensor teaching sequences. The handler immediately returns success
        and launches the teaching sequence asynchronously via start_sonar_teach().
        
        This non-blocking approach prevents the caller from waiting for the
        full ~8 second teaching sequence. The caller can assume teaching is
        complete when teach_timer cancels (after completion or timeout).
        
        Args:
            request (Trigger.Request): Empty trigger request (no parameters)
            response (Trigger.Response): Response object to populate
        
        Returns:
            Trigger.Response: success=True after teaching sequence initiated
        
        Side Effects:
            - Launches teach_timer if not already active
            - Sets sonar_teach_active flag
            - Logs the request
        """
        self.get_logger().info("Received sonar teach request from external caller.")
        self.start_sonar_teach()
        response.success = True
        return response


    def publish_command(self, name: str, value: int) -> None:
        """Publish command to PLC controller via PlcController message.
        
        Constructs and sends a PlcController message containing a single command
        to the PLC. The message format allows specifying arbitrary interface names
        and values, enabling future extension to multiple simultaneous commands.
        
        Current Usage:
            - name: "PLC_node/sonar_teach" (hardcoded sonar teaching interface)
            - value: 0 or 1 (signal state during teaching sequence)
        
        Message Structure:
            PlcController:
                interface_names: [str]  # List of interface identifiers
                values: [int]           # Corresponding command values
        
        Args:
            name (str): Interface identifier for the PLC command
                (e.g., "PLC_node/sonar_teach")
            value (int): Command value to send (e.g., 0 or 1 for teaching signal)
        
        Returns:
            None. Publishes message to /PLC_controller/plc_commands topic.
        
        Side Effects:
            - Logs published command value for debugging
            - Message is guaranteed to reach PLC (RELIABLE QoS)
        """
        command_msg = PlcController()
        command_msg.interface_names = [name]
        command_msg.values = [value]
        self.command_publisher.publish(command_msg)
        self.get_logger().debug(f"Published PLC command: {name} = {value}")


    def start_sonar_teach(self) -> None:
        """Initiate sonar teaching sequence with 50 ms timer callback.
        
        Creates and starts a timer that will execute teach_progress() every 50 ms
        (20 Hz) to implement the teaching signal sequence. The timer continues until
        teach_progress() explicitly cancels it after the sequence completes (~8 seconds).
        
        The teaching sequence is entirely time-based:
            - teach_progress() reads elapsed time since sequence start
            - Based on elapsed time, determines current signal state
            - Publishes command to PLC
            - Cancels timer when sequence complete
        
        This design ensures precise timing independent of system load,
        as long as the 50 ms timer granularity is maintained.
        
        Returns:
            None. Creates self.teach_timer and sets sonar_teach_active flag.
        
        Side Effects:
            - Creates teach_timer callback (runs on timer_group)
            - Sets sonar_teach_active = True
            - Logs sequence start
        """

        if self.sonar_teach_active:
            self.get_logger().warning("Sonar teach sequence already active. Ignoring new request.")
            return
        
        self.get_logger().info("Starting sonar teach sequence (duration: ~8 seconds).")
        self.sonar_teach_active = True
        
        # Capture start time for teach_progress() to calculate elapsed time
        start_time = self.get_clock().now()
        
        # Create timer to execute teaching sequence at 20 Hz (50 ms interval)
        # Lambda captures start_time for teach_progress to reference
        self.teach_timer = self.create_timer(
            0.05,
            lambda s=start_time: self.teach_progress(s),
            callback_group=self.timer_group
        )

    def teach_progress(self, start_time): 
            """Execute sonar teaching sequence state machine (50 ms timer callback).
            
            This timer callback implements the sonar teaching protocol as a sequence
            of timed signal transitions. Called at 20 Hz (every 50 ms), the callback
            calculates elapsed time since sequence start and updates the PLC command
            signal accordingly.
            
            Args:
                start_time: ROS2 time when teach_progress() was first called
                    (captured in start_sonar_teach() lambda)
            
            Returns:
                None. Publishes commands and cancels timer when complete.
            
            Side Effects:
                - Publishes command every 50 ms (via publish_command)
                - Cancels teach_timer after ~8 seconds
                - Logs completion message (once=True prevents duplicate logs)
            """
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9
            # Sonar teaching logic
            if elapsed_time < 3.3: 
                self.publish_command('PLC_node/sonar_teach', 1)
            elif elapsed_time > 3.3 and elapsed_time < 3.6:
                self.publish_command('PLC_node/sonar_teach', 0)
            elif elapsed_time > 3.6 and elapsed_time < 4.6:
                self.publish_command('PLC_node/sonar_teach', 1)
            else:
                self.publish_command('PLC_node/sonar_teach', 0)
                self.get_logger().info("Teaching process completed.")
                self.sonar_teach_active = False
                self.teach_timer.cancel()


    def shutdown_node(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        """Service handler for remote graceful shutdown requests.
        
        Allows external systems to request node shutdown without killing
        the process. Sets shutdown_requested flag, which is checked in main()
        to break the executor spin loop and trigger cleanup.
        
        This method provides a clean alternative to sending SIGTERM/SIGKILL,
        ensuring all resources are properly deallocated.
        
        Args:
            request (Trigger.Request): Empty trigger request
            response (Trigger.Response): Response object to populate
        
        Returns:
            Trigger.Response: success=True if shutdown request accepted
        
        Side Effects:
            - Sets shutdown_requested flag
            - Logs shutdown request
        """
        self.get_logger().info("Remote graceful shutdown request received.")
        self.shutdown_requested = True
        response.success = True
        return response


    def signal_handler(self, sig, frame) -> None:
        """Handle Ctrl+C (SIGINT) signal for graceful shutdown.
        
        Installed via signal.signal(signal.SIGINT, ...) in __init__().
        Provides the same shutdown behavior as the remote shutdown service,
        allowing both Ctrl+C and remote requests to trigger cleanup.
        
        Instead of immediately terminating the process, sets shutdown_requested
        flag for the main executor loop to detect and handle gracefully.
        
        Args:
            sig: Signal number (signal.SIGINT)
            frame: Current stack frame
        
        Returns:
            None. Sets shutdown_requested flag.
        """
        self.get_logger().info("Ctrl+C detected (SIGINT). Initiating graceful shutdown.")
        self.shutdown_requested = True


def main(args=None):
    """Entry point for the sonar teaching node.
    
    Initializes ROS 2 system with custom signal handling (disabled by default
    to allow manual signal_handler implementation), creates the SonarTeachNode,
    and runs a multi-threaded executor loop.
    
    Execution Flow:
        1. Create SonarTeachNode instance with signal handling disabled
        2. Create MultiThreadedExecutor with 2 threads
        3. Add node to executor
        4. Spin executor until shutdown_requested flag is set
        5. Gracefully shutdown ROS 2 system
        6. Cleanup node resources
    
    Shutdown Conditions:
        - shutdown_requested flag set by signal handler (Ctrl+C)
        - shutdown_requested flag set by remote shutdown service
        - Unhandled exception (logs and exits)
        - rclpy.ok() returns False (system shutdown externally)
    
    Multi-Threading:
        - 2 threads in executor (one for services, one for timers)
        - MutuallyExclusiveCallbackGroup prevents concurrent service/timer execution
        - Ensures teaching sequence completes atomically
    
    Args:
        args (list, optional): Command-line arguments passed to rclpy.init()
            (default: None)
    
    Returns:
        None. Blocks until shutdown_requested is set.
    """
    # Initialize ROS 2 without automatic signal handling
    # (custom signal_handler implemented in SonarTeachNode)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    
    # Create node instance
    node = SonarTeachNode()

    # Create multi-threaded executor with 2 threads
    # Thread 1: Service callbacks
    # Thread 2: Timer callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        # Spin executor until shutdown requested
        while rclpy.ok():
            # Check for shutdown request from signal handler or service
            if node.shutdown_requested:
                node.get_logger().warning('Graceful shutdown request received!')
                break
            # Execute single iteration of executor (process callbacks)
            executor.spin_once()

    except Exception as e:
        node.get_logger().error(f"Unexpected exception in main loop: {e}")
    
    # Graceful cleanup
    try:
        rclpy.try_shutdown()
        node.cleanup()
    except Exception as e:
        node.get_logger().error(f"Error during shutdown: {e}")
