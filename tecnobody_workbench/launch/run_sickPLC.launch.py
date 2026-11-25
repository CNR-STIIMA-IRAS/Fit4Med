# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from typing import List
import atexit
import signal
import subprocess

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, EmitEvent, RegisterEventHandler, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
import launch.logging



# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT']="[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"

import os
import sys
os.sched_setaffinity(0, {2})


def _housekeeping_prefix():
    """taskset prefix that starts a process on the housekeeping CPUs.

    This launch runs on CPU 2 (above) so that the PLC ros2_control_node, with
    its RT loop, inherits it. Everything else would inherit it too: the
    spawner and the Python nodes would start there, competing with the RT
    loop (in run_platform_control each spawner took 6-15 s instead of < 1 s).
    """
    try:
        from tecnobody_workbench_utils.cpu_affinity import housekeeping_cpus
        cpus = ','.join(str(cpu) for cpu in sorted(housekeeping_cpus()))
    except Exception as exc:  # noqa: BLE001
        print(f'[run_sickPLC] housekeeping CPUs unknown ({exc}): '
              'spawner and nodes stay on CPU 2', file=sys.stderr)
        return []
    return [f'taskset -c {cpus}']


HOUSEKEEPING_PREFIX = _housekeeping_prefix()

plc_controller_manager_node_name="plc_controller_manager"

# Per-run log archive (see bash_scripts/fit4med_session_log.sh). Disable with
# FIT4MED_SESSION_LOG=0.
SESSION_LOG_SCRIPT = os.environ.get(
    'FIT4MED_SESSION_LOG_SCRIPT',
    '/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/fit4med_session_log.sh')
_shutdown_reason = ['launch exited (all processes ended)']


def start_session_log(context):
    """Open the log session of this run and point every process to it.

    The environment set here is inherited by all the nodes and, through
    plc_manager, by launch_ros2_env*.sh / launch_ros2_bridge.sh, which put
    the logs of each of their starts in a numbered folder of the session.
    """
    if os.environ.get('FIT4MED_SESSION_LOG', '1') == '0':
        return [LogInfo(msg='Session log disabled (FIT4MED_SESSION_LOG=0)')]
    if not os.access(SESSION_LOG_SCRIPT, os.X_OK):
        return [LogInfo(msg=f'Session log disabled: {SESSION_LOG_SCRIPT} not found')]
    gui_ip = LaunchConfiguration('gui_ip').perform(context)
    try:
        result = subprocess.run(
            [SESSION_LOG_SCRIPT, 'start', str(os.getpid()), gui_ip],
            capture_output=True, text=True, timeout=20, check=True)
        session_dir = result.stdout.strip().splitlines()[-1]
    except (subprocess.SubprocessError, OSError, IndexError) as exc:
        return [LogInfo(msg=f'Session log not started: {exc}')]
    # atexit: runs once every launched process has exited, also when the
    # launch is stopped with Ctrl-C or by systemd (SIGINT/SIGTERM).
    atexit.register(finalize_session_log, session_dir)
    return [
        SetEnvironmentVariable('FIT4MED_SESSION_DIR', session_dir),
        SetEnvironmentVariable('ROS_LOG_DIR', os.path.join(session_dir, 'sickPLC')),
        LogInfo(msg=f'Session logs: {session_dir}'),
    ]


def finalize_session_log(session_dir):
    # A second Ctrl-C (or the SIGTERM of a cleanup script) must not cut the
    # archiving short: ignored here, and so in the child too.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    try:
        launch_log_dir = launch.logging.launch_config.log_dir
    except Exception:
        launch_log_dir = ''
    print(f'[fit4med] Archiving the logs of this run: {session_dir} ...', flush=True)
    try:
        subprocess.run(
            [SESSION_LOG_SCRIPT, 'finalize', session_dir, launch_log_dir, _shutdown_reason[0]],
            timeout=90)
    except (subprocess.SubprocessError, OSError) as exc:
        print(f'[fit4med] Session log not archived: {exc}', flush=True)


def clean_shutdown(event, context):
    _shutdown_reason[0] = event.reason
    # ros2_control_node is excluded: it handles SIGINT correctly on its own.
    # -P: only the nodes started by this launch (its direct children; taskset
    # execs the node, so the pid is the node's). Without it a second bring-up
    # that failed to start killed the plc_manager of the running one, which
    # opened the safety chain (runs 0111/0112), and a Ctrl-C here also hit the
    # robot_state_publisher of the platform launch, which plc_manager stops.
    sig = '-SIGINT' if event.reason == 'ctrl-c (SIGINT)' else '-SIGTERM'
    nodes_names = ['robot_state_publisher', 'plc_manager_node', 'sonar_teach_node']
    for nm in nodes_names:
        # pkill matches the process name, at most 15 characters.
        subprocess.run(['pkill', sig, '-P', str(os.getpid()), nm[:15]], check=False)
    return [
        LogInfo(msg=f'Shutdown Callback "{event.reason}" forwarded to {nodes_names}'),
    ]
    
def generate_launch_description():
    # Before any process and before the session log (which archives the logs
    # of the bring-up it finds): a second bring-up cannot get EtherCAT master 0
    # nor UDP port 5006 and, failing, stopped the running one (runs 0111/0112).
    # Held until this launch exits; see plc_manager/bringup_lock.py.
    from plc_manager.bringup_lock import acquire, BringupRunning
    try:
        acquire()
    except BringupRunning as exc:
        print(f'[run_sickPLC] NOT STARTED: {exc}', file=sys.stderr, flush=True)
        raise

    controllers_file = 'plc_controller.yaml'
    description_package = 'tecnobody_workbench'
    
    declare_gui_ip = DeclareLaunchArgument(
            'gui_ip',
            default_value='127.0.0.0',
            description='IP of the GUI'
    )

    declare_eeg_delay_ms = DeclareLaunchArgument(
            'eeg_delay_ms',
            default_value='4000',
            description='Base EEG pause duration in milliseconds'
    )

    declare_debug_delta = DeclareLaunchArgument(
            'debug_delta',
            default_value='false',
            description='Debug delta mode: skip plc_manager and publish mode_of_operation=7'
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'sickPLC.config.urdf']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name=f'{plc_controller_manager_node_name}',
        remappings=[('robot_description', 'plc_robot_description')],
        arguments=[],
        parameters=[initial_joint_controllers],
        # shutdown_guard: on shutdown this node must outlive plc_manager, which
        # raises the SW ESTOP through it and waits for the PLC to confirm. Got
        # together with plc_manager (launch shutdown, Ctrl-C, systemd stop), the
        # SIGINT stopped the EtherCAT master first and plc_manager's shutdown
        # watchdog expired (run 0092). The guard forwards SIGINT only once
        # plc_manager_node has exited (max 15 s); sigterm_timeout leaves it that
        # time before launch escalates to SIGTERM.
        # stdbuf: line-buffered stdout for the EtherCAT driver printf (see
        # run_platform_control).
        prefix=[
            PathJoinSubstitution([FindPackagePrefix('plc_manager'), 'lib', 'plc_manager', 'shutdown_guard']),
            ' --after plc_manager_node --max-wait 15 -- stdbuf -oL -eL',
        ],
        sigterm_timeout='20',
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        prefix=HOUSEKEEPING_PREFIX,
        name='plc_state_publisher',
        remappings=[('robot_description', 'plc_robot_description')],
        output='screen',
        parameters=[robot_description]
    )

    plc_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['PLC_controller', '-c', f'/{plc_controller_manager_node_name}'],
        output='screen',
    )

    debug_delta = LaunchConfiguration('debug_delta')

    plc_manager = Node(
        package='plc_manager',
        executable='plc_manager_node',
        output = 'screen',
        arguments=[
            LaunchConfiguration('gui_ip', default='127.0.0.0'),
            LaunchConfiguration('eeg_delay_ms', default='4000'),
        ],
        condition=UnlessCondition(debug_delta),
        # Without plc_manager the rest of the bring-up looks alive but nothing
        # drives the PLC: stop everything (systemd then shows the unit down).
        on_exit=[EmitEvent(event=Shutdown(reason='plc_manager_node exited'))],
    )

    ethercat_slaves_status_check_node = Node(
        package='plc_manager',
        executable='ethercat_slaves_status_check_node',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
    )

    debug_delta_publish = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '/PLC_controller/plc_commands',
            'tecnobody_msgs/msg/PlcController',
            "{interface_names: ['PLC_node/mode_of_operation', 'PLC_node/estop'], values: [7, 1]}",
        ],
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        condition=IfCondition(debug_delta),
    )

    plc_manager_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=plc_controller_spawner,
            on_exit=[plc_manager, ethercat_slaves_status_check_node, debug_delta_publish],
        )
    )

    sonar_teach_node = Node(
        package='tecnobody_workbench_utils',
        executable='sonar_teach_node',
        prefix=HOUSEKEEPING_PREFIX,
        name='sonar_teach_node',
        output='screen',
    )

    nodes_killer = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=clean_shutdown # type: ignore
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(declare_gui_ip)
    ld.add_action(declare_eeg_delay_ms)
    ld.add_action(declare_debug_delta)

    # before any process: they inherit the session environment
    ld.add_action(OpaqueFunction(function=start_session_log))

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(rsp)
    ld.add_action(plc_controller_spawner)
    ld.add_action(plc_manager_launcher)
    ld.add_action(sonar_teach_node)
    ld.add_action(nodes_killer)
    return ld


