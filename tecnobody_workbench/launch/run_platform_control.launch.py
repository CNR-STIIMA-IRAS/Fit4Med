# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, LogInfo, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource

import os
import sys
os.sched_setaffinity(0, {3})


def _housekeeping_prefix():
    """taskset prefix that starts a process on the housekeeping CPUs.

    This launch runs on CPU 3 (above) so that ros2_control_node, with its RT
    loop, inherits it. Everything else would inherit it too: the ~9 spawners
    and the Python nodes start together there, competing with the RT loop for
    one CPU, and each spawner took 6-15 s instead of well under 1 s (run 0091).
    """
    try:
        from tecnobody_workbench_utils.cpu_affinity import housekeeping_cpus
        cpus = ','.join(str(cpu) for cpu in sorted(housekeeping_cpus()))
    except Exception as exc:  # noqa: BLE001
        print(f'[run_platform_control] housekeeping CPUs unknown ({exc}): '
              'spawners and nodes stay on CPU 3', file=sys.stderr)
        return []
    return [f'taskset -c {cpus}']


HOUSEKEEPING_PREFIX = _housekeeping_prefix()


def generate_launch_description():
    ros2_controllers = 'safemod_controllers.yaml'
    description_package = 'tecnobody_workbench'
    declare_perform_homing = DeclareLaunchArgument(
            'perform_homing',
            default_value='false',
            description='Se true lancia il nodo extra'
    )

    declare_eeg_delay_ms = DeclareLaunchArgument(
            'eeg_delay_ms',
            default_value='6000',
            description='Base EEG pause duration in milliseconds'
    )

    

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf",'platform_complete.config.urdf']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ros2_controllers]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[initial_joint_controllers],
        # prefix=['xterm -e gdb -ex run --args'],
        # Line-buffered stdout: the EtherCAT driver reports WC/slave state
        # changes with printf, which would otherwise reach the logs in 4 KB
        # blocks (or never, if the node dies).
        prefix=['stdbuf -oL -eL'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'ros2_control_node_%p_%t.log'}
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        parameters=[robot_description],
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'robot_state_publisher_%p_%t.log'}
    )

    ssb = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['state_controller'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'state_controller_%p_%t.log'}

    )

    # Controllers are spawned by a few spawners with several controllers each
    # (loaded and activated in the given order, the spawner stops at the first
    # failure) instead of one spawner process per controller:
    #   state_controller (alone: the homing waits for it)
    #   -> broadcasters_spawner              (active)
    #   -> active_controllers_spawner        (active; after the homing if any)
    #      -> inactive_controllers_spawner   (--inactive)
    # plc_manager needs all of them anyway (platform readiness monitor).
    broadcasters_spawner = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['joint_state_broadcaster', 'ft_sensor_command_broadcaster'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'broadcasters_spawner_%p_%t.log'}
    )

    homing = Node(
        package='tecnobody_workbench_utils',
        executable='boot_hw',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['MODE_CYCLIC_SYNC_POSITION'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('perform_homing')),
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'homing_%p_%t.log'}
    )

    homing_done_publisher = Node(
        package='tecnobody_workbench_utils',
        executable='homing_completion_publisher',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
    )

    eth_checker = Node(
        package='tecnobody_workbench_utils',
        executable='ethercat_checker_node',
        prefix=HOUSEKEEPING_PREFIX,
        name='tecnobody_ethercat_checker_node',
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'ethercat_checker_%p_%t.log'}
    )

    # Ring buffer of /joint_states (incl. effort = drive torque): saved as
    # fault_snapshot_*.csv in the folder of this start when a drive faults.
    fault_snapshot_node = Node(
        package='tecnobody_workbench_utils',
        executable='fault_snapshot_node',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'fault_snapshot_%p_%t.log'}
    )

    ft_offset_updater = Node(
        package='tecnobody_workbench_utils',
        executable='ft_offset_updater',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'ft_offset_updater_%p_%t.log'}
    )

    # remapping_controller first: joint_trajectory_controller commands through
    # its reference interfaces (remapping_controller/joint_*), which exist only
    # once it is active. forward_position_controller drives joint_yaw only.
    active_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=[
            'remapping_controller',
            'forward_position_controller',
            'joint_trajectory_controller',
        ],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'active_controllers_spawner_%p_%t.log'}
    )

    # Started after active_controllers_spawner: go_to_start_controller also
    # commands through remapping_controller.
    inactive_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=[
            'forward_velocity_controller',
            'admittance_controller',
            'go_to_start_controller',
            '--inactive',
        ],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'inactive_controllers_spawner_%p_%t.log'}
    )

    homing_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[homing, broadcasters_spawner],
        )
    )

    fct_manager_node = Node(
        package='tecnobody_workbench_utils',
        executable='fct_manager_node',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['joint_trajectory_controller'],
        parameters=[{
            'eeg_delay_ms': ParameterValue(
                LaunchConfiguration('eeg_delay_ms'),
                value_type=int,
            ),
        }],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'fct_manager_node_%p_%t.log'}
    )

    bag_recorder_node = Node(
        package='tecnobody_workbench_utils',
        executable='bag_recorder_node',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'bag_recorder_node_%p_%t.log'}
    )

    filter_commands_node = Node(
        package='tecnobody_workbench_utils',
        executable='filter_commands_node',
        prefix=HOUSEKEEPING_PREFIX,
        parameters=[{'forward_velocity_controller_name': 'forward_velocity_controller', 
                   'joint_trajectory_controller_name':'joint_trajectory_controller', 
                   'default_sample_rate':250,
                   'desampled_rate':25,
                   'joint_names': ['joint_x', 'joint_y', 'joint_z']}],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'filter_commands_node_%p_%t.log'}
    )

    group1 = GroupAction(
                actions=[
                    active_controllers_spawner,
                    homing_done_publisher,
                    eth_checker
                ],
                condition=IfCondition(LaunchConfiguration('perform_homing'))
            )

    group2 = GroupAction(
                actions=[
                    active_controllers_spawner,
                    homing_done_publisher,
                    eth_checker
                ],
                condition=UnlessCondition(LaunchConfiguration('perform_homing'))
            )

    controllers_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[group1],
        )
    )

    controllers_launcher_no_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[group2],
        )
    )

    # joint_trajectory_controller is active: the inactive controllers, and the
    # nodes that use the trajectory controller (as before, after its spawner).
    joint_controller_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=active_controllers_spawner,
            on_exit=[inactive_controllers_spawner],
        )
    )

    fct_manager_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=active_controllers_spawner,
            on_exit=[fct_manager_node, filter_commands_node, bag_recorder_node],
        )
    )

    ft_offset_updater_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=broadcasters_spawner,
            on_exit=[ft_offset_updater],
        )
    )

    #rosbridge_pkg = FindPackageShare('tecnobody_workbench').find('tecnobody_workbench')
    #xml_launch = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')

    #rosbridge_launch = IncludeLaunchDescription(
    #    AnyLaunchDescriptionSource(xml_launch)
    #)

    
    # No unspawners on shutdown: launch never starts a process requested while
    # it is shutting down (ExecuteLocal returns early on context.is_shutdown),
    # so they never ran; and the second Shutdown (from the handler below)
    # re-executed the same Node actions -> "executed more than once" and exit
    # code 1. ros2_control_node deactivates and shuts down all controllers
    # itself when it receives SIGINT.

    # If ros2_control_node dies (crash, kill) the drives lose the EtherCAT
    # loop and torque, while the launch would keep running: plc_manager only
    # watches the launch process, so it would not stop the platform and close
    # the brake. Shut the whole launch down instead (ethercat_checker closes
    # the brake on exit, plc_manager sees the launch gone and runs its STOP).
    shutdown_on_control_node_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros2_control_node,
            on_exit=[
                LogInfo(msg='ros2_control_node exited: shutting the launch down.'),
                EmitEvent(event=Shutdown(reason='ros2_control_node exited')),
            ],
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_perform_homing)
    ld.add_action(declare_eeg_delay_ms)
    ld.add_action(ros2_control_node)
    ld.add_action(ssb)
    ld.add_action(rsp)
    ld.add_action(fault_snapshot_node)
    ld.add_action(homing_launcher)
    ld.add_action(controllers_launcher)
    ld.add_action(shutdown_on_control_node_exit)
    ld.add_action(controllers_launcher_no_homing)
    ld.add_action(fct_manager_launcher)
    ld.add_action(joint_controller_launcher)
    ld.add_action(ft_offset_updater_launcher)
    #ld.add_action(rosbridge_launch)
    return ld
