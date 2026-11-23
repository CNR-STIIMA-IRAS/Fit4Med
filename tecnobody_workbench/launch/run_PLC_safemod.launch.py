# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, SetEnvironmentVariable

import os
import sys
os.sched_setaffinity(0, {2,3})

LAUNCH_NAME = 'run_PLC_safemod'

def _housekeeping_prefix():
    """taskset prefix that starts a process on the housekeeping CPUs.

    This launch runs on the isolated CPU(s) above so that ros2_control_node,
    with its RT loop, inherits them. Everything else would inherit them too:
    spawners and Python nodes then start there, competing with the RT loop
    (in run_platform_control each spawner took 6-15 s instead of < 1 s).
    """
    try:
        from tecnobody_workbench_utils.cpu_affinity import housekeeping_cpus
        cpus = ','.join(str(cpu) for cpu in sorted(housekeeping_cpus()))
    except Exception as exc:  # noqa: BLE001
        print(f'[{LAUNCH_NAME}] housekeeping CPUs unknown ({exc}): '
              'spawners and nodes stay on the isolated CPUs', file=sys.stderr)
        return []
    return [f'taskset -c {cpus}']


HOUSEKEEPING_PREFIX = _housekeeping_prefix()


def generate_launch_description():
    ros2_controllers = 'safemod_controllers.yaml'
    description_package = 'tecnobody_workbench'
    

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf",'platform_complete_PLC.config.urdf']),
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
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['joint_state_broadcaster'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        parameters=[robot_description],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )
    
    plc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['PLC_controller'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    plc_manager = Node(
        package='plc_manager',
        executable='plc_manager_node',
        output = 'screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    ssb = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['state_controller'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    fsb = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['ft_sensor_command_broadcaster'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    homing = Node(
        package='tecnobody_workbench_utils',
        executable='boot_hw',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['MODE_CYCLIC_SYNC_POSITION'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    homing_done_publisher = Node(
        package='tecnobody_workbench_utils',
        executable='homing_completion_publisher',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    eth_checker = Node(
        package='tecnobody_workbench_utils',
        executable='ethercat_checker_node',
        prefix=HOUSEKEEPING_PREFIX,
        name='tecnobody_ethercat_checker_node',
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    ft_offset_updater = Node(
        package='tecnobody_workbench_utils',
        executable='ft_offset_updater',
        prefix=HOUSEKEEPING_PREFIX,
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    joint_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=[
            'joint_trajectory_controller'
            # 'scaled_trajectory_controller'
        ],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    forward_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['forward_velocity_controller', '--inactive'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    forward_pos_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['forward_position_controller'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    admittance_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        prefix=HOUSEKEEPING_PREFIX,
        arguments=['admittance_controller', '--inactive'],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity}] [{time}] [{name}]: {message}'}
    )

    homing_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[homing, jsb, fsb],
        )
    )
    
    controllers_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[joint_controller_node, 
                     forward_controller_node,
                     forward_pos_controller_node,
                     admittance_controller_node, 
                     homing_done_publisher,
                     eth_checker,
                     plc_manager],
        )
    )

    
    # No unspawners on shutdown: launch never starts a process requested while
    # it is shutting down (ExecuteLocal returns early on context.is_shutdown),
    # so they never ran. ros2_control_node deactivates and shuts down all
    # controllers itself when it receives SIGINT.

    ld = LaunchDescription()
    ld.add_action(
        SetEnvironmentVariable(
            name='RCUTILS_CONSOLE_OUTPUT_FORMAT',
            value='[{severity}] [{time}] [{name}]: {message}'
        ))
    ld.add_action(ros2_control_node)
    ld.add_action(plc_spawner)
    ld.add_action(ssb)
    ld.add_action(rsp)
    ld.add_action(homing_launcher)
    ld.add_action(controllers_launcher)
    return ld