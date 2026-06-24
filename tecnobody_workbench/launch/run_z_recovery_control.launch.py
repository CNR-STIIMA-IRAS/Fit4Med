# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Minimal launch for Z-axis limit-switch recovery.
# Starts only the infrastructure + forward_velocity_controller (active) + filter_commands_node,
# so the operator can jog Z+ out of the limit without loading the full platform stack.
# NOT included: joint_trajectory_controller, admittance_controller, remapping_controller,
#               forward_position_controller, go_to_start_controller, fct_manager_node,
#               bag_recorder_node, homing nodes.

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.actions import RegisterEventHandler, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

import os
os.sched_setaffinity(0, {3})


def generate_launch_description():
    ros2_controllers = 'safemod_controllers.yaml'
    description_package = 'tecnobody_workbench'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'platform_complete.config.urdf']),
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
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'ros2_control_node_%p_%t.log'}
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'robot_state_publisher_%p_%t.log'}
    )

    ssb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['state_controller'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'state_controller_%p_%t.log'}
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'joint_state_broadcaster_%p_%t.log'}
    )

    fsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_sensor_command_broadcaster'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'ft_sensor_command_broadcaster_%p_%t.log'}
    )

    ft_offset_updater = Node(
        package='tecnobody_workbench_utils',
        executable='ft_offset_updater',
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'ft_offset_updater_%p_%t.log'}
    )

    # Spawned ACTIVE (no --inactive): immediately available for Z+ jogging via filter_commands_node
    forward_vel_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller'],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'forward_velocity_controller_%p_%t.log'}
    )

    eth_checker = Node(
        package='tecnobody_workbench_utils',
        executable='ethercat_checker_node',
        name='tecnobody_ethercat_checker_node',
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'ethercat_checker_%p_%t.log'}
    )

    # filter_commands_node provides the soft_movement_start/stop services used by GUI jogging
    filter_commands_node = Node(
        package='tecnobody_workbench_utils',
        executable='filter_commands_node',
        parameters=[{
            'forward_velocity_controller_name': 'forward_velocity_controller',
            'joint_trajectory_controller_name': 'joint_trajectory_controller',
            'default_sample_rate': 250,
            'desampled_rate': 25,
            'joint_names': ['joint_x', 'joint_y', 'joint_z'],
        }],
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'filter_commands_node_%p_%t.log'}
    )

    # Automatically jogs Z+ by 5 cm once filter_commands_node is ready
    auto_z_recovery_node = Node(
        package='tecnobody_workbench_utils',
        executable='auto_z_recovery_node',
        output='screen',
        additional_env={'RCUTILS_LOGGING_FILE_NAME': 'auto_z_recovery_%p_%t.log'}
    )

    # After state_controller is spawned: bring up joint/ft broadcasters, vel controller, checker
    after_ssb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[jsb, fsb, forward_vel_controller_node, eth_checker],
        )
    )

    # After forward_velocity_controller is spawned: start filter_commands_node + auto recovery
    after_forward_vel = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_vel_controller_node,
            on_exit=[filter_commands_node, auto_z_recovery_node],
        )
    )

    after_fsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fsb,
            on_exit=[ft_offset_updater],
        )
    )

    # Clean shutdown: unspawn all spawned controllers
    controller_unspawner = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                LogInfo(msg=['Z-recovery launch shutting down — unspawning controllers...']),
                Node(package='controller_manager', executable='unspawner',
                     arguments=['joint_state_broadcaster']),
                Node(package='controller_manager', executable='unspawner',
                     arguments=['state_controller']),
                Node(package='controller_manager', executable='unspawner',
                     arguments=['ft_sensor_command_broadcaster']),
                Node(package='controller_manager', executable='unspawner',
                     arguments=['forward_velocity_controller']),
            ]
        )
    )

    rosbridge_pkg = FindPackageShare('tecnobody_workbench').find('tecnobody_workbench')
    xml_launch = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(xml_launch)
    )

    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(ssb)
    ld.add_action(rsp)
    ld.add_action(after_ssb)
    ld.add_action(after_forward_vel)
    ld.add_action(after_fsb)
    ld.add_action(controller_unspawner)
    ld.add_action(rosbridge_launch)
    return ld
