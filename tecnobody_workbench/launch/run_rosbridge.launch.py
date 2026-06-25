# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import threading

import os
os.sched_setaffinity(0, {3})

# Funzione per lanciare il nodo che pubblica il flag
def launch_status_node(context, *args, **kwargs):
    import rclpy
    from tecnobody_workbench_utils.launch_status_publisher import LaunchStatusPublisher

    def run():
        rclpy.init()
        node = LaunchStatusPublisher()
        context._launch_status_node = node
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    
# Funzione per cambiare lo stato a True
def set_node_names(context, *args, **kwargs):
    node = getattr(context, "_launch_status_node", None)
    if node:
        node.get_logger().info(f"Setting launch status {args}")
        node.set_ready(args)
    return []

def clean_shutdown():
    joint_state_broadcaster_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=['joint_state_broadcaster'],
    )
    state_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=['state_controller'],
    )
    force_torque_sensor_broadcaster_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "ft_sensor_command_broadcaster"
        ],
    )
    joint_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "joint_trajectory_controller"
            # 'scaled_joint_trajectory_controller'
        ],
    )
    forward_pos_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "forward_position_controller"
        ],
    )
    forward_vel_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "forward_velocity_controller"
        ],
    )
    admittance_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "admittance_controller"
        ],
    )


    return LaunchDescription([
        joint_state_broadcaster_unspawner,
        state_controller_unspawner,
        force_torque_sensor_broadcaster_unspawner,
        forward_pos_controller_unspawner,
        forward_vel_controller_unspawner,
        admittance_controller_unspawner,
        joint_controller_unspawner,
    ])


def generate_launch_description():
    
    rosbridge_pkg = FindPackageShare('tecnobody_workbench').find('tecnobody_workbench')
    xml_launch = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(xml_launch)
    )

    ld = LaunchDescription()
    ld.add_action(rosbridge_launch)
    return ld
