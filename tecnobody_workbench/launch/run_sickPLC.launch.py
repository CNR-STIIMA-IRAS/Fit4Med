# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

from typing import List
import subprocess

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, EmitEvent, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown



# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT']="[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"

import os
os.sched_setaffinity(0, {2})

plc_controller_manager_node_name="plc_controller_manager"

def clean_shutdown(event, context):
    import os
    nodes_names = ['robot_state_publisher', 'plc_manager_node', 'sonar_teach_node', 'ros2_control_node']
    for nm in nodes_names:
        _nm = nm[0:15] if len(nm)>15  else nm
        if event.reason == "ctrl-c (SIGINT)":
            os.system(f'pkill -SIGINT {_nm}')
        elif event.reason == "ctrl-z (SIGTERM)":
            os.system(f'pkill -SIGTERM {_nm}')
        else:
            os.system(f'pkill -SIGTERM {_nm}')
    return [
        LogInfo(msg=f'Shutdown Callback "{event.reason}" forwarded to {nodes_names}'),
    ]
    
def generate_launch_description():
    controllers_file = 'plc_controller.yaml'
    description_package = 'tecnobody_workbench'
    
    declare_gui_ip = DeclareLaunchArgument(
            'gui_ip',
            default_value='127.0.0.0',
            description='IP of the GUI'
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
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='plc_state_publisher',
        remappings=[('robot_description', 'plc_robot_description')],
        output='screen',
        parameters=[robot_description]
    )

    plc_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['PLC_controller', '-c', f'/{plc_controller_manager_node_name}'],
        output='screen',
    )

    debug_delta = LaunchConfiguration('debug_delta')

    plc_manager = Node(
        package='plc_manager',
        executable='plc_manager_node',
        output = 'screen',
        arguments=[LaunchConfiguration('gui_ip', default='127.0.0.0')],
        condition=UnlessCondition(debug_delta),
    )

    debug_delta_publish = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '/PLC_controller/plc_commands',
            'tecnobody_msgs/msg/PlcController',
            "{interface_names: ['PLC_node/mode_of_operation', 'PLC_node/estop'], values: [7, 1]}",
        ],
        output='screen',
        condition=IfCondition(debug_delta),
    )

    plc_manager_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=plc_controller_spawner,
            on_exit=[plc_manager, debug_delta_publish],
        )
    )

    sonar_teach_node = Node(
        package='tecnobody_workbench_utils',
        executable='sonar_teach_node',
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
    ld.add_action(declare_debug_delta)

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(rsp)
    ld.add_action(plc_controller_spawner)
    ld.add_action(plc_manager_launcher)
    ld.add_action(sonar_teach_node)
    ld.add_action(nodes_killer)
    return ld



