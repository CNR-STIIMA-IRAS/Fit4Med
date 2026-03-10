# Copyright 2026 CNR-STIIMA-IRAS
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def load_yaml(package_name, file_path):
    package_share = FindPackageShare(package=package_name).find(package_name)
    file_path = PathJoinSubstitution([package_share, file_path]).perform(LaunchContext())
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    moveit_config_package = "fmrrehab_moveit_config"
    srdf_file = "tecnobody.srdf"
    ros2_controllers = 'ros2_controllers.yaml'
    launch_rviz = "true"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", 'tecnobody_real.urdf.xacro']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", ros2_controllers]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[initial_joint_controllers],
        output='screen',
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])
    
    ssb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['state_controller'],
        output='screen',
    )

    fsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_sensor_command_broadcaster'],
        output='screen',
    )

    joint_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller'
        ],
        output='screen',
    )

    remapping_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'remapping_controller'
        ],
        output='screen',
    )

    homing = Node(
        package='tecnobody_workbench_utils',
        executable='boot_hw',
        arguments=['MODE_CYCLIC_SYNC_POSITION'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('perform_homing'))
    )

    eth_checker = Node(
        package='tecnobody_workbench_utils',
        executable='ethercat_checker_node',
        name='tecnobody_ethercat_checker_node',
        output='screen',
    )

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", srdf_file]),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics_real.yaml"]
    )

    robot_description_planning = load_yaml(moveit_config_package, "config/joint_limits.yaml")

    moveit_config = (
        MoveItConfigsBuilder("fmrrehab")
        .robot_description(file_path="config/tecnobody_real.urdf.xacro")
        .robot_description_semantic(file_path="config/tecnobody.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    group1 = GroupAction(
                actions=[
                    eth_checker,
                    remapping_controller_node,
                    move_group_node,
                    rviz_node
                ],
                condition=IfCondition(LaunchConfiguration('perform_homing'))
            )
    
    group2 = GroupAction(
                actions=[
                    eth_checker,
                    remapping_controller_node,
                    move_group_node,
                    rviz_node
                ],
                condition=UnlessCondition(LaunchConfiguration('perform_homing'))
            )
    
    controllers_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[group1],
        )
    )

    homing_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[homing, jsb, fsb],
        )
    )

    controllers_launcher_no_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[group2],
        )
    )

    joint_controller_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=remapping_controller_node,
            on_exit=[joint_controller_node],
        )
    )

    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(rsp)
    ld.add_action(ssb)
    ld.add_action(homing_launcher)
    ld.add_action(controllers_launcher)
    ld.add_action(controllers_launcher_no_homing)
    ld.add_action(joint_controller_launcher)

    return ld