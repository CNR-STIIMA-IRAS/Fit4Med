# Copyright 2024 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from fcntl import F_GET_SEALS
import controller_manager
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def kill_nodes():
    return ExecuteProcess(
        cmd=['killall', '-9', 'ros2'],
        shell=True,
        output='screen'
    )

def clean_shutdown(namespace, robot_controllers, ft_controllers):
    joint_state_broadcaster_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=['joint_state_broadcaster'],
    )
    forward_command_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=['forward_command_controller'],
    )
    force_torque_sensor_broadcaster_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        namespace=namespace,
        arguments=[
            "robotiq_force_torque_sensor_broadcaster",
        ],
    )
    ft_commands_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        namespace=namespace,
        arguments=[
            "admittance_controller",
        ],
    )
    ft_commands_broadcaster_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
        ],
    )
    return LaunchDescription([
        joint_state_broadcaster_unspawner,
        forward_command_controller_unspawner,
        force_torque_sensor_broadcaster_unspawner,
        ft_commands_controller_unspawner,
        ft_commands_broadcaster_unspawner
    ])

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='')
    namespace = LaunchConfiguration("namespace", default="ft_sensor")
    description_package = LaunchConfiguration("description_package", default="tecnobody_workbench")
    description_file = LaunchConfiguration("description_file", default="urdf/robotiq_ft300.urdf.xacro")
    tf_prefix = LaunchConfiguration("tf_prefix", default='""')
    max_retries = LaunchConfiguration("max_retries", default="100")
    read_rate = LaunchConfiguration("read_rate", default="10")
    ftdi_id = LaunchConfiguration("ftdi_id", default="_")
    use_fake_mode = LaunchConfiguration("use_fake_mode", default="false")

    # ROBOT DESCRIPTIONS
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("tecnobody_workbench"),
                 "urdf", "robotiq_ft300.urdf.xacro"]
            ),
            " ",
            "name:=",
            "tecnobody_gazebo",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_mode:=",
            use_fake_mode,
            " ",
            "max_retries:=",
            max_retries,
            " ",
            "read_rate:=",
            read_rate,
            " ",
            "ftdi_id:=",
            ftdi_id,
            " ",
            "control_configuration:=",
            "robot_controller",
            " ",
            "simulated:=",
            "true",
            " ",
        ]
    )

    ft_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("tecnobody_workbench"),
                 "urdf", "robotiq_ft300.urdf.xacro"]
            ),
            " ",
            "name:=",
            "tecnobody_gazebo",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_mode:=",
            use_fake_mode,
            " ",
            "max_retries:=",
            max_retries,
            " ",
            "read_rate:=",
            read_rate,
            " ",
            "ftdi_id:=",
            ftdi_id,
            " ",
            "control_configuration:=",
            "ft_controller",
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    ft_robot_description = {"robot_description": ft_description_content}


    # CONTROLLER CONFIGURATIONS
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('tecnobody_workbench'),
            'config',
            'controllers.yaml',
        ]
    )

    ft_controllers = PathJoinSubstitution(
        [
            FindPackageShare('tecnobody_workbench'),
            'config',
            'ft_sensor_controllers.yaml',
        ]
    )

    # GAZEBO CONTROLLERS    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    ft_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[ft_robot_description],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=["-topic", "robot_description",
                   "-name", "cart", "-allow_renaming", "true"],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'scaled_trajectory_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    forward_command_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_command_controller'],
    )

    # FT CONTROLLERS
    ft_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[ft_controllers],
        output="both",
    )

    force_torque_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            "robotiq_force_torque_sensor_broadcaster",
        ],
    )

    ft_commands_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            "admittance_controller",
        ],
    )

    ft_commands_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
        ],
    )

    ft_commands_forwarder_node = Node(
        package='tecnobody_workbench_utils',
        executable='ft_forwarder',
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Declare Launch Arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("namespace", default_value="ft_sensor"))
    declared_arguments.append(DeclareLaunchArgument("max_retries", default_value="100"))
    declared_arguments.append(DeclareLaunchArgument("read_rate", default_value="10"))
    declared_arguments.append(DeclareLaunchArgument("ftdi_id", default_value=""))
    declared_arguments.append(
        DeclareLaunchArgument("frame_id", default_value="robotiq_ft_frame_id")
    )

    return LaunchDescription([

        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 empty.sdf'])]),
        
        # Event Handlers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[forward_command_controller_spawner, ft_commands_forwarder_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=forward_command_controller_spawner,
                on_exit=[ft_commands_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[LogInfo(msg=['Launch was asked to shutdown. Unspawning controllers...']),
                    clean_shutdown(namespace, robot_controllers, ft_controllers),
                    LogInfo(msg=['Killing remaining nodes']),
                    kill_nodes()]
            )
        ),

        # Nodes
        bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        ft_state_publisher_node,
        ft_control_node,
        force_torque_sensor_broadcaster_spawner,
        ft_commands_broadcaster_spawner,
        
        # Gazebo Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        
        # FT Sensor Launch Arguments
        DeclareLaunchArgument("namespace", default_value=namespace),
        DeclareLaunchArgument("description_package", default_value=description_package),
        DeclareLaunchArgument("description_file", default_value=description_file),
        DeclareLaunchArgument("tf_prefix", default_value=tf_prefix),
        DeclareLaunchArgument("max_retries", default_value=max_retries),
        DeclareLaunchArgument("read_rate", default_value=read_rate),
        DeclareLaunchArgument("ftdi_id", default_value=ftdi_id),
        DeclareLaunchArgument("use_fake_mode", default_value=use_fake_mode)
    ])
