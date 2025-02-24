from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.actions import RegisterEventHandler, LogInfo
from moveit_configs_utils import MoveItConfigsBuilder


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
            "ft_sensor_command_broadcaster",
        ],
    )
    joint_trajectory_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "joint_trajectory_controller",
        ],
    )
    gpio_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "gpio_command_controller",
        ],
    )
    return LaunchDescription([
        joint_state_broadcaster_unspawner,
        state_controller_unspawner,
        force_torque_sensor_broadcaster_unspawner,
        joint_trajectory_controller_unspawner,
        gpio_controller_unspawner
    ])


def generate_launch_description():
    ros2_controllers = 'safemod_controllers.yaml'
    description_package = 'tecnobody_workbench'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf",'tecnobody_single_safemod.config.urdf']),
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
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    #PROVA
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    

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

    homing = Node(
        package='tecnobody_workbench_utils',
        executable='homing_node',
        output='screen',
    )

    homing_done_publisher = Node(
        package='tecnobody_workbench_utils',
        executable='homing_completion_publisher',
        output='screen',
    )

    eth_checker = Node(
        package='tecnobody_workbench_utils',
        executable='ethercat_checker_node',
        output='screen',
    )

    trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    gpio_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gpio_command_controller', '--param-file', initial_joint_controllers],
        output='screen',
    )

    controller_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[trajectory_controller_node, homing_done_publisher, eth_checker],
        )
    )

    controller_unspawner = RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[LogInfo(msg=['Launch was asked to shutdown. Unspawning controllers...']),
                    clean_shutdown()]
            )
        ),

    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(jsb)
    ld.add_action(ssb)
    ld.add_action(gpio_controller_node)
    ld.add_action(rsp)
    ld.add_action(fsb)
    ld.add_action(homing)
    ld.add_action(controller_launcher)
    ld.add_action(controller_unspawner)

    return ld