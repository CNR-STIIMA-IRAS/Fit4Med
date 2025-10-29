from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():
    ros2_controllers = 'safemod_controllers.yaml'
    description_package = 'tecnobody_workbench'
    

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
        parameters=[robot_description]
    )

    ssb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['state_controller'],
        output='screen',
    )

    eth_checker = Node(
        package='tecnobody_workbench_utils',
        executable='ethercat_checker_node',
        name='tecnobody_ethercat_checker_node',
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

    joint_controller_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=remapping_controller_node,
            on_exit=[joint_controller_node],
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(ssb)
    ld.add_action(jsb)
    ld.add_action(rsp)
    ld.add_action(eth_checker)
    ld.add_action(remapping_controller_node)
    ld.add_action(joint_controller_launcher)
    return ld