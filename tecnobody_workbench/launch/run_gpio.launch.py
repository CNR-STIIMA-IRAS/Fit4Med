from launch import LaunchDescription, substitutions
from launch.actions import DeclareLaunchArgument , RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT']="[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"

def generate_launch_description():
    controllers_file = 'gpio_only.yaml'
    description_package = 'tecnobody_workbench'
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'gpio_only.config.urdf']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )
        
    info_logger = DeclareLaunchArgument(
        'info_log_level',
        default_value='info',
        description='Logging level'
    )
    debug_logger = DeclareLaunchArgument(
        'debug_log_level',
        default_value='debug',
        description='Logging level'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[initial_joint_controllers],
        arguments=[],
#        prefix=['xterm -e gdb -ex run --args'],
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

    gpio_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gpio_command_controller'], # '--param-file', controllers_file],
        output='screen',
    )

    gpio_command_publisher = Node(
        package='tecnobody_workbench_utils',
        executable='gpio_command_publisher',
        output='screen',
    )

    gpio_publisher_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gpio_spawner,
            on_exit=[gpio_command_publisher],
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_entity(info_logger)
    #ld.add_entity(debug_logger)

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(rsp)
    ld.add_action(gpio_spawner)
    ld.add_action(gpio_publisher_launcher)

    return ld



