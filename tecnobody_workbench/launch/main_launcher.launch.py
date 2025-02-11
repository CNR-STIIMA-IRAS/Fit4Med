from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    package_share = FindPackageShare(package=package_name).find(package_name)
    file_path = PathJoinSubstitution([package_share, file_path]).perform(None)
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    controllers_file = 'controllers.yaml'

    description_package = 'tecnobody_workbench'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'platform.config.urdf']),
            
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, initial_joint_controllers],
        output='screen',
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    ssb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['state_controller'],
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

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scaled_fmrrehab_controller'],
        output='screen',
    )

    controller_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[trajectory_controller_node, homing_done_publisher],
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(jsb)
    ld.add_action(ssb)
    ld.add_action(homing)
    ld.add_action(controller_launcher)
    ld.add_action(rsp)

    return ld