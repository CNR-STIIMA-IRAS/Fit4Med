from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = 'safemod_controllers.yaml'

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

    joint_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
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

    # Create the launch description and populate
    ld = LaunchDescription()

    # declared_arguments

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(jsb)
    ld.add_action(rsp)
    ld.add_action(joint_controller_node)

    return ld
