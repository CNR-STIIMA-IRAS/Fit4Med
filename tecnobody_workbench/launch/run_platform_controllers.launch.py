from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    controllers_file = 'controllers.yaml'
    description_package = 'tecnobody_workbench'
    chosen_controller = 'scaled_trajectory_controller'

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


    velocity_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[chosen_controller],
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

    urdf_launch_package = FindPackageShare('urdf_launch')
    default_rviz_config_path = os.path.join(get_package_share_directory(description_package), 'rviz', 'platform_facsimile_simulation.rviz')

    rviz_config = DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file')

    urdf_launch_description = IncludeLaunchDescription(
        PathJoinSubstitution([urdf_launch_package, 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'tecnobody_workbench',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'platform_complete.urdf'])}.items()
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # declared_arguments

    # nodes_to_start
    ld.add_action(rsp)
    ld.add_action(velocity_controller_node)
    ld.add_action(rviz_config)
    ld.add_action(urdf_launch_description)
    ld.add_action(rviz)
  
    return ld
