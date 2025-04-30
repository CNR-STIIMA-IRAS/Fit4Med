from numpy import add
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_package = "tecnobody_workbench"
    ros2_controllers = 'controllers.yaml'
    launch_rviz = "true"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'tecnobody_fake.urdf.xacro']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ros2_controllers]
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

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])
    

    trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )
    
    controller_logger = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['controller_logger'],
        output='screen',
    )
    
    logger_launcher = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=trajectory_controller_node,
                on_exit=[controller_logger],
            )
        )
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "robot_model.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(launch_rviz),
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
        ],
    )

    rehab_gui = Node(
        package="rehab_gui",
        executable="rehab_gui",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(ros2_control_node)
    ld.add_action(jsb)
    ld.add_action(rsp)
    ld.add_action(trajectory_controller_node)
    ld.add_action(rviz_node)
    ld.add_action(logger_launcher)

    return ld
