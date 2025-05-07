from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    controllers_file = 'sick_PLC.yaml'
    description_package = 'tecnobody_workbench'
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'sickPLC.config.urdf']),
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}

    initial_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='plc_controller_manager',
        remappings=[('/robot_description', '/plc_description')],
        arguments=[],
        parameters=[initial_controllers],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='plc_state_publisher',
        remappings=[('/robot_description', '/plc_description')],
        output='screen',
        parameters=[robot_description]
    )

    plc_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['PLC_controller', '-c', '/plc_controller_manager'],
        output='screen',
    )
    
    plc_manager = Node(
        package='plc_manager',
        executable='plc_manager_node',
        output='screen',
    )
    
    plc_manager_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=plc_controller_spawner,
            on_exit=[plc_manager],
        )
    )
     
    # Create the launch description and populate
    ld = LaunchDescription()

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(rsp)
    ld.add_action(plc_controller_spawner)
    ld.add_action(plc_manager_launcher)

    return ld
