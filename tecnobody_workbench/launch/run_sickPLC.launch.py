from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT']="[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"

def generate_launch_description():
    controllers_file = 'plc_controller.yaml'
    description_package = 'tecnobody_workbench'
    
    DeclareLaunchArgument(
            'gui_ip',
            default_value='127.0.0.0',
            description='IP of the GUI'
    ),
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'sickPLC.config.urdf']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", controllers_file]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='plc_controller_manager',
        remappings=[('robot_description', 'plc_robot_description')],
        arguments=[],
        parameters=[initial_joint_controllers],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='plc_state_publisher',
        remappings=[('robot_description', 'plc_robot_description')],
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
        output = 'screen',
        arguments=[LaunchConfiguration('gui_ip', default='127.0.0.0')],
    )

    plc_manager_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=plc_controller_spawner,
            on_exit=[plc_manager],
        )
    )

    sonar_teach_node = Node(
        package='tecnobody_workbench_utils',
        executable='sonar_teach_node',
        name='sonar_teach_node',
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # nodes_to_start
    ld.add_action(ros2_control_node)
    ld.add_action(rsp)
    ld.add_action(plc_controller_spawner)
    # ld.add_action(plc_manager_launcher)
    # ld.add_action(sonar_teach_node)
    return ld



