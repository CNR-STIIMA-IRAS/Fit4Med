from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction
from moveit_configs_utils import MoveItConfigsBuilder
import threading

# Funzione per lanciare il nodo che pubblica il flag
def launch_status_node(context, *args, **kwargs):
    import rclpy
    from tecnobody_workbench_utils.launch_status_publisher import LaunchStatusPublisher

    def run():
        rclpy.init()
        node = LaunchStatusPublisher()
        context._launch_status_node = node
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    
# Funzione per cambiare lo stato a True
def set_node_names(context, *args, **kwargs):
    node = getattr(context, "_launch_status_node", None)
    if node:
        node.get_logger().info(f"Setting launch status {args}")
        node.set_ready(args)
    return []

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
    joint_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "joint_trajectory_controller",
        ],
    )
    return [
        joint_state_broadcaster_unspawner,
        state_controller_unspawner,
        joint_controller_unspawner
    ]


def generate_launch_description():
    ros2_controllers = 'safemod_controllers.yaml'
    description_package = 'tecnobody_workbench'
    
    nodes_names = []

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf",'platform_complete_PLC.config.urdf']),
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
    nodes_names.append('controller_manager')

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
    nodes_names.append('robot_state_publisher')
    
    gpio_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['PLC_controller'],
        output='screen',
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
        arguments=['MODE_CYCLIC_SYNC_POSITION'],
        output='screen',
    )
    nodes_names.append('homing_node')

    homing_done_publisher = Node(
        package='tecnobody_workbench_utils',
        executable='homing_completion_publisher',
        output='screen',
    )

    eth_checker = Node(
        package='tecnobody_workbench_utils',
        executable='ethercat_checker_node',
        name='tecnobody_ethercat_checker_node',
        output='screen',
    )
    nodes_names.append('tecnobody_ethercat_checker_node')

    ft_offset_updater = Node(
        package='tecnobody_workbench_utils',
        executable='ft_offset_updater',
        output='screen',
    )

    joint_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    forward_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller', '--inactive'],
        output='screen',
    )

    forward_pos_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
        output='screen',
    )

    admittance_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['admittance_controller', '--inactive'],
        output='screen',
    )

    homing_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[homing, jsb, fsb],
        )
    )
    
    controllers_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[joint_controller_node, 
                     forward_controller_node,
                     forward_pos_controller_node,
                     admittance_controller_node, 
                     homing_done_publisher, 
                     eth_checker],
        )
    )

    node_names_launcher = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_controller_node,
            on_exit=[
                OpaqueFunction(function=set_node_names, args=nodes_names),
            ],
        )
    )
    
    controller_unspawner = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[LogInfo(msg=['Launch was asked to shutdown. Unspawning controllers...']),
                clean_shutdown()] # type: ignore
        )
    )

    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(gpio_spawner)
    ld.add_action(ssb)
    ld.add_action(rsp)
    ld.add_action(homing_launcher)
    ld.add_action(controllers_launcher)
    ld.add_action(node_names_launcher)
    ld.add_action(controller_unspawner)

    return ld