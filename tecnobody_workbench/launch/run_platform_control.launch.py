from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction, DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import threading

import os
os.sched_setaffinity(0, {3,7})

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
            "ft_sensor_command_broadcaster"
        ],
    )
    joint_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "joint_trajectory_controller"
            # 'scaled_joint_trajectory_controller'
        ],
    )
    forward_pos_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "forward_position_controller"
        ],
    )
    forward_vel_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "forward_velocity_controller"
        ],
    )
    admittance_controller_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=[
            "admittance_controller"
        ],
    )
    return LaunchDescription([
        joint_state_broadcaster_unspawner,
        state_controller_unspawner,
        force_torque_sensor_broadcaster_unspawner,
        joint_controller_unspawner,
        forward_pos_controller_unspawner,
        forward_vel_controller_unspawner,
        admittance_controller_unspawner
    ])


def generate_launch_description():
    ros2_controllers = 'safemod_controllers.yaml'
    description_package = 'tecnobody_workbench'
    DeclareLaunchArgument(
            'perform_homing',
            default_value='false',
            description='Se true lancia il nodo extra'
    ),

    
    nodes_names = []

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

    fsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_sensor_command_broadcaster'],
        output='screen',
    )

    homing = Node(
        package='tecnobody_workbench_utils',
        executable='boot_hw',
        arguments=['MODE_CYCLIC_SYNC_POSITION'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('perform_homing'))
    )

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

    ft_offset_updater = Node(
        package='tecnobody_workbench_utils',
        executable='ft_offset_updater',
        output='screen',
    )

    joint_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            # 'joint_trajectory_controller'
            'scaled_trajectory_controller'
        ],
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

    fct_manager_node = Node(
        package='tecnobody_workbench_utils',
        executable='fct_manager_node',
        arguments=['scaled_trajectory_controller']
    )
    
    group1 = GroupAction(
                actions=[
                    joint_controller_node,
                    forward_controller_node,
                    forward_pos_controller_node,
                    admittance_controller_node,
                    homing_done_publisher,
                    eth_checker
                ],
                condition=IfCondition(LaunchConfiguration('perform_homing'))
            )
    
    group2 = GroupAction(
                actions=[
                    joint_controller_node,
                    forward_controller_node,
                    forward_pos_controller_node,
                    admittance_controller_node,
                    homing_done_publisher,
                    eth_checker
                ],
                condition=UnlessCondition(LaunchConfiguration('perform_homing'))
            )

    controllers_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[group1],
        )
    )

    controllers_launcher_no_homing = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ssb,
            on_exit=[group2],
        )
    )

    fct_manager_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_controller_node,
            on_exit=[fct_manager_node],
        )
    )

    rosbridge_pkg = FindPackageShare('rosbridge_server').find('rosbridge_server')
    xml_launch = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(xml_launch)
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
    ld.add_action(ssb)
    ld.add_action(rsp)
    ld.add_action(homing_launcher)
    ld.add_action(controllers_launcher)
    ld.add_action(controller_unspawner)
    ld.add_action(controllers_launcher_no_homing)
    ld.add_action(fct_manager_launcher)
    ld.add_action(rosbridge_launch)
    return ld