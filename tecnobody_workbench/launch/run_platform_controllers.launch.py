from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler


def generate_launch_description():
    
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

    ld = LaunchDescription()
    ld.add_action(ssb)
    ld.add_action(homing_launcher)
    ld.add_action(controllers_launcher)

    return ld