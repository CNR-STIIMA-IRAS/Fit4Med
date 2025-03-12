import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnShutdown

def generate_launch_description():
    # Declare the launch arguments
    ssh_username_arg = DeclareLaunchArgument(
        'ssh_username',
        default_value='pauli',
        description='Username for SSH connection'
    )
    ssh_address_arg = DeclareLaunchArgument(
        'ssh_address',
        default_value='192.168.46.101',
        description='Address for SSH connection'
    )

    # Get the launch configurations
    ssh_username = LaunchConfiguration('ssh_username')
    ssh_address = LaunchConfiguration('ssh_address')

    # Check if there are alive ros2 nodes from previous sessions
    kill_remote_nodes = ExecuteProcess(
        cmd=[
            'ssh', [ssh_username, '@', ssh_address],
            '/home/', ssh_username, '/projects/ros2_ws/src/Fit4Med/bash_scripts/kill_ros2_nodes.sh'
        ],
        shell=True,
        output='screen'
    )

    # Launch remote control nodes via SSH on remote PC
    remote_node_launch = ExecuteProcess(
        cmd=[
            'ssh', [ssh_username, '@', ssh_address],
            '"echo Remote connection successful && sudo /etc/init.d/ethercat start && source /home/', ssh_username, '/ros2_ws/install/setup.bash && ros2 launch tecnobody_workbench run_platform_moveit.launch.py"'
        ],
        shell=True,
        output='screen'
    )

    # Check the completion of homing procedure before triggering the GUI node
    homing_listener = Node(
        package='check_homing_done',
        executable='homing_done_listener',
        output='screen',
    )

    remote_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=kill_remote_nodes,
            on_exit=[remote_node_launch, homing_listener],
        )
    )

    # Node for GUI that should start after homing completion
    gui_node = Node(
        package='rehab_gui',
        executable='rehab_gui',
        output='screen',
    )

    gui_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing_listener,
            on_exit=[gui_node],
        )
    )

    # Create the launch description and add actions
    ld = launch.LaunchDescription()
    ld.add_action(ssh_username_arg)
    ld.add_action(ssh_address_arg)
    ld.add_action(kill_remote_nodes)
    ld.add_action(remote_launcher)
    ld.add_action(gui_launcher)
    return ld
