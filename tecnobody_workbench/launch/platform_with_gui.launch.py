import launch
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnShutdown


def generate_launch_description():
    # Check if there are alive ros2 nodes from previous sessions
    kill_remote_nodes = ExecuteProcess(
        cmd=[
            'ssh', 'pauli@192.168.46.101',
            '/home/pauli/scripts/kill_ros2_nodes.sh'
        ],
        shell=True,
        output='screen'
    )

    # Launch remote control nodes via SSH on remote PC
    remote_node_launch = ExecuteProcess(
    cmd=[
        'ssh', 'pauli@192.168.46.101', 
        '"echo Remote connection successful && sudo /etc/init.d/ethercat start && source /home/pauli/ros2_ws/install/setup.bash && ros2 launch tecnobody_workbench run_platform_moveit.launch.py"'
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
    ld.add_action(kill_remote_nodes)
    ld.add_action(remote_launcher)
    ld.add_action(gui_launcher)
    return ld
