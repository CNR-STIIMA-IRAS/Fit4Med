from launch import LaunchDescription
from launch.actions import EmitEvent, TimerAction
from launch.events import Shutdown
from launch_ros.actions import Node
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    plc_manager_node = Node(
        package='plc_manager',
        executable='plc_manager_node',
        name='plc_manager_node',
        output='screen',
        additional_env={'PLC_MANAGER_SKIP_ETHERCAT': '1'},
        arguments=['127.0.0.1'],
    )

    return (
        LaunchDescription([
            plc_manager_node,
            launch_testing.actions.ReadyToTest(),
            TimerAction(period=3.0, actions=[EmitEvent(event=Shutdown())]),
        ]),
        {'plc_manager_node': plc_manager_node},
    )


def test_plc_manager_process_exits_cleanly(proc_info, plc_manager_node):
    """Ensure the plc_manager node can start and shut down via the launch timer."""
    proc_info.assertWaitForShutdown(process=plc_manager_node, timeout=5.0)
