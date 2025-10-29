from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT']="[{severity}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"


import os
os.sched_setaffinity(0, {4})

def generate_launch_description():
    rosbridge_pkg = FindPackageShare('rosbridge_server').find('rosbridge_server')
    xml_launch = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(xml_launch)
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(rosbridge_launch)
    return ld



