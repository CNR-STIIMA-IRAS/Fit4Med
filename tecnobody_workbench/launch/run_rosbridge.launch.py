import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():

    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    xml_launch = os.path.join(
        rosbridge_pkg,
        'launch',
        'rosbridge_websocket_launch.xml'
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(xml_launch),
        launch_arguments={
            'port': '9090',
            'address': '192.168.1.1',
            'websocket_ping_interval': '5.0',
            'websocket_ping_timeout': '10.0',
            'unregister_timeout': '2.0',
            'respawn': 'true',
        }.items()
    )

    return LaunchDescription([
        rosbridge_launch
    ])