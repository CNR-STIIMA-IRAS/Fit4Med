import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
            'address': LaunchConfiguration('address'),
            'websocket_ping_interval': '5.0',
            'websocket_ping_timeout': '10.0',
            'unregister_timeout': '2.0',
            'respawn': 'true',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'address',
            default_value='',
            description='Interface address for rosbridge. Empty listens on all interfaces.'
        ),
        rosbridge_launch
    ])
