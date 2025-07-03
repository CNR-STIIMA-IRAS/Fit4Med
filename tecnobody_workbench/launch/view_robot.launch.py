from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare 
from launch_ros.actions import Node

def generate_launch_description():
    description_package = 'tecnobody_workbench'
    tf_prefix = LaunchConfiguration("tf_prefix", default='""')
    max_retries = LaunchConfiguration("max_retries", default="100")
    read_rate = LaunchConfiguration("read_rate", default="10")
    ftdi_id = LaunchConfiguration("ftdi_id", default="_")
    use_fake_mode = LaunchConfiguration("use_fake_mode", default="false")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(description_package), "urdf",'platform_complete.config.urdf']),
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "robot_model.rviz"]
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description])
    
    jsp = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
        ],
    )
    
    ld = LaunchDescription()
    ld.add_action(rsp)
    ld.add_action(jsp)
    ld.add_action(rviz_node)
    return ld