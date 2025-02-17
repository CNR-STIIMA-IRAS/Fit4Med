from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchContext, LaunchDescription
import yaml


def load_yaml(package_name, file_path, context):
    package_share = FindPackageShare(package=package_name).find(package_name)
    file_path = PathJoinSubstitution([package_share, file_path]).perform(context)
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    context = LaunchContext()
    moveit_config_package = "fmrrehab_moveit_config"
    description_package = "tecnobody.urdf.xacro"
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")
    
    # Robot description and semantic params
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", 'tecnobody.urdf.xacro']),
            
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_description_planning = load_yaml(moveit_config_package, "config/joint_limits.yaml", context)

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "urdf", "tecnobody.srdf"]),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # ROS2 control
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", 'ros2_controllers.yaml']
    )
    
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, initial_joint_controllers],
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
        parameters=[robot_description])
    
    ros2_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fmrrehab_controller'],
        output='screen',
    )

    # MoveIt!
    moveit_config = (
        MoveItConfigsBuilder("fmrrehab")
        .robot_description(
            file_path="urdf/tecnobody_real.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="urdf/tecnobody.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            load_all=False
        )
        .to_moveit_configs()
    )
    
    moveit_config_dict = moveit_config.to_dict()
        
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    #Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    
    return LaunchDescription(
        [
            ros2_control_hardware_type,
            rviz_node,
            ros2_control_node,
            rsp,
            jsb,
            move_group_node,
            ros2_controller,
            DeclareLaunchArgument("launch_rviz", default_value=launch_rviz)
        ]
    )
