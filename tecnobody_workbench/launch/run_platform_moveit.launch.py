from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchContext
import yaml

def load_yaml(package_name, file_path, context):
    package_share = FindPackageShare(package=package_name).find(package_name)
    file_path = PathJoinSubstitution([package_share, file_path]).perform(context)
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    context = LaunchContext()
    moveit_config_package = "fmrrehab_moveit_config"
    srdf_file = "tecnobody.srdf"
    ros2_controllers = 'ros2_controllers.yaml'
    launch_rviz = "true"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "urdf", 'tecnobody_real.urdf.xacro']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", ros2_controllers]
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
        output='screen',
    )

    ft_transformer = Node(
        package='ft_sensor_utils',
        executable='wrench_transformer',
        output='screen',
    )

    trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fmrrehab_controller'],
        output='screen',
    )

    spawner_admittance_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fmrrehab_admittance_controller"], #"--ros-args", "--log-level", "debug"],
        output="screen",
    )

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "urdf", srdf_file]),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics_real.yaml"]
    )

    robot_description_planning = load_yaml(moveit_config_package, "config/joint_limits.yaml", context)

    moveit_config = (
        MoveItConfigsBuilder("fmrrehab")
        .robot_description(file_path="urdf/tecnobody_real.urdf.xacro")
        .robot_description_semantic(file_path="urdf/tecnobody.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

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
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    controller_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing,
            on_exit=[spawner_admittance_controller, homing_done_publisher, eth_checker, ft_transformer, move_group_node],
        )
    )

    moveit_rviz_launcher = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=homing_done_publisher,
            on_exit=rviz_node
        )
    )

    ld = LaunchDescription()
    ld.add_action(ros2_control_node)
    ld.add_action(jsb)
    ld.add_action(ssb)
    ld.add_action(rsp)
    ld.add_action(fsb)
    ld.add_action(homing)
    ld.add_action(controller_launcher)
    # ld.add_action(moveit_rviz_launcher)

    return ld