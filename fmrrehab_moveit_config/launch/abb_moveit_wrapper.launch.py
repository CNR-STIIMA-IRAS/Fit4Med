from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
# from nav2_common.launch import RewrittenYaml
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import yaml
import os
import string
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterValue

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def moveit_cpp_node_builder(context: LaunchContext):
    abb_ns = context.perform_substitution(LaunchConfiguration('namespace'))
    


    urdf_xacro_file = os.path.join(get_package_share_directory('abb_description'), 'robots',
                                     'irb4600.urdf.xacro')

    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', urdf_xacro_file, ' arm_id:=', abb_ns])


    robot_description = {"robot_description": ParameterValue(robot_description_config)}


    srdf_xacro_file = os.path.join(get_package_share_directory('abb_moveit_config'), 'srdf',
                                     'abb.srdf.xacro')

    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', srdf_xacro_file, ' prefix:=', abb_ns])



    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_config)
    }

    kinematics_yaml = load_yaml(
        "abb_moveit_config", "config/kinematics.yaml"
    )

    kinematic_plugin = {"robot_description_kinematics": kinematics_yaml}


    planning_yaml = load_yaml(
        "abb_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning = {"ompl": planning_yaml}

    joint_limit_yaml = load_yaml(
        "abb_moveit_config", "config/joint_limits_dx.yaml"
    )
    
    cartesian_limit_yaml = load_yaml(
        "abb_moveit_config", "config/pilz_cartesian_limits.yaml"
    )
    moveit_cpp_yaml = load_yaml(
        "abb_moveit_config", "config/moveit_cpp_dx.yaml"
    )

    robot_description_planning = {"robot_description_planning": joint_limit_yaml}
    robot_description_planning["robot_description_planning"].update(cartesian_limit_yaml)

    controller_yaml = load_yaml(
        "abb_moveit_config", "config/moveit_controllers_dx.yaml"
    )



    node_list = []

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning,
            kinematic_plugin,
            robot_description_planning,
            controller_yaml,
            moveit_cpp_yaml,

        ],
    )
    node_list.append(rviz_node)

    return node_list


def generate_launch_description():

    # get namespaced parameters
    abb_bringup_params = DeclareLaunchArgument(
        "namespace",
        default_value="abb",
        description="namespace of the robot",
    )



    return LaunchDescription(
        [
            abb_bringup_params,
            OpaqueFunction(function=moveit_cpp_node_builder),
        ]
    )