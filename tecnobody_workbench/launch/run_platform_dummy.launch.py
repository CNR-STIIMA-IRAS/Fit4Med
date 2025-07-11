from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
import threading

# Funzione per lanciare il nodo che pubblica il flag
def launch_status_node(context, *args, **kwargs):
    import rclpy
    from tecnobody_workbench_utils.launch_status_publisher import LaunchStatusPublisher

    def run():
        rclpy.init()
        node = LaunchStatusPublisher()
        context._launch_status_node = node
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()

# Funzione per cambiare lo stato a True
def set_node_names(context, *args, **kwargs):
    node = getattr(context, "_launch_status_node", None)
    if node:
        node.get_logger().info(f"Setting launch status {args}")
        node.set_ready(args)
    return []

def generate_launch_description():
    description_package = "tecnobody_workbench"
    ros2_controllers = 'controllers.yaml'
    launch_rviz = "true"
    
    nodes_names = []

    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", 'tecnobody_fake.urdf.xacro']),
    ])
    robot_description = {'robot_description': robot_description_content}

    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare(description_package), "config", ros2_controllers
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, initial_joint_controllers],
        namespace='tecnobody_fake_hardware',
        output='screen',
        name='tecnobody_controller_manager',
    )
    nodes_names.append('tecnobody_controller_manager')
    
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        namespace='tecnobody_fake_hardware',
        arguments=['joint_state_broadcaster', '-c', '/tecnobody_fake_hardware/tecnobody_controller_manager'],
        output='screen',
    )
    
    fmrrehab_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace='tecnobody_fake_hardware',
        arguments=['joint_trajectory_controller', '-c', '/tecnobody_fake_hardware/tecnobody_controller_manager'],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='tecnobody_fake_hardware',
        parameters=[robot_description],
        name='tecnobody_state_publisher',
    )
    nodes_names.append('tecnobody_state_publisher')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare(description_package), "config", "robot_model.rviz"
        ])],
        namespace='tecnobody_fake_hardware',
        parameters=[robot_description],
    )
    nodes_names.append("rviz2_moveit")

    rsp_launcher = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[rsp],
        )
    )

    controllers_launcher = RegisterEventHandler(
        OnProcessStart(
            target_action=rsp,
            on_start=[jsb, fmrrehab_controller],
        )
    )

    rviz_launcher = RegisterEventHandler(
        OnProcessExit(
            target_action=fmrrehab_controller,
            on_exit=[
                rviz_node,
                OpaqueFunction(function=set_node_names, args=nodes_names),
            ],
        )
    )

    return LaunchDescription([
        OpaqueFunction(function=launch_status_node),  # Avvia il publisher
        ros2_control_node,
        rsp_launcher,
        controllers_launcher,
        rviz_launcher,
    ])
