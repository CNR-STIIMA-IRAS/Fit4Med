from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    fake_follow_joint_trajectory = Node(
        package="tecnobody_workbench_utils",
        executable="fake_fjt",
        output="screen",
    )
    
    rehab_gui = Node(
        package="rehab_gui",
        executable="rehab_gui",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(fake_follow_joint_trajectory)
    ld.add_action(rehab_gui)

    return ld