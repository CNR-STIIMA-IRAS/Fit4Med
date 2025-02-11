from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("tecnobody", package_name="fmrrehab_moveit_config").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)
    return ld
