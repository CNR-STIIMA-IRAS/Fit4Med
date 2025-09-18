import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tecnobody_workbench_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('tecnobody_workbench_utils/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pauli',
    maintainer_email='pauli@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'homing_node = tecnobody_workbench_utils.homing:main',
            'boot_hw = tecnobody_workbench_utils.boot_hw:main',
            'ethercat_checker_node = tecnobody_workbench_utils.eth_checker:main',
            'ethercat_checker_no_try_turn_on_node = tecnobody_workbench_utils.eth_checker_no_try_turn_on:main',
            'homing_completion_publisher = tecnobody_workbench_utils.homing_completion_publisher:main',
            'trajectory_action_client = tecnobody_workbench_utils.send_trajectory:main',
            'ft_forwarder = tecnobody_workbench_utils.ft_forwarder:main',
            'ft_offset_updater = tecnobody_workbench_utils.ft_offset_updater:main',
            'gpio_command_publisher = tecnobody_workbench_utils.gpio_command:main',
            'ros_controllers_checker = tecnobody_workbench_utils.check_active_controllers:main',
            'fake_fjt = tecnobody_workbench_utils.fake_follow_joint_trajectory:main',
            'launch_status_publisher = tecnobody_workbench_utils.launch_status_publisher',
            'apriltag_azure_detector = tecnobody_workbench_utils.apriltag_azure_detector:main',
            'extract_bags = tecnobody_workbench_utils.extract_bags:main',
            'fct_manager_node = tecnobody_workbench_utils.gui_trajectory_manager:main'
        ],
    },
)
