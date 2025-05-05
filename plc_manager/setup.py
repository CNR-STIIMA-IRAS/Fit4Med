import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'plc_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('plc_manager/config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('plc_manager/launch/*.launch.py')),  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adriano',
    maintainer_email='adriano@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plc_manager_node = plc_manager.plc_manager:main',
        ],
    },
)
