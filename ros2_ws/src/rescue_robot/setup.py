import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rescue_robot'


def files(pattern):
    """Return only files for setup(data_files), never directories."""
    return [path for path in glob(pattern, recursive=True) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), files('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), files('rviz/*')),
        (os.path.join('share', package_name, 'behavior_trees'), files('behavior_trees/*')),
        (os.path.join('share', package_name, 'test_data'), files('test_data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IA712 Search and Rescue Team',
    maintainer_email='yimou.zhang@telecom-paris.fr',
    description='Autonomous search and rescue robot nodes — exploration, perception, results, BT.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer_node = rescue_robot.exploration.frontier_explorer_node:main',
            'victim_detector_node = rescue_robot.detection.victim_detector_node:main',
            'coverage_evaluator_node = rescue_robot.results.coverage_evaluator_node:main',
            'result_exporter_node = rescue_robot.results.result_exporter_node:main',
            'rviz_marker_node = rescue_robot.results.rviz_marker_node:main',
            'mock_map_publisher = rescue_robot.mocks.mock_map_publisher:main',
            'mock_victim_publisher = rescue_robot.mocks.mock_victim_publisher:main',
            'mock_coverage_publisher = rescue_robot.mocks.mock_coverage_publisher:main',
            'bt_supervisor_node = rescue_robot.bt.bt_supervisor_node:main',
            'waypoint_follower_node = rescue_robot.navigation.waypoint_follower_node:main',
            'tf_relay_node = rescue_robot.utils.tf_relay_node:main',
            'cmd_vel_relay_node = rescue_robot.utils.cmd_vel_relay_node:main',
        ],
    },
)
