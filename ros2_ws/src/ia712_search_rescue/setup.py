import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ia712_search_rescue'


def files(pattern):
    """Return only files for setup(data_files), never directories."""
    return [path for path in glob(pattern, recursive=True) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), files('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), files('worlds/*')),
        (os.path.join('share', package_name, 'models'), files('models/**/*')),
        (os.path.join('share', package_name, 'maps'), files('maps/*')),
        (os.path.join('share', package_name, 'rviz'), files('rviz/*')),
        (os.path.join('share', package_name, 'behavior_trees'), files('behavior_trees/*')),
        (os.path.join('share', package_name, 'test_data'), files('test_data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IA712 Team',
    maintainer_email='team@example.com',
    description='IA712 Project B - Autonomous Search and Rescue.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer_node = ia712_search_rescue.exploration.frontier_explorer_node:main',
            'victim_detector_node = ia712_search_rescue.detection.victim_detector_node:main',
            'coverage_evaluator_node = ia712_search_rescue.results.coverage_evaluator_node:main',
            'result_exporter_node = ia712_search_rescue.results.result_exporter_node:main',
            'rviz_marker_node = ia712_search_rescue.results.rviz_marker_node:main',
            'mock_map_publisher = ia712_search_rescue.mocks.mock_map_publisher:main',
            'mock_victim_publisher = ia712_search_rescue.mocks.mock_victim_publisher:main',
            'mock_coverage_publisher = ia712_search_rescue.mocks.mock_coverage_publisher:main',
            'bt_supervisor_node = ia712_search_rescue.bt.bt_supervisor_node:main',
        ],
    },
)
