from setuptools import setup

package_name = 'team_b_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julien GIMENEZ, Hugo FANCHINI, Paul CINTRA, Yimou ZHANG',
    maintainer_email='julien.gimenez@telecom-paris.fr',
    description='Frontier-based and information-gain exploration nodes (IA712 Project B).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'frontier_greedy = team_b_exploration.frontier_greedy_node:main',
            # 'information_gain = team_b_exploration.information_gain_node:main',
        ],
    },
)
