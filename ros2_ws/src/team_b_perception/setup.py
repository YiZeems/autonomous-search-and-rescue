from setuptools import setup

package_name = 'team_b_perception'

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
    description='Victim registry (AprilTag detections → map frame via TF2) — IA712 Project B.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'victim_registry = team_b_perception.victim_registry_node:main',
        ],
    },
)
