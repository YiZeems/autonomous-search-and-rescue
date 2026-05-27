from setuptools import setup

package_name = 'team_b_metrics'

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
    description='Coverage evaluator + benchmark runner (IA712 Project B bonus).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'coverage_evaluator = team_b_metrics.coverage_evaluator_node:main',
            # 'benchmark_runner = team_b_metrics.benchmark_runner:main',
        ],
    },
)
