from setuptools import setup

package_name = 'ros2agnocast_discovery_agent'

setup(
    name=package_name,
    version='2.3.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/discovery_agent.launch.xml']),
        ('lib/' + package_name, ['scripts/discovery_agent']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={'test': ['pytest']},
    maintainer='Keita Morisaki, Takahiro Ishikawa-Aso, Koichi Imai, Takumi Jin',
    maintainer_email=(
        'keita.morisaki@tier4.jp, sykwer@gmail.com, '
        'koichi.imai.2@tier4.jp, primenumber_2_3_5@yahoo.co.jp'
    ),
    description='Per-IPC-namespace daemon publishing Agnocast state on '
                '/_agnocast_discovery for cross-NS/ECU observability and bridge generation.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'discovery_agent = ros2agnocast_discovery_agent.agent:main',
        ],
    },
)
