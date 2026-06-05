from setuptools import find_packages, setup

package_name = 'ros2agnocast'

setup(
    name=package_name,
    version='2.3.4',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    package_data={
        'ros2agnocast.templates': [
            '*.em',
        ],
        'ros2agnocast.static_sources': [
            '*.hpp',
            '*.cpp',
        ],
    },
    extras_require={'test': ['pytest']},
    entry_points={
        'ros2cli.command': [
            'agnocast = ros2agnocast.command.agnocast:AgnocastCommand',
        ],
        'ros2agnocast.verb': [
            'bridge-daemon-status = ros2agnocast.verb.agnocast_bridge_daemon_status:BridgeDaemonStatusVerb',
            'generate-bridge-plugins = ros2agnocast.verb.agnocast_generate_bridge_plugins:GenerateBridgePluginsVerb',
            'version = ros2agnocast.verb.agnocast_version:VersionVerb',
            'discovery-daemon-status = ros2agnocast.verb.agnocast_discovery_daemon_status:DiscoveryDaemonStatusVerb',
        ],
        'ros2topic.verb': [
            'list_agnocast = ros2agnocast.verb.topic_list_agnocast:ListAgnocastVerb',
            'info_agnocast = ros2agnocast.verb.topic_info_agnocast:TopicInfoAgnocastVerb',
            'hz_agnocast = ros2agnocast.verb.topic_hz_agnocast:TopicHzAgnocastVerb',
            'echo_agnocast = ros2agnocast.verb.topic_echo_agnocast:TopicEchoAgnocastVerb',
            'delay_agnocast = ros2agnocast.verb.topic_delay_agnocast:TopicDelayAgnocastVerb',
        ],
        'ros2node.verb': [
            'list_agnocast = ros2agnocast.verb.node_list_agnocast:ListAgnocastVerb',
            'info_agnocast = ros2agnocast.verb.node_info_agnocast:NodeInfoAgnocastVerb',
        ],
        'ros2bag.verb': [
            'record_agnocast = ros2agnocast.verb.bag_record_agnocast:BagRecordAgnocastVerb',
        ],
    },
)
