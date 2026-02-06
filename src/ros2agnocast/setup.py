from setuptools import find_packages, setup

package_name = 'ros2agnocast'

setup(
    name=package_name,
    version='2.1.2',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    entry_points={
        'ros2topic.verb': [
            'list_agnocast = ros2agnocast.verb.list_agnocast:ListAgnocastVerb',
            'info_agnocast = ros2agnocast.verb.topic_info_agnocast:TopicInfoAgnocastVerb',
        ],
        'ros2node.verb': [
            'list_agnocast = ros2agnocast.verb.node_list_agnocast:ListAgnocastVerb',
            'info_agnocast = ros2agnocast.verb.node_info_agnocast:NodeInfoAgnocastVerb',
        ]
    }
)
