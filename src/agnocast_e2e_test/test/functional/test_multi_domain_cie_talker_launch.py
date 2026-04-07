import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_test_description():
    prerun_node = launch_ros.actions.Node(
        package='agnocast_cie_thread_configurator',
        executable='prerun_node',
        name='prerun_node',
        output='screen',
        parameters=[{'domains': [0, 1]}],
        additional_env={'ROS_DOMAIN_ID': '0'}
    )

    container_domain_0 = ComposableNodeContainer(
        name='test_container_domain_0',
        namespace='',
        package='agnocast_components',
        executable='agnocast_component_container_cie',
        composable_node_descriptions=[
            ComposableNode(
                package='agnocast_e2e_test',
                plugin='agnocast_e2e_test::TestPublisherComponent',
                name='test_publisher_domain_0',
            ),
        ],
        output='screen',
        additional_env={'ROS_DOMAIN_ID': '0'}
    )

    container_domain_1 = ComposableNodeContainer(
        name='test_container_domain_1',
        namespace='',
        package='agnocast_components',
        executable='agnocast_component_container_cie',
        composable_node_descriptions=[
            ComposableNode(
                package='agnocast_e2e_test',
                plugin='agnocast_e2e_test::TestPublisherComponent',
                name='test_publisher_domain_1',
            ),
        ],
        output='screen',
        additional_env={'ROS_DOMAIN_ID': '1'}
    )

    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
            prerun_node,
            launch.actions.TimerAction(
                period=3.0,
                actions=[container_domain_0, container_domain_1]
            ),
            launch.actions.TimerAction(
                period=6.0,
                actions=[launch_testing.actions.ReadyToTest()]
            )
        ]),
        {
            'prerun_node': prerun_node,
            'container_domain_0': container_domain_0,
            'container_domain_1': container_domain_1,
        }
    )


class TestMultiDomainCieTalker(unittest.TestCase):

    def test_domain_0_publishes(self, proc_output, container_domain_0):
        proc_output.assertWaitFor(
            'Publishing:',
            timeout=10.0,
            process=container_domain_0
        )

    def test_domain_1_publishes(self, proc_output, container_domain_1):
        proc_output.assertWaitFor(
            'Publishing:',
            timeout=10.0,
            process=container_domain_1
        )

    def test_prerun_receives_callback_info_from_both_domains(self, proc_output, prerun_node):
        proc_output.assertWaitFor(
            'domain=0',
            timeout=10.0,
            process=prerun_node
        )
        proc_output.assertWaitFor(
            'domain=1',
            timeout=10.0,
            process=prerun_node
        )


@launch_testing.post_shutdown_test()
class TestMultiDomainCieTalkerShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

    @classmethod
    def tearDownClass(cls):
        template_yaml = os.path.join(os.path.expanduser("~"), "agnocast", "template.yaml")
        if os.path.exists(template_yaml):
            os.remove(template_yaml)
