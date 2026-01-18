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
    thread_configurator_node = launch_ros.actions.Node(
        package='cie_thread_configurator',
        executable='thread_configurator_node',
        name='thread_configurator_node',
        output='screen',
        arguments=['--prerun']
    )

    component_container = ComposableNodeContainer(
        name='test_component_container',
        namespace='',
        package='agnocastlib',
        executable='agnocast_component_container_cie',
        composable_node_descriptions=[
            ComposableNode(
                package='agnocastlib',
                plugin='agnocastlib_test::TestPublisherComponent',
                name='test_publisher_node',
            ),
            ComposableNode(
                package='agnocastlib',
                plugin='agnocastlib_test::TestSubscriptionComponent',
                name='test_subscription_node',
            )
        ],
        output='screen',
        parameters=[{'get_next_timeout_ms': 50}]
    )

    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
            thread_configurator_node,
            launch.actions.TimerAction(
                period=3.0,
                actions=[component_container]
            ),
            launch.actions.TimerAction(
                period=6.0,
                actions=[launch_testing.actions.ReadyToTest()]
            )
        ]),
        {
            'thread_configurator': thread_configurator_node,
            'component_container': component_container
        }
    )


class TestComponentContainerCIE(unittest.TestCase):

    def test_component_publishes(self, proc_output, component_container):
        proc_output.assertWaitFor(
            'Publishing:',
            timeout=10.0,
            process=component_container
        )

    def test_component_receives(self, proc_output, component_container):
        proc_output.assertWaitFor(
            'Received:',
            timeout=10.0,
            process=component_container
        )

    def test_thread_configurator_receives_callback_info(self, proc_output, thread_configurator):
        with launch_testing.asserts.assertSequentialStdout(proc_output, process=thread_configurator) as cm:
            output_text = "".join(cm._output)
            callback_info_count = output_text.count('Received CallbackGroupInfo:')

            # Total expected: 2 (not 3, because the callback group with `automatically_add_to_executor = false` should be skipped)
            self.assertEqual(
                callback_info_count, 2,
                f"Expected exactly 2 'Received CallbackGroupInfo:' messages, but got {callback_info_count}"
            )


@launch_testing.post_shutdown_test()
class TestComponentContainerCIEShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

    def test_cleanup(self):
        import os
        template_yaml = os.path.join(os.path.expanduser("~"), "agnocast", "template.yaml")
        if os.path.exists(template_yaml):
            os.remove(template_yaml)
