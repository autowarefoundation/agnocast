import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts


def generate_test_description():
    thread_configurator_node = launch_ros.actions.Node(
        package='agnocast_cie_thread_configurator',
        executable='thread_configurator_node',
        name='thread_configurator_node',
        output='screen',
        arguments=['--prerun']
    )

    # Standalone executable with CallbackIsolatedAgnocastExecutor
    test_cie_publisher = launch_ros.actions.Node(
        package='agnocast_e2e_test',
        executable='test_cie_publisher',
        name='test_cie_publisher',
        output='screen',
    )

    # Standalone executable with CallbackIsolatedAgnocastExecutor
    test_cie_subscriber = launch_ros.actions.Node(
        package='agnocast_e2e_test',
        executable='test_cie_subscriber',
        name='test_cie_subscriber',
        output='screen',
    )

    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
            thread_configurator_node,
            launch.actions.TimerAction(
                period=3.0,
                actions=[test_cie_publisher]
            ),
            launch.actions.TimerAction(
                period=4.0,
                actions=[test_cie_subscriber]
            ),
            launch.actions.TimerAction(
                period=8.0,
                actions=[launch_testing.actions.ReadyToTest()]
            )
        ]),
        {
            'thread_configurator': thread_configurator_node,
            'test_cie_publisher': test_cie_publisher,
            'test_cie_subscriber': test_cie_subscriber,
        }
    )


class TestCallbackIsolatedExecutor(unittest.TestCase):

    def test_publisher_outputs(self, proc_output, test_cie_publisher):
        proc_output.assertWaitFor(
            'Publishing:',
            timeout=10.0,
            process=test_cie_publisher
        )

    def test_subscriber_outputs(self, proc_output, test_cie_subscriber):
        proc_output.assertWaitFor(
            'Received:',
            timeout=10.0,
            process=test_cie_subscriber
        )

    def test_thread_configurator_receives_callback_info(self, proc_output, thread_configurator):
        # Wait for at least one CallbackGroupInfo to ensure output is available
        proc_output.assertWaitFor(
            'Received CallbackGroupInfo:',
            timeout=10.0,
            process=thread_configurator
        )

        # assertWaitFor returns on first match; give time for the second message to arrive
        import time
        time.sleep(2)

        filtered_output_text = "".join(
            line
            for output in proc_output[thread_configurator]
            for line in output.text.decode('utf-8').splitlines(keepends=True)
            if 'agnocast_bridge_node' not in line
        )
        callback_info_count = filtered_output_text.count('Received CallbackGroupInfo:')

        # Expected: 2 (1 default group from publisher + 1 default group from subscriber;
        # the publisher's callback group with automatically_add_to_executor=false is skipped)
        self.assertEqual(
            callback_info_count, 2,
            f"Expected exactly 2 'Received CallbackGroupInfo:' messages, but got {callback_info_count}"
        )

    def test_thread_configurator_receives_non_ros_thread_info(self, proc_output, thread_configurator):
        # spawn_non_ros2_thread creates a fresh rclcpp context with its own DDS participant,
        # so DDS discovery can be slow on loaded CI machines. Wait for the message to appear.
        proc_output.assertWaitFor(
            'Received NonRosThreadInfo:',
            timeout=10.0,
            process=thread_configurator
        )

        output_text = "".join(
            output.text.decode('utf-8') for output in proc_output[thread_configurator]
        )
        non_ros_thread_info_count = output_text.count('Received NonRosThreadInfo:')

        # Expected: 1 (for test_non_ros_worker from the publisher)
        self.assertEqual(
            non_ros_thread_info_count, 1,
            f"Expected exactly 1 'Received NonRosThreadInfo:' message, but got {non_ros_thread_info_count}"
        )


@launch_testing.post_shutdown_test()
class TestCallbackIsolatedExecutorShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

    @classmethod
    def tearDownClass(cls):
        import os
        template_yaml = os.path.join(os.path.expanduser("~"), "agnocast", "template.yaml")
        if os.path.exists(template_yaml):
            os.remove(template_yaml)
