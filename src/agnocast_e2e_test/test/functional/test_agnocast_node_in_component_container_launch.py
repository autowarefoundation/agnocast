"""Regression test for loading an ``agnocast::Node`` into a Component Container."""

import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _heaphook_env():
    return {'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}"}


def generate_test_description():
    component_container = ComposableNodeContainer(
        name='agnocast_node_container',
        namespace='',
        package='agnocast_components',
        executable='agnocast_component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='agnocast_sample_application',
                plugin='NoRclcppSubscriber',
                name='no_rclcpp_listener_node',
                # use_sim_time makes NodeTimeSource spawn an AgnocastOnlySingleThreadedExecutor
                # for the clock thread while the node is being constructed.
                parameters=[{'use_sim_time': True}],
            )
        ],
        output='screen',
        additional_env=_heaphook_env(),
    )

    talker = launch_ros.actions.Node(
        package='agnocast_sample_application',
        executable='no_rclcpp_talker',
        name='no_rclcpp_talker_node',
        output='screen',
        additional_env=_heaphook_env(),
    )

    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
            component_container,
            launch.actions.TimerAction(period=3.0, actions=[talker]),
            launch.actions.TimerAction(
                period=5.0,
                actions=[launch_testing.actions.ReadyToTest()]
            )
        ]),
        {
            'component_container': component_container,
            'talker': talker,
        }
    )


class TestAgnocastNodeInComponentContainer(unittest.TestCase):

    def test_node_is_loaded(self, proc_output, component_container):
        """The container must survive constructing the agnocast::Node."""
        proc_output.assertWaitFor(
            '=== NoRclcppSubscriber Node Info ===',
            timeout=15.0,
            process=component_container
        )

        # The Agnocast-only executors the node spawns register their shutdown eventfd with the
        # signal handler while it is being constructed, and log these when that fails. Asserted
        # here rather than as a case of its own so that the wait above has already happened.
        output = ''.join(
            output.text.decode('utf-8') for output in proc_output[component_container]
        )
        self.assertNotIn('signal handler is not installed', output)
        self.assertNotIn('Failed to register shutdown eventfd', output)

    def test_composable_node_receives_messages(self, proc_output, component_container):
        """The loaded node must still be functional, not merely constructed."""
        proc_output.assertWaitFor(
            'I heard dynamic size array message',
            timeout=15.0,
            process=component_container
        )
