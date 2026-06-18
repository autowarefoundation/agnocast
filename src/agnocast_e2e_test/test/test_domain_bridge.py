"""Case 2 (same IPC namespace, cross-domain) zero-copy delivery e2e.

A domain bridge rule (FROM_DOMAIN -> TO_DOMAIN) must already be registered with
the kmod (scripts/test/e2e_test_domain_bridge.bash does this before launching).
An Agnocast publisher runs in FROM_DOMAIN and an Agnocast subscriber in
TO_DOMAIN within the same IPC namespace; with the rule in place the subscriber
receives the publisher's messages straight from shared memory (no DDS / bridge
node), i.e. zero-copy across domains.

The subscriber starts first so volatile messages are not missed.
"""
import os
import unittest

import launch_testing
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

TOPIC_NAME = os.environ.get('E2E_TOPIC_NAME', '/test_domain_bridge_topic')
FROM_DOMAIN = os.environ.get('E2E_FROM_DOMAIN', '1')
TO_DOMAIN = os.environ.get('E2E_TO_DOMAIN', '2')
QOS_DEPTH = 10
PUB_NUM = 10


def _agnocast_env(domain_id):
    return {
        'ROS_DOMAIN_ID': str(domain_id),
        'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
    }


def generate_test_description():
    sub_container = ComposableNodeContainer(
        name='db_listener_container',
        namespace='',
        package='agnocast_components',
        executable='agnocast_component_container',
        parameters=[{'get_next_timeout_ms': 1}],
        composable_node_descriptions=[
            ComposableNode(
                package='agnocast_e2e_test',
                plugin='TestSubscriber',
                name='db_listener_node',
                parameters=[{
                    'topic_name': TOPIC_NAME,
                    'qos_depth': QOS_DEPTH,
                    'transient_local': False,
                    'forever': False,
                    'target_end_id': PUB_NUM - 1,
                }],
            )
        ],
        output='screen',
        additional_env=_agnocast_env(TO_DOMAIN),
    )

    pub_container = ComposableNodeContainer(
        name='db_talker_container',
        namespace='',
        package='agnocast_components',
        executable='agnocast_component_container',
        parameters=[{'get_next_timeout_ms': 1}],
        composable_node_descriptions=[
            ComposableNode(
                package='agnocast_e2e_test',
                plugin='TestPublisher',
                name='db_talker_node',
                parameters=[{
                    'topic_name': TOPIC_NAME,
                    'qos_depth': QOS_DEPTH,
                    'transient_local': False,
                    'init_pub_num': 0,
                    'pub_num': PUB_NUM,
                    'planned_pub_count': 0,  # Case 2 is pure Agnocast; no DDS sub to wait for.
                    'forever': False,
                }],
            )
        ],
        output='screen',
        additional_env=_agnocast_env(FROM_DOMAIN),
    )

    # Subscriber first (t=0) so it is connected before the publisher's volatile
    # messages; publisher after a delay; ReadyToTest once both have run.
    return (
        LaunchDescription([
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
            sub_container,
            TimerAction(period=1.0, actions=[pub_container]),
            TimerAction(period=10.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {'test_pub': pub_container, 'test_sub': sub_container},
    )


class TestDomainBridgeCase2(unittest.TestCase):

    def test_pub(self, proc_output, test_pub):
        output = ''.join(o.text.decode('utf-8') for o in proc_output[test_pub])
        for i in range(PUB_NUM):
            self.assertEqual(output.count(f'Publishing {i}.'), 1)
        self.assertEqual(output.count('All messages published. Shutting down.'), 1)

    def test_sub_receives_across_domains(self, proc_output, test_sub):
        output = ''.join(o.text.decode('utf-8') for o in proc_output[test_sub])
        # Every published message reaches the other domain's subscriber.
        for i in range(PUB_NUM):
            self.assertEqual(output.count(f'Receiving {i}.'), 1)
        self.assertEqual(output.count('All messages received. Shutting down.'), 1)
