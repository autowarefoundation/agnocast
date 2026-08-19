import os
import unittest

import launch_testing
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import EXIT_OK, assertExitCodes, assertInStderr

SERVICE_NAME = '/test_service'
QOS_DEPTH = 10
TARGET_COUNT = 3


@launch_testing.parametrize(
    'server_plugin, use_deferred_callback, client_plugin, use_response_callback, wait_response',
    [
        # === synchronous client (wait_response = true) ===
        # basic arrangement
        ('TestServer', False, 'TestClient', False, True),
        # deferred server callback
        ('TestServer', True, 'TestClient', False, True),
        # callback-based async_send_request()
        ('TestServer', False, 'TestClient', True, True),
        # ROS2 server to test A2R service bridge
        ('TestROS2Server', False, 'TestClient', False, True),
        # ROS2 client to test R2A service bridge
        ('TestServer', False, 'TestROS2Client', True, True),

        # === asynchronous client (wait_response = false) ===
        # basic arrangement
        ('TestServer', False, 'TestClient', True, False),
        # deferred server callback
        ('TestServer', True, 'TestClient', True, False),
        # ROS2 server to test A2R service bridge
        ('TestROS2Server', False, 'TestClient', True, False),
        # ROS2 client to test R2A service bridge
        ('TestServer', False, 'TestROS2Client', True, False),
    ],
)
def generate_test_description(
    server_plugin,
    use_deferred_callback,
    client_plugin,
    use_response_callback,
    wait_response,
):
    server_container = ComposableNodeContainer(
        name='test_server_container',
        namespace='',
        package='agnocast_components',
        executable='agnocast_component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='agnocast_e2e_test',
                plugin=server_plugin,
                name='test_server_node',
                parameters=[
                    {
                        'service_name': SERVICE_NAME,
                        'qos_depth': QOS_DEPTH,
                        'use_deferred_callback': use_deferred_callback,
                        'target_count': TARGET_COUNT,
                    }
                ],
            )
        ],
        output='screen',
        emulate_tty=True,
        additional_env={'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}"},
    )

    client_container = ComposableNodeContainer(
        name='test_client_container',
        namespace='',
        package='agnocast_components',
        executable='agnocast_component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='agnocast_e2e_test',
                plugin=client_plugin,
                name='test_client_node',
                parameters=[
                    {
                        'service_name': SERVICE_NAME,
                        'qos_depth': QOS_DEPTH,
                        'use_response_callback': use_response_callback,
                        'wait_response': wait_response,
                        'target_count': TARGET_COUNT,
                    }
                ],
            )
        ],
        output='screen',
        emulate_tty=True,
        additional_env={'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}"},
    )

    return (
        LaunchDescription(
            [
                SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
                server_container,
                client_container,
                ReadyToTest(),
            ]
        ),
        {'server_container': server_container, 'client_container': client_container},
    )


class TestWaitForAllShutdown(unittest.TestCase):

    # This test ensures that post-shutdown tests are not run until all processes have exited.
    def test_wait_for_all_shutdown(self, proc_info, server_container, client_container):
        proc_info.assertWaitForShutdown(process=server_container, timeout=10)
        proc_info.assertWaitForShutdown(process=client_container, timeout=10)


@launch_testing.post_shutdown_test()
class TestService(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info, allowable_exit_codes=[EXIT_OK])

    def test_server_received_all_requests(self, proc_output, server_container):
        for i in range(TARGET_COUNT):
            assertInStderr(proc_output, f'Receiving {i}.', server_container)
        assertInStderr(
            proc_output,
            'All requests have been handled. Shutting down.',
            server_container,
        )

    def test_client_received_all_responses(self, proc_output, client_container):
        for i in range(TARGET_COUNT):
            assertInStderr(proc_output, f'Receiving {i}.', client_container)
        assertInStderr(
            proc_output,
            'All responses have been received. Shutting down.',
            client_container,
        )
