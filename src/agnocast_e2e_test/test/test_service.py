import os
import unittest

import launch_testing
from launch import LaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import EXIT_OK, assertExitCodes, assertInStderr
from osrf_pycommon.terminal_color import remove_ansi_escape_sequences

SERVICE_NAME = '/test_service'
QOS_DEPTH = 10
TARGET_COUNT = 3


def stderr_lines(proc_output, process):
    text = ''.join(
        io.text.decode(errors='replace') for io in proc_output[process] if io.from_stderr
    )
    return remove_ansi_escape_sequences(text).splitlines()


PARAMETER_NAMES = (
    'server_plugin, use_deferred_callback, client_plugin, use_response_callback, wait_response'
)
PARAMETER_SETS = [
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
]


def print_progress(params):
    names = [name.strip() for name in PARAMETER_NAMES.split(',')]
    width = max(len(name) for name in names)
    print(
        f'\n====================== '
        f'{PARAMETER_SETS.index(params) + 1} / {len(PARAMETER_SETS)}'
        f' ======================'
    )
    print('----------------------------------------------')
    for name, value in zip(names, params):
        print(f'{name:{width}} : {value}')
    print('----------------------------------------------', flush=True)


@launch_testing.parametrize(PARAMETER_NAMES, PARAMETER_SETS)
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

    params = (
        server_plugin,
        use_deferred_callback,
        client_plugin,
        use_response_callback,
        wait_response,
    )

    return (
        LaunchDescription(
            [
                OpaqueFunction(function=lambda context: print_progress(params)),
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
        proc_info.assertWaitForShutdown(process=server_container, timeout=30)
        proc_info.assertWaitForShutdown(process=client_container, timeout=30)


@launch_testing.post_shutdown_test()
class TestService(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info, allowable_exit_codes=[EXIT_OK])

    def test_no_error_logs(self, proc_output, server_container, client_container):
        for container in (server_container, client_container):
            errors = [line for line in stderr_lines(proc_output, container) if '[ERROR]' in line]
            self.assertEqual(errors, [])

    def test_server_received_all_requests(self, proc_output, server_container):
        lines = stderr_lines(proc_output, server_container)
        for i in range(TARGET_COUNT):
            matched = [line for line in lines if f'Receiving {i}.' in line]
            self.assertEqual(len(matched), 1, f'Receiving {i}. was logged {len(matched)} times')
        assertInStderr(
            proc_output,
            'All requests have been handled. Shutting down.',
            server_container,
        )

    def test_client_received_all_responses(self, proc_output, client_container):
        lines = stderr_lines(proc_output, client_container)
        for i in range(TARGET_COUNT):
            matched = [line for line in lines if f'Receiving {i}.' in line]
            self.assertEqual(len(matched), 1, f'Receiving {i}. was logged {len(matched)} times')
        assertInStderr(
            proc_output,
            'All responses have been received. Shutting down.',
            client_container,
        )
