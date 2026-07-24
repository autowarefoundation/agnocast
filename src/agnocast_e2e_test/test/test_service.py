import os
import unittest

import launch_testing
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_testing.asserts import assertInStderr


def generate_test_description():
    server_container = ComposableNodeContainer(
        name="test_server_container",
        namespace="",
        package="agnocast_components",
        executable="agnocast_component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="agnocast_e2e_test",
                plugin="TestServer",
                name="test_server_node",
                parameters=[
                    {
                        "service_name": "/test_service",
                        "qos_depth": 10,
                        "use_deferred_callback": False,
                        "target_count": 10,
                    }
                ],
            )
        ],
        output="screen",
        emulate_tty=True,
        additional_env={ "LD_PRELOAD": f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}" },
    )

    client_container = ComposableNodeContainer(
        name="test_client_container",
        namespace="",
        package="agnocast_components",
        executable="agnocast_component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="agnocast_e2e_test",
                plugin="TestClient",
                name="test_client_node",
                parameters=[
                    {
                        "service_name": "/test_service",
                        "qos_depth": 10,
                        "use_response_callback": False,
                        "target_count": 10,
                    }
                ],
            )
        ],
        output="screen",
        emulate_tty=True,
        additional_env={ "LD_PRELOAD": f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}" },
    )

    return (
        LaunchDescription(
            [
                SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "0"),
                server_container,
                client_container,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        { "test_server": server_container, "test_client": client_container },
    )

class TestWaitForAllShutdown(unittest.TestCase):

    # This test ensures that post-shutdown tests are not run until all processes have exited.
    def test_wait_for_all_shutdown(self, proc_info, test_server, test_client):
        proc_info.assertWaitForShutdown(process=test_server, timeout=10)
        proc_info.assertWaitForShutdown(process=test_client, timeout=10)

@launch_testing.post_shutdown_test()
class TestService(unittest.TestCase):

    def test_server_received_all_requests(self, proc_output, test_server):
        for i in range(10):
            assertInStderr(proc_output, f"Receiving {i}.", test_server)
        assertInStderr(proc_output, "All requests have been handled. Shutting down.", test_server)

    def test_client_received_response(self, proc_output, test_client):
        for i in range(10):
            assertInStderr(proc_output, f"Receiving {i}.", test_client)
        assertInStderr(proc_output, "All responses have been received. Shutting down.", test_client)
