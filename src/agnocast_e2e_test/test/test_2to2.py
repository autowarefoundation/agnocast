import os
import unittest

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import yaml
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config_test_2to2.yaml')
with open(CONFIG_PATH, 'r') as f:
    CONFIG = yaml.safe_load(f)
QOS_DEPTH = 10
PUB_NUM = int(QOS_DEPTH / 2)
TIMEOUT = float(os.environ.get('STRESS_TEST_TIMEOUT', 8.0))
FOREVER = True if (os.environ.get('STRESS_TEST_TIMEOUT')) else False


def generate_test_description():
    pub_i = 0
    sub_i = 0
    containers = []
    testing_processes = {}
    for i, nodes in enumerate(CONFIG.values()):
        composable_nodes = []
        for node in nodes:
            if node == 'p':
                composable_nodes.append(
                    ComposableNode(
                        package='agnocast_e2e_test',
                        plugin='TestPublisher',
                        name=f'test_talker_node_{pub_i}',
                        parameters=[
                            {
                                "qos_depth": QOS_DEPTH,
                                "transient_local": False,
                                "init_pub_num": 0,
                                "pub_num": PUB_NUM,
                                "planned_pub_count": 1, # Number of ROS 2 inside the bridge.
                                "planned_sub_count": 2, # Number of external Agnocast.
                                "forever": FOREVER,
                            }
                        ],
                    )
                )
                pub_i += 1
            else:  # s
                composable_nodes.insert(
                    0,
                    ComposableNode(
                        package='agnocast_e2e_test',
                        plugin='TestSubscriber',
                        name=f'test_listener_node_{sub_i}',
                        parameters=[
                            {
                                "qos_depth": QOS_DEPTH,
                                "transient_local": False,
                                "forever": FOREVER,
                                "target_end_id" : PUB_NUM - 1,
                                "target_end_count": 2,  # Number of external Agnocast.
                            }
                        ],
                    )
                )
                sub_i += 1

        container = ComposableNodeContainer(
            name=f'test_container_{i}',
            namespace='',
            package='agnocastlib',
            executable='agnocast_component_container_mt',
            composable_node_descriptions=composable_nodes,
            output='screen',
            parameters=[{'number_of_ros2_threads': 8, 'number_of_agnocast_threads': 8}],
            additional_env={
                'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
                'AGNOCAST_MEMPOOL_SIZE': '134217728',
            }
        )
        containers.append(container)
        testing_processes[f'container{i}'] = container

    return (
        LaunchDescription(
            [
                SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
                *containers,
                TimerAction(period=TIMEOUT, actions=[launch_testing.actions.ReadyToTest()])
            ]
        ), testing_processes
    )


class Test2To2(unittest.TestCase):
    pub_i_ = 0
    sub_i_ = 0

    def common_assert(self, proc_output, container_proc, nodes):
        if not nodes:
            return

        with launch_testing.asserts.assertSequentialStdout(proc_output, process=container_proc) as cm:
            proc_output = "".join(cm._output)

            # The display order is not guaranteed, so the message order is not checked.
            for node in nodes:
                if node == 'p':
                    prefix = f"[test_talker_node_{self.pub_i_}]: "
                    for i in range(PUB_NUM):
                        self.assertEqual(proc_output.count(f"{prefix}Publishing {i}."), 1)
                    self.assertEqual(proc_output.count(
                        f"{prefix}All messages published. Shutting down."), 1)
                    self.pub_i_ += 1
                else:  # s
                    prefix = f"[test_listener_node_{self.sub_i_}]: "
                    for i in range(PUB_NUM):
                        self.assertEqual(proc_output.count(f"{prefix}Receiving {i}."), 2)
                    self.assertEqual(proc_output.count(
                        f"{prefix}All messages received. Shutting down."), 1)
                    self.sub_i_ += 1

    def test_all_container(self, proc_output, container0, container1, container2, container3):
        nodes = CONFIG['container0']
        self.common_assert(proc_output, container0, nodes)

        nodes = CONFIG['container1']
        self.common_assert(proc_output, container1, nodes)

        nodes = CONFIG['container2']
        self.common_assert(proc_output, container2, nodes)

        nodes = CONFIG['container3']
        self.common_assert(proc_output, container3, nodes)
