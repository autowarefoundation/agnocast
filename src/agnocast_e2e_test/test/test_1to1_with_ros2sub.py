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

CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config_test_1to1_with_ros2sub.yaml')

USE_AGNOCAST_PUB: bool
EXPECT_INIT_PUB_NUM: int
EXPECT_PUB_NUM: int
EXPECT_INIT_SUB_NUM: int
EXPECT_SUB_NUM: int
EXPECT_INIT_ROS2_SUB_NUM: int
EXPECT_ROS2_SUB_NUM: int

TIMEOUT = os.environ.get('STRESS_TEST_TIMEOUT')
FOREVER = True if (os.environ.get('STRESS_TEST_TIMEOUT')) else False


def calc_expect_pub_sub_num(config: dict) -> None:
    global USE_AGNOCAST_PUB, EXPECT_PUB_NUM, EXPECT_INIT_PUB_NUM, EXPECT_INIT_SUB_NUM, EXPECT_SUB_NUM, EXPECT_INIT_ROS2_SUB_NUM, EXPECT_ROS2_SUB_NUM

    USE_AGNOCAST_PUB = config['use_agnocast_pub']
    EXPECT_INIT_PUB_NUM = config['pub_qos_depth'] if (
        config['launch_pub_before_sub'] and config['pub_transient_local']) else 0
    EXPECT_PUB_NUM = config['pub_qos_depth']

    is_pub_volatile = not config['pub_transient_local']
    is_sub_transient_local = config['sub_transient_local']

    if is_pub_volatile and is_sub_transient_local and not USE_AGNOCAST_PUB:
        EXPECT_ROS2_SUB_NUM = 0
        EXPECT_INIT_ROS2_SUB_NUM = 0
        EXPECT_SUB_NUM = 0
        EXPECT_INIT_SUB_NUM = 0
        return

    EXPECT_ROS2_SUB_NUM = min(EXPECT_PUB_NUM, config['sub_qos_depth'])
    EXPECT_SUB_NUM = min(EXPECT_PUB_NUM, config['sub_qos_depth'])
    if config['sub_transient_local']:
        EXPECT_INIT_ROS2_SUB_NUM = min(
            EXPECT_INIT_PUB_NUM, config['sub_qos_depth']) if config['pub_transient_local'] else 0
        EXPECT_INIT_SUB_NUM = min(
            EXPECT_INIT_PUB_NUM, config['sub_qos_depth'])
    else:
        EXPECT_INIT_ROS2_SUB_NUM = 0
        EXPECT_INIT_SUB_NUM = 0


def calc_action_delays(config: dict) -> tuple:
    unit_delay = 1.0
    pub_delay = 0.0 if config['launch_pub_before_sub'] else unit_delay
    sub_delay = 0.01 * EXPECT_INIT_PUB_NUM + unit_delay if config['launch_pub_before_sub'] else 0.0
    ready_delay = float(TIMEOUT) if TIMEOUT else pub_delay + sub_delay + 10.0
    return pub_delay, sub_delay, ready_delay


def generate_test_description():
    with open(CONFIG_PATH, 'r') as f:
        config = yaml.safe_load(f)
    calc_expect_pub_sub_num(config)
    pub_delay, sub_delay, ready_delay = calc_action_delays(config)

    if config['use_agnocast_pub']:
        pub_container = ComposableNodeContainer(
            name='test_talker_container',
            namespace='',
            package='agnocastlib',
            executable='agnocast_component_container',
            parameters=[
                {"get_next_timeout_ms": 1},
            ],
            composable_node_descriptions=[
                ComposableNode(
                    package='agnocast_e2e_test',
                    plugin='TestPublisher',
                    name='test_talker_node',
                    parameters=[
                        {
                            "qos_depth": config['pub_qos_depth'],
                            "transient_local": config['pub_transient_local'],
                            "init_pub_num": EXPECT_INIT_PUB_NUM,
                            "pub_num": EXPECT_PUB_NUM,
                            "forever": FOREVER
                        }
                    ],
                )
            ],
            output='screen',
            additional_env={
                'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
                'AGNOCAST_MEMPOOL_SIZE': '134217728',
            }
        )
    else:
        pub_container = ComposableNodeContainer(
            name='test_ros2_talker_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='agnocast_e2e_test',
                    plugin='TestROS2Publisher',
                    name='test_ros2_talker_node',
                    parameters=[
                        {
                            "qos_depth": config['pub_qos_depth'],
                            "transient_local": config['pub_transient_local'],
                            "init_pub_num": EXPECT_INIT_PUB_NUM,
                            "pub_num": EXPECT_PUB_NUM,
                            # If 0, skip the connection wait to avoid hanging in incompatible QoS scenarios.
                            "planned_sub_count": 1 if EXPECT_SUB_NUM > 0 else 0,
                            "forever": FOREVER
                        }
                    ],
                )
            ],
            output='screen',
        )

    pub_node = TimerAction(
        period=pub_delay,
        actions=[pub_container]
    )

    sub_nodes_actions = [
        ComposableNodeContainer(
            name='test_ros2_listener_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='agnocast_e2e_test',
                        plugin='TestROS2Subscriber',
                        name='test_ros2_listener_node',
                        parameters=[
                            {
                                "qos_depth": config['sub_qos_depth'],
                                "transient_local": config['sub_transient_local'],
                                "forever": FOREVER,
                                "target_end_id": (EXPECT_INIT_PUB_NUM + EXPECT_SUB_NUM) - 1
                            }
                        ],
                    )
            ],
            output='screen',
        )
    ]
    if config['use_take_sub']:
        sub_nodes_actions.append(
            ComposableNodeContainer(
                name='test_taker_container',
                namespace='',
                package='agnocastlib',
                executable='agnocast_component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='agnocast_e2e_test',
                        plugin='TestTakeSubscriber',
                        name='test_taker_node',
                        parameters=[
                            {
                                "qos_depth": config['sub_qos_depth'],
                                "transient_local": config['sub_transient_local'],
                                "forever": FOREVER,
                                "target_end_id": (EXPECT_INIT_PUB_NUM + EXPECT_SUB_NUM) - 1
                            }
                        ],
                    )
                ],
                output='screen',
                additional_env={
                    'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
                    'AGNOCAST_MEMPOOL_SIZE': '134217728',
                }
            )
        )
    else:
        sub_nodes_actions.append(
            ComposableNodeContainer(
                name='test_listener_container',
                namespace='',
                package='agnocastlib',
                executable='agnocast_component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='agnocast_e2e_test',
                        plugin='TestSubscriber',
                        name='test_listener_node',
                        parameters=[
                            {
                                "qos_depth": config['sub_qos_depth'],
                                "transient_local": config['sub_transient_local'],
                                "forever": FOREVER,
                                "target_end_id": (EXPECT_INIT_PUB_NUM + EXPECT_SUB_NUM) - 1
                            }
                        ],
                    )
                ],
                output='screen',
                additional_env={
                    'LD_PRELOAD': f"libagnocast_heaphook.so:{os.getenv('LD_PRELOAD', '')}",
                    'AGNOCAST_MEMPOOL_SIZE': '134217728',
                }
            )
        )

    sub_nodes = TimerAction(
        period=sub_delay,
        actions=sub_nodes_actions
    )

    return (
        LaunchDescription(
            [
                SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
                pub_node,
                sub_nodes,
                TimerAction(period=ready_delay, actions=[launch_testing.actions.ReadyToTest()])
            ]
        ),
        {
            'test_pub': pub_node.actions[0],
            'test_ros2_sub': sub_nodes.actions[0],
            'test_sub': sub_nodes.actions[1],
        }
    )


class Test1To1(unittest.TestCase):

    def test_pub(self, proc_output, test_pub):
        with launch_testing.asserts.assertSequentialStdout(proc_output, process=test_pub) as cm:
            proc_output = "".join(cm._output)

            total_expected_count = EXPECT_INIT_PUB_NUM + EXPECT_PUB_NUM
            self.assertGreater(total_expected_count, 0, "Expected publisher count must be greater than 0.")

            # The display order is not guaranteed, so the message order is not checked.
            for i in range(total_expected_count):
                self.assertEqual(proc_output.count(f"Publishing {i}."), 1)
            self.assertEqual(proc_output.count("All messages published. Shutting down."), 1)

    # Bridges's Behavior Analysis: A2R vs R2A (Transient Local)
    # Both scenarios ultimately follow ROS 2 (DDS) semantics, but the burst location differs.
    #
    # Case 1: A2R (Agnocast -> ROS 2) => "Exit Burst"
    # - In:  Real-time stream. The Bridge starts with the Agnocast Pub, receiving data live.
    # - Out: Burst. The Bridge buffers the data via DDS. When the ROS 2 Sub joins late,
    #        the Bridge dumps the history to the Sub.
    # -> ROS 2 Sub receives a burst of history data.
    #
    # Case 2: R2A (ROS 2 -> Agnocast) => "Entry Burst & Replay"
    # - In:  Burst. The ROS 2 Pub dumps history to the Bridge upon connection.
    # - Out: Fast Replay. The Bridge iterates through the burst input and re-publishes sequentially.
    #        The Agnocast Sub sees this as a high-speed stream of "fresh" messages, not history lookup.
    # -> Agnocast Sub receives a "fast replay" of the history.
    def test_sub(self, proc_output, test_sub):
        with launch_testing.asserts.assertSequentialStdout(proc_output, process=test_sub) as cm:
            output_str = "".join(cm._output)

            start_index = EXPECT_INIT_PUB_NUM - EXPECT_INIT_SUB_NUM
            total_expected_count = EXPECT_INIT_SUB_NUM + EXPECT_SUB_NUM
            
            if total_expected_count > 0:
                if USE_AGNOCAST_PUB:
                    # ---------------------------------------------------------
                    # Pattern A: Agnocast Publisher
                    # ---------------------------------------------------------
                    # Agnocast uses a pull mechanism via shared memory.
                    # The subscriber strictly respects the history depth, ensuring deterministic behavior.
                    # We verify that exactly the expected sequence is received.
                    for i in range(start_index, start_index + total_expected_count):
                        self.assertEqual(output_str.count(f"Receiving {i}."), 1)
                    self.assertEqual(output_str.count("All messages received. Shutting down."), 1)
                else:
                    # ---------------------------------------------------------
                    # Pattern B: ROS 2 Publisher (R2A Bridge [Case 2])
                    # ---------------------------------------------------------
                    # [Context: ROS 2 Transient Local behavior with depth mismatch]
                    # ROS 2 (DDS) bursts history data. If the Subscriber's depth is smaller than the Publisher's,
                    # the receive buffer in the Bridge may be overwritten before processing.
                    #
                    # 1. History (Init) Check: Fuzzy
                    # We expect to receive *at least* the minimum required history.
                    actual_init_count = sum(
                        1 for i in range(EXPECT_INIT_PUB_NUM)
                        if output_str.count(f"Receiving {i}.") > 0
                    )
                    self.assertGreaterEqual(actual_init_count, EXPECT_INIT_SUB_NUM)
                    
                    # 2. Live Message Check: Strict
                    # New messages arriving after connection should be received reliably.
                    for i in range(EXPECT_INIT_PUB_NUM, EXPECT_INIT_PUB_NUM + EXPECT_SUB_NUM):
                        self.assertEqual(output_str.count(f"Receiving {i}."), 1)
                    self.assertEqual(output_str.count("All messages received. Shutting down."), 1)
            else:
                # ---------------------------------------------------------
                # Pattern C: Incompatible QoS (Connection Rejected)
                # ---------------------------------------------------------
                # The Bridge's ROS 2 Subscriber inherits settings.
                # Violation of RxO rules (e.g., Volatile Pub vs. Transient Local Sub) prevents connection.
                self.assertEqual(output_str.count("Receiving"), 0)
                
                # Scan full log for the incompatibility warning.
                full_log = "".join([
                    line.text.decode() if isinstance(line.text, bytes) else line.text
                    for line in proc_output
                ])
                self.assertIn("incompatible QoS", full_log)

    def test_ros2_sub(self, proc_output, test_ros2_sub):
        with launch_testing.asserts.assertSequentialStdout(proc_output, process=test_ros2_sub) as cm:
            output_str = "".join(cm._output)

            total_expected_count = EXPECT_INIT_ROS2_SUB_NUM + EXPECT_ROS2_SUB_NUM
            if total_expected_count > 0:
                # ---------------------------------------------------------
                # Pattern B: Agnocast Publisher (A2R Bridge [Case 1]) or ROS 2 Publisher
                # ---------------------------------------------------------
                # Similar to the Bridge case, standard ROS 2 subscribers are subject to DDS burst behavior
                # when Transient Local is used with small depth (Depth Mismatch).
                
                # 1. History (Init) Check: Fuzzy
                actual_init_count = sum(
                    1 for i in range(EXPECT_INIT_PUB_NUM)
                    if output_str.count(f"Receiving {i}.") > 0
                )
                self.assertGreaterEqual(actual_init_count, EXPECT_INIT_ROS2_SUB_NUM)
                
                # 2. Live Message Check: Strict
                for i in range(EXPECT_INIT_PUB_NUM, EXPECT_INIT_PUB_NUM + EXPECT_ROS2_SUB_NUM):
                    self.assertEqual(output_str.count(f"Receiving {i}."), 1)
                self.assertEqual(output_str.count("All messages received. Shutting down."), 1)
            else:
                # ---------------------------------------------------------
                # Pattern C: Incompatible QoS (Connection Rejected)
                # ---------------------------------------------------------
                self.assertEqual(output_str.count("Receiving"), 0)
                
                # Scan full log for the incompatibility warning.
                full_log = "".join([
                    line.text.decode() if isinstance(line.text, bytes) else line.text
                    for line in proc_output
                ])
                self.assertIn("incompatible QoS", full_log)

