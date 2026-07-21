import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import yaml
from ament_index_python.packages import get_package_prefix

import rclpy
from agnocast_cie_config_msgs.srv import ReapplyConfig

CONFIG_DIR = os.path.join(
    tempfile.gettempdir(), 'agnocast_test_thread_configurator_precedence'
)
CONFIG_FILE = os.path.join(CONFIG_DIR, 'template.yaml')

REAPPLY_SERVICE = '/thread_configurator_node/reapply_config'

TARGET_NODE = '/test_publisher_component'
WILDCARD_ID = TARGET_NODE + '/*'

# The publisher's exact entries from the prerun output, all kept alongside a
# covering wildcard: with precedence intact every announced instance is taken
# by its exact entry and the wildcard never matches anything.
_PUBLISHER_ENTRIES = None
_DOMAIN_ID = None


def _run_prerun():
    """Run prerun_node alongside test_cie_publisher to generate config YAML."""
    if os.path.exists(CONFIG_DIR):
        shutil.rmtree(CONFIG_DIR)
    os.makedirs(CONFIG_DIR)

    tc_prefix = get_package_prefix('agnocast_cie_thread_configurator')
    prerun_exe = os.path.join(
        tc_prefix, 'lib', 'agnocast_cie_thread_configurator', 'prerun_node'
    )

    e2e_prefix = get_package_prefix('agnocast_e2e_test')
    publisher_exe = os.path.join(
        e2e_prefix, 'lib', 'agnocast_e2e_test', 'test_cie_publisher'
    )

    prerun_log = tempfile.NamedTemporaryFile(
        mode='w', suffix='.log', delete=False
    )
    prerun_proc = subprocess.Popen(
        [prerun_exe],
        cwd=CONFIG_DIR,
        stdout=prerun_log,
        stderr=subprocess.STDOUT,
    )
    publisher_proc = subprocess.Popen(
        [publisher_exe],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    time.sleep(5)

    try:
        publisher_proc.send_signal(signal.SIGINT)
        publisher_proc.wait(timeout=10)
        prerun_proc.send_signal(signal.SIGINT)
        prerun_proc.wait(timeout=10)
    finally:
        for proc in [publisher_proc, prerun_proc]:
            if proc.poll() is None:
                proc.kill()
                proc.wait()

    prerun_log.close()
    if not os.path.exists(CONFIG_FILE):
        with open(prerun_log.name) as f:
            prerun_output = f.read()
        os.unlink(prerun_log.name)
        raise RuntimeError(
            f'prerun_node failed to generate {CONFIG_FILE}.\n'
            f'Output:\n{prerun_output}'
        )
    os.unlink(prerun_log.name)


def _write_config(cfg):
    with open(CONFIG_FILE, 'w') as f:
        yaml.safe_dump(cfg, f)


def _add_wildcard_alongside_exact_entries():
    """Add a wildcard covering the publisher while keeping its exact entries.

    Exact-over-wildcard precedence is decided at announcement time, so both
    forms must already be in the config when the configurator starts.
    """
    global _PUBLISHER_ENTRIES, _DOMAIN_ID
    # BaseLoader keeps every scalar as a string: this rewrite happens before
    # the configurator starts, and safe_load would turn hardware_info values
    # like 'cpu_max_mhz: 2101.0000' into floats whose re-dumped form ('2101.0')
    # fails the configurator's startup hardware validation.
    with open(CONFIG_FILE) as f:
        cfg = yaml.load(f, Loader=yaml.BaseLoader)
    _PUBLISHER_ENTRIES = [
        e for e in cfg.get('callback_groups', [])
        if e['id'].split('@', 1)[0] == TARGET_NODE
    ]
    if len(_PUBLISHER_ENTRIES) == 0:
        raise RuntimeError(
            f'prerun produced no callback_groups for {TARGET_NODE}'
        )
    _DOMAIN_ID = _PUBLISHER_ENTRIES[0].get('domain_id', 0)
    # SCHED_OTHER + positive nice value: lowering priority needs no
    # CAP_SYS_NICE, which is not guaranteed in CI sandboxes.
    wildcard_entry = {
        'id': WILDCARD_ID,
        'domain_id': _DOMAIN_ID,
        'policy': 'SCHED_OTHER',
        'priority': 10,
        'affinity': [],
    }
    cfg['callback_groups'] = cfg['callback_groups'] + [wildcard_entry]
    _write_config(cfg)


def _call_reapply(timeout_sec=10.0):
    """Call the reapply_config service synchronously and return the response."""
    rclpy.init()
    try:
        node = rclpy.create_node('test_reapply_client')
        client = node.create_client(ReapplyConfig, REAPPLY_SERVICE)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.destroy_node()
            raise RuntimeError(f'service {REAPPLY_SERVICE} not available')
        future = client.call_async(ReapplyConfig.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            node.destroy_node()
            raise RuntimeError('reapply service call timed out')
        response = future.result()
        node.destroy_node()
        return response
    finally:
        rclpy.shutdown()


def generate_test_description():
    _run_prerun()
    _add_wildcard_alongside_exact_entries()

    thread_configurator = launch_ros.actions.Node(
        package='agnocast_cie_thread_configurator',
        executable='thread_configurator_node',
        name='thread_configurator_node',
        output='screen',
        parameters=[{'config_file': CONFIG_FILE}],
    )

    test_app = launch_ros.actions.Node(
        package='agnocast_e2e_test',
        executable='test_cie_publisher',
        output='screen',
    )

    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable(
                'RCUTILS_LOGGING_BUFFERED_STREAM', '0'
            ),

            thread_configurator,

            # Start the target app after DDS discovery has time to settle.
            launch.actions.TimerAction(
                period=2.0,
                actions=[test_app],
            ),

            launch.actions.TimerAction(
                period=8.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {
            'thread_configurator': thread_configurator,
            'test_app': test_app,
        },
    )


class TestThreadConfiguratorPrecedence(unittest.TestCase):

    def test_exact_entries_take_precedence_over_wildcard(
            self, proc_output, thread_configurator):
        for entry in _PUBLISHER_ENTRIES:
            proc_output.assertWaitFor(
                entry['id'],
                timeout=20.0,
                process=thread_configurator,
            )

        # The reapply report exposes which entry tracks each instance: with
        # precedence intact every instance is applied under its exact entry
        # and the never-matched wildcard reports its pattern key as skipped.
        # Were precedence broken, the wildcard would have captured the
        # instances (their keys applied via matched_tids, pattern key absent
        # from skipped) and the exact entries would be skipped (no known tid).
        response = _call_reapply()
        self.assertTrue(
            response.success,
            f'reapply failed unexpectedly: error_message={response.error_message!r}',
        )
        applied = list(response.applied_callback_groups)
        for entry in _PUBLISHER_ENTRIES:
            key = f"{_DOMAIN_ID}:{entry['id']}"
            self.assertEqual(
                applied.count(key), 1,
                'each instance must be applied exactly once via its exact '
                'entry; twice would mean the wildcard also recorded it',
            )
            self.assertNotIn(
                key, list(response.skipped_callback_groups),
                'an exact entry must have received the announcement itself, '
                'not lost it to the wildcard',
            )
            self.assertNotIn(key, list(response.failed_callback_groups))
        pattern_key = f'{_DOMAIN_ID}:{WILDCARD_ID}'
        self.assertIn(
            pattern_key, list(response.skipped_callback_groups),
            'the wildcard must have matched nothing while every instance is '
            'covered by an exact entry',
        )
        for arr in (
            response.applied_callback_groups,
            response.failed_callback_groups,
        ):
            self.assertNotIn(pattern_key, list(arr))

        proc_output.assertWaitFor(
            'Reapply done:',
            timeout=10.0,
            process=thread_configurator,
        )


@launch_testing.post_shutdown_test()
class TestThreadConfiguratorPrecedenceShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -signal.SIGINT]
        )

    @classmethod
    def tearDownClass(cls):
        if os.path.isdir(CONFIG_DIR):
            shutil.rmtree(CONFIG_DIR)
