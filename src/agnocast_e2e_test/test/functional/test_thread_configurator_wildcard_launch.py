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
    tempfile.gettempdir(), 'agnocast_test_thread_configurator_wildcard'
)
CONFIG_FILE = os.path.join(CONFIG_DIR, 'template.yaml')

REAPPLY_SERVICE = '/thread_configurator_node/reapply_config'

TARGET_NODE = '/test_publisher_component'
WILDCARD_ID = TARGET_NODE + '/*'

# The publisher's exact entries captured from the prerun output before they
# are replaced by the wildcard; their ids equal the announced callback-group
# ids, which the wildcard reports per instance.
_PUBLISHER_ENTRIES = None
_WILDCARD_DOMAIN_ID = None


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


def _read_config():
    with open(CONFIG_FILE) as f:
        return yaml.safe_load(f)


def _write_config(cfg):
    with open(CONFIG_FILE, 'w') as f:
        yaml.safe_dump(cfg, f)


def _replace_publisher_entries_with_wildcard():
    """Rewrite the prerun config so the publisher is covered by a wildcard.

    Instances are matched to entries at announcement time only, so the
    wildcard must already be in the config when the configurator starts.
    """
    global _PUBLISHER_ENTRIES, _WILDCARD_DOMAIN_ID
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
    kept = [
        e for e in cfg.get('callback_groups', [])
        if e['id'].split('@', 1)[0] != TARGET_NODE
    ]
    if len(_PUBLISHER_ENTRIES) == 0:
        raise RuntimeError(
            f'prerun produced no callback_groups for {TARGET_NODE}'
        )
    _WILDCARD_DOMAIN_ID = _PUBLISHER_ENTRIES[0].get('domain_id', 0)
    # SCHED_OTHER + positive nice value: lowering priority needs no
    # CAP_SYS_NICE, which is not guaranteed in CI sandboxes.
    wildcard_entry = {
        'id': WILDCARD_ID,
        'domain_id': _WILDCARD_DOMAIN_ID,
        'policy': 'SCHED_OTHER',
        'priority': 10,
        'affinity': [],
    }
    cfg['callback_groups'] = kept + [wildcard_entry]
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


# Snapshot of the wildcard-rewritten YAML, restored by setUp before each test.
_ORIGINAL_CONFIG_BYTES = None


def _snapshot_original_config():
    global _ORIGINAL_CONFIG_BYTES
    with open(CONFIG_FILE, 'rb') as f:
        _ORIGINAL_CONFIG_BYTES = f.read()


def _restore_original_config():
    if _ORIGINAL_CONFIG_BYTES is None:
        raise RuntimeError(
            '_restore_original_config called before _snapshot_original_config'
        )
    with open(CONFIG_FILE, 'wb') as f:
        f.write(_ORIGINAL_CONFIG_BYTES)


def generate_test_description():
    _run_prerun()
    _replace_publisher_entries_with_wildcard()
    _snapshot_original_config()

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


class TestThreadConfiguratorWildcard(unittest.TestCase):

    def setUp(self):
        _restore_original_config()

    def _wait_for_all_instances(self, proc_output, thread_configurator):
        proc_output.assertWaitFor(
            f"matched wildcard entry '{WILDCARD_ID}'",
            timeout=20.0,
            process=thread_configurator,
        )
        # The configurator only knows tids for announced callback groups; wait
        # for each instance id in its log so the reapply below covers all.
        for entry in _PUBLISHER_ENTRIES:
            proc_output.assertWaitFor(
                entry['id'],
                timeout=20.0,
                process=thread_configurator,
            )

    def test_wildcard_matches_at_announcement(
            self, proc_output, thread_configurator):
        self._wait_for_all_instances(proc_output, thread_configurator)

    def test_reapply_wildcard_attribute_change(
            self, proc_output, thread_configurator):
        self._wait_for_all_instances(proc_output, thread_configurator)

        cfg = _read_config()
        wildcard_entries = [
            e for e in cfg.get('callback_groups', []) if e['id'] == WILDCARD_ID
        ]
        self.assertEqual(len(wildcard_entries), 1)
        wildcard_entries[0]['priority'] = 11
        _write_config(cfg)

        response = _call_reapply()
        self.assertTrue(
            response.success,
            f'reapply failed unexpectedly: error_message={response.error_message!r}',
        )
        # The wildcard reports one key per matched instance, so every announced
        # instance must be applied under its full id, not under the pattern.
        applied = list(response.applied_callback_groups)
        for entry in _PUBLISHER_ENTRIES:
            key = f"{_WILDCARD_DOMAIN_ID}:{entry['id']}"
            self.assertEqual(
                applied.count(key), 1,
                'each matched instance must be applied once under its full id',
            )
        pattern_key = f'{_WILDCARD_DOMAIN_ID}:{WILDCARD_ID}'
        for arr in (
            response.applied_callback_groups,
            response.skipped_callback_groups,
            response.failed_callback_groups,
        ):
            self.assertNotIn(
                pattern_key, list(arr),
                'pattern key must not be reported once instances are known',
            )

        proc_output.assertWaitFor(
            'Reapply done:',
            timeout=10.0,
            process=thread_configurator,
        )

    def test_reapply_added_exact_override_stays_pending(
            self, proc_output, thread_configurator):
        # Pins the same-form-only carry-over semantics: an exact entry added
        # for an instance already matched by the wildcard has no known tid on
        # reapply, so it is reported as skipped and the wildcard keeps applying
        # to the instance until the target application re-announces.
        self._wait_for_all_instances(proc_output, thread_configurator)

        cfg = _read_config()
        exact_entry = dict(_PUBLISHER_ENTRIES[0])
        exact_entry['policy'] = 'SCHED_OTHER'
        exact_entry['priority'] = 15
        exact_entry['affinity'] = []
        cfg['callback_groups'] = cfg['callback_groups'] + [exact_entry]
        _write_config(cfg)

        response = _call_reapply()
        self.assertTrue(
            response.success,
            f'reapply failed unexpectedly: error_message={response.error_message!r}',
        )
        # The pending exact entry and the wildcard instance share the same
        # reporting key: skipped for the former, applied for the latter.
        key = f"{_WILDCARD_DOMAIN_ID}:{exact_entry['id']}"
        self.assertIn(
            key, list(response.skipped_callback_groups),
            'exact entry added while its instance is wildcard-matched must '
            'stay pending until the next announcement',
        )
        self.assertEqual(
            list(response.applied_callback_groups).count(key), 1,
            'the instance must still be applied via the wildcard',
        )
        self.assertNotIn(key, list(response.failed_callback_groups))

        proc_output.assertWaitFor(
            'Reapply done:',
            timeout=10.0,
            process=thread_configurator,
        )


@launch_testing.post_shutdown_test()
class TestThreadConfiguratorWildcardShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -signal.SIGINT]
        )

    @classmethod
    def tearDownClass(cls):
        if os.path.isdir(CONFIG_DIR):
            shutil.rmtree(CONFIG_DIR)
