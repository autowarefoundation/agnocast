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
    tempfile.gettempdir(), 'agnocast_test_thread_configurator_reapply'
)
CONFIG_FILE = os.path.join(CONFIG_DIR, 'template.yaml')

REAPPLY_SERVICE = '/thread_configurator_node/reapply_config'


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


# Snapshot of the prerun-generated YAML, restored by setUp before each test.
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


class TestThreadConfiguratorReapply(unittest.TestCase):
    # Tests wait for 'Received CallbackGroupInfo' (not 'All applied') because
    # the publisher's worker thread race with the configurator can leave the
    # initial apply pass incomplete; receiving one CallbackGroupInfo is the
    # precondition the reapply tests actually need.

    def setUp(self):
        _restore_original_config()

    def test_configurator_receives_registration(self, proc_output, thread_configurator):
        proc_output.assertWaitFor(
            'Received CallbackGroupInfo',
            timeout=20.0,
            process=thread_configurator,
        )

    def test_reapply_after_priority_change(self, proc_output, thread_configurator):
        proc_output.assertWaitFor(
            'Received CallbackGroupInfo',
            timeout=20.0,
            process=thread_configurator,
        )

        cfg = _read_config()
        self.assertGreater(
            len(cfg.get('callback_groups', [])), 0,
            'prerun must produce at least one callback_group',
        )
        first = cfg['callback_groups'][0]
        # SCHED_OTHER + positive nice value: lowering priority needs no
        # CAP_SYS_NICE, which is not guaranteed in CI sandboxes.
        first['policy'] = 'SCHED_OTHER'
        first['priority'] = 10
        first.setdefault('affinity', [])
        _write_config(cfg)

        response = _call_reapply()
        self.assertTrue(
            response.success,
            f'reapply failed unexpectedly: error_message={response.error_message!r}',
        )
        # Equality (not >=) catches both the silent no-op case and any
        # future bug that double-routes an entry into multiple arrays.
        outcomes_total = (
            len(response.applied_callback_groups)
            + len(response.skipped_callback_groups)
            + len(response.failed_callback_groups)
        )
        self.assertEqual(
            outcomes_total,
            len(cfg['callback_groups']),
            'reapply must visit every callback_group entry exactly once',
        )
        first_key = f"{first['domain_id']}:{first['id']}"
        self.assertIn(
            first_key, list(response.applied_callback_groups),
            'modified callback_group must appear in applied_callback_groups',
        )

        proc_output.assertWaitFor(
            'Reapply done:',
            timeout=10.0,
            process=thread_configurator,
        )

    def test_reapply_accepts_added_entry(self, proc_output, thread_configurator):
        proc_output.assertWaitFor(
            'Received CallbackGroupInfo',
            timeout=20.0,
            process=thread_configurator,
        )

        cfg = _read_config()
        added_entry = {
            'id': 'fictitious_unannounced_cbg',
            'domain_id': 0,
            'policy': 'SCHED_OTHER',
            'priority': 0,
            'affinity': [],
        }
        cfg.setdefault('callback_groups', []).append(added_entry)
        _write_config(cfg)

        response = _call_reapply()
        self.assertTrue(
            response.success,
            f'reapply rejected unexpectedly: error_message={response.error_message!r}',
        )
        added_key = f"{added_entry['domain_id']}:{added_entry['id']}"
        self.assertIn(added_key, list(response.skipped_callback_groups))

    # 'zz_' prefix forces alphabetical-last execution: removing the cb-group
    # clears in-memory state and there is no re-announcement mechanism, so
    # this test would poison every test that runs after it.
    def test_zz_reapply_accepts_removed_entry(self, proc_output, thread_configurator):
        proc_output.assertWaitFor(
            'Received CallbackGroupInfo',
            timeout=20.0,
            process=thread_configurator,
        )

        cfg = _read_config()
        self.assertGreater(
            len(cfg.get('callback_groups', [])), 0,
            'prerun must produce at least one callback_group',
        )
        removed = cfg['callback_groups'].pop(0)
        _write_config(cfg)

        response = _call_reapply()
        self.assertTrue(
            response.success,
            f'reapply rejected unexpectedly: error_message={response.error_message!r}',
        )
        removed_key = f"{removed.get('domain_id', 0)}:{removed['id']}"
        for arr in (
            response.applied_callback_groups,
            response.skipped_callback_groups,
            response.failed_callback_groups,
        ):
            self.assertNotIn(removed_key, list(arr))

    def test_reapply_non_ros_thread_priority_change(self, proc_output, thread_configurator):
        proc_output.assertWaitFor(
            'Received NonRosThreadInfo',
            timeout=20.0,
            process=thread_configurator,
        )

        cfg = _read_config()
        nrts = cfg.get('non_ros_threads', []) or []
        target = next(
            (e for e in nrts if e.get('name') == 'test_non_ros_worker'), None
        )
        if target is None:
            self.skipTest(
                'prerun did not produce test_non_ros_worker non_ros_thread entry'
            )
        target['policy'] = 'SCHED_OTHER'
        target['priority'] = 10
        target.setdefault('affinity', [])
        _write_config(cfg)

        response = _call_reapply()
        self.assertTrue(
            response.success,
            f'reapply failed unexpectedly: error_message={response.error_message!r}',
        )
        self.assertIn(
            'test_non_ros_worker', list(response.applied_non_ros_threads),
            'modified non_ros_thread must appear in applied_non_ros_threads',
        )

    def test_reapply_rejects_invalid_policy(self, proc_output, thread_configurator):
        proc_output.assertWaitFor(
            'Received CallbackGroupInfo',
            timeout=20.0,
            process=thread_configurator,
        )

        cfg = _read_config()
        if len(cfg.get('callback_groups', [])) == 0:
            self.skipTest('no callback_groups produced by prerun')
        cfg['callback_groups'][0]['policy'] = 'NOT_A_POLICY'
        _write_config(cfg)

        response = _call_reapply()
        self.assertFalse(response.success)
        self.assertIn('Unknown scheduling policy', response.error_message)


@launch_testing.post_shutdown_test()
class TestThreadConfiguratorReapplyShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -signal.SIGINT]
        )

    @classmethod
    def tearDownClass(cls):
        if os.path.isdir(CONFIG_DIR):
            shutil.rmtree(CONFIG_DIR)
