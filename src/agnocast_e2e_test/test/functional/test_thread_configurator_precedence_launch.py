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

CONFIG_DIR = os.path.join(
    tempfile.gettempdir(), 'agnocast_test_thread_configurator_precedence'
)
CONFIG_FILE = os.path.join(CONFIG_DIR, 'template.yaml')

TARGET_NODE = '/test_publisher_component'
WILDCARD_ID = TARGET_NODE + '/*'

# The publisher's exact entries from the prerun output, all kept alongside a
# covering wildcard: with precedence intact every announced instance is taken
# by its exact entry and the wildcard never matches anything.
_PUBLISHER_ENTRIES = None


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
    global _PUBLISHER_ENTRIES
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
    wildcard_entry = {
        'id': WILDCARD_ID,
        'domain_id': _PUBLISHER_ENTRIES[0].get('domain_id', 0),
        'policy': 'SCHED_OTHER',
        'priority': 0,
        'affinity': [],
    }
    cfg['callback_groups'] = cfg['callback_groups'] + [wildcard_entry]
    _write_config(cfg)


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

    def test_every_instance_matches_an_entry(
            self, proc_output, thread_configurator):
        # Positive half of the precedence proof: the "| <id>" shape only
        # occurs in the matched-announcement log; the no-configuration path
        # logs "id=<id>" instead. Together with the post-shutdown check that
        # the wildcard never matched, this pins each instance to its exact
        # entry. The positive half is essential: without it, a run where
        # nothing is announced at all would pass vacuously.
        for entry in _PUBLISHER_ENTRIES:
            proc_output.assertWaitFor(
                f"| {entry['id']}",
                timeout=20.0,
                process=thread_configurator,
            )


@launch_testing.post_shutdown_test()
class TestThreadConfiguratorPrecedenceShutdown(unittest.TestCase):

    def test_wildcard_never_matched(self, proc_output):
        # Negative half of the precedence proof, deterministic only here: the
        # processes have exited, so the captured output is complete.
        # Recording an instance into a wildcard entry always logs this line,
        # so its absence proves the wildcard captured nothing.
        for event in proc_output:
            self.assertNotIn(
                'matched wildcard entry',
                event.text.decode(errors='replace'),
            )

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -signal.SIGINT]
        )

    @classmethod
    def tearDownClass(cls):
        if os.path.isdir(CONFIG_DIR):
            shutil.rmtree(CONFIG_DIR)
