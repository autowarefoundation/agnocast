import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

import launch
import launch.actions
import launch.event_handlers
import launch.events.process
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from ament_index_python.packages import get_package_prefix

CONFIG_DIR = os.path.join(
    tempfile.gettempdir(), 'agnocast_test_thread_configurator_restart'
)
CONFIG_FILE = os.path.join(CONFIG_DIR, 'template.yaml')


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

    # Wait for DDS discovery and data collection
    time.sleep(5)

    # Shutdown app first, then prerun (so prerun captures all data)
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


def generate_test_description():
    _run_prerun()

    thread_configurator = launch_ros.actions.Node(
        package='agnocast_cie_thread_configurator',
        executable='thread_configurator_node',
        name='thread_configurator_node',
        output='screen',
        parameters=[{'config_file': CONFIG_FILE}],
    )

    # First instance of the target application.
    test_app_1 = launch_ros.actions.Node(
        package='agnocast_e2e_test',
        executable='test_cie_publisher',
        output='screen',
    )

    # Second instance, started after the first exits.
    test_app_2 = launch_ros.actions.Node(
        package='agnocast_e2e_test',
        executable='test_cie_publisher',
        output='screen',
    )

    return (
        launch.LaunchDescription([
            launch.actions.SetEnvironmentVariable(
                'RCUTILS_LOGGING_BUFFERED_STREAM', '0'
            ),

            # T=0: Start thread_configurator_node
            thread_configurator,

            # T=2: Start first target app (DDS discovery needs time)
            launch.actions.TimerAction(
                period=2.0,
                actions=[test_app_1],
            ),

            # T=6: Send SIGINT to the first target app
            launch.actions.TimerAction(
                period=6.0,
                actions=[
                    launch.actions.EmitEvent(
                        event=launch.events.process.SignalProcess(
                            signal_number=signal.SIGINT,
                            process_matcher=lambda action: action is test_app_1,
                        )
                    ),
                ],
            ),

            # When the first app exits, start the second app
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=test_app_1,
                    on_exit=[test_app_2],
                )
            ),

            # T=12: ReadyToTest (enough time for second round of configuration)
            launch.actions.TimerAction(
                period=12.0,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {
            'thread_configurator': thread_configurator,
            'test_app_1': test_app_1,
            'test_app_2': test_app_2,
        },
    )


class TestThreadConfiguratorRestart(unittest.TestCase):

    def test_reapply_logged(self, proc_output, thread_configurator):
        """Verify that the re-application path was exercised on restart."""
        proc_output.assertWaitFor(
            'Re-applying configuration',
            timeout=15.0,
            process=thread_configurator,
        )

    def test_first_app_publishes(self, proc_output, test_app_1):
        proc_output.assertWaitFor(
            'Publishing:',
            timeout=15.0,
            process=test_app_1,
        )

    def test_second_app_publishes(self, proc_output, test_app_2):
        proc_output.assertWaitFor(
            'Publishing:',
            timeout=15.0,
            process=test_app_2,
        )


@launch_testing.post_shutdown_test()
class TestThreadConfiguratorRestartShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -signal.SIGINT]
        )

    @classmethod
    def tearDownClass(cls):
        if os.path.isdir(CONFIG_DIR):
            shutil.rmtree(CONFIG_DIR)
