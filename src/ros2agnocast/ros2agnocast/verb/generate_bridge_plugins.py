import argparse
import os
import re
import subprocess
import sys
from importlib.resources import files

import em
from ros2cli.verb import VerbExtension


def camel_to_snake(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


class GenerateBridgePluginsVerb(VerbExtension):
    """Generate bridge plugins for Performance Bridge mode."""

    def add_arguments(self, parser, cli_name):
        parser.formatter_class = argparse.RawDescriptionHelpFormatter
        parser.epilog = '''Example:
  ros2 agnocast generate-bridge-plugins --message-types std_msgs/msg/String geometry_msgs/msg/Pose
  ros2 agnocast generate-bridge-plugins --all
'''
        parser.add_argument(
            '--output-dir',
            default='./agnocast_bridge_plugins',
            help='Output directory for generated package (default: ./agnocast_bridge_plugins)'
        )
        group = parser.add_mutually_exclusive_group(required=True)
        group.add_argument(
            '--message-types',
            nargs='+',
            metavar='TYPE',
            help='Space-separated list of message types (e.g., std_msgs/msg/String geometry_msgs/msg/Pose)'
        )
        group.add_argument(
            '--all',
            action='store_true',
            help='Generate for all available message types (via ros2 interface list -m)'
        )

    def main(self, *, args):
        output_dir = os.path.abspath(args.output_dir)

        if args.all:
            message_types = self._get_all_message_types()
            if not message_types:
                print('Error: No message types found.', file=sys.stderr)
                return 1
        else:
            message_types = args.message_types

        message_types = self._validate_message_types(message_types)
        if not message_types:
            print('Error: No valid message types provided.', file=sys.stderr)
            return 1

        print(f'Generating bridge plugins for {len(message_types)} message type(s)...')
        print(f'Output directory: {output_dir}')

        try:
            self._generate_package(output_dir, message_types)
        except Exception as e:
            print(f'Error: Failed to generate package: {e}', file=sys.stderr)
            return 1

        print(f'\nSuccessfully generated package at: {output_dir}')
        print('\nNext steps:')
        print(f'  1. colcon build --packages-select agnocast_bridge_plugins')
        print(f'  2. source install/setup.bash')
        print(f'  3. export AGNOCAST_BRIDGE_MODE=performance')
        return 0

    def _get_all_message_types(self):
        """Get all available message types via ros2 interface list -m."""
        try:
            result = subprocess.run(
                ['ros2', 'interface', 'list', '-m'],
                capture_output=True,
                text=True,
                check=True
            )
            return [stripped for line in result.stdout.strip().split('\n') if '/' in (stripped := line.strip())]
        except subprocess.CalledProcessError as e:
            print(f'Error: Failed to run "ros2 interface list -m": {e}', file=sys.stderr)
            return []

    def _validate_message_types(self, message_types):
        """Validate message type format (package/msg/Type)."""
        valid_types = []
        for msg_type in message_types:
            msg_type = msg_type.strip()
            parts = msg_type.split('/')
            if len(parts) == 3 and parts[1] == 'msg':
                valid_types.append(msg_type)
            else:
                print(f'Warning: Invalid message type format: {msg_type} (expected package/msg/Type)', file=sys.stderr)
        return valid_types

    def _generate_package(self, output_dir, message_types):
        """Generate the complete colcon package."""
        src_dir = os.path.join(output_dir, 'src')
        os.makedirs(src_dir, exist_ok=True)

        package_names = set()
        for msg_type in message_types:
            pkg_name = msg_type.split('/')[0]
            package_names.add(pkg_name)

        templates_pkg = files('ros2agnocast.templates')

        # Generate C++ source files
        for msg_type in message_types:
            self._generate_plugin_source(src_dir, msg_type, 'r2a', templates_pkg)
            self._generate_plugin_source(src_dir, msg_type, 'a2r', templates_pkg)

        # Generate CMakeLists.txt
        self._generate_cmake(output_dir, message_types, package_names, templates_pkg)

        # Generate package.xml
        self._generate_package_xml(output_dir, package_names, templates_pkg)

    def _generate_plugin_source(self, src_dir, msg_type, direction, templates_pkg):
        """Generate a single plugin C++ source file."""
        flat_type = msg_type.replace('/', '_')
        output_file = os.path.join(src_dir, f'register_{direction}_{flat_type}.cpp')

        cpp_type = msg_type.replace('/', '::')

        parts = msg_type.split('/')
        parts[-1] = camel_to_snake(parts[-1])
        header_path = '/'.join(parts) + '.hpp'

        function_name = f'create_{direction}_bridge'

        data = {
            'msg_type': msg_type,
            'cpp_type': cpp_type,
            'header_path': header_path,
            'function_name': function_name
        }

        template_file = templates_pkg.joinpath(f'{direction}_bridge_plugin.cpp.em')
        template_content = template_file.read_text()

        with open(output_file, 'w') as f:
            interpreter = em.Interpreter(output=f, globals=data)
            interpreter.string(template_content)
            interpreter.shutdown()

    def _generate_cmake(self, output_dir, message_types, package_names, templates_pkg):
        """Generate CMakeLists.txt for the plugin package."""
        output_file = os.path.join(output_dir, 'CMakeLists.txt')

        data = {
            'message_types': message_types,
            'package_names': sorted(package_names)
        }

        template_file = templates_pkg.joinpath('CMakeLists.txt.em')
        template_content = template_file.read_text()

        with open(output_file, 'w') as f:
            interpreter = em.Interpreter(output=f, globals=data)
            interpreter.string(template_content)
            interpreter.shutdown()

    def _generate_package_xml(self, output_dir, package_names, templates_pkg):
        """Generate package.xml for the plugin package."""
        output_file = os.path.join(output_dir, 'package.xml')

        data = {
            'package_names': sorted(package_names)
        }

        template_file = templates_pkg.joinpath('package.xml.em')
        template_content = template_file.read_text()

        with open(output_file, 'w') as f:
            interpreter = em.Interpreter(output=f, globals=data)
            interpreter.string(template_content)
            interpreter.shutdown()
