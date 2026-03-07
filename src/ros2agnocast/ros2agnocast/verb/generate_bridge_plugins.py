import argparse
from enum import Enum, auto
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

class InterfaceType(Enum):
    MESSAGE = auto()
    SERVICE = auto()

class GenerateBridgePluginsVerb(VerbExtension):
    """Generate bridge plugins for Performance Bridge mode."""

    def add_arguments(self, parser, cli_name):
        parser.formatter_class = argparse.RawDescriptionHelpFormatter
        parser.epilog = '''Example:
  ros2 agnocast generate-bridge-plugins --message-types std_msgs/msg/String geometry_msgs/msg/Pose
  ros2 agnocast generate-bridge-plugins --service-types example_interfaces/srv/AddTwoInts
  ros2 agnocast generate-bridge-plugins --all
'''
        parser.add_argument(
            '--output-dir',
            default='./agnocast_bridge_plugins',
            help='Output directory for generated package (default: ./agnocast_bridge_plugins)'
        )
        parser.add_argument(
            '--message-types',
            nargs='+',
            metavar='TYPE',
            help='Space-separated list of message types (e.g., std_msgs/msg/String geometry_msgs/msg/Pose)'
        )
        parser.add_argument(
            '--service-types',
            nargs='+',
            metavar='TYPE',
            help='Space-separated list of service types (e.g., example_interfaces/srv/AddTwoInts)'
        )
        parser.add_argument(
            '--all',
            action='store_true',
            help='Generate for all available message and service types (via ros2 interface list)'
        )

    def main(self, *, args):
        if not args.all and not args.message_types and not args.service_types:
            print(
                'Error: At least one of --message-types, --service-types, or --all is required.',
                file=sys.stderr
            )
            return 1

        output_dir = os.path.abspath(args.output_dir)

        if args.all:
            message_types = self._get_all_types(InterfaceType.MESSAGE)
            service_types = self._get_all_types(InterfaceType.SERVICE)
        else:
            message_types = args.message_types or []
            service_types = args.service_types or []

        message_types = self._validate_types(message_types, InterfaceType.MESSAGE)
        service_types = self._validate_types(service_types, InterfaceType.SERVICE)
        if not message_types and not service_types:
            print('Error: No valid types provided.', file=sys.stderr)
            return 1

        print(f'Generating bridge plugins for {len(message_types)} message type(s) and {len(service_types)} service type(s)...')
        print(f'Output directory: {output_dir}')

        try:
            self._generate_package(output_dir, message_types, service_types)
        except Exception as e:
            print(f'Error: Failed to generate package: {e}', file=sys.stderr)
            return 1

        print(f'\nSuccessfully generated package at: {output_dir}')
        print('\nNext steps:')
        print(f'  1. colcon build --packages-select agnocast_bridge_plugins')
        print(f'  2. source install/setup.bash')
        print(f'  3. export AGNOCAST_BRIDGE_MODE=performance')
        return 0

    def _get_all_types(self, interface_type):
        """Get all available message or service types via ros2 interface list."""
        try:
            opt = '-m' if interface_type == InterfaceType.MESSAGE else '-s'
            result = subprocess.run(
                ['ros2', 'interface', 'list', opt],
                capture_output=True,
                text=True,
                check=True
            )
            return [stripped for line in result.stdout.strip().split('\n') if '/' in (stripped := line.strip())]
        except subprocess.CalledProcessError as e:
            print(f'Error: Failed to run "ros2 interface list {opt}": {e}', file=sys.stderr)
            return []

    def _validate_types(self, types, interface_type):
        """Validate type format (package/msg/Type for messages, package/srv/Type for services)."""
        valid_types = []
        expected_mid = 'msg' if interface_type == InterfaceType.MESSAGE else 'srv'
        for t in types:
            t = t.strip()
            parts = t.split('/')
            if len(parts) == 3 and parts[1] == expected_mid:
                valid_types.append(t)
            else:
                print(f'Warning: Invalid {interface_type.name.lower()} type format: {t} (expected package/{expected_mid}/Type)', file=sys.stderr)
        return valid_types

    def _generate_package(self, output_dir, message_types, service_types):
        """Generate the complete colcon package."""
        src_dir = os.path.join(output_dir, 'src')
        os.makedirs(src_dir, exist_ok=True)

        package_names = set()
        for msg_type in message_types:
            package_names.add(msg_type.split('/')[0])
        for srv_type in service_types:
            package_names.add(srv_type.split('/')[0])

        templates_pkg = files('ros2agnocast.templates')

        # Generate C++ source files
        for msg_type in message_types:
            self._generate_plugin_source(src_dir, msg_type, InterfaceType.MESSAGE, templates_pkg)
        for srv_type in service_types:
            self._generate_plugin_source(src_dir, srv_type, InterfaceType.SERVICE, templates_pkg)

        # Generate CMakeLists.txt
        self._generate_cmake(output_dir, message_types, service_types, package_names, templates_pkg)

        # Generate package.xml
        self._generate_package_xml(output_dir, package_names, templates_pkg)

    def _generate_plugin_source(self, src_dir, typ, interface_type, templates_pkg):
        """Generate a single plugin C++ source file."""
        flat_type = typ.replace('/', '_')
        if interface_type == InterfaceType.MESSAGE:
            output_file = os.path.join(src_dir, f'bridge_plugin_{flat_type}.cpp')
        else:
            output_file = os.path.join(src_dir, f'service_bridge_plugin_{flat_type}.cpp')

        cpp_type = typ.replace('/', '::')

        parts = typ.split('/')
        parts[-1] = camel_to_snake(parts[-1])
        header_path = '/'.join(parts) + '.hpp'

        if interface_type == InterfaceType.MESSAGE:
            data = {
                'msg_type': typ,
                'cpp_type': cpp_type,
                'header_path': header_path,
            }
            template_file = templates_pkg.joinpath('bridge_plugin.cpp.em')
        else:
            data = {
                'srv_type': typ,
                'cpp_type': cpp_type,
                'header_path': header_path,
            }
            template_file = templates_pkg.joinpath('service_bridge_plugin.cpp.em')

        template_content = template_file.read_text()

        with open(output_file, 'w') as f:
            interpreter = em.Interpreter(output=f, globals=data)
            interpreter.string(template_content)
            interpreter.shutdown()

    def _generate_cmake(self, output_dir, message_types, service_types, package_names, templates_pkg):
        """Generate CMakeLists.txt for the plugin package."""
        output_file = os.path.join(output_dir, 'CMakeLists.txt')

        data = {
            'message_types': message_types,
            'service_types': service_types,
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
