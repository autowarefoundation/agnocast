import ctypes
import ctypes.util
import os
import re

from ament_index_python.packages import get_package_prefix
from ros2cli.verb import VerbExtension


def _parse_semver(version_str):
    """Parse a version string into (major, minor, patch) tuple, or None on failure."""
    m = re.match(r'^(\d+)\.(\d+)\.(\d+)$', version_str)
    if m:
        return (int(m.group(1)), int(m.group(2)), int(m.group(3)))
    return None


def _get_lib_path(package_name, lib_filename):
    """Resolve absolute path to a shared library from its ROS2 package prefix."""
    try:
        prefix = get_package_prefix(package_name)
        path = os.path.join(prefix, 'lib', lib_filename)
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return None


def _check_version_compatibility(kmod_version, agnocastlib_version, heaphook_version):
    """Check version compatibility using the same rules as agnocastlib's is_version_consistent.

    Rules:
      - heaphook and agnocastlib must match exactly (major.minor.patch)
      - kmod and agnocastlib must share the same major.minor
    """
    warnings = []

    # Extract the leading version string (strip trailing warnings like "(WARN: ...)")
    def extract_version(v):
        return v.split()[0] if v else v

    lib_ver = _parse_semver(extract_version(agnocastlib_version))
    heaphook_ver = _parse_semver(extract_version(heaphook_version))
    kmod_ver = _parse_semver(extract_version(kmod_version))

    if lib_ver and heaphook_ver:
        if lib_ver != heaphook_ver:
            warnings.append(
                'Agnocast Heaphook and Agnocastlib versions must match exactly '
                '(major.minor.patch). '
                f'heaphook={extract_version(heaphook_version)}, '
                f'agnocastlib={extract_version(agnocastlib_version)}')

    if lib_ver and kmod_ver:
        if lib_ver[:2] != kmod_ver[:2]:
            warnings.append(
                'Agnocast Kernel Module and Agnocastlib must share the same '
                'major.minor version. '
                f'kmod={extract_version(kmod_version)}, '
                f'agnocastlib={extract_version(agnocastlib_version)}')

    return warnings


class VersionVerb(VerbExtension):
    """Show version information for Agnocast components."""

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        kmod_version = self._get_kmod_version()
        agnocastlib_version = self._get_agnocastlib_version()
        heaphook_version = self._get_heaphook_version()
        ros2agnocast_version = self._get_ros2agnocast_version()

        print(f'agnocast_kmod:     {kmod_version}')
        print(f'agnocastlib:       {agnocastlib_version}')
        print(f'agnocast_heaphook: {heaphook_version}')
        print(f'ros2agnocast:      {ros2agnocast_version}')

        # Version compatibility checks (same rules as agnocastlib's is_version_consistent)
        warnings = _check_version_compatibility(
            kmod_version, agnocastlib_version, heaphook_version)
        for w in warnings:
            print(f'\nWARN: {w}')

        return 0

    def _get_kmod_version(self):
        # Try /sys/module first (available when MODULE_VERSION is set)
        sys_path = '/sys/module/agnocast/version'
        if os.path.exists(sys_path):
            try:
                with open(sys_path) as f:
                    return f.read().strip()
            except OSError:
                pass

        # Fall back to ioctl via wrapper library
        lib_path = _get_lib_path('agnocast_ioctl_wrapper', 'libagnocast_ioctl_wrapper.so')
        if lib_path is None:
            return '(not available - ioctl wrapper not found)'
        try:
            lib = ctypes.CDLL(lib_path)
            lib.get_agnocast_kmod_version.argtypes = []
            lib.get_agnocast_kmod_version.restype = ctypes.c_void_p
            lib.free_agnocast_kmod_version.argtypes = [ctypes.c_void_p]
            lib.free_agnocast_kmod_version.restype = None

            version_ptr = lib.get_agnocast_kmod_version()
            if version_ptr:
                version = ctypes.c_char_p(version_ptr).value.decode('utf-8')
                lib.free_agnocast_kmod_version(version_ptr)
                return version
            return '(not available - module not loaded?)'
        except OSError:
            return '(not available - ioctl wrapper not found)'
        except AttributeError:
            return '(not available - ioctl wrapper is outdated or not installed)'

    def _get_agnocastlib_version(self):
        lib_path = _get_lib_path('agnocastlib', 'libagnocast.so')
        if lib_path is None:
            return '(not available - library not found)'
        try:
            lib = ctypes.CDLL(lib_path)
            lib.agnocast_get_version.argtypes = []
            lib.agnocast_get_version.restype = ctypes.c_char_p
            version = lib.agnocast_get_version()
            if version:
                return version.decode('utf-8')
            return '(not available)'
        except OSError:
            return '(not available - library not found)'
        except AttributeError:
            return '(not available - agnocastlib is outdated or not installed)'

    def _get_heaphook_version(self):
        # Find heaphook from LD_PRELOAD, which is how it's loaded at runtime.
        lib_path = None
        not_in_ld_preload = False
        ld_preload = os.environ.get('LD_PRELOAD', '')
        for entry in ld_preload.split(':'):
            if 'libagnocast_heaphook.so' in entry:
                if os.path.isabs(entry) and os.path.exists(entry):
                    lib_path = entry
                else:
                    # Bare soname (e.g., "libagnocast_heaphook.so") — resolve via
                    # the dynamic linker search path.
                    resolved = ctypes.util.find_library('agnocast_heaphook')
                    if resolved:
                        lib_path = resolved
                    else:
                        # find_library returns the soname; try loading directly
                        lib_path = entry
                break

        if lib_path is None:
            not_in_ld_preload = True
            # LD_PRELOAD not set; fall back to the default install path.
            ros_distro = os.environ.get('ROS_DISTRO', '')
            if ros_distro:
                candidate = f'/opt/ros/{ros_distro}/lib/libagnocast_heaphook.so'
                if os.path.exists(candidate):
                    lib_path = candidate

        if lib_path is None:
            return '(not available - library not found)'

        # Load with RTLD_LOCAL | RTLD_LAZY to prevent the heaphook's malloc/free
        # symbols from replacing the process-wide allocator.
        try:
            lib = ctypes.CDLL(
                lib_path,
                mode=os.RTLD_LAZY | ctypes.RTLD_LOCAL,
            )
            lib.agnocast_heaphook_get_version.argtypes = []
            lib.agnocast_heaphook_get_version.restype = ctypes.c_char_p
            version = lib.agnocast_heaphook_get_version()
            if version:
                ver = version.decode('utf-8')
                if not_in_ld_preload:
                    return f'{ver} (WARN: not in LD_PRELOAD, found at {lib_path})'
                return ver
            return '(not available)'
        except OSError:
            return '(not available - library not found)'
        except AttributeError:
            return '(not available - heaphook is outdated or not installed)'

    def _get_ros2agnocast_version(self):
        try:
            from importlib.metadata import version
            return version('ros2agnocast')
        except Exception:
            return '(not available)'
