import ctypes
import os

from ros2cli.verb import VerbExtension

# Use c_void_p for malloc'd pointers to preserve the raw address.
# c_char_p auto-converts to Python bytes, losing the original pointer and causing
# crashes on free().
_c_void_p = ctypes.c_void_p


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
        try:
            lib = ctypes.CDLL('libagnocast_ioctl_wrapper.so')
            lib.get_agnocast_kmod_version.argtypes = []
            lib.get_agnocast_kmod_version.restype = _c_void_p
            lib.free_agnocast_kmod_version.argtypes = [_c_void_p]
            lib.free_agnocast_kmod_version.restype = None

            version_ptr = lib.get_agnocast_kmod_version()
            if version_ptr:
                version = ctypes.c_char_p(version_ptr).value.decode('utf-8')
                lib.free_agnocast_kmod_version(version_ptr)
                return version
            return '(not available - module not loaded?)'
        except OSError:
            return '(not available - ioctl wrapper not found)'

    def _get_agnocastlib_version(self):
        try:
            lib = ctypes.CDLL('libagnocast.so')
            lib.agnocast_get_version.argtypes = []
            lib.agnocast_get_version.restype = ctypes.c_char_p
            version = lib.agnocast_get_version()
            if version:
                return version.decode('utf-8')
            return '(not available)'
        except OSError:
            return '(not available - library not found)'

    def _get_heaphook_version(self):
        # Load with RTLD_LOCAL | RTLD_LAZY to prevent the heaphook's malloc/free
        # symbols from replacing the process-wide allocator.
        try:
            lib = ctypes.CDLL(
                'libagnocast_heaphook.so',
                mode=os.RTLD_LAZY | ctypes.RTLD_LOCAL,
            )
            lib.agnocast_heaphook_get_version.argtypes = []
            lib.agnocast_heaphook_get_version.restype = ctypes.c_char_p
            version = lib.agnocast_heaphook_get_version()
            if version:
                return version.decode('utf-8')
            return '(not available)'
        except OSError:
            return '(not available - library not found)'

    def _get_ros2agnocast_version(self):
        try:
            from importlib.metadata import version
            return version('ros2agnocast')
        except Exception:
            return '(not available)'
