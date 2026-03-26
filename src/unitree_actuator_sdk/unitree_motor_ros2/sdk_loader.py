import ctypes
import importlib
from pathlib import Path
import platform


def _native_dir() -> Path:
    return Path(__file__).resolve().parent / 'native'


def _platform_library_name() -> str:
    machine = platform.machine().lower()
    if machine in ('x86_64', 'amd64'):
        return 'libUnitreeMotorSDK_Linux64.so'
    if machine in ('aarch64', 'arm64'):
        return 'libUnitreeMotorSDK_Arm64.so'
    raise RuntimeError(f'Unsupported architecture: {platform.machine()}')


def load_sdk():
    native_dir = _native_dir()
    ctypes.CDLL(str(native_dir / _platform_library_name()), mode=ctypes.RTLD_GLOBAL)

    try:
        module = importlib.import_module('.' + '_unitree_actuator_sdk', package=__package__)
    except ImportError as exc:
        raise RuntimeError(
            'Failed to load the bundled Unitree Python extension. '
            'Rebuild the package in this workspace to produce a Python-compatible binary.'
        ) from exc
    return module
