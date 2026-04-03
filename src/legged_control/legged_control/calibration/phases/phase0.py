"""Phase 0: environment checks and gamepad emergency-stop binding."""

from __future__ import annotations
import os
import subprocess
import time

from legged_control.calibration import ui
from legged_control.calibration.config_io import ConfigIO


def run(config_path: str, ros_client) -> int:
    """Run environment checks and bind ESTOP button.

    Returns the button index bound as ESTOP, or -1 if no gamepad found
    and user chose keyboard-only mode.

    Raises SystemExit on any unrecoverable check failure.
    """
    ui.phase_banner(0, 'Environment Check + Gamepad Setup')

    _check_serial(config_path)
    _check_ros_env()
    _check_yaml_writable(config_path)
    _check_python_deps()

    ui.success('Environment checks passed.')
    return _setup_gamepad(config_path, ros_client)


def _check_serial(config_path: str) -> None:
    cfg = ConfigIO(config_path).read()
    port = cfg['control']['serial_port']
    if not os.path.exists(port):
        ui.error(f'Serial port {port} not found.')
        ui.info(f'  Fix: plug in the motor USB cable, or update control.serial_port in robot.yaml')
        raise SystemExit(1)
    ui.success(f'Serial port {port} found.')


def _check_ros_env() -> None:
    if not os.environ.get('ROS_DISTRO'):
        ui.error('ROS2 environment not sourced.')
        ui.info('  Fix: source /opt/ros/humble/setup.bash && source install/setup.bash')
        raise SystemExit(1)
    ui.success(f'ROS2 ({os.environ["ROS_DISTRO"]}) sourced.')


def _check_yaml_writable(config_path: str) -> None:
    if not os.access(config_path, os.W_OK):
        ui.error(f'robot.yaml is not writable: {config_path}')
        ui.info('  Fix: chmod u+w ' + config_path)
        raise SystemExit(1)
    ui.success('robot.yaml is writable.')


def _check_python_deps() -> None:
    missing = []
    try:
        import ruamel.yaml  # noqa: F401
    except ImportError:
        missing.append('ruamel.yaml')
    try:
        import colorama  # noqa: F401
    except ImportError:
        missing.append('colorama')
    if missing:
        ui.error(f'Missing Python packages: {", ".join(missing)}')
        ui.info('  Fix: pip install ' + ' '.join(missing))
        raise SystemExit(1)
    ui.success('Python dependencies present.')


def _setup_gamepad(config_path: str, ros_client) -> int:
    if not os.path.exists('/dev/input/js0'):
        ui.warn('No gamepad found at /dev/input/js0.')
        if not ui.confirm('Continue in keyboard-only mode? (Ctrl+C = only ESTOP)'):
            raise SystemExit(0)
        return -1

    joy_proc = subprocess.Popen(
        ['ros2', 'run', 'joy', 'joy_node'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

    ui.info('\nGamepad detected. Press the button you want as EMERGENCY STOP.')
    button_index = _wait_for_button_press(ros_client)
    ui.info(f'Button {button_index} detected.')

    ui.info(f'Press button {button_index} again to confirm.')
    confirmed = _wait_for_button_press(ros_client)

    if confirmed != button_index:
        ui.warn('Different button pressed. Using first button as ESTOP.')

    ConfigIO(config_path).patch({'teleop': {'btn_emergency_stop': button_index}})
    ui.set_estop_label(f'[BTN{button_index}=ESTOP]')
    ui.success(f'Emergency stop armed on button {button_index}.')
    return button_index


def _wait_for_button_press(ros_client, timeout: float = 30.0) -> int:
    """Block until any Joy button is pressed. Returns button index."""
    import time
    deadline = time.time() + timeout
    prev_buttons: list[int] = []
    while time.time() < deadline:
        joy = ros_client.get_joy()
        if joy is not None:
            buttons = list(joy.buttons)
            if prev_buttons and buttons != prev_buttons:
                for i, (old, new) in enumerate(zip(prev_buttons, buttons)):
                    if old == 0 and new == 1:
                        return i
            prev_buttons = buttons
        time.sleep(0.02)
    ui.error('Timeout waiting for button press.')
    raise SystemExit(1)
