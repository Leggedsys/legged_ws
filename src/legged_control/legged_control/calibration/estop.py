"""Background thread that monitors a gamepad button for emergency stop."""

from __future__ import annotations
import threading
from typing import Callable

from sensor_msgs.msg import Joy


class EStopMonitor:
    """Watches /joy via a RosClient and fires a callback when the ESTOP button
    is pressed.

    After the callback fires, the monitor is automatically re-armed so it will
    fire again if pressed a second time.

    Args:
        ros_client: A RosClient instance already spinning.
        button_index: Index into Joy.buttons array.
        on_estop: Callable invoked (from the monitor thread) when button pressed.
    """

    def __init__(self,
                 ros_client,
                 button_index: int,
                 on_estop: Callable[[], None]) -> None:
        self._client = ros_client
        self._btn = button_index
        self._on_estop = on_estop
        self._running = False
        self._thread: threading.Thread | None = None
        self._prev_pressed = False

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _run(self) -> None:
        import time
        while self._running:
            joy = self._client.get_joy()
            if joy is not None and self._btn < len(joy.buttons):
                pressed = bool(joy.buttons[self._btn])
                if pressed and not self._prev_pressed:
                    self._on_estop()
                self._prev_pressed = pressed
            time.sleep(0.02)


class PauseController:
    """Coordinates pause/resume state between EStopMonitor and phase code.

    Usage in a phase:
        pause_ctrl.check()   # call periodically; blocks if paused
    """

    def __init__(self, motor_manager, ui_module) -> None:
        self._mm = motor_manager
        self._ui = ui_module
        self._paused = threading.Event()
        self._paused.set()   # not paused initially (set = allowed to proceed)

    def trigger(self) -> None:
        """Called by EStopMonitor. Zeros torque and enters paused state."""
        self._paused.clear()
        self._mm.zero_torque()
        self._ui.warn('\nEMERGENCY STOP — motors zeroed.')

    def check(self) -> None:
        """Block until not paused. Call this in any phase loop."""
        if not self._paused.is_set():
            answer = self._ui.prompt('Paused. Continue? [y/n]:')
            if answer.lower() == 'y':
                self._paused.set()
                self._ui.info('Resuming...')
            else:
                self._ui.info('Exiting.')
                raise SystemExit(0)
