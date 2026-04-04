"""
Keyboard-enabled Communicator Base

Extends CommunicatorBase with keyboard-driven task mode switching.
Communicators that need interactive keyboard control (e.g. teleop leaders,
LCM communicators) should inherit from this class instead of CommunicatorBase.

Usage:
    class MyCommunicator(KeyboardCommunicatorBase):
        def connect(self):
            # ... your connection logic ...
            self.start_keyboard_listener()

        def disconnect(self):
            self.stop_keyboard_listener()
            # ... your cleanup logic ...
"""

import sys
import time
import threading
import select

from RynnMotion.common.communicator_base import CommunicatorBase
from RynnMotion.manager.robot_manager import RobotManager


class KeyboardCommunicatorBase(CommunicatorBase):
    """CommunicatorBase with built-in keyboard listener for task mode switching.

    Provides:
        - Background thread reading stdin key presses
        - Configurable key-to-mode mapping via ``self._available_modes``
        - ``start_keyboard_listener()`` / ``stop_keyboard_listener()`` lifecycle
        - Default ``process_task_command()`` returning the current mode string
    """

    def __init__(self, robot_model: RobotManager, communicator_config, logger=None):
        super().__init__(
            robot_model=robot_model,
            communicator_config=communicator_config,
            logger=logger,
        )
        self._init_keyboard_listener()

    # ── Keyboard state & configuration ─────────────────────────────────

    def _init_keyboard_listener(self):
        """Initialize keyboard listener state."""
        self._keyboard_stop_event = threading.Event()
        self._keyboard_thread = None
        self._last_key_msg_time = 0.0

        # keyboard-driven task mode
        self._current_task_mode = "go_standby"
        self._available_modes = {
            "r": "run",
            "g": "go_standby",
        }

    # ── Thread lifecycle ───────────────────────────────────────────────

    def start_keyboard_listener(self):
        """Start the keyboard listener thread. Call from connect()."""
        self._keyboard_stop_event.clear()
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener, daemon=True
        )
        self._keyboard_thread.start()

    def stop_keyboard_listener(self):
        """Stop the keyboard listener thread. Call from disconnect()."""
        self._keyboard_stop_event.set()
        if self._keyboard_thread and self._keyboard_thread.is_alive():
            self._keyboard_thread.join(timeout=2.0)
            if self._keyboard_thread.is_alive():
                self.logger.warning(
                    "Keyboard listener thread did not terminate gracefully."
                )

    # ── Background thread ──────────────────────────────────────────────

    def _keyboard_listener(self):
        """Background thread that reads stdin key presses (Unix/Linux/macOS)."""
        mode_keys = ", ".join(
            f"'{k}' = {v}" for k, v in self._available_modes.items()
        )
        self.logger.info(f"Keyboard listener started. Available keys: {mode_keys}")

        while not self._keyboard_stop_event.is_set():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).strip().lower()
                if key in self._available_modes:
                    mode = self._available_modes[key]
                    self._current_task_mode = mode
                    self.logger.info(
                        f"Task command switched to: '{mode}' via key '{key}'"
                    )
                    self._last_key_msg_time = time.time()
                elif key:
                    self.logger.warning(
                        f"Unknown key pressed: '{key}'. Use: {mode_keys}"
                    )

    # ── Default process_task_command ───────────────────────────────────

    def process_task_command(self):
        """Return current task mode set by keyboard input."""
        return self._current_task_mode
