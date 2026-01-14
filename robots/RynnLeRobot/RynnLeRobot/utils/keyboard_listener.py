#!/usr/bin/env python3
"""Keyboard listener for recording control."""

import logging
import traceback
from functools import cache


@cache
def is_headless():
    """Detects if python is running without a monitor."""
    try:
        import pynput  # noqa

        return False
    except Exception:
        print(
            "Error trying to import pynput. Switching to headless mode. "
            "As a result, you won't be able to change the control flow with keyboards. "
            "For more info, see traceback below.\n"
        )
        traceback.print_exc()
        print()
        return True


def init_keyboard_listener():
    """Initialize keyboard listener for recording controls."""
    # Allow to exit early while recording an episode or resetting the environment,
    # by tapping the right arrow key '->'. This might require a sudo permission
    # to allow your terminal to monitor keyboard events.
    events = {}
    events["exit_early"] = False
    events["rerecord_episode"] = False
    events["stop_recording"] = False

    if is_headless():
        logging.warning(
            "Headless environment detected. Keyboard inputs will not be available."
        )
        listener = None
        return listener, events

    # Only import pynput if not in a headless environment
    try:
        from pynput import keyboard
    except ImportError:
        logging.warning("pynput not available. Keyboard controls disabled.")
        listener = None
        return listener, events

    def on_press(key):
        try:
            if key == keyboard.Key.right:
                print(
                    "\n→ Right arrow pressed: Ending episode early and saving data..."
                )
                events["exit_early"] = True
            elif key == keyboard.Key.left:
                print("\n← Left arrow pressed: Discarding episode and re-recording...")
                events["rerecord_episode"] = True
                events["exit_early"] = True
            elif key == keyboard.Key.esc:
                print("\n⏹️ Escape pressed: Stopping entire recording session...")
                events["stop_recording"] = True
                events["exit_early"] = True
        except Exception as e:
            print(f"Error handling key press: {e}")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print("\n🎮 Keyboard Controls (Active During Recording):")
    print("   → (Right Arrow): End episode early and SAVE partial data")
    print("   ← (Left Arrow):  Discard episode and re-record from start")
    print("   Esc:             Stop entire recording session")
    print(
        "   💡 Note: Press right arrow if you want to end episode early but keep the data"
    )
    print(
        "           Press left arrow if you made a mistake and want to redo completely"
    )
    print()

    return listener, events


def cleanup_keyboard_listener(listener):
    """Stop the keyboard listener."""
    if listener:
        try:
            listener.stop()
            listener.join()
        except Exception as e:
            logging.warning(f"Error stopping keyboard listener: {e}")
