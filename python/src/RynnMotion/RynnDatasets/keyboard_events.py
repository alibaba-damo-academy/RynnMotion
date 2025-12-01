#!/usr/bin/env python3

"""
Keyboard event handling for data recording control
Provides real-time control during recording sessions
"""

import logging
import threading
import time
from typing import Dict, Callable, Optional


try:
    import pynput
    from pynput import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    logging.warning("pynput not available. Keyboard controls disabled.")


class KeyboardEventHandler:
    """
    Handles keyboard events for recording control
    Based on LeRobot keyboard event system
    """
    
    def __init__(self):
        self.events = {
            "exit_early": False,
            "stop_recording": False, 
            "rerecord_episode": False,
            "continue": False,
        }
        
        self.callbacks = {}
        self.listener = None
        self.is_headless = not KEYBOARD_AVAILABLE
        
        if not self.is_headless:
            self._setup_listener()
    
    def _setup_listener(self):
        """Setup keyboard listener"""
        try:
            self.listener = keyboard.Listener(on_press=self._on_press)
            self.listener.start()
            logging.info("Keyboard event handler initialized")
        except Exception as e:
            logging.warning(f"Failed to setup keyboard listener: {e}")
            self.is_headless = True
    
    def _on_press(self, key):
        """Handle key press events"""
        try:
            if key == keyboard.Key.esc:
                # ESC: Emergency stop
                self.events["exit_early"] = True
                logging.info("ESC pressed: Emergency stop")
                self._trigger_callback("emergency_stop")
            
            elif key == keyboard.Key.left:
                # LEFT ARROW: Stop recording current episode (don't save)
                self.events["stop_recording"] = True
                self.events["rerecord_episode"] = True
                logging.info("LEFT ARROW pressed: Stop and rerecord episode")
                self._trigger_callback("rerecord_episode")
            
            elif key == keyboard.Key.right:
                # RIGHT ARROW: Stop recording current episode early (but save)
                self.events["stop_recording"] = True
                logging.info("RIGHT ARROW pressed: Stop recording early")
                self._trigger_callback("stop_early")
            
            elif key == keyboard.Key.enter:
                # ENTER: Continue to next episode (after pause)
                self.events["continue"] = True
                logging.info("ENTER pressed: Continue to next episode")
                self._trigger_callback("continue")
            
            elif key == keyboard.Key.space:
                # SPACE: Pause/resume
                logging.info("SPACE pressed: Pause/resume")
                self._trigger_callback("pause_resume")
            
            elif hasattr(key, 'char') and key.char == 'q':
                # Q: Quit gracefully
                self.events["exit_early"] = True
                logging.info("Q pressed: Graceful quit")
                self._trigger_callback("quit")
                
        except AttributeError:
            # Handle special keys that don't have char attribute
            pass
    
    def _trigger_callback(self, event_name: str):
        """Trigger registered callback for event"""
        if event_name in self.callbacks:
            try:
                self.callbacks[event_name]()
            except Exception as e:
                logging.error(f"Error in callback for {event_name}: {e}")
    
    def register_callback(self, event_name: str, callback: Callable):
        """Register callback for specific event"""
        self.callbacks[event_name] = callback
        logging.info(f"Registered callback for event: {event_name}")
    
    def get_events(self) -> Dict[str, bool]:
        """Get current event states"""
        return self.events.copy()
    
    def reset_events(self):
        """Reset all event flags"""
        for key in self.events:
            self.events[key] = False
    
    def reset_event(self, event_name: str):
        """Reset specific event flag"""
        if event_name in self.events:
            self.events[event_name] = False
    
    def wait_for_key(self, timeout: float = None) -> Optional[str]:
        """Wait for specific key press with optional timeout"""
        if self.is_headless:
            return None
        
        start_time = time.time()
        original_events = self.events.copy()
        
        while True:
            if timeout and (time.time() - start_time) > timeout:
                return None
            
            # Check for any event change
            for event_name, state in self.events.items():
                if state != original_events[event_name] and state:
                    return event_name
            
            time.sleep(0.01)  # Small sleep to prevent busy waiting
    
    def print_controls(self):
        """Print available keyboard controls"""
        controls = [
            "=== Keyboard Controls ===",
            "ESC: Emergency stop",
            "LEFT ARROW: Discard current episode and rerecord",
            "RIGHT ARROW: Stop current episode early (but save)",
            "ENTER: Continue to next episode",
            "SPACE: Pause/resume",
            "Q: Quit gracefully",
            "========================="
        ]
        
        for line in controls:
            logging.info(line)
            print(line)
    
    def cleanup(self):
        """Cleanup resources"""
        if self.listener:
            self.listener.stop()
            logging.info("Keyboard event handler cleaned up")


class RecordingController:
    """
    High-level controller for recording sessions with keyboard controls
    """
    
    def __init__(self):
        self.keyboard_handler = KeyboardEventHandler()
        self.is_paused = False
        self.current_episode = 0
        
        # Register callbacks
        self.keyboard_handler.register_callback("pause_resume", self._toggle_pause)
        self.keyboard_handler.register_callback("emergency_stop", self._emergency_stop)
    
    def _toggle_pause(self):
        """Toggle pause state"""
        self.is_paused = not self.is_paused
        state = "paused" if self.is_paused else "resumed"
        logging.info(f"Recording {state}")
    
    def _emergency_stop(self):
        """Handle emergency stop"""
        logging.warning("Emergency stop activated!")
    
    def start_recording_session(self):
        """Start a new recording session"""
        logging.info("Starting recording session")
        self.keyboard_handler.print_controls()
        self.keyboard_handler.reset_events()
    
    def check_episode_control(self) -> Dict[str, bool]:
        """Check for episode control events"""
        events = self.keyboard_handler.get_events()
        
        # Handle pause
        if self.is_paused:
            logging.info("Recording paused. Press SPACE to resume.")
            while self.is_paused and not events.get("exit_early", False):
                time.sleep(0.1)
                events = self.keyboard_handler.get_events()
        
        return events
    
    def wait_for_episode_start(self, timeout: float = None) -> bool:
        """Wait for user to start next episode"""
        if self.keyboard_handler.is_headless:
            return True  # Auto-continue in headless mode
        
        logging.info("Press ENTER to start next episode, or ESC to quit")
        
        event = self.keyboard_handler.wait_for_key(timeout)
        
        if event == "exit_early":
            return False
        elif event == "continue":
            self.keyboard_handler.reset_event("continue")
            return True
        elif timeout is None:  # If no timeout, keep waiting
            return self.wait_for_episode_start()
        else:
            return True  # Timeout reached, auto-continue
    
    def cleanup(self):
        """Cleanup resources"""
        self.keyboard_handler.cleanup()


# Global instance for easy access
recording_controller = RecordingController()


def get_recording_controller() -> RecordingController:
    """Get global recording controller instance"""
    return recording_controller