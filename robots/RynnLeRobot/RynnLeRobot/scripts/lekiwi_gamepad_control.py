#!/usr/bin/env python3
# Copyright 2025 Alibaba Damo Academy. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""LeKiwi gamepad control for chassis.

Usage:
    $ lekiwi-gamepad-control --mode real
"""

import argparse
import sys
import time
import yaml
import os
import pygame
import numpy as np

# Add the project root to the Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from RynnLeRobot.hardware.robots.lekiwi import LeKiwiChassisOnly
from RynnLeRobot.hardware.robots.lekiwi.config_lekiwi import LeKiwiConfig


class LeKiWiGamepadController:
    """Main controller class for LeKiWi gamepad control."""

    def __init__(self, mode: str = "real", frequency: int = 30, config_path: str = "configs/lekiwi.yaml"):
        """
        Initialize the LeKiWi gamepad controller.

        Args:
            mode: Control mode - "real" for real robot
            frequency: Control frequency in Hz
            config_path: Path to configuration file
        """
        self.mode = mode
        self.frequency = max(30, min(frequency, 100))  # LeKiwi typically uses lower frequencies
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path

        # Initialize components
        self.gamepad = None
        self.robot_interface = None
        self.is_running = False

        # Control parameters
        self.max_linear_velocity = 0.3  # m/s
        self.max_angular_velocity = 0.8  # rad/s
        self.deadzone = 0.1

        # F710 gamepad axis mapping
        self.LEFT_JOYSTICK_Y_AXIS = 1  # Forward/backward motion
        self.RIGHT_JOYSTICK_X_AXIS = 3  # Turning

        self.init_config()
        self.init_gamepad()
        self.init_robot_interface()

    def init_config(self):
        """Initialize configuration."""
        try:
            with open(self.config_path, "r") as f:
                config_dict = yaml.safe_load(f)
                self.config = LeKiwiConfig.from_dict(config_dict.get("robot", {}))
        except FileNotFoundError:
            print(f"Warning: Config file {self.config_path} not found, using defaults")
            self.config = LeKiwiConfig()

    def init_gamepad(self):
        """Initialize the gamepad."""
        try:
            pygame.init()
            pygame.joystick.init()
            
            # Check if any joysticks are available
            joystick_count = pygame.joystick.get_count()
            if joystick_count == 0:
                raise RuntimeError("No joysticks found. Please connect your Logitech F710 gamepad.")
                
            # Use the first available joystick
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            
            print(f"Detected joystick: {self.gamepad.get_name()}")
            print("✓ Gamepad initialized successfully")
        except Exception as e:
            print(f"✗ Failed to initialize gamepad: {e}")
            raise

    def init_robot_interface(self):
        """Initialize robot interface."""
        try:
            # Use the chassis-only robot class
            self.robot_interface = LeKiwiChassisOnly(self.config)
            self.robot_interface.connect()
            self.is_connected = True
            print(f"✓ Robot interface ready in {self.mode} mode")
        except Exception as e:
            print(f"✗ Failed to setup robot interface: {e}")
            raise

    def apply_deadzone(self, value, deadzone):
        """Apply deadzone filtering to joystick values."""
        if abs(value) < deadzone:
            return 0.0
        return value

    def apply_nonlinear_mapping(self, value):
        """
        Apply non-linear mapping to improve control feel.
        Uses a cubic function to provide fine control at low speeds
        and full range at high speeds.
        """
        # Cubic mapping: preserves sign, compresses small values
        return value * abs(value) * abs(value)

    def map_joystick_to_velocity(self, joystick_value, max_velocity):
        """
        Map joystick value (-1.0 to 1.0) to velocity (-max_velocity to max_velocity).
        
        Args:
            joystick_value: Raw joystick value from pygame (-1.0 to 1.0)
            max_velocity: Maximum velocity for this axis
            
        Returns:
            Mapped velocity value
        """
        # Apply deadzone
        filtered_value = self.apply_deadzone(joystick_value, self.deadzone)
        
        # Apply non-linear mapping for better control feel
        filtered_value = self.apply_nonlinear_mapping(filtered_value)
        
        # Scale to max velocity
        return filtered_value * max_velocity

    def get_gamepad_input(self):
        """
        Read input from the gamepad.
        
        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        if not self.gamepad:
            return 0.0, 0.0

        # Process pygame events
        for event in pygame.event.get():
            pass  # Just process events to prevent queue buildup

        # Read joystick values
        left_y = self.gamepad.get_axis(self.LEFT_JOYSTICK_Y_AXIS)
        right_x = self.gamepad.get_axis(self.RIGHT_JOYSTICK_X_AXIS)

        # Map to velocities
        # Invert Y-axis so up = forward
        linear_velocity = self.map_joystick_to_velocity(-left_y, self.max_linear_velocity)
        # Use X-axis as is for turning
        angular_velocity = self.map_joystick_to_velocity(right_x, self.max_angular_velocity)

        return linear_velocity, angular_velocity

    def run(self):
        """Run the main control loop."""
        print(f"LeKiWi Gamepad Control - Mode: {self.mode}")
        print("Controls:")
        print("  Left joystick Y-axis: Forward/backward motion")
        print("  Right joystick X-axis: Turning")
        print("Press Ctrl+C to stop")
        print("=" * 50)

        self.is_running = True

        try:
            while self.is_running:
                # Get start time for timing control
                start_time = time.time()

                # Get velocity commands from gamepad
                x_vel, theta_vel = self.get_gamepad_input()
                y_vel = 0.0  # No lateral movement in this control scheme

                # Print current velocities for debugging (only when there's significant input)
                if abs(x_vel) > 0.01 or abs(theta_vel) > 0.01:
                    print(f"Velocity commands: x={x_vel:.3f} m/s, theta={theta_vel:.3f} rad/s")

                # Send commands to robot
                # Create data dictionary with only chassis joint commands
                data = {
                    "x.vel": x_vel,
                    "y.vel": y_vel,
                    "theta.vel": theta_vel * 180 / np.pi  # Convert rad/s to deg/s for real robot
                }
                self.robot_interface.send_action(data)

                # Maintain control frequency
                elapsed_time = time.time() - start_time
                sleep_time = self.timestep - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nStopping LeKiWi gamepad control...")
        except Exception as e:
            print(f"\nError during control: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources."""
        print("Cleaning up...")

        # Send zero velocity command to stop
        try:
            stop_data = {
                "x.vel": 0.0,
                "y.vel": 0.0,
                "theta.vel": 0.0
            }
            self.robot_interface.send_action(stop_data)
        except:
            pass

        if self.gamepad:
            pygame.quit()
            print("✓ Gamepad disconnected")

        if self.robot_interface:
            self.robot_interface.disconnect()
            print("✓ Robot interface disconnected")

        print("✓ Cleanup completed")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="LeKiWi Gamepad Control")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["real"],
        default="real",
        help="Control mode: real (real robot)",
    )
    parser.add_argument(
        "--frequency", type=int, default=30, help="Control frequency in Hz"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="configs/lekiwi.yaml",
        help="Path to configuration file",
    )

    args = parser.parse_args()

    print(f"Mode: {args.mode}")
    print(f"Frequency: {args.frequency} Hz")

    try:
        controller = LeKiWiGamepadController(
            mode=args.mode,
            frequency=args.frequency,
            config_path=args.config,
        )
        controller.run()
    except Exception as e:
        print(f"Error running LeKiWi gamepad controller: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())