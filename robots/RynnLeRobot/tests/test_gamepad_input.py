#!/usr/bin/env python3
"""
Gamepad Input Test Script

This script tests if the Logitech F710 gamepad is properly connected
and reading inputs correctly.

Usage:
    cd robots/RynnLeRobot
    python -m tests.test_gamepad_input
"""

import pygame
import sys
import time


def initialize_gamepad():
    """Initialize pygame and the gamepad."""
    try:
        pygame.init()
        pygame.joystick.init()
        
        # Check if any joysticks are available
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("No joysticks found. Please connect your Logitech F710 gamepad.")
            return None
            
        # Use the first available joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        
        print(f"Detected joystick: {joystick.get_name()}")
        print(f"Number of axes: {joystick.get_numaxes()}")
        print(f"Number of buttons: {joystick.get_numbuttons()}")
        print(f"Number of hats: {joystick.get_numhats()}")
        print("\nControls:")
        print("  Left joystick Y-axis: Forward/backward motion")
        print("  Right joystick X-axis: Turning")
        print("Press Ctrl+C to exit\n")
        
        return joystick
        
    except Exception as e:
        print(f"Failed to initialize gamepad: {e}")
        return None


def main():
    """Main function to test gamepad input."""
    print("Logitech F710 Gamepad Input Test")
    print("=" * 40)
    
    joystick = initialize_gamepad()
    if not joystick:
        sys.exit(1)
    
    # F710 gamepad axis mapping
    LEFT_JOYSTICK_Y_AXIS = 1  # Forward/backward motion
    RIGHT_JOYSTICK_X_AXIS = 3  # Turning
    
    try:
        print("Reading gamepad inputs (Press Ctrl+C to exit)...")
        last_print_time = time.time()
        
        while True:
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    break
            
            # Read joystick values
            left_y = joystick.get_axis(LEFT_JOYSTICK_Y_AXIS)
            right_x = joystick.get_axis(RIGHT_JOYSTICK_X_AXIS)
            
            # Print values every 0.1 seconds
            current_time = time.time()
            if current_time - last_print_time > 0.1:
                print(f"Left Y: {left_y:6.3f} | Right X: {right_x:6.3f}")
                last_print_time = current_time
            
            time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error during gamepad input reading: {e}")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()