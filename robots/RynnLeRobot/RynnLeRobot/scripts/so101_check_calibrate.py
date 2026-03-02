#!/usr/bin/env python3
"""SO101 Calibration Check Script.

Moves the robot to predefined joint configurations to verify calibration.

Usage:
    $ check-calibrate        # Go to config 1 (all zeros)
    $ check-calibrate 1      # Go to config 1 (all zeros)
    $ check-calibrate 2      # Go to config 2 (point to E-F-12-13)
    $ check-calibrate 3      # Go to config 3 (point to H-J-8-9 and J-K-8-9)
"""

import argparse
import os
import time
import traceback
import numpy as np

from RynnLeRobot.interface.so101_interface import create_robot_interface
from RynnMotion.common.data.robot_state import RobotState
from RynnMotion.algorithms.policy_interpolator import lerp


# Predefined calibration configs (5 joints in radians + gripper 0-1)
CALIBRATION_CONFIGS = {
    1: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # All zeros (home)
    2: [0.5746, 0.8799, -0.6438, 0.4580, -1.4807, 0.05],  # point to E-F-12-13
    3: [0.0, -0.4035, 0.8823, 1.0487, -0.0184, 0.28],  # point to H-J-8-9 and J-K-8-9
    4: [0.0, 0.0, 1.57, 0.0, 0.0, 0.0],  # Elbow bent 90deg
}

# Rest position from so101_robot.xml keyframe "rest"
REST_POSITION = np.array([0.0, -1.75, 1.57, 1.18, 0.0, 0])

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

LERP_DURATION = 2.0  # seconds
CONTROL_RATE = 50  # Hz


def print_joint_positions(positions: np.ndarray, title: str = "Joint Positions"):
    """Print joint positions in a formatted way."""
    print(f"\n=== {title} ===")
    for i, (name, pos) in enumerate(zip(JOINT_NAMES, positions)):
        if i < 5:
            print(f"  {name:>15}: {pos:7.4f} rad ({np.degrees(pos):7.2f} deg)")
        else:
            print(f"  {name:>15}: {pos:7.4f}")
    print("=" * 40)


def send_joint_command(interface, positions: np.ndarray):
    """Send joint positions to robot."""
    robot_command = RobotState()
    robot_command.num_joints = len(positions)
    for i, pos in enumerate(positions):
        robot_command.joint_pos[i] = pos
    interface.set_robot_command(robot_command)


def lerp_to_position(interface, start_pos: np.ndarray, target_pos: np.ndarray, duration: float = LERP_DURATION):
    """Lerp from start to target position."""
    dt = 1.0 / CONTROL_RATE
    start_time = time.time()

    print(f"Moving to target position over {duration:.1f}s...")

    while True:
        elapsed = time.time() - start_time
        if elapsed >= duration:
            break

        current_target = lerp(start_pos, target_pos, elapsed, duration)
        send_joint_command(interface, current_target)

        time.sleep(dt)

    # Final command at exact target
    send_joint_command(interface, target_pos)
    print("Reached target position.")


def main():
    parser = argparse.ArgumentParser(
        description="SO101 Calibration Check",
        epilog="Configs: 1=zeros, 2=shoulder, 3=base, 4=elbow, 5=wrist_flex, 6=wrist_roll, 7=gripper",
    )
    parser.add_argument(
        "config",
        type=int,
        nargs="?",
        default=1,
        choices=list(CALIBRATION_CONFIGS.keys()),
        help="Config number (default: 1)",
    )
    parser.add_argument(
        "--config-path",
        type=str,
        default="configs/so101.yaml",
        help="Path to config file",
    )

    args = parser.parse_args()

    target_config = np.array(CALIBRATION_CONFIGS[args.config])

    print("SO101 Calibration Check")
    print("=" * 50)
    print(f"Target config: {args.config}")
    print_joint_positions(target_config, f"Target Config {args.config}")
    print("\nPress Ctrl+C to return to rest position and exit.")
    print("=" * 50)

    interface = None
    exit_requested = False

    try:
        # Create and connect interface
        interface = create_robot_interface(
            name="so101",
            mode="real",
            config_path=args.config_path,
        )
        interface.connect()
        print("Connected to robot.")
        initial_pos = interface.get_joint_positions()
        lerp_to_position(interface, initial_pos, np.array(CALIBRATION_CONFIGS[1]))

        # Read initial position
        initial_pos = interface.get_joint_positions()
        print_joint_positions(initial_pos, "Initial Position (qFb)")

        # Lerp to target position
        lerp_to_position(interface, initial_pos, target_config)

        # Hold at target and print feedback
        print("\nHolding at target position. Press Ctrl+C to exit.")
        loop_count = 0
        dt = 1.0 / CONTROL_RATE

        while True:
            send_joint_command(interface, target_config)

            # Print feedback every 2 seconds
            if loop_count % (CONTROL_RATE * 2) == 0:
                current_pos = interface.get_joint_positions()
                error = target_config - current_pos
                loop_count and print("\033[5A", end="")
                print(f"\nTime: {loop_count / CONTROL_RATE:.1f}s")
                print(f"  Target:  {np.array2string(target_config, precision=3, suppress_small=True)}")
                print(f"  Current: {np.array2string(current_pos, precision=3, suppress_small=True)}")
                print(f"  Error:   {np.array2string(error, precision=4, suppress_small=True)}")

            loop_count += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        exit_requested = True
        print("\n\nCtrl+C received. Returning to rest position...")

    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()

    finally:
        if interface and interface.is_connected:
            try:
                if exit_requested:
                    # Lerp to rest position
                    current_pos = interface.get_joint_positions()
                    print_joint_positions(REST_POSITION, "Rest Position")
                    lerp_to_position(interface, current_pos, REST_POSITION)

                print("\nDisconnecting...")
                interface.disconnect()
                print("Disconnected.")

            except Exception as e:
                print(f"Error during cleanup: {e}")

    # Force clean exit
    os._exit(0)


if __name__ == "__main__":
    main()
