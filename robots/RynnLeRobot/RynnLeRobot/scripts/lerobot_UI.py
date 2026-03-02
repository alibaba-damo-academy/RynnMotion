"""LeRobot MuJoCo viewer interface.

Usage:
    $ lerobot-ui
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import traceback
import yaml

from RynnLeRobot.hardware.robots.so101_follower.config_so101_follower import (
    SO101FollowerConfig,
)
from RynnLeRobot.hardware.robots.so101_follower.so101_follower import SO101Follower

# SO101 joint limits (degrees)
SO101_JOINT_RANGE = {
    "shoulder_pan.pos": [-120, 120],
    "shoulder_lift.pos": [-100, 100],
    "elbow_flex.pos": [-95, 95],
    "wrist_flex.pos": [-105, 105],
    "wrist_roll.pos": [-100, 180],
    "gripper.pos": [1, 100],
}


class LeRobotUIController:
    def __init__(self):
        # Configuration constants
        self._mjcf_path = "../../models/3.robot_arm/24.so101/scene/scene.xml"
        self._render_freq = 60  # Hz
        self._warmup_steps = 500
        self._max_steps = 100000
        self._reset_steps = 10000
        self._sleep_interval = 0.001
        self._max_velocity = 1.0
        self._time_step = 0.01
        self._angle_threshold = 0.1
        self._rest_position = [0, -1.71, 1.67, 1.0, 0, -0.174]

        # Initialize MuJoCo
        self.model = mujoco.MjModel.from_xml_path(self._mjcf_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.render_interval = 1.0 / self._render_freq
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.camera.distance = 3.0
        self.camera.azimuth = 90.0
        self.camera.elevation = -30.0
        self.camera.lookat = np.array([0.0, 0.0, 0.0])

        self.qFb = np.zeros(self.model.nu)
        self.qdFb = np.zeros(self.model.nu)
        self.current_time = time.time()
        self.last_control_commands_rad = []

    def update_feedback(self):
        self.qFb = self.data.qpos
        self.qdFb = self.data.qvel

    def get_project_root_path(self):
        project_name = "RynnMotion"
        current_file_path = str(os.path.abspath(__file__))
        current_path_list = current_file_path.split(project_name)
        project_root_path = current_path_list[0] + project_name
        return project_root_path

    def get_ctrl_input(self):
        qCmd = self.data.ctrl
        return qCmd

    def run(self):
        joint_mapping, joint_range, robot = self.real_robot()
        step_counter = 0
        while self.viewer.is_running():
            self.update_feedback()
            qCmd = self.get_ctrl_input()
            if step_counter > self._warmup_steps:
                self.real_robot_control(qCmd, joint_mapping, joint_range, robot)
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(self._sleep_interval)
            step_counter += 1
            if step_counter > self._max_steps:
                step_counter = self._reset_steps
        self.go_to_rest_position(robot, joint_mapping, joint_range)
        robot.disconnect()

    def load_config(self, config_path):
        """Load configuration from YAML file."""
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        return config

    def get_config_info(self):
        config = None
        config_path = "configs/so101.yaml"
        if config_path and os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

        port = config.get("follower", {}).get("port", "/dev/ttyACM0")
        joint_names = list(SO101_JOINT_RANGE.keys())
        joint_mapping = {i + 1: name for i, name in enumerate(joint_names)}
        return joint_mapping, SO101_JOINT_RANGE, port

    def real_robot(self):
        joint_mapping, joint_range, port = self.get_config_info()

        robot_config = SO101FollowerConfig(port=port)
        robot = SO101Follower(robot_config)

        robot.connect()

        initial_positions = robot.get_observation()

        try:
            for joint_name in joint_range.keys():
                current_position_rad = self.real_joint_pos_2_rad(
                    joint_name, initial_positions
                )
                self.last_control_commands_rad.append(current_position_rad)

            if not initial_positions:
                raise Exception("Failed to read initial positions")

            # Warning and confirmation
            print("\n" + "⚠️ " * 20)
            print("WARNING: The arm will start moving!")
            print("Make sure the workspace is clear and safe.")
            print("Press Ctrl+C at any time to stop the motion.")
            print("⚠️ " * 20)

            print("\nStarting control real robot move...")
            print("Press Ctrl+C to stop")
            return joint_mapping, joint_range, robot

        except Exception as e:
            print(f"\n❌ Error: {e}")
            print("\nTroubleshooting tips:")
            print("1. Check that the SO100 arm is connected via USB")
            print("2. Verify the USB port (default: /dev/ttyACM0)")
            print("3. Check that you have permission to access the USB port")
            print("4. Ensure the arm is powered on")
            print("5. Make sure the workspace is clear for motion")
            print("\nFull error trace:")
            traceback.print_exc()
            return None, None, None

    def real_joint_pos_2_rad(self, joint_name, current_positions):
        """Convert target joint positions to radians."""
        return current_positions[joint_name] * np.pi / 180.0

    def real_robot_control(
        self,
        target_control_commands,
        joint_mapping,
        joint_range,
        robot,
        go_to_rest_flag=False,
    ):
        target_control_commands_rad = np.array(target_control_commands)
        current_positions = robot.get_observation()

        dt = self._time_step
        if time.time() - self.current_time < dt:
            dt = time.time() - self.current_time
        self.current_time = time.time()

        # Vectorized velocity limiting
        last_commands = np.array(self.last_control_commands_rad)
        angle_errors = target_control_commands_rad - last_commands
        max_change = self._max_velocity * dt

        # Apply velocity limits using np.clip
        limited_changes = np.clip(angle_errors, -max_change, max_change)
        target_control_commands_rad = last_commands + limited_changes

        data = dict()

        for i in range(6):
            target_joint = joint_mapping[i + 1]
            joint_range_min, joint_range_max = joint_range[target_joint]
            if go_to_rest_flag:
                if target_joint == "wrist_flex":
                    shoulder_lift_angle = self.real_joint_pos_2_rad(
                        "shoulder_lift", current_positions
                    )
                    if shoulder_lift_angle > -0.5:
                        if target_control_commands_rad[i] > 0:
                            target_control_commands_rad[i] = 0
            target_position = (
                target_control_commands_rad[i] * 180 / np.pi
            )  # Convert radians to degrees
            target_position = np.clip(target_position, joint_range_min, joint_range_max)
            data[target_joint] = target_position
        robot.send_action(data)
        self.last_control_commands_rad = target_control_commands_rad.tolist()

    def go_to_rest_position(self, robot, joint_mapping, joint_range):
        """Move the robot to a predefined rest position."""
        try:
            print("\nMoving to rest position...")
            max_iterations = 1000
            iteration = 0

            while iteration < max_iterations:
                self.real_robot_control(
                    self._rest_position, joint_mapping, joint_range, robot, True
                )

                # Check if all joints have reached the target position
                current_positions = robot.get_observation()

                # Get current joint positions as array
                current_joint_rads = np.array(
                    [
                        self.real_joint_pos_2_rad(joint_name, current_positions)
                        for joint_name in joint_range.keys()
                    ]
                )

                # Calculate angle errors (skip gripper at index 5)
                rest_positions = np.array(self._rest_position)
                angle_errors = np.abs(current_joint_rads - rest_positions)

                # Check if all non-gripper joints are within threshold
                non_gripper_errors = np.delete(angle_errors, 5)  # Remove gripper index
                all_joints_reached = np.all(non_gripper_errors <= self._angle_threshold)

                if all_joints_reached:
                    break

                iteration += 1
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage

            if iteration >= max_iterations:
                print(
                    "⚠️ Warning: Max iterations reached, may not have fully reached rest position"
                )
            else:
                print("✓ Moved to rest position successfully")

        except Exception as e:
            print(f"Error moving to rest position: {e}")


def main():
    try:
        controller = LeRobotUIController()
        controller.run()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
