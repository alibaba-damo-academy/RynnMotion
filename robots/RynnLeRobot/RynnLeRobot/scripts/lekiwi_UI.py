"""LeKiwi MuJoCo viewer interface.

Usage:
    $ lekiwi-ui
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import traceback
import yaml

from RynnLeRobot.hardware.robots.lekiwi.config_lekiwi import (
    LeKiwiConfig,
)  # , LeKiwiHostConfig
from RynnLeRobot.hardware.robots.lekiwi.lekiwi import LeKiwi
from RynnLeRobot.interface.lekiwi_interface import body_to_wheel_raw

# LeKiwi arm joint limits (degrees)
LEKIWI_JOINT_RANGE = {
    "arm_shoulder_pan.pos": [-120, 120],
    "arm_shoulder_lift.pos": [-100, 100],
    "arm_elbow_flex.pos": [-95, 95],
    "arm_wrist_flex.pos": [-105, 105],
    "arm_wrist_roll.pos": [-100, 180],
    "arm_gripper.pos": [1, 100],
}


class LeKiwiUIController:
    def __init__(self):
        # Configuration constants
        self._render_freq = 60  # Hz
        self._warmup_steps = 500
        self._max_steps = 100000
        self._reset_steps = 10000
        self._sleep_interval = 0.001
        self._max_velocity = 1.0
        self._time_step = 0.01
        self._angle_threshold = 0.1
        self._rest_position = [0, 0, 0, 0, 0, 0, 0, -1.75, 1.67, 1.0, 0, -0.174]
        self._gripper_joint_name = "arm_gripper.pos"

        # Initialize MuJoCo
        project_path = self.get_project_root_path()
        mjcf_path = (
            project_path + "/models/6.mobile_manipulator/60.lekiwi/scene/scene.xml"
        )
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
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
        joint_names = list(LEKIWI_JOINT_RANGE.keys())
        joint_mapping = {i + 1: name for i, name in enumerate(joint_names)}
        return joint_mapping, LEKIWI_JOINT_RANGE

    def real_robot(self):

        robot_config = LeKiwiConfig()
        robot = LeKiwi(robot_config)

        robot.connect()

        initial_positions = robot.get_observation()

        joint_mapping, joint_range = self.get_config_info()

        try:
            self.last_control_commands_rad = [
                self.real_joint_pos_2_rad(joint_name, initial_positions)
                for joint_name in joint_range.keys()
            ]
            self.last_control_commands_rad.extend([0, 0, 0])  # for x, y, theta

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
        # try:
        # Extract arm joint commands (indices 6+) and mobility commands
        arm_commands = np.array(target_control_commands[6:])
        mobility_commands = np.array(
            [
                target_control_commands[1],
                target_control_commands[2],
                target_control_commands[0] * 180 / np.pi,
            ]
        )
        target_control_commands_rad = np.concatenate([arm_commands, mobility_commands])

        chassis_wheel_vels = body_to_wheel_raw(
            target_control_commands[1],
            target_control_commands[2],
            target_control_commands[0],
        )
        self.data.ctrl[3:6] = [
            -1 * chassis_wheel_vels["base_back_wheel"],
            -1 * chassis_wheel_vels["base_right_wheel"],
            -1 * chassis_wheel_vels["base_left_wheel"],
        ]

        current_positions = robot.get_observation()

        dt = self._time_step
        if time.time() - self.current_time < dt:
            dt = time.time() - self.current_time
        self.current_time = time.time()

        # Vectorized velocity limiting (exclude the last element for mobility)
        last_commands = np.array(
            self.last_control_commands_rad[:-3]
        )  # Exclude mobility commands
        arm_errors = target_control_commands_rad[:-3] - last_commands
        max_change = self._max_velocity * dt

        # Apply velocity limits using np.clip for arm joints only
        limited_arm_changes = np.clip(arm_errors, -max_change, max_change)
        target_control_commands_rad[:-3] = last_commands + limited_arm_changes

        # Default joint positions for LeKiwi
        data = {
            "arm_shoulder_pan.pos": 0.0,
            "arm_shoulder_lift.pos": -85.0,
            "arm_elbow_flex.pos": 95.0,
            "arm_wrist_flex.pos": 30.0,
            "arm_wrist_roll.pos": 0.0,
            "arm_gripper.pos": 0.0,
            "x.vel": target_control_commands_rad[6],
            "y.vel": target_control_commands_rad[7],
            "theta.vel": target_control_commands_rad[8],
        }

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

                # Get current arm joint positions as array (skip mobility joints)
                arm_joint_names = [
                    name
                    for name in joint_range.keys()
                    if name != self._gripper_joint_name
                ]
                current_arm_rads = np.array(
                    [
                        self.real_joint_pos_2_rad(joint_name, current_positions)
                        for joint_name in arm_joint_names
                    ]
                )

                # Calculate angle errors for arm joints (starting from index 7 in rest_position)
                arm_rest_positions = np.array(
                    self._rest_position[7:-1]
                )  # Exclude gripper
                angle_errors = np.abs(current_arm_rads - arm_rest_positions)

                # Check if all arm joints are within threshold
                all_joints_reached = np.all(angle_errors <= self._angle_threshold)

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
        controller = LeKiwiUIController()
        controller.run()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
