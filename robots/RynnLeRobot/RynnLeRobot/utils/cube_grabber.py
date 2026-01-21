"""Cube grabbing utilities for pick-and-place demonstrations."""

import numpy as np
import time
import argparse
import logging
import pinocchio as pin
import yaml
from enum import Enum, auto
from abc import ABC, abstractmethod
from RynnMotion.core.robotinterface_base import robotinterface_factory
from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.utils.path_config import get_models_root
from RynnMotion.utils.policy_interpolator import lerp
from RynnLeRobot.utils.ik_solver import IKSolver


class ArmState(Enum):
    """Enumeration for the states of the robotic arm."""

    IDLE = auto()
    MOVING_TO_CUBE = auto()
    GRABBING = auto()
    LIFTING_CUBE = auto()
    MOVING_TO_MUG = auto()
    RELEASING = auto()
    MOVING_HOME = auto()
    FINISHED = auto()


class CubeGrabberBase(ABC):
    "Abstract base class for cubegrabber"

    def __init__(
        self,
        cube_names=[],
        home_joint_positions=None,
        mug_pose=None,
    ):
        self.state = ArmState.IDLE

        self.current_joint_positions = home_joint_positions
        self.generated_joint_command = home_joint_positions

        self.succeeded_cube_names = []
        self.cube_names = cube_names
        self.current_cube_name = None
        self.current_cube_pose = None
        self.mug_pose = mug_pose

        self.current_time = None
        self.state_start_time = None

        self.max_try_times = 3
        self.current_try_times = 1

        self.pos_only_flag = False
        self.lerp_duration = 1.2

        # For distance-based cube selection
        self.all_cube_positions = {}  # {cube_name: [x, y, z]} updated by main loop
        self.home_ee_position = None  # Set by derived classes

    def get_distance(self, q1, q2):
        """
        Calculate the distance between two vectors.
        """
        return np.linalg.norm(q1 - q2)

    def find_closest_cube(self):
        """Find the closest cube to home position that is still on the table."""
        if not self.all_cube_positions or self.home_ee_position is None:
            return None

        closest_cube = None
        min_distance = float("inf")

        for cube_name, cube_pos in self.all_cube_positions.items():
            # Skip cubes near/in the mug (within 0.05m XY distance)
            if self.get_distance(cube_pos[:2], self.mug_pose[:2]) < 0.05:
                continue

            distance = self.get_distance(self.home_ee_position, cube_pos)
            if distance < min_distance:
                min_distance = distance
                closest_cube = cube_name

        return closest_cube

    def get_target_pose(
        self,
        input_target_pose,
        orientation_name,
        z_bias=0.1,
    ):
        target_pose = input_target_pose.copy()
        target_pose[2] = z_bias

        target_orientation_dict = {
            "grab": pin.Quaternion(0.5, 0.5, 0.5, 0.5),
            "above_mug": pin.Quaternion(np.array([0.196, 0.471, 0.466, 0.722])),
        }
        target_orientation = target_orientation_dict[orientation_name]

        if orientation_name == "grab":
            target_angle = 2 * np.arctan2(abs(target_pose[6]), target_pose[3])
            if target_angle < np.pi / 2:
                target_angle = target_angle if target_angle < abs(target_angle - np.pi / 2) else target_angle - np.pi / 2
            if target_angle > np.pi / 2 and target_angle < np.pi:
                target_angle = target_angle - np.pi / 2 if target_angle - np.pi / 2 < abs(target_angle - np.pi) else target_angle - np.pi
            if target_angle > np.pi and target_angle < 1.5 * np.pi:
                target_angle = target_angle - np.pi if target_angle - np.pi < abs(target_angle - 1.5 * np.pi) else target_angle - 1.5 * np.pi
            if target_angle > 1.5 * np.pi:
                target_angle = target_angle - 1.5 * np.pi if target_angle - 1.5 * np.pi < abs(target_angle - 2 * np.pi) else target_angle - 2 * np.pi

            target_pose[0] += 0.1 * np.sin(target_angle)
            target_pose[1] += 0.01 * np.cos(target_angle)
            target_orientation = pin.Quaternion(np.cos(target_angle / 2), 0.0, 0.0, np.sin(target_angle / 2)) * target_orientation

        target_position = target_pose[:3]
        target_orientation = target_orientation.toRotationMatrix()

        return target_position, target_orientation

    def check_cube_in_mug(self):
        if self.get_distance(self.current_cube_pose[:2], self.mug_pose[:2]) < 0.05:
            print("✓ Cube placed successfully")
            self.succeeded_cube_names.append(self.current_cube_name)
            self.cube_names.remove(self.current_cube_name)
            self.current_try_times = 1
            # Prepare for next cube - select closest remaining
            closest = self.find_closest_cube()
            if closest:
                self.current_cube_name = closest
                print(f"🎯 Next target: {self.current_cube_name}")
            else:
                self.current_cube_name = None
            self.current_cube_pose = None
        elif self.current_try_times >= self.max_try_times:
            print(f"✗ Failed to grab {self.current_cube_name} after {self.max_try_times} attempts")
            self.cube_names.remove(self.current_cube_name)
            self.current_try_times = 1
            # Prepare for next cube - select closest remaining
            closest = self.find_closest_cube()
            if closest:
                self.current_cube_name = closest
                print(f"🎯 Next target: {self.current_cube_name}")
            else:
                self.current_cube_name = None
            self.current_cube_pose = None
        else:
            print("↻ Retrying...")
            self.current_try_times += 1

    def update_state(self, input_state):
        self.state = input_state
        self.pos_only_flag = False
        if (input_state == ArmState.GRABBING and np.linalg.norm(self.current_cube_pose[:2]) > 0.3) or input_state == ArmState.LIFTING_CUBE:
            self.pos_only_flag = True

    def step(self):
        """
        Run one step of the cube grabber.
        """

        if self.mug_pose is None:
            print("Mug not detected! Maintain current joint positions")
            return

        if self.state == ArmState.IDLE:
            self.check_need_to_grab()

        elif self.state == ArmState.MOVING_TO_CUBE:
            self.moving_to_cube()

        elif self.state == ArmState.GRABBING:
            self.grab_cube()

        elif self.state == ArmState.LIFTING_CUBE:
            self.lift_cube()

        elif self.state == ArmState.MOVING_TO_MUG:
            self.moving_to_mug()

        elif self.state == ArmState.RELEASING:
            self.release_cube()

        elif self.state == ArmState.MOVING_HOME:
            self.moving_to_home()

        else:
            pass

    def check_need_to_grab(self):
        if len(self.cube_names) == 0:
            print("\n=== Task Complete ===")
            if len(self.succeeded_cube_names):
                print(f"✓ Successfully placed {len(self.succeeded_cube_names)} cubes: {self.succeeded_cube_names}")
            else:
                print("✗ No cubes were successfully placed")
            self.update_state(ArmState.FINISHED)
        elif self.current_cube_pose is None:
            # Select closest cube to home position
            closest = self.find_closest_cube()
            if closest:
                self.current_cube_name = closest
            else:
                self.current_cube_name = self.cube_names[0]  # Fallback
            print(f"🎯 Target: {self.current_cube_name}")
        else:
            print("→ Moving to cube...")

    def release_cube(self):
        self.generated_joint_command[-1] = 20
        # Gripper is open when feedback > 1.5 (radians) - fully open is ~1.74
        if self.current_joint_positions[-1] > 1.5:
            self.check_cube_in_mug()

    @abstractmethod
    def moving_to_cube(self):
        pass

    @abstractmethod
    def grab_cube(self):
        pass

    @abstractmethod
    def lift_cube(self):
        pass

    @abstractmethod
    def moving_to_mug(self):
        pass

    @abstractmethod
    def moving_to_home(self):
        pass

    @abstractmethod
    def generate_joint_command(self, current_time):
        pass


class CubeGrabberLerp(CubeGrabberBase):
    """
    CubeGrabberLerp class is responsible for controlling the robotic arm to grab
    randomly generated cubes in the scene and place them into a mug using lerp.
    """

    def __init__(
        self,
        home_joint_positions=None,
        cube_names=[],
        mug_pose=None,
        model_mjcf_path=None,
    ):
        """
        Initialize an instance of the CubeGrabberLerp class.
        """
        super().__init__(
            cube_names=cube_names,
            home_joint_positions=home_joint_positions,
            mug_pose=mug_pose,
        )

        self.home_joint_positions = home_joint_positions
        self.initial_joint_positions = None
        self.target_joint_positions = None

        self.ik_solver = IKSolver(
            model_mjcf_path,
            "ee",
            np.array([0.0, 0.0, -0.141, 0.193, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0]),
        )

        # Compute home EE position using FK for distance-based cube selection
        pin.forwardKinematics(self.ik_solver.pin_model, self.ik_solver.pin_data, home_joint_positions[:6])
        pin.updateFramePlacements(self.ik_solver.pin_model, self.ik_solver.pin_data)
        ee_id = self.ik_solver.pin_model.getFrameId("ee")
        self.home_ee_position = self.ik_solver.pin_data.oMf[ee_id].translation.copy()

    def get_target_joint_positions(
        self,
        input_target_pose,
        z_bias=0.1,
        opening_degree=10,
    ):
        target_position, target_orientation = self.get_target_pose(input_target_pose, "grab", z_bias=z_bias)

        self.ik_solver.pos_only_flag = self.pos_only_flag

        self.target_joint_positions = self.ik_solver.compute_ik(target_position, target_orientation)
        self.target_joint_positions[-1] = opening_degree

    def update_state(self, input_state):

        super().update_state(input_state)
        self.initial_joint_positions = self.current_joint_positions
        self.initial_joint_positions[-1] = self.target_joint_positions[-1]
        self.state_start_time = self.current_time

    def generate_joint_command(self, current_time):

        return lerp(
            self.initial_joint_positions,
            self.target_joint_positions,
            current_time,
            self.lerp_duration,
        )

    def check_need_to_grab(self):

        super().check_need_to_grab()
        if len(self.cube_names) != 0 and self.current_cube_pose is not None:
            self.get_target_joint_positions(
                self.current_cube_pose,
                z_bias=0.07,
                opening_degree=20,
            )
            self.update_state(ArmState.MOVING_TO_CUBE)

    def moving_to_cube(self):
        lerp_time = self.current_time - self.state_start_time
        if self.get_distance(self.current_joint_positions[:5], self.target_joint_positions[:5]) < 0.05 or lerp_time > 2.0 * self.lerp_duration:
            if lerp_time > 2.0 * self.lerp_duration:
                print("⏰ Timeout - skipping to next state")
            else:
                print("🤏 Grabbing...")
            self.get_target_joint_positions(
                self.current_cube_pose,
                z_bias=0.00,
                opening_degree=20,
            )
            self.update_state(ArmState.GRABBING)
        else:
            self.generated_joint_command = self.generate_joint_command(lerp_time)

    def grab_cube(self):
        lerp_time = self.current_time - self.state_start_time
        if self.get_distance(self.current_joint_positions[:5], self.target_joint_positions[:5]) < 0.07 or lerp_time > 2.0 * self.lerp_duration:
            for i in range(5):
                self.generated_joint_command[i] = self.target_joint_positions[i]
            self.generated_joint_command[-1] = 0  # Close gripper

            if lerp_time > self.lerp_duration + 1.0:
                print("💪 Lifting the cube...")
                self.get_target_joint_positions(
                    self.current_cube_pose,
                    z_bias=0.1,
                    opening_degree=0,
                )
                self.update_state(ArmState.LIFTING_CUBE)
        else:
            self.generated_joint_command = self.generate_joint_command(lerp_time)

    def lift_cube(self):
        lerp_time = self.current_time - self.state_start_time
        if self.get_distance(self.current_joint_positions[:5], self.target_joint_positions[:5]) < 0.05 or lerp_time > 2.0 * self.lerp_duration:
            if lerp_time > 2.0 * self.lerp_duration:
                print("⏰ Lift time timeout - skipping to next state")
            else:
                print("☕ Moving to mug...")
            self.get_target_joint_positions(
                self.mug_pose,
                z_bias=0.15,
                opening_degree=0,
            )
            self.update_state(ArmState.MOVING_TO_MUG)
        else:
            self.generated_joint_command = self.generate_joint_command(lerp_time)

    def moving_to_mug(self):
        lerp_time = self.current_time - self.state_start_time
        if self.get_distance(self.current_joint_positions[:5], self.target_joint_positions[:5]) < 0.05 or lerp_time > 2.0 * self.lerp_duration:
            if lerp_time > 2.0 * self.lerp_duration:
                print("⏰ Mug approach timeout - skipping to next state")
            else:
                print("📦 Releasing cube...")
            super().update_state(ArmState.RELEASING)
        else:
            self.generated_joint_command = self.generate_joint_command(lerp_time)

    def release_cube(self):

        super().release_cube()

        # Gripper is open when feedback > 1.5 (radians)
        if self.current_joint_positions[-1] > 1.5:
            self.target_joint_positions = self.home_joint_positions
            self.update_state(ArmState.MOVING_HOME)

    def moving_to_home(self):
        lerp_time = self.current_time - self.state_start_time
        if self.get_distance(self.current_joint_positions[:5], self.target_joint_positions[:5]) < 0.05 or lerp_time > 2.0 * self.lerp_duration:
            if lerp_time > 2.0 * self.lerp_duration:
                print("⏰ Home return timeout - skipping to next state")
            else:
                print("🏠 Returned home\n")
            super().update_state(ArmState.IDLE)
        else:
            self.generated_joint_command = self.generate_joint_command(lerp_time)


class CubeGrabberPControl(CubeGrabberBase):
    """
    CubeGrabberPControl class is responsible for controlling the robotic arm to grab
    randomly generated cubes in the scene and place them into a mug using proportional control.
    """

    def __init__(
        self,
        home_pose=None,
        home_joint_positions=None,
        cube_names=[],
        mug_pose=None,
        model_mjcf_path=None,
        control_interval=0.01,
    ):
        """
        Initialize an instance of the CubeGrabberPControl class.
        """
        super().__init__(
            cube_names=cube_names,
            home_joint_positions=home_joint_positions,
            mug_pose=mug_pose,
        )

        self.home_pose = self.get_SE3_pose(home_pose["position"], home_pose["orientation"]) if home_pose is not None else None
        self.current_pose = self.home_pose
        self.target_pose = self.home_pose
        self.initial_pose = self.home_pose

        self.pin_model, self.collision_model, self.visual_model = pin.buildModelsFromMJCF(model_mjcf_path)
        self.pin_data = self.pin_model.createData()
        self.ee_id = self.pin_model.getFrameId("ee")

        self.q_min = self.pin_model.lowerPositionLimit
        self.q_max = self.pin_model.upperPositionLimit
        self.q_min[-1] = 0
        self.q_max[-1] = 100

        self.generated_joint_command = home_joint_positions

        self.Kp = np.diag([40, 40, 40, 40, 40, 40])
        self.control_interval = control_interval

        # Set home EE position for distance-based cube selection
        if self.home_pose is not None:
            self.home_ee_position = self.home_pose.translation.copy()

    def get_SE3_pose(self, position, orientation):
        orientation = np.array(orientation).reshape(3, 3).astype(np.float64)
        if np.allclose(orientation.T @ orientation, np.eye(3), atol=1e-6) and np.isclose(np.linalg.det(orientation), 1.0, atol=1e-6):
            return pin.SE3(orientation, position)
        elif np.allclose(orientation, np.zeros(3), atol=1e-6):
            pass
        else:
            raise ValueError("Orientation must be a valid rotation matrix.")

    def get_target_pose(self, input_target_pose, orientation_name, z_bias=0.1):
        target_position, target_orientation = super().get_target_pose(input_target_pose, orientation_name, z_bias=z_bias)
        self.target_pose = pin.SE3(target_orientation, target_position)

    def get_ref_pose(self, initial_pose, target_pose, current_time):
        SE3_lerp_velocity = pin.log6(initial_pose.actInv(target_pose)) / self.lerp_duration
        if current_time > 0 and current_time <= self.lerp_duration:
            self.feedforward_velocity = SE3_lerp_velocity.copy()
        else:
            self.feedforward_velocity = np.zeros(6)

        if current_time > self.lerp_duration:
            return target_pose
        current_ref_pose = initial_pose * pin.exp6(SE3_lerp_velocity * current_time)
        return current_ref_pose

    def get_ee_error(self, target_pose, current_pose, pos_only_flag=False):
        SE3_pose_error = current_pose.actInv(target_pose)
        pose_error = pin.log6(SE3_pose_error).vector
        return pose_error[:3] if pos_only_flag or self.pos_only_flag else pose_error

    def compute_inverse_jacobian(self):
        J = pin.computeFrameJacobian(
            self.pin_model,
            self.pin_data,
            self.current_joint_positions,
            self.ee_id,
            pin.LOCAL,
        )
        if self.pos_only_flag:
            J = J[:3, :]
        J_DLS = np.linalg.pinv(J.T @ J + 0.005 * np.eye(J.shape[1])) @ J.T
        return J_DLS

    def proportional_control(self, input_error):

        ee_v_command = self.Kp[:3, :3] @ input_error if self.pos_only_flag else self.Kp @ input_error
        if self.feedforward_velocity is None:
            dq = self.compute_inverse_jacobian() @ ee_v_command
        else:
            dq = self.feedforward_velocity + self.compute_inverse_jacobian() @ ee_v_command

        current_q = self.current_joint_positions.copy()
        current_q[-1] = self.generated_joint_command[-1]
        result = np.clip(
            current_q + dq * self.control_interval,
            self.q_min,
            self.q_max,
        )

        return result

    def update_state(self, input_state):
        super().update_state(input_state)
        self.initial_pose = self.current_pose
        self.state_start_time = self.current_time

    def generate_joint_command(self, current_time):
        ref_pose = self.get_ref_pose(self.initial_pose, self.target_pose, current_time)
        pose_error = self.get_ee_error(ref_pose, self.current_pose)
        return self.proportional_control(pose_error)

    def check_need_to_grab(self):
        super().check_need_to_grab()
        if len(self.cube_names) != 0 and self.current_cube_pose is not None:
            self.get_target_pose(self.current_cube_pose, "grab", z_bias=0.1)
            self.generated_joint_command[-1] = 20
            self.update_state(ArmState.MOVING_TO_CUBE)

    def moving_to_cube(self):
        pos_error = self.get_ee_error(self.target_pose, self.current_pose, pos_only_flag=True)
        state_duration = self.current_time - self.state_start_time
        if np.linalg.norm(pos_error) < 0.005 or state_duration > 2.0 * self.lerp_duration:
            if state_duration > 2.0 * self.lerp_duration:
                print("⏰ Timeout - skipping to next state")
            else:
                print("🤏 Grabbing...")
            self.get_target_pose(self.current_cube_pose, "grab", z_bias=0.012)
            self.update_state(ArmState.GRABBING)
        else:
            self.generated_joint_command = self.generate_joint_command(state_duration)

    def grab_cube(self):
        pos_error = self.get_ee_error(self.target_pose, self.current_pose, pos_only_flag=True)
        state_duration = self.current_time - self.state_start_time
        if np.linalg.norm(pos_error[1:]) < 0.007 or state_duration > 2.0 * self.lerp_duration:
            if state_duration < 0.5 + self.lerp_duration:
                self.generated_joint_command[-1] = 0
            else:
                if state_duration > 2.0 * self.lerp_duration:
                    print("⏰ Grab timeout - skipping to next state")
                else:
                    print("💪 Lifting the cube...")
                self.get_target_pose(self.current_cube_pose, "grab", z_bias=0.1)
                self.update_state(ArmState.LIFTING_CUBE)
        else:
            self.generated_joint_command = self.generate_joint_command(state_duration)

    def lift_cube(self):
        pos_error = self.get_ee_error(self.target_pose, self.current_pose, pos_only_flag=True)
        state_duration = self.current_time - self.state_start_time
        if np.linalg.norm(pos_error) < 0.005 or state_duration > 2.0 * self.lerp_duration:
            if state_duration > 2.0 * self.lerp_duration:
                print("⏰ Lift timeout - skipping to next state")
            else:
                print("☕ Moving to mug...")
            self.get_target_pose(self.mug_pose, "above_mug", z_bias=0.15)
            self.update_state(ArmState.MOVING_TO_MUG)
        else:
            self.generated_joint_command = self.generate_joint_command(state_duration)

    def moving_to_mug(self):
        pos_error = self.get_ee_error(self.target_pose, self.current_pose, pos_only_flag=True)
        state_duration = self.current_time - self.state_start_time
        if np.linalg.norm(pos_error) < 0.005 or state_duration > 2.0 * self.lerp_duration:
            if state_duration > 2.0 * self.lerp_duration:
                print("⏰ Mug approach timeout - skipping to next state")
            else:
                print("📦 Releasing cube...")
            super().update_state(ArmState.RELEASING)
        else:
            self.generated_joint_command = self.generate_joint_command(state_duration)

    def release_cube(self):

        super().release_cube()

        # Gripper is open when feedback > 1.5 (radians)
        if self.current_joint_positions[-1] > 1.5:
            self.target_pose = self.home_pose
            self.update_state(ArmState.MOVING_HOME)

    def moving_to_home(self):
        pos_error = self.get_ee_error(self.target_pose, self.current_pose, pos_only_flag=True)
        state_duration = self.current_time - self.state_start_time
        if np.linalg.norm(pos_error) < 0.005 or state_duration > 2.0 * self.lerp_duration:
            if state_duration > 2.0 * self.lerp_duration:
                print("⏰ Home return timeout - skipping to next state")
            else:
                print("🏠 Returned home\n")
            super().update_state(ArmState.IDLE)
        else:
            self.generated_joint_command = self.generate_joint_command(state_duration)


def main():
    """
    Main function to run the cube grabber.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--step", action="store_true", help="Step mode for simulation")

    parser.add_argument(
        "--mode",
        type=str,
        choices=["lerp", "Pcontrol"],
        default="lerp",
        help="Choose the control mode for the cube grabber",
    )
    args = parser.parse_args()

    models_root = get_models_root()
    model_mjcf_path = str(models_root / "3.robot_arm/24.so101/mjcf/so101_new_calib.xml")
    cube_names = ["cube1", "cube2", "cube3", "cube4", "cube5", "cube6"]
    config_path = "configs/so101.yaml"
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)
    scene = str(models_root / "3.robot_arm/24.so101/scene/scene_grip.xml")
    frequency = 100.0
    time_step = 1.0 / frequency

    # Initialize logger
    logger = logging.getLogger(__name__)

    # Build robotmodel_config for RobotManager
    robotmodel_config = {
        "robot_name": "so101",
        "robot_control_freq": frequency,
        "robot_mjcf": scene,
        "pino_mjcf": str(models_root / "3.robot_arm/24.so101/so101_pinocchio.xml"),
    }

    # Initialize robot model and interface using new pattern
    robot_model = RobotManager(robotmodel_config, logger)
    robotinterface_config = {"timestep": time_step}
    mujoco_interface = robotinterface_factory("mujoco_sim_robot", robot_model, robotinterface_config)
    mujoco_interface.connect()

    # Get the initial joint positions and mug position from mujoco interface
    home_joint_positions = mujoco_interface.get_joint_positions()
    home_pose = {
        "position": mujoco_interface.data.site("ee").xpos,
        "orientation": mujoco_interface.data.site("ee").xmat,
    }
    mug_position = mujoco_interface.data.body("mug").xpos.copy()
    mug_orientation = mujoco_interface.data.body("mug").xquat
    mug_pose = np.concatenate((mug_position, mug_orientation))

    # Create an cubegrabber instance
    if args.mode == "lerp":
        cube_grabber = CubeGrabberLerp(
            home_joint_positions=home_joint_positions,
            cube_names=cube_names,
            mug_pose=mug_pose,
            model_mjcf_path=model_mjcf_path,
        )
    else:
        cube_grabber = CubeGrabberPControl(
            home_pose=home_pose,
            home_joint_positions=home_joint_positions,
            cube_names=cube_names,
            mug_pose=mug_pose,
            model_mjcf_path=model_mjcf_path,
            control_interval=time_step,
        )

    # All initialization finished, start main loop
    try:
        while True:
            if args.step:
                input("\nPress Enter to run one step of the cubegrabber")
            precall_time = time.time()
            nextcall_time = precall_time + time_step

            cube_grabber.current_joint_positions = mujoco_interface.get_joint_positions()
            if args.mode == "Pcontrol":
                cube_grabber.current_pose = cube_grabber.get_SE3_pose(
                    mujoco_interface.data.site("ee").xpos,
                    mujoco_interface.data.site("ee").xmat,
                )
            if cube_grabber.current_cube_name is not None:
                cube_grabber.current_cube_pose = mujoco_interface.data.joint(cube_grabber.current_cube_name).qpos
            cube_grabber.current_time = mujoco_interface.get_current_time()
            cube_grabber.step()
            if cube_grabber.generated_joint_command is not None:
                mujoco_interface.set_joint_positions(cube_grabber.generated_joint_command.copy())

            mujoco_interface.step()

            postcall_time = time.time()
            sleep_time = nextcall_time - postcall_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Keyboard interruption detected, exiting...")
        mujoco_interface.disconnect()


if __name__ == "__main__":
    main()
