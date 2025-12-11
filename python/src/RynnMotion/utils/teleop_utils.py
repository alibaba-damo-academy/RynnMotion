"""Shared utilities for SO101 teleoperation applications.

Common constants, helper functions, and base functionality.
"""

import time
import threading
import mujoco
import mujoco.viewer
import numpy as np
import glfw
import logging
from threading import Lock, Event
from typing import Tuple, Optional

from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.utils.pyFSM import PyFSM
from RynnMotion.utils.policy_interpolator import lerp
from RynnMotion.utils.path_config import get_models_root


ROBOT_NUMBERS = {"fr3": 20, "ur5e": 21, "piper": 22, "rm75": 23, "so101": 24}
ROBOT_ALIASES = {
    "fr3": ["FR3", "franka", "panda"],
    "ur5e": ["UR5E", "ur5-e", "ur5_e"],
    "piper": ["PIPER", "Piper"],
    "rm75": ["RM75", "realman75", "realman-rm75"],
}

ROBOT_MAP = {
    20: ("fr3", "3.robot_arm/20.fr3/mjcf/fr3_robot.xml", "3.robot_arm/20.fr3/mjcf/fr3_pinocchio.xml"),
    21: ("ur5e", "3.robot_arm/21.ur5e/mjcf/ur5e_robot.xml", "3.robot_arm/21.ur5e/mjcf/ur5e_pinocchio.xml"),
    22: ("piper", "3.robot_arm/22.piper/mjcf/piper_robot.xml", "3.robot_arm/22.piper/mjcf/piper_pinocchio.xml"),
    23: ("rm75", "3.robot_arm/23.rm75/mjcf/rm75_robot.xml", "3.robot_arm/23.rm75/mjcf/rm75_pinocchio.xml"),
    24: ("so101", "3.robot_arm/24.so101/mjcf/so101_robot.xml", "3.robot_arm/24.so101/mjcf/so101_pinocchio.xml"),
}


def resolve_robot_number(name: str) -> int:
    """Resolve robot alias to robot number."""
    lower = name.lower()
    if lower in ROBOT_NUMBERS:
        return ROBOT_NUMBERS[lower]

    for canonical, aliases in ROBOT_ALIASES.items():
        if lower in [a.lower() for a in aliases]:
            return ROBOT_NUMBERS[canonical]

    available = ", ".join(ROBOT_NUMBERS.keys())
    raise ValueError(f"Unknown robot: {name}. Available: {available}")


def create_robot_config(robot_number: int) -> dict:
    """Create robot config dict from robot number."""
    models_root = get_models_root()
    name, robot_rel, pino_rel = ROBOT_MAP[robot_number]

    return {
        "robot_name": name,
        "robot_control_freq": 100,
        "robot_mjcf": str(models_root / robot_rel),
        "pino_mjcf": str(models_root / pino_rel),
    }


def create_robot_manager(robot_number: int, logger=None) -> RobotManager:
    """Create RobotManager from robot number."""
    logger = logger or logging.getLogger(__name__)
    config = create_robot_config(robot_number)
    return RobotManager(config, logger)


class TeleopAppBase:
    """Base class for teleoperation applications.

    Handles common setup: GLFW, MuJoCo models, rendering, FSM, threading.
    Subclasses implement: init_teleop(), run_teleop()
    """

    def __init__(
        self,
        master_robot_number: int,
        slave_robot_number: int,
        arm_side: str = "left",
        verbose: bool = False,
        window_title: str = "Teleoperation - Slave Robot",
    ):
        self.master_robot_number = master_robot_number
        self.slave_robot_number = slave_robot_number
        self.arm_side = arm_side
        self.verbose = verbose
        self.window_title = window_title

        self.logger = logging.getLogger(__name__)
        self.master_robot_manager = create_robot_manager(master_robot_number, self.logger)
        self.slave_robot_manager = create_robot_manager(slave_robot_number, self.logger)

        self.simulation_mutex = Lock()
        self.exit_request = Event()

        self.ctrl_freq = 100.0
        self.mjstep_freq = 500.0
        self.render_freq = 60.0
        self.ctrl_dt = 1.0 / self.ctrl_freq
        self.mjstep_dt = 1.0 / self.mjstep_freq
        self.render_dt = 1.0 / self.render_freq

        self.slave_robot_name = self.slave_robot_manager.getRobotName()
        self.q_home = self.slave_robot_manager.get_qHome()
        self.q_stand = self.slave_robot_manager.get_qStandby(self.arm_side)
        self.site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]

        self.pyfsm = PyFSM(verbose=self.verbose)
        self.lerp_duration = 1.0

        # GLFW
        self.window = None
        self.glfw_initialized = False

        # MuJoCo rendering
        self.scene = None
        self.context = None
        self.camera = None
        self.option = None

        # Master
        self.master_viewer = None
        self.master_model = None
        self.master_data = None
        self.master_gripper_range = 0.0

        # Slave
        self.model = None
        self.data = None
        self.nctrl = 0

        self._init_glfw()
        self._init_scene()
        self._init_master_viewer()
        self._init_mujoco_rendering()
        self.init_teleop()  # Subclass implements

        if self.verbose:
            self._print_info()

    def _init_glfw(self):
        """Initialize GLFW window."""
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")
        self.glfw_initialized = True

        primary_monitor = glfw.get_primary_monitor()
        video_mode = glfw.get_video_mode(primary_monitor)
        screen_width = video_mode.size.width

        base_width = screen_width // 2
        window_width = int(base_width * 1.25)
        window_height = int(window_width * 9 / 16)

        self.window = glfw.create_window(window_width, window_height, self.window_title, None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window")

        glfw.set_window_pos(self.window, base_width, 0)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        glfw.set_key_callback(self.window, self._key_callback)

        if self.verbose:
            print("✅ GLFW window initialized")

    def _key_callback(self, window, key, scancode, action, mods):
        """Handle keyboard input."""
        if action == glfw.PRESS:
            if key == glfw.KEY_R:
                if self.pyfsm.is_DoAction1() or self.pyfsm.is_GoStandby1():
                    if self.verbose:
                        print("🔄 Reset requested")
                    self.pyfsm.to_reset()
            elif key == glfw.KEY_ENTER:
                if self.pyfsm.is_GoStandby1():
                    if self.verbose:
                        print("▶️  Entering DoAction1")
                    self.pyfsm.to_do_action1()
            elif key == glfw.KEY_ESCAPE:
                if self.verbose:
                    print("🛑 Exit requested")
                self.exit_request.set()

    def _init_scene(self):
        """Load slave MuJoCo model."""
        slave_scene_path = self.slave_robot_manager.getRobotMJCF()
        if self.verbose:
            print(f"Loading slave model: {slave_scene_path}")

        self.model = mujoco.MjModel.from_xml_path(slave_scene_path)
        self.data = mujoco.MjData(self.model)
        self.nctrl = self.model.nu

        n_motion_joints = len(self.q_home)
        self.data.qpos[:n_motion_joints] = self.q_home
        mujoco.mj_forward(self.model, self.data)

        if self.verbose:
            print(f"✅ Slave model loaded: {self.model.nq} DOF, {n_motion_joints} motion joints, {self.nctrl} actuators")

    def _init_master_viewer(self):
        """Launch master MuJoCo viewer."""
        master_scene_path = self.master_robot_manager.getRobotMJCF()
        if self.verbose:
            print(f"Loading master model: {master_scene_path}")

        self.master_model = mujoco.MjModel.from_xml_path(master_scene_path)
        self.master_data = mujoco.MjData(self.master_model)

        master_qHome = self.master_robot_manager.get_qHome()
        n_master_motion_joints = len(master_qHome)
        self.master_data.qpos[:n_master_motion_joints] = master_qHome
        mujoco.mj_forward(self.master_model, self.master_data)

        self.master_viewer = mujoco.viewer.launch_passive(
            self.master_model, self.master_data, show_left_ui=False, show_right_ui=True
        )
        self.master_gripper_range = (
            self.master_model.actuator_ctrlrange[-1][1] - self.master_model.actuator_ctrlrange[-1][0]
        )

        if self.verbose:
            print(f"✅ Master model loaded: {self.master_model.nq} DOF, {n_master_motion_joints} motion joints")
            print(f"🎮 Master viewer ENABLED (drag joints to control)")

    def _init_mujoco_rendering(self):
        """Initialize MuJoCo rendering for slave window."""
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        self.camera = mujoco.MjvCamera()
        self.camera.azimuth = 90
        self.camera.elevation = -20
        self.camera.distance = 2.0
        self.camera.lookat[:] = [0, 0, 0.5]

        self.option = mujoco.MjvOption()

        if self.verbose:
            print("✅ MuJoCo rendering context initialized")

    def _print_info(self):
        """Print initialization info."""
        print("=" * 60)
        print(f"Master: {self.master_robot_manager.getRobotName()} (#{self.master_robot_number})")
        print(f"Slave:  {self.slave_robot_manager.getRobotName()} (#{self.slave_robot_number})")
        print(f"Arm side: {self.arm_side}")
        print("=" * 60)

    def init_teleop(self):
        """Initialize teleop-specific components. Override in subclass."""
        pass

    def run_teleop(self):
        """Run one step of teleoperation. Override in subclass."""
        pass

    def go_standby(self):
        """LERP from qHome to qStand."""
        elapsed = self.pyfsm.get_elapsed_time()

        slave_q_lerp = lerp(self.q_home, self.q_stand, elapsed, self.lerp_duration)

        master_qHome = self.master_robot_manager.get_qHome()
        master_qStand = self.master_robot_manager.get_qStandby(self.arm_side)
        master_q_lerp = lerp(master_qHome, master_qStand, elapsed, self.lerp_duration)

        with self.simulation_mutex:
            self.data.ctrl[:len(slave_q_lerp)] = slave_q_lerp
            self.master_data.ctrl[:len(master_q_lerp)] = master_q_lerp

        if self.pyfsm.should_auto_transition(self.lerp_duration):
            if self.verbose:
                print("✅ LERP complete, entering DoAction1")
            self.pyfsm.to_do_action1()

    def map_gripper_command(self, master_gripper_cmd: float) -> float:
        """Map master gripper command to slave gripper range."""
        slave_gripper_idx = self.nctrl - 1
        slave_min = self.model.actuator_ctrlrange[slave_gripper_idx][0]
        slave_max = self.model.actuator_ctrlrange[slave_gripper_idx][1]
        slave_cmd = slave_min + slave_max * master_gripper_cmd / self.master_gripper_range
        if self.slave_robot_name in ["lerobot", "so101"]:
            slave_cmd *= 180 / 3.14
        return slave_cmd

    def _simulation_thread(self):
        """Background thread: control + physics."""
        last_control_time = time.time()

        while not self.exit_request.is_set():
            loop_start = time.time()

            if self.pyfsm.is_Initr():
                self.pyfsm.to_go_standby1()
            elif self.pyfsm.is_GoStandby1():
                self.go_standby()
            elif self.pyfsm.is_DoAction1():
                if time.time() - last_control_time >= self.ctrl_dt:
                    last_control_time = time.time()
                    self.run_teleop()
            elif self.pyfsm.is_Reset():
                self.go_standby()
                if self.pyfsm.should_auto_transition(self.lerp_duration):
                    if self.verbose:
                        print("✅ Reset complete")
                    self.pyfsm.finish_reset()

            with self.simulation_mutex:
                mujoco.mj_step(self.master_model, self.master_data)
                mujoco.mj_step(self.model, self.data)

            if self.master_viewer and self.master_viewer.is_running():
                self.master_viewer.sync()

            elapsed = time.time() - loop_start
            sleep_time = self.mjstep_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _render_slave(self):
        """Render slave to GLFW window."""
        width, height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)

        with self.simulation_mutex:
            mujoco.mjv_updateScene(
                self.model, self.data, self.option, None, self.camera,
                mujoco.mjtCatBit.mjCAT_ALL, self.scene
            )

        mujoco.mjr_rectangle(viewport, 0.1, 0.1, 0.1, 1.0)
        mujoco.mjr_render(viewport, self.scene, self.context)

        time_msg = f"time: {self.data.time:.2f}"
        mujoco.mjr_overlay(
            mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
            viewport, time_msg, "", self.context
        )

    def run(self):
        """Main entry point."""
        sim_thread = None
        try:
            sim_thread = threading.Thread(target=self._simulation_thread, daemon=True)
            sim_thread.start()

            if self.verbose:
                print("\n🚀 Starting teleoperation")
                print("Controls: ENTER=start, R=reset, ESC=exit\n")

            last_render_time = time.time()
            while not self.exit_request.is_set() and not glfw.window_should_close(self.window):
                current_time = time.time()
                if current_time - last_render_time >= self.render_dt:
                    glfw.make_context_current(self.window)
                    self._render_slave()
                    glfw.swap_buffers(self.window)
                    glfw.poll_events()
                    last_render_time = current_time
                else:
                    time.sleep(0.001)

        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.exit_request.set()
            if sim_thread and sim_thread.is_alive():
                sim_thread.join(timeout=1.0)
            if self.master_viewer:
                self.master_viewer.close()
            if self.window:
                glfw.destroy_window(self.window)
            if self.glfw_initialized:
                glfw.terminate()
