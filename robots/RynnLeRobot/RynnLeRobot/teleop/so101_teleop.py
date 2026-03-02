#!/usr/bin/env python3
"""SO101 master to dual-arm slave teleoperation.

Usage:
    $ so101-teleop --robot fr3
    $ so101-teleop --robot 20
"""

import argparse
import mujoco
import numpy as np
import os
import platform
import time
import yaml
from pathlib import Path

# Platform-specific imports
_PLATFORM = platform.system()
if _PLATFORM != "Darwin":
    # GLFW only needed on non-macOS platforms
    import glfw
    import threading
    from threading import Lock

from RynnMotion.utils.pyFSM import PyFSM
from RynnMotion.utils.path_config import get_models_root
from RynnMotion.algorithms.policy_interpolator import lerp
from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.algorithms import PinKine
import logging
from RynnLeRobot.interface.so101_interface import create_robot_interface
from RynnLeRobot.teleop.so101_teleop_config import get_robot_config, compute_slave_command


class SO101Teleop:
    """Unified SO101 teleoperation for multiple slave robot types."""

    def __init__(self, robot_identifier, use_viewer=None, config_path="configs/so101.yaml"):
        """
        Initialize SO101 teleoperation.

        Args:
            robot_identifier: Robot name (str) or number (int)
            use_viewer: If None, auto-detect platform. If True, use viewer mode.
                       If False, use GLFW mode.
            config_path: Path to lerobot configuration file
        """
        # Get robot configuration
        self.robot_name, self.robot_config = get_robot_config(robot_identifier)
        print(f"Initializing teleoperation for {self.robot_config['name']} (robot #{self.robot_config['number']})")

        # Determine rendering mode
        if use_viewer is None:
            self.use_viewer = platform.system() == "Darwin"
        else:
            self.use_viewer = use_viewer

        # Fixed configuration
        self.is_dual_arm = True
        self.master_number = 24
        self.slave_numbers = [self.robot_config["number"]]
        self.config_path = config_path

        # Robot managers
        logger = logging.getLogger(__name__)

        # Get models root for absolute path resolution
        models_root = get_models_root()

        # Robot number to config mapping (relative to models_root)
        robot_map = {
            20: ("fr3", "3.robot_arm/20.fr3/mjcf/fr3_robot.xml", "3.robot_arm/20.fr3/mjcf/fr3_pinocchio.xml"),
            21: ("ur5e", "3.robot_arm/21.ur5e/mjcf/ur5e_robot.xml", "3.robot_arm/21.ur5e/mjcf/ur5e_pinocchio.xml"),
            22: ("piper", "3.robot_arm/22.piper/mjcf/piper_robot.xml", "3.robot_arm/22.piper/mjcf/piper_pinocchio.xml"),
            23: ("rm75", "3.robot_arm/23.rm75/mjcf/rm75_robot.xml", "3.robot_arm/23.rm75/mjcf/rm75_pinocchio.xml"),
            24: ("so101", "3.robot_arm/24.so101/mjcf/so101_robot.xml", "3.robot_arm/24.so101/mjcf/so101_pinocchio.xml"),
        }

        def create_config(robot_num):
            name, robot_rel, pino_rel = robot_map[robot_num]
            return {
                "robot_name": name,
                "robot_control_freq": 100,
                "robot_mjcf": str(models_root / robot_rel),
                "pino_mjcf": str(models_root / pino_rel),
            }

        self.master_robot_manager = RobotManager(create_config(self.master_number), logger)
        self.slave_robot_managers = {
            num: RobotManager(create_config(num), logger) for num in self.slave_numbers
        }

        # Get slave robot name for convenience
        self.slave_robot_name = self.slave_robot_managers[self.robot_config["number"]].getRobotName()

        # Control parameters
        self.control_freq = 100.0
        self.control_dt = 1.0 / self.control_freq

        # MuJoCo state
        self.model = None
        self.data = None

        # FSM
        self.fsm = PyFSM(verbose=True, lerp_duration=1.0)

        # Lerp state
        self.lerp_start_time = None
        self.lerp_q0 = {}
        self.lerp_q1 = {}

        # Mode-specific attributes
        if not self.use_viewer:
            # GLFW mode
            self.simulation_mutex = Lock()
            self.render_mutex = Lock()
            self.exit_request = threading.Event()
            self.pause_sim = False

            self.mjstep_freq = 5 * self.control_freq
            self.render_freq = 60.0
            self.mjstep_dt = 1.0 / self.mjstep_freq
            self.render_dt = 1.0 / self.render_freq

            self.window = None
            self.viewport_width = 1600
            self.viewport_height = 1200
            self.scene = None
            self.context = None
            self.camera = None
            self.option = None
            self.glfw_initialized = False

            self.cameras = []
            self.camera_names = []
            self.render_buffer = []
            self.render_buffer_updated = threading.Event()
        else:
            # Viewer mode
            self.last_control_time = time.time()

        # Initialize system
        if not self.use_viewer:
            self._init_glfw()

        self._init_scene()

        if not self.use_viewer:
            self._init_mujoco_rendering()

        self._get_master_info()
        self._get_slaves_info()
        self._init_teleop()
        self._init_joint_positions()

    def _init_joint_positions(self):
        """Initialize lerp position arrays."""
        for slave_num, robot_manager in self.slave_robot_managers.items():
            robot_name = robot_manager.getRobotName()
            if robot_name in self.slaves_njnt:
                s_dof = self.slaves_pin_kine[robot_name].pin_model.nq
                self.lerp_q0[robot_name] = np.zeros(s_dof)
                self.lerp_q1[robot_name] = np.zeros(s_dof)

                if self.is_dual_arm:
                    self.lerp_q0[robot_name + "_2"] = np.zeros(s_dof)
                    self.lerp_q1[robot_name + "_2"] = np.zeros(s_dof)

    def _calc_standby_pos(self):
        """Calculate standby positions for lerp initialization."""
        m_full_pos = self.master_interface.get_joint_positions()
        m_motion = self._get_motion_dof_from_master(m_full_pos)

        if self.is_dual_arm:
            m2_full_pos = self.master2_interface.get_joint_positions()
            m2_motion = self._get_motion_dof_from_master(m2_full_pos)

        # Get standby positions from PRobotManager
        master_qStand_left = self.master_robot_manager.get_qStandby("left")
        master_qStand_right = self.master_robot_manager.get_qStandby("right")
        slave_qStand_left = self.slave_robot_managers[self.robot_config["number"]].get_qStandby("left")
        slave_qStand_right = self.slave_robot_managers[self.robot_config["number"]].get_qStandby("right")

        for key in self.slaves_range_in_qpos.keys():
            s_dof = self.slaves_pin_kine[key].pin_model.nq
            qpos_start = self.slaves_range_in_qpos[key][0]

            self.lerp_q0[key] = self.data.qpos[qpos_start : qpos_start + s_dof].copy()

            # Compute standby position using new matrix-based formula (arm 1 = left)
            qDirection_left = self.robot_config["qDirection_left"]
            s_standby = compute_slave_command(
                m_motion, master_qStand_left, slave_qStand_left, qDirection_left
            )
            self.lerp_q1[key] = s_standby

            if self.is_dual_arm:
                qpos2_start = self.slaves2_range_in_qpos[key][0]
                self.lerp_q0[key + "_2"] = self.data.qpos[qpos2_start : qpos2_start + s_dof].copy()

                # Compute standby position for arm 2 (right)
                qDirection_right = self.robot_config["qDirection_right"]
                s2_standby = compute_slave_command(
                    m2_motion, master_qStand_right, slave_qStand_right, qDirection_right
                )
                self.lerp_q1[key + "_2"] = s2_standby

    def _init_glfw(self):
        """Initialize GLFW window (Linux only)."""
        if self.use_viewer:
            return

        try:
            if not glfw.init():
                raise RuntimeError("Failed to initialize GLFW")

            self.window = glfw.create_window(
                self.viewport_width,
                self.viewport_height,
                f"SO101-to-{self.robot_config['name']} Teleoperation",
                None,
                None,
            )

            if not self.window:
                glfw.terminate()
                raise RuntimeError("Failed to create GLFW window")

            glfw.make_context_current(self.window)
            glfw.swap_interval(1)

            def key_callback(window, key, scancode, action, mods):
                if action == glfw.PRESS:
                    if key == glfw.KEY_R:
                        if self.fsm.current_state in [self.fsm.go_standby1, self.fsm.do_action1]:
                            self.fsm.to_reset()
                            print("Reset requested")
                    elif key == glfw.KEY_ESCAPE:
                        self.exit_request.set()

            glfw.set_key_callback(self.window, key_callback)

            print("GLFW initialized successfully")
            self.glfw_initialized = True

        except Exception as e:
            print(f"Error initializing GLFW: {e}")
            raise

    def _init_scene(self):
        """Initialize MuJoCo scene."""
        try:
            models_root = get_models_root()
            scene_mjcf = str(models_root / self.robot_config["scene_path"])

            if not os.path.exists(scene_mjcf):
                raise FileNotFoundError(f"Scene file not found: {scene_mjcf}")

            print(f"Loading scene from: {scene_mjcf}")

            self.model = mujoco.MjModel.from_xml_path(scene_mjcf)
            self.data = mujoco.MjData(self.model)
            mujoco.mj_forward(self.model, self.data)

            if not self.use_viewer:
                self.model.opt.timestep = self.mjstep_dt
                self._init_cameras()

            self.whole_joint_command = np.array([])
            self.master_joint_positions = None
            self.slaves_joint_positions = {}
            self.slaves_ref_pose = {}

            if self.is_dual_arm:
                self.master2_joint_positions = None
                self.slaves2_joint_positions = {}
                self.slaves2_ref_pose = {}

            print("Scene initialized successfully")

        except Exception as e:
            print(f"Error initializing scene: {e}")
            raise

    def _init_cameras(self):
        """Initialize camera list for rendering (GLFW mode only)."""
        if self.use_viewer:
            return

        self.cameras = []
        self.camera_names = []
        target_cameras = ["overhead_cam"]

        for i in range(self.model.ncam):
            camera_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            if camera_name and camera_name in target_cameras:
                self.cameras.append(i)
                self.camera_names.append(camera_name)

        with self.render_mutex:
            self.render_buffer = [
                (cam_id, cam_name) for cam_id, cam_name in zip(self.cameras, self.camera_names)
            ]
            self.render_buffer_updated.set()

        print(f"Initialized {len(self.cameras)} cameras: {self.camera_names}")

    def _init_mujoco_rendering(self):
        """Initialize MuJoCo rendering context (GLFW mode only)."""
        if self.use_viewer:
            return

        try:
            self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
            self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self.camera = mujoco.MjvCamera()
            self.option = mujoco.MjvOption()

            mujoco.mjv_defaultCamera(self.camera)
            mujoco.mjv_defaultOption(self.option)

            print("MuJoCo rendering components initialized successfully")

        except Exception as e:
            print(f"Error initializing MuJoCo rendering: {e}")
            raise

    def _get_master_info(self):
        """Load master robot configuration from YAML."""
        try:
            with open(self.config_path, "r") as f:
                self.master_config = yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load {self.config_path}: {e}")
            print("Using default port settings")
            self.master_config = {}

        self.master_port = self.master_config.get("leader", {}).get("port", "/dev/ttyACM1")
        self.master_port_2 = self.master_config.get("leader", {}).get("port_2", "/dev/ttyACM2")

        self.master_calibration_dir = self.master_config.get("leader", {}).get("calibration_dir")
        self.master_calibration_dir_2 = self.master_config.get("leader", {}).get("calibration_dir_2")

    def _get_slaves_info(self):
        """Set up slave robot joint indexing from configuration."""
        self.slaves_range_in_qpos = {}
        self.slaves_range_in_ctrl = {}
        self.slaves2_range_in_qpos = {}
        self.slaves2_range_in_ctrl = {}
        self.slaves_njnt = {}
        self.slaves_nctrl = {}

        robot_name = self.slave_robot_managers[self.robot_config["number"]].getRobotName()

        # Use configuration ranges
        cfg = self.robot_config
        self.slaves_range_in_qpos[robot_name] = list(cfg["qpos_arm1"])
        self.slaves_range_in_ctrl[robot_name] = list(cfg["ctrl_arm1"])
        self.slaves2_range_in_qpos[robot_name] = list(cfg["qpos_arm2"])
        self.slaves2_range_in_ctrl[robot_name] = list(cfg["ctrl_arm2"])

        # Compute njnt and nctrl from ranges
        self.slaves_njnt[robot_name] = cfg["qpos_arm1"][1] - cfg["qpos_arm1"][0]
        self.slaves_nctrl[robot_name] = cfg["ctrl_arm1"][1] - cfg["ctrl_arm1"][0]

        print(f"\n=== Dual {cfg['name']} Joint Indexing ===")
        print(f"Robot: {robot_name}")
        print(f"Arm 1 qpos: {self.slaves_range_in_qpos[robot_name]}")
        print(f"Arm 1 ctrl: {self.slaves_range_in_ctrl[robot_name]}")
        print(f"Arm 2 qpos: {self.slaves2_range_in_qpos[robot_name]}")
        print(f"Arm 2 ctrl: {self.slaves2_range_in_ctrl[robot_name]}")
        print(f"njnt: {self.slaves_njnt[robot_name]}, nctrl: {self.slaves_nctrl[robot_name]}")
        print("=" * 40)

    def _get_motion_dof_from_master(self, full_joint_positions):
        """Extract motion DOF from master robot (exclude gripper).

        Uses pin_model.nq to determine motion DOF instead of assuming
        gripper is always the last joint.
        """
        m_dof = self.master_pin_kine.pin_model.nq
        return full_joint_positions[:m_dof]

    def _get_motion_dof_from_slave(self, key, full_joint_positions):
        """Extract motion DOF from slave robot."""
        motion_dof = self.slaves_pin_kine[key].pin_model.nq
        return full_joint_positions[:motion_dof]

    def _load_arm_length(self, pinkine: PinKine, q_config: np.ndarray) -> dict:
        """Load arm segment lengths from robot kinematics."""
        pinkine.update(q_config)

        base_pos = np.zeros(3)
        shoulder_pos = pinkine.getSitePosByName("shoulderSite")
        elbow_pos = pinkine.getSitePosByName("elbowSite")
        wrist_pos = pinkine.getSitePosByName("wristSite")
        ee_pos = pinkine.getSitePosByName("EE")

        return {
            "l0": shoulder_pos - base_pos,
            "l1": elbow_pos - shoulder_pos,
            "l2": wrist_pos - elbow_pos,
            "l3": ee_pos - wrist_pos,
        }

    def _init_teleop(self):
        """Initialize teleoperation interfaces and kinematics."""
        self.site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]

        master_robot_name = self.master_robot_manager.getRobotName()
        master_pin_mjcf = self.master_robot_manager.getPinoMJCF()

        # Initialize master interface (arm 1)
        self.master_interface = create_robot_interface(
            name=master_robot_name,
            mode="teleop",
            config_path=self.config_path,
            port=self.master_port,
            calibration_dir=self.master_calibration_dir,
        )
        self.master_interface.init()
        self.master_interface.disable_robot_torque()

        self.master_pin_kine = PinKine(master_pin_mjcf, self.site_names)
        master_qStand = self.master_robot_manager.get_qStand()
        self.master_armlength_config = self._load_arm_length(self.master_pin_kine, master_qStand)

        # Initialize master interface (arm 2) for dual-arm
        if self.is_dual_arm:
            self.master2_interface = create_robot_interface(
                name=master_robot_name,
                mode="teleop",
                config_path=self.config_path,
                port=self.master_port_2,
                calibration_dir=self.master_calibration_dir_2,
            )
            self.master2_interface.init()
            self.master2_interface.disable_robot_torque()
            self.master2_pin_kine = PinKine(master_pin_mjcf, self.site_names)
            self.master2_armlength_config = self._load_arm_length(self.master2_pin_kine, master_qStand)

        # Initialize slave kinematics
        self.slaves_pin_kine = {}
        self.slaves_armlength_config = {}

        if self.is_dual_arm:
            self.slaves2_pin_kine = {}
            self.slaves2_armlength_config = {}

        for slave_num, robot_manager in self.slave_robot_managers.items():
            robot_name = robot_manager.getRobotName()
            slave_pin_mjcf = robot_manager.getPinoMJCF()
            slave_qStand = robot_manager.get_qStand()

            self.slaves_pin_kine[robot_name] = PinKine(slave_pin_mjcf, self.site_names)
            self.slaves_armlength_config[robot_name] = self._load_arm_length(
                self.slaves_pin_kine[robot_name], slave_qStand
            )

            if self.is_dual_arm:
                self.slaves2_pin_kine[robot_name] = PinKine(slave_pin_mjcf, self.site_names)
                self.slaves2_armlength_config[robot_name] = self._load_arm_length(
                    self.slaves2_pin_kine[robot_name], slave_qStand
                )

    def _lerp_to_target(self):
        """Interpolate to target position."""
        if self.lerp_start_time is None:
            self.lerp_start_time = time.time()

        t_elapsed = time.time() - self.lerp_start_time

        if not self.use_viewer:
            with self.simulation_mutex:
                self._apply_lerp(t_elapsed)
        else:
            self._apply_lerp(t_elapsed)

        return t_elapsed >= self.fsm.lerp_duration

    def _apply_lerp(self, t_elapsed):
        """Apply lerp commands to data.ctrl."""
        for key in self.slaves_range_in_qpos.keys():
            s_dof = self.slaves_pin_kine[key].pin_model.nq

            qCmd = lerp(self.lerp_q0[key], self.lerp_q1[key], t_elapsed, self.fsm.lerp_duration)

            ctrl_start = self.slaves_range_in_ctrl[key][0]
            self.data.ctrl[ctrl_start : ctrl_start + s_dof] = qCmd

            if self.is_dual_arm:
                qCmd2 = lerp(
                    self.lerp_q0[key + "_2"],
                    self.lerp_q1[key + "_2"],
                    t_elapsed,
                    self.fsm.lerp_duration,
                )

                ctrl_start2 = self.slaves2_range_in_ctrl[key][0]
                self.data.ctrl[ctrl_start2 : ctrl_start2 + s_dof] = qCmd2

    def _get_feedback(self):
        """Read joint positions from master robots."""
        master_full_positions = self.master_interface.get_joint_positions()
        self.master_joint_positions = self._get_motion_dof_from_master(master_full_positions)
        self.master_gripper_position = master_full_positions[-1]

        if self.is_dual_arm:
            master2_full_positions = self.master2_interface.get_joint_positions()
            self.master2_joint_positions = self._get_motion_dof_from_master(master2_full_positions)
            self.master2_gripper_position = master2_full_positions[-1]

        for key in self.slaves_range_in_qpos.keys():
            qpos_range = self.slaves_range_in_qpos[key]
            self.slaves_joint_positions[key] = self.data.qpos[qpos_range[0] : qpos_range[1]].copy()

            if self.is_dual_arm:
                qpos2_range = self.slaves2_range_in_qpos[key]
                self.slaves2_joint_positions[key] = self.data.qpos[qpos2_range[0] : qpos2_range[1]].copy()

    def _set_control(self):
        """Compute and set slave robot control commands."""
        self.whole_joint_command = np.zeros(self.model.nu)

        m_dof = self.master_pin_kine.pin_model.nq
        master_motion_joints = self.master_joint_positions[:m_dof]

        # Get standby positions from PRobotManager
        master_qStand_left = self.master_robot_manager.get_qStandby("left")
        master_qStand_right = self.master_robot_manager.get_qStandby("right")
        slave_qStand_left = self.slave_robot_managers[self.robot_config["number"]].get_qStandby("left")
        slave_qStand_right = self.slave_robot_managers[self.robot_config["number"]].get_qStandby("right")

        if self.is_dual_arm:
            m2_dof = self.master2_pin_kine.pin_model.nq
            master2_motion_joints = self.master2_joint_positions[:m2_dof]

        for key in self.slaves_range_in_qpos.keys():
            s_dof = self.slaves_pin_kine[key].pin_model.nq

            # Compute motion command using new matrix-based formula (arm 1 = left)
            qDirection_left = self.robot_config["qDirection_left"]
            slave_motion_command = compute_slave_command(
                master_motion_joints, master_qStand_left, slave_qStand_left, qDirection_left
            )

            # Compute gripper command
            ctrl_range = self.slaves_range_in_ctrl[key]
            gripper_idx = ctrl_range[1] - 1
            slave_gripper_ctrlrange_min = self.model.actuator_ctrlrange[gripper_idx][0]
            slave_gripper_ctrlrange_max = self.model.actuator_ctrlrange[gripper_idx][1]
            slave_gripper_cmd = (
                slave_gripper_ctrlrange_min
                + slave_gripper_ctrlrange_max * self.master_gripper_position * 2.0
            )

            slave_joint_command = np.append(slave_motion_command, slave_gripper_cmd)
            self.whole_joint_command[ctrl_range[0] : ctrl_range[1]] = slave_joint_command

            if self.is_dual_arm:
                s2_dof = self.slaves2_pin_kine[key].pin_model.nq

                # Compute motion command for arm 2 (right)
                qDirection_right = self.robot_config["qDirection_right"]
                slave2_motion_command = compute_slave_command(
                    master2_motion_joints, master_qStand_right, slave_qStand_right, qDirection_right
                )

                ctrl2_range = self.slaves2_range_in_ctrl[key]
                gripper2_idx = ctrl2_range[1] - 1
                slave2_gripper_ctrlrange_min = self.model.actuator_ctrlrange[gripper2_idx][0]
                slave2_gripper_ctrlrange_max = self.model.actuator_ctrlrange[gripper2_idx][1]
                slave2_gripper_cmd = (
                    slave2_gripper_ctrlrange_min
                    + slave2_gripper_ctrlrange_max * self.master2_gripper_position * 2.0
                )

                slave2_joint_command = np.append(slave2_motion_command, slave2_gripper_cmd)

                self.whole_joint_command[ctrl2_range[0] : ctrl2_range[1]] = slave2_joint_command

        self.data.ctrl = self.whole_joint_command.copy()

    # =============================================================================
    # GLFW Mode Methods (Linux)
    # =============================================================================

    def _simulation_thread_func(self):
        """Physics simulation thread (GLFW mode only)."""
        print(f"Starting simulation thread at {self.mjstep_freq}Hz, control at {self.control_freq}Hz")
        last_control_time = time.time()

        while not self.exit_request.is_set():
            loop_start_time = time.time()

            if self.pause_sim:
                time.sleep(0.001)
                continue

            if self.fsm.current_state == self.fsm.initr:
                self._calc_standby_pos()
                self.fsm.to_go_standby1()
            elif self.fsm.current_state == self.fsm.go_standby1:
                if self._lerp_to_target():
                    self.fsm.to_do_action1()
                    self.lerp_start_time = None
                self._simulation_step()
            elif self.fsm.current_state == self.fsm.do_action1:
                if time.time() - last_control_time > self.control_dt:
                    last_control_time = time.time()
                    with self.simulation_mutex:
                        self._get_feedback()
                        self._set_control()
                self._simulation_step()
            elif self.fsm.current_state == self.fsm.reset:
                self._calc_standby_pos()
                self.fsm.finish_reset()

            elapsed = time.time() - loop_start_time
            sleep_time = self.mjstep_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _simulation_step(self):
        """Single physics step (GLFW mode only)."""
        with self.simulation_mutex:
            mujoco.mj_step(self.model, self.data)

    def _render_cameras(self):
        """Render camera views (GLFW mode only)."""
        if not self.glfw_initialized or self.scene is None or self.context is None:
            return

        if glfw.get_current_context() != self.window:
            glfw.make_context_current(self.window)

        with self.simulation_mutex:
            width, height = glfw.get_framebuffer_size(self.window)

            if len(self.cameras) == 0:
                return

            cam_id = self.cameras[0]

            try:
                cam = mujoco.MjvCamera()
                mujoco.mjv_defaultCamera(cam)
                cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                cam.fixedcamid = cam_id

                mujoco.mjv_updateScene(
                    self.model,
                    self.data,
                    self.option,
                    None,
                    cam,
                    mujoco.mjtCatBit.mjCAT_ALL,
                    self.scene,
                )

                viewport = mujoco.MjrRect(0, 0, width, height)
                mujoco.mjr_render(viewport, self.scene, self.context)

            except Exception as e:
                print(f"Error rendering camera: {e}")

    def _update_display(self):
        """Update GLFW display (GLFW mode only)."""
        if not self.window or glfw.window_should_close(self.window):
            return

        if not self.glfw_initialized or self.context is None:
            return

        glfw.make_context_current(self.window)

        width, height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)

        mujoco.mjr_rectangle(viewport, 0.1, 0.1, 0.1, 1.0)

        self._render_cameras()

        with self.simulation_mutex:
            time_msg = f"time: {self.data.time:.2f}"

        mujoco.mjr_overlay(
            mujoco.mjtFont.mjFONT_BIG,
            mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
            viewport,
            time_msg,
            "",
            self.context,
        )

        glfw.swap_buffers(self.window)

    def _run_glfw(self):
        """Main loop for GLFW mode (Linux)."""
        try:
            print("=" * 60)
            print(f"SO101-to-{self.robot_config['name']} Teleoperation Application")
            print("=" * 60)
            print(f"Master: SO101 (robot #{self.master_number})")
            print(f"Slave: Dual {self.robot_config['name']} (robot #{self.robot_config['number']})")
            print(f"Mode: Dual-arm (GLFW)")
            print("=" * 60)
            print("\nControls:")
            print("  R - Reset simulation")
            print("  ESC - Exit")
            print("\nNote: Teleoperation starts automatically after initialization")
            print()

            physics_thread = threading.Thread(target=self._simulation_thread_func, daemon=True)
            physics_thread.start()

            print("Simulation thread started")
            print("Starting main display loop...")

            last_time = time.time()

            while not self.exit_request.is_set() and not glfw.window_should_close(self.window):
                glfw.poll_events()

                current_time = time.time()
                if current_time - last_time >= self.render_dt:
                    self._update_display()
                    last_time = current_time
                else:
                    time.sleep(0.001)

            print("Shutting down...")
            self.exit_request.set()

            if physics_thread.is_alive():
                physics_thread.join(timeout=1.0)

            print("Application shut down successfully")

        except Exception as e:
            print(f"Error in main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if self.window:
                glfw.destroy_window(self.window)
            if self.glfw_initialized:
                glfw.terminate()

    # =============================================================================
    # Viewer Mode Methods (macOS)
    # =============================================================================

    def _control_callback(self, model, data):
        """Control callback for viewer mode."""
        if self.fsm.current_state == self.fsm.initr:
            self._calc_standby_pos()
            self.fsm.to_go_standby1()
        elif self.fsm.current_state == self.fsm.go_standby1:
            if self._lerp_to_target():
                self.fsm.to_do_action1()
                self.lerp_start_time = None
        elif self.fsm.current_state == self.fsm.do_action1:
            if time.time() - self.last_control_time > self.control_dt:
                self.last_control_time = time.time()
                self._get_feedback()
                self._set_control()
        elif self.fsm.current_state == self.fsm.reset:
            self._calc_standby_pos()
            self.fsm.finish_reset()

    def _run_viewer(self):
        """Main loop for viewer mode (macOS)."""
        try:
            print("=" * 60)
            print(f"SO101-to-{self.robot_config['name']} Teleoperation Application (macOS)")
            print("=" * 60)
            print(f"Master: SO101 (robot #{self.master_number})")
            print(f"Slave: Dual {self.robot_config['name']} (robot #{self.robot_config['number']})")
            print(f"Mode: Dual-arm (Viewer)")
            print("=" * 60)
            print("\nControls:")
            print("  R - Reset simulation")
            print("  ESC - Exit")
            print("\nNote: Teleoperation starts automatically after initialization")
            print()

            # Import mujoco.viewer here to avoid import on Linux
            import mujoco.viewer

            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=False,
                show_right_ui=True
            ) as viewer:
                # Set camera to overhead_cam
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_CAMERA, "overhead_cam"
                )

                # Main control loop
                while viewer.is_running():
                    step_start = time.time()

                    # Call control logic
                    self._control_callback(self.model, self.data)

                    # Step simulation
                    mujoco.mj_step(self.model, self.data)

                    # Sync viewer
                    viewer.sync()

                    # Timing
                    elapsed = time.time() - step_start
                    sleep_time = self.model.opt.timestep - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

            print("Application shut down successfully")

        except Exception as e:
            print(f"Error in main loop: {e}")
            import traceback
            traceback.print_exc()

    # =============================================================================
    # Public API
    # =============================================================================

    def run(self):
        """Run teleoperation application."""
        if self.use_viewer:
            self._run_viewer()
        else:
            self._run_glfw()


def main():
    """Entry point for so101-teleop command."""
    parser = argparse.ArgumentParser(
        description="SO101 Teleoperation for dual-arm robots",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  so101-teleop --robot fr3              # By name
  so101-teleop --robot 20               # By number
  so101-teleop --robot ur5e --mode glfw # Force GLFW mode
  so101-teleop --robot piper --mode viewer # Force viewer mode

Supported robots:
  fr3   (20) - Franka FR3
  ur5e  (21) - Universal Robots UR5e
  piper (22) - Piper dual-arm
  rm75  (23) - RealMan RM75
        """
    )

    parser.add_argument(
        "--robot",
        type=str,
        default="fr3",
        help="Robot name (fr3/ur5e/piper/rm75) or number (20/21/22/23). Default: fr3"
    )

    parser.add_argument(
        "--mode",
        type=str,
        choices=["auto", "glfw", "viewer"],
        default="auto",
        help="Rendering mode. Default: auto (detects platform)"
    )

    parser.add_argument(
        "--config",
        type=str,
        default="configs/so101.yaml",
        help="Path to so101 config file. Default: configs/so101.yaml"
    )

    args = parser.parse_args()

    # Determine use_viewer from mode
    if args.mode == "auto":
        use_viewer = None  # Auto-detect
    elif args.mode == "viewer":
        use_viewer = True
    else:  # glfw
        use_viewer = False

    try:
        app = SO101Teleop(args.robot, use_viewer=use_viewer, config_path=args.config)
        app.run()
    except ValueError as e:
        print(f"Error: {e}")
        parser.print_help()
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130

    return 0


if __name__ == "__main__":
    exit(main())
