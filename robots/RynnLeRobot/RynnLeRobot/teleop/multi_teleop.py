"""Multi-robot teleoperation with MuJoCo visualization.

Usage:
    $ multi-teleop --dual --master_number 24
    $ multi-teleop -d -mn 24 -sn 20,21,22,23
"""

import argparse
import mujoco
import glfw
import numpy as np
import time
import threading
import os
import yaml
from threading import Lock
from RynnMotion.utils.pyFSM import PyFSM
from RynnMotion.utils.path_config import get_models_root
from RynnMotion.utils.policy_interpolator import lerp
from RynnMotion.manager.robot_manager import RobotManager
import logging
from RynnLeRobot.interface.so101_interface import create_robot_interface
from RynnLeRobot.teleop.so101_teleop_config import ROBOT_CONFIGS, compute_slave_command


class UnifiedMujocoGLFW:
    """Multi-robot teleoperation with MuJoCo GLFW viewer.

    Attributes:
        is_dual_arm: Dual-arm mode enabled.
        master_number: Master robot number.
        slave_numbers: List of slave robot numbers.
    """

    def __init__(self, is_dual_arm: bool, master_number: int, slave_numbers: list):
        self.is_dual_arm = is_dual_arm
        self.master_number = master_number
        self.slave_numbers = slave_numbers

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

        self.master_robot_manager = RobotManager(create_config(master_number), logger)
        self.slave_robot_managers = {num: RobotManager(create_config(num), logger) for num in slave_numbers}

        self.simulation_mutex = Lock()
        self.render_mutex = Lock()
        self.exit_request = threading.Event()
        self.pause_sim = False
        self.reset_sim = False

        self.control_freq = 100.0
        self.mjstep_freq = 5 * self.control_freq
        self.render_freq = 60.0
        self.control_dt = 1.0 / self.control_freq
        self.mjstep_dt = 1.0 / self.mjstep_freq
        self.render_dt = 1.0 / self.render_freq

        self.model = None
        self.data = None

        self.cameras = []
        self.camera_names = []
        self.camera_frames = []

        self.window = None
        self.viewport_width = 1200
        self.viewport_height = 800

        self.viewport_width = 1600
        self.viewport_height = 1200

        self.scene = None
        self.context = None
        self.camera = None
        self.option = None

        self.render_buffer = []
        self.render_buffer_updated = threading.Event()

        self.glfw_initialized = False

        self.fsm = PyFSM(verbose=True, lerp_duration=1.0)

        self.lerp_start_time = None
        self.lerp_q0 = {}
        self.lerp_q1 = {}

        self.init_glfw()
        self.init_scene()
        self.init_mujoco_rendering()
        self.get_master_info()
        self.get_slaves_info()
        self.init_teleop()
        self.init_joint_positions()

    def init_joint_positions(self):
        """Initialize joint positions for all slaves to qStand"""
        for slave_num, robot_manager in self.slave_robot_managers.items():
            robot_name = robot_manager.getRobotName()
            if robot_name in self.slaves_njnt:
                qStand = robot_manager.get_qStand()
                n_dof = len(qStand)
                self.lerp_q0[robot_name] = np.zeros(n_dof)
                self.lerp_q1[robot_name] = qStand.copy()

                if self.is_dual_arm:
                    self.lerp_q0[robot_name + "_2"] = np.zeros(n_dof)
                    self.lerp_q1[robot_name + "_2"] = qStand.copy()

    def calc_standby_pos(self):
        """Calculate mapped standby positions from master for each slave type"""
        m_full_pos = self.master_interface.get_joint_positions()
        m_motion = m_full_pos[:5]

        if self.is_dual_arm:
            m2_full_pos = self.master2_interface.get_joint_positions()
            m2_motion = m2_full_pos[:5]

        for key in self.slaves_range_in_qpos.keys():
            n_dof = len(self.lerp_q1[key])
            qpos_start = self.slaves_range_in_qpos[key][0]
            self.lerp_q0[key] = self.data.qpos[qpos_start : qpos_start + n_dof].copy()

            # Use universal mapping with left arm
            self.lerp_q1[key] = self._get_joint_mapping(key, m_motion, arm_side="left")

            if self.is_dual_arm:
                qpos_start2 = self.slaves2_range_in_qpos[key][0]
                self.lerp_q0[key + "_2"] = self.data.qpos[qpos_start2 : qpos_start2 + n_dof].copy()

                # Use universal mapping with right arm
                self.lerp_q1[key + "_2"] = self._get_joint_mapping(key, m2_motion, arm_side="right")

    def init_glfw(self):
        """Initialize GLFW window and OpenGL context"""
        try:
            if not glfw.init():
                raise RuntimeError("Failed to initialize GLFW")

            self.window = glfw.create_window(
                self.viewport_width,
                self.viewport_height,
                "Unified MuJoCo Multi-Robot Viewer",
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
                        if self.fsm.current_state == self.fsm.go_standby1 or self.fsm.current_state == self.fsm.do_action1:
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

    def init_scene(self):
        """Initialize the multi-robot scene"""
        try:
            models_root = get_models_root()

            if self.is_dual_arm:
                scene_mjcf = models_root / "3.robot_arm/24.so101/scene/multi_dual_robots.xml"
            else:
                scene_mjcf = models_root / "3.robot_arm/24.so101/scene/multi_robots.xml"
            scene_mjcf = str(scene_mjcf)

            if not os.path.exists(scene_mjcf):
                raise FileNotFoundError(f"Scene file not found: {scene_mjcf}")

            print(f"Loading scene from: {scene_mjcf}")

            self.model = mujoco.MjModel.from_xml_path(scene_mjcf)
            self.data = mujoco.MjData(self.model)
            mujoco.mj_forward(self.model, self.data)
            self.model.opt.timestep = self.mjstep_dt

            self.init_cameras()

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

    def lerp_to_target(self):
        """Linear interpolation to target joint positions"""
        if self.lerp_start_time is None:
            self.lerp_start_time = time.time()

        t_elapsed = time.time() - self.lerp_start_time

        with self.simulation_mutex:
            for key in self.slaves_range_in_qpos.keys():
                n_dof = len(self.lerp_q1[key])

                qCmd = lerp(self.lerp_q0[key], self.lerp_q1[key], t_elapsed, self.fsm.lerp_duration)

                ctrl_start = self.slaves_range_in_ctrl[key][0]
                self.data.ctrl[ctrl_start : ctrl_start + n_dof] = qCmd

                if self.is_dual_arm:
                    qCmd2 = lerp(self.lerp_q0[key + "_2"], self.lerp_q1[key + "_2"], t_elapsed, self.fsm.lerp_duration)

                    ctrl_start2 = self.slaves2_range_in_ctrl[key][0]
                    self.data.ctrl[ctrl_start2 : ctrl_start2 + n_dof] = qCmd2

        return t_elapsed >= self.fsm.lerp_duration

    def init_cameras(self):
        """Initialize all fixed cameras from the scene"""
        self.cameras = []
        self.camera_names = []
        self.camera_frames = []

        target_cameras = ["camera_franka", "camera_ur5e", "camera_piper", "camera_rm75"]

        for i in range(self.model.ncam):
            camera_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            if camera_name and camera_name in target_cameras:
                self.cameras.append(i)
                self.camera_names.append(camera_name)
                self.camera_frames.append(
                    np.zeros(
                        (self.viewport_height // 2, self.viewport_width // 3, 3),
                        dtype=np.uint8,
                    )
                )

        with self.render_mutex:
            self.render_buffer = [(cam_id, cam_name) for cam_id, cam_name in zip(self.cameras, self.camera_names)]
            self.render_buffer_updated.set()

        print(f"Initialized {len(self.cameras)} cameras: {self.camera_names}")

    def init_mujoco_rendering(self):
        """Initialize MuJoCo rendering components"""
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

    def get_master_info(self):
        """Load master robot configuration - ports and calibration from so101 config"""
        try:
            config_path = "configs/so101.yaml"
            with open(config_path, "r") as f:
                self.master_config = yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load {config_path}: {e}")
            print("Using default port settings")
            self.master_config = {}

        self.master_port = self.master_config.get("teleoperate", {}).get("port", "/dev/ttyACM1")
        self.master_port_2 = self.master_config.get("teleoperate", {}).get("port_2", "/dev/ttyACM2")

        self.master_calibration_dir = self.master_config.get("teleoperate", {}).get("calibration_dir")
        self.master_calibration_dir_2 = self.master_config.get("teleoperate", {}).get("calibration_dir_2")

    def get_slaves_info(self):
        """Initialize slave robot information using PRobotManager"""
        self.slaves_range_in_qpos = {}
        self.slaves_range_in_ctrl = {}
        if self.is_dual_arm:
            self.slaves2_range_in_qpos = {}
            self.slaves2_range_in_ctrl = {}
        self.slaves_njnt = {}
        self.slaves_nctrl = {}

        self.get_index_info(index_type="qpos")
        self.get_index_info(index_type="ctrl")

        for slave_num, robot_manager in self.slave_robot_managers.items():
            robot_name = robot_manager.getRobotName()

            if robot_name in self.slaves_range_in_qpos:
                self.slaves_njnt[robot_name] = self.slaves_range_in_qpos[robot_name][1] - self.slaves_range_in_qpos[robot_name][0]
                self.slaves_nctrl[robot_name] = self.slaves_range_in_ctrl[robot_name][1] - self.slaves_range_in_ctrl[robot_name][0]

                if self.is_dual_arm:
                    self.slaves_njnt[robot_name] = int(self.slaves_njnt[robot_name] / 2)
                    self.slaves_nctrl[robot_name] = int(self.slaves_nctrl[robot_name] / 2)
                    self.slaves2_range_in_qpos[robot_name] = [
                        self.slaves_range_in_qpos[robot_name][1] - self.slaves_njnt[robot_name],
                        self.slaves_range_in_qpos[robot_name][1],
                    ]
                    self.slaves_range_in_qpos[robot_name] = [
                        self.slaves_range_in_qpos[robot_name][0],
                        self.slaves_range_in_qpos[robot_name][0] + self.slaves_njnt[robot_name],
                    ]
                    self.slaves2_range_in_ctrl[robot_name] = [
                        self.slaves_range_in_ctrl[robot_name][1] - self.slaves_nctrl[robot_name],
                        self.slaves_range_in_ctrl[robot_name][1],
                    ]
                    self.slaves_range_in_ctrl[robot_name] = [
                        self.slaves_range_in_ctrl[robot_name][0],
                        self.slaves_range_in_ctrl[robot_name][0] + self.slaves_nctrl[robot_name],
                    ]

    def get_index_info(self, index_type: str):
        """Parse MuJoCo model to find index ranges for each slave robot"""
        if index_type not in ["qpos", "ctrl"]:
            raise ValueError("Index_type must be 'qpos' or 'ctrl'")

        iter_range = self.model.njnt if index_type == "qpos" else self.model.nu
        mujoco_obj = mujoco.mjtObj.mjOBJ_JOINT if index_type == "qpos" else mujoco.mjtObj.mjOBJ_ACTUATOR
        current_slave_name = "init_slave_name"

        slave_robot_names = [mgr.getRobotName() for mgr in self.slave_robot_managers.values()]

        for i in range(iter_range):
            current_name_from_mujoco = mujoco.mj_id2name(self.model, mujoco_obj, i)
            if current_slave_name not in current_name_from_mujoco:
                slave_name_before = current_slave_name
                for robot_name in slave_robot_names:
                    if robot_name in current_name_from_mujoco:
                        current_slave_name = robot_name
                        if index_type == "qpos":
                            self.slaves_range_in_qpos[current_slave_name] = [i]
                        else:
                            self.slaves_range_in_ctrl[current_slave_name] = [i]
                        break
                if slave_name_before != "init_slave_name":
                    if index_type == "qpos" and len(self.slaves_range_in_qpos[slave_name_before]) == 1:
                        self.slaves_range_in_qpos[slave_name_before].append(i)
                    if index_type == "ctrl" and len(self.slaves_range_in_ctrl[slave_name_before]) == 1:
                        self.slaves_range_in_ctrl[slave_name_before].append(i)
            if i == iter_range - 1 and current_slave_name in current_name_from_mujoco:
                if index_type == "qpos":
                    self.slaves_range_in_qpos[current_slave_name].append(i + 1)
                else:
                    self.slaves_range_in_ctrl[current_slave_name].append(i + 1)

    def get_motion_dof_from_master(self, full_joint_positions):
        """
        Extract motion DOF (exclude gripper) from master joint positions.

        For SO101: 6 DOF total (5 arm joints + 1 gripper)
        Motion DOF: first 5 joints
        Action DOF: last 1 joint (gripper)

        Args:
            full_joint_positions: Full joint positions from master robot

        Returns:
            Motion-only joint positions (excludes gripper)
        """
        return full_joint_positions[:-1]  # Return first 5 joints, exclude gripper

    def init_teleop(self):
        # Get master robot info from PRobotManager
        master_robot_name = self.master_robot_manager.getRobotName()
        master_pin_mjcf = self.master_robot_manager.getPinoMJCF()

        # Initialize master hardware interface
        self.master_interface = create_robot_interface(
            name=master_robot_name,
            mode="teleop",
            config_path="configs/so101.yaml",
            port=self.master_port,
            calibration_dir=self.master_calibration_dir,
        )
        self.master_interface.init()
        self.master_interface.disable_robot_torque()

        if self.is_dual_arm:
            self.master2_interface = create_robot_interface(
                name=master_robot_name,
                mode="teleop",
                config_path="configs/so101.yaml",
                port=self.master_port_2,
                calibration_dir=self.master_calibration_dir_2,
            )
            self.master2_interface.init()
            self.master2_interface.disable_robot_torque()

    def render_cameras(self):
        """Render all camera views in split-screen layout"""
        if not self.glfw_initialized or self.scene is None or self.context is None:
            return

        if glfw.get_current_context() != self.window:
            glfw.make_context_current(self.window)

        with self.simulation_mutex:
            width, height = glfw.get_framebuffer_size(self.window)

            num_cameras = len(self.cameras)
            if num_cameras == 0:
                return

            cols = 2
            rows = 2
            cam_width = width // cols
            cam_height = height // rows

            for i, cam_id in enumerate(self.cameras):
                try:
                    col = i % cols
                    row = i // cols

                    if row >= rows:
                        break

                    viewport_x = col * cam_width
                    viewport_y = height - (row + 1) * cam_height

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

                    viewport = mujoco.MjrRect(viewport_x, viewport_y, cam_width, cam_height)
                    mujoco.mjr_render(viewport, self.scene, self.context)

                except Exception as e:
                    print(f"Error rendering camera {i} ({self.camera_names[i] if i < len(self.camera_names) else 'Unknown'}): {e}")
                    continue

    def update_display(self):
        """Update GLFW window display"""
        if not self.window or glfw.window_should_close(self.window):
            return

        if not self.glfw_initialized or self.context is None:
            return

        glfw.make_context_current(self.window)

        width, height = glfw.get_framebuffer_size(self.window)

        viewport = mujoco.MjrRect(0, 0, width, height)

        mujoco.mjr_rectangle(viewport, 0.1, 0.1, 0.1, 1.0)

        self.render_cameras()

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

    def simulation_thread_func(self):
        """Simulation thread function running at 100Hz"""
        print(f"Starting simulation thread at {self.mjstep_freq}Hz, control at {self.control_freq}Hz")
        last_control_time = time.time()

        while not self.exit_request.is_set():
            loop_start_time = time.time()

            if self.pause_sim:
                time.sleep(0.001)
                continue

            if self.fsm.current_state == self.fsm.initr:
                self.calc_standby_pos()
                self.fsm.to_go_standby1()
            elif self.fsm.current_state == self.fsm.go_standby1:
                if self.lerp_to_target():
                    self.fsm.to_do_action1()
                    self.lerp_start_time = None
                    print("Auto-starting teleoperation...")
                self.simulation_step()
            elif self.fsm.current_state == self.fsm.do_action1:
                if time.time() - last_control_time > self.control_dt:
                    last_control_time = time.time()
                    self.get_feedback()
                    self.set_control()
                self.simulation_step()
            elif self.fsm.current_state == self.fsm.reset:
                self.calc_standby_pos()
                self.fsm.finish_reset()

            elapsed = time.time() - loop_start_time
            sleep_time = self.mjstep_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_feedback(self):
        """Get feedback from the simulation"""
        with self.simulation_mutex:
            master_full_positions = self.master_interface.get_joint_positions()
            self.master_joint_positions = self.get_motion_dof_from_master(master_full_positions)
            self.master_gripper_position = master_full_positions[-1]

            if self.is_dual_arm:
                master2_full_positions = self.master2_interface.get_joint_positions()
                self.master2_joint_positions = self.get_motion_dof_from_master(master2_full_positions)
                self.master2_gripper_position = master2_full_positions[-1]

            for key in self.slaves_range_in_qpos.keys():
                self.slaves_joint_positions[key] = self.data.qpos[self.slaves_range_in_qpos[key][0] : self.slaves_range_in_qpos[key][1]].copy()
                if self.is_dual_arm:
                    self.slaves2_joint_positions[key] = self.data.qpos[self.slaves2_range_in_qpos[key][0] : self.slaves2_range_in_qpos[key][1]].copy()

    def _get_joint_mapping(self, slave_robot_name, master_motion_joints, arm_side="left"):
        """
        Get appropriate joint mapping based on slave robot name.

        Args:
            slave_robot_name: Name of slave robot (fr3, ur5e, piper, rm75)
            master_motion_joints: Master robot joint positions
            arm_side: "left" or "right"

        Returns:
            Mapped slave joint commands
        """
        return self._joint_mapping(master_motion_joints, slave_robot_name, arm_side)

    def _joint_mapping(self, master_qCmd, slave_robot_name, arm_side="left"):
        """
        Universal joint mapping using matrix-based formula.

        Args:
            master_qCmd: Master robot joint positions
            slave_robot_name: Slave robot name (fr3, ur5e, piper, rm75)
            arm_side: "left" or "right"

        Returns:
            Slave robot joint commands
        """
        # Get robot config and standby positions
        robot_config = ROBOT_CONFIGS.get(slave_robot_name)
        if not robot_config:
            return np.zeros(len(master_qCmd))

        master_qStand = self.master_robot_manager.get_qStandby(arm_side)
        slave_mgr = self.slave_robot_managers.get(robot_config["number"])
        if not slave_mgr:
            return np.zeros(len(master_qCmd))

        slave_qStand = slave_mgr.get_qStandby(arm_side)

        # Get qDirection based on arm side
        qDirection_key = f"qDirection_{arm_side}"
        qDirection = robot_config[qDirection_key]

        # Use clean matrix-based formula
        return compute_slave_command(master_qCmd, master_qStand, slave_qStand, qDirection)

    def set_control(self):
        """Set control commands using matrix-based joint mapping"""
        with self.simulation_mutex:
            self.whole_joint_command = np.zeros(self.model.nu)

            m_dof = len(self.master_robot_manager.get_qHome())
            master_motion_joints = self.master_joint_positions[:m_dof]

            if self.is_dual_arm:
                m2_dof = len(self.master_robot_manager.get_qHome())
                master2_motion_joints = self.master2_joint_positions[:m2_dof]

            for key in self.slaves_range_in_qpos.keys():
                # Compute motion command for left arm
                slave_motion_command = self._get_joint_mapping(key, master2_motion_joints if self.is_dual_arm else master_motion_joints, arm_side="left")

                # Compute gripper command
                slave_gripper_ctrlrange_min = self.model.actuator_ctrlrange[self.slaves_range_in_ctrl[key][1] - 1][0]
                slave_gripper_ctrlrange_max = self.model.actuator_ctrlrange[self.slaves_range_in_ctrl[key][1] - 1][1]
                slave_gripper_cmd = slave_gripper_ctrlrange_min + slave_gripper_ctrlrange_max * (self.master2_gripper_position if self.is_dual_arm else self.master_gripper_position) / 100.0 * 2.0

                slave_joint_command = np.append(slave_motion_command, slave_gripper_cmd)
                self.whole_joint_command[self.slaves_range_in_ctrl[key][0] : self.slaves_range_in_ctrl[key][1]] = slave_joint_command

                if self.is_dual_arm:
                    # Compute motion command for right arm
                    slave2_motion_command = self._get_joint_mapping(key, master_motion_joints, arm_side="right")

                    slave2_gripper_ctrlrange_min = self.model.actuator_ctrlrange[self.slaves2_range_in_ctrl[key][1] - 1][0]
                    slave2_gripper_ctrlrange_max = self.model.actuator_ctrlrange[self.slaves2_range_in_ctrl[key][1] - 1][1]
                    slave2_gripper_cmd = slave2_gripper_ctrlrange_min + slave2_gripper_ctrlrange_max * self.master_gripper_position / 100.0 * 2.0

                    slave2_joint_command = np.append(slave2_motion_command, slave2_gripper_cmd)
                    self.whole_joint_command[self.slaves2_range_in_ctrl[key][0] : self.slaves2_range_in_ctrl[key][1]] = slave2_joint_command

            self.data.ctrl = self.whole_joint_command.copy()

    def simulation_step(self):
        """Perform one step of the simulation"""
        with self.simulation_mutex:
            mujoco.mj_step(self.model, self.data)

    def run(self):
        """Main run function"""
        try:
            print("Starting unified MuJoCo GLFW application...")
            print("Controls:")
            print("  R - Reset simulation")
            print("  ENTER - Start teleoperation")
            print("  ESC - Exit")

            physics_thread = threading.Thread(target=self.simulation_thread_func, daemon=True)
            physics_thread.start()

            print("Simulation thread started")
            print("Starting main display loop...")

            last_time = time.time()

            while not self.exit_request.is_set() and not glfw.window_should_close(self.window):
                glfw.poll_events()

                current_time = time.time()
                if current_time - last_time >= self.render_dt:
                    self.update_display()
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


def main():
    parser = argparse.ArgumentParser(description="Multi-Robot Teleoperation using PRobotManager")
    parser.add_argument("--dual", "-d", action="store_true", help="Enable dual arm mode")
    parser.add_argument(
        "--master_number",
        "-mn",
        type=int,
        default=24,
        help="Master robot number (default: 24)",
    )
    parser.add_argument(
        "--slave_numbers",
        "-sn",
        type=str,
        default="20,21,22,23",
        help="Slave robot numbers separated by commas (default: 20,21,22,23)",
    )
    args = parser.parse_args()

    try:
        slave_numbers = [int(num.strip()) for num in args.slave_numbers.split(",") if num.strip().isdigit()]
        if not slave_numbers:
            raise ValueError("No valid slave numbers provided.")
    except Exception as e:
        print(f"Error parsing slave numbers: {e}")
        return

    print("=" * 60)
    print("Multi-Robot Teleoperation")
    print("=" * 60)
    print(f"Master robot number: {args.master_number}")
    print(f"Slave robot numbers: {slave_numbers}")
    print(f"Dual-arm mode: {'Enabled' if args.dual else 'Disabled'}")
    print("=" * 60)

    app = UnifiedMujocoGLFW(args.dual, args.master_number, slave_numbers)
    app.run()


if __name__ == "__main__":
    main()
