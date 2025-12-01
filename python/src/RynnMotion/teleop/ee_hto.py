#!/usr/bin/env python3
"""
Heterogeneous Teleoperation (End-Effector based) - single process, no LCM
Master: MuJoCo passive viewer (drag control)
Slave: GLFW rendering with OSC + Differential IK control

This implementation uses end-effector based teleoperation where master EE pose
is mapped to slave workspace using PoseMapper, then OSC + Differential IK
computes slave joint commands.
For joint-level teleoperation, see joint_hto.py
"""

import time
import threading
import argparse
import mujoco
import mujoco.viewer
import numpy as np
import glfw
from threading import Lock, Event

from RynnMotion.manager.robot_manager import RobotManager
import logging
from RynnMotion.algorithms import PoseMapper, PinKine
from RynnMotion.algorithms.osc import OSCtrl
from RynnMotion.utils.pyFSM import PyFSM
from RynnMotion.utils.policy_interpolator import lerp
from RynnMotion.utils.path_config import get_models_root


class EEHTOApp:
    def __init__(
        self,
        master_robot_number: int,
        slave_robot_number: int,
        verbose: bool = False,
    ):
        """
        Initialize heterogeneous teleoperation application (joint mapping)

        Args:
            master_robot_number: Master robot number
            slave_robot_number: Slave robot number
            verbose: Enable verbose output (default: False)
        """
        self.master_robot_number = master_robot_number
        self.slave_robot_number = slave_robot_number
        self.verbose = verbose

        # Robot managers - create config dicts inline
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

        master_name, master_robot_rel, master_pino_rel = robot_map[master_robot_number]
        slave_name, slave_robot_rel, slave_pino_rel = robot_map[slave_robot_number]

        # Resolve to absolute paths
        master_robot_mjcf = str(models_root / master_robot_rel)
        master_pino_mjcf = str(models_root / master_pino_rel)
        slave_robot_mjcf = str(models_root / slave_robot_rel)
        slave_pino_mjcf = str(models_root / slave_pino_rel)

        master_config = {
            "robot_name": master_name,
            "robot_control_freq": 100,
            "robot_mjcf": master_robot_mjcf,
            "pino_mjcf": master_pino_mjcf,
        }
        slave_config = {
            "robot_name": slave_name,
            "robot_control_freq": 100,
            "robot_mjcf": slave_robot_mjcf,
            "pino_mjcf": slave_pino_mjcf,
        }

        self.master_robot_manager = RobotManager(master_config, logger)
        self.slave_robot_manager = RobotManager(slave_config, logger)

        # Threading
        self.simulation_mutex = Lock()
        self.exit_request = Event()

        # Timing
        self.ctrl_freq = 100.0  # Control Hz
        self.mjstep_freq = 500.0  # Physics Hz
        self.render_freq = 60.0  # Render Hz

        self.ctrl_dt = 1.0 / self.ctrl_freq
        self.mjstep_dt = 1.0 / self.mjstep_freq
        self.render_dt = 1.0 / self.render_freq

        # Robot data
        self.slave_robot_name = self.slave_robot_manager.getRobotName()
        self.q_home = self.slave_robot_manager.get_qHome()
        self.q_stand = self.slave_robot_manager.get_qStand()
        self.site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]

        # FSM
        self.pyfsm = PyFSM(verbose=self.verbose)
        self.lerp_duration = 1.0

        # Components (initialized later) - PinKine, PoseMapper, OSC for EE-based teleop
        self.master_pinkine = None
        self.slave_pinkine = None
        self.pose_mapper = None
        self.osc_ctrl = None

        # GLFW window
        self.window = None
        self.glfw_initialized = False

        # MuJoCo rendering components
        self.scene = None
        self.context = None
        self.camera = None
        self.option = None

        # Master viewer
        self.master_viewer = None
        self.master_model = None
        self.master_data = None

        # Slave model/data
        self.model = None
        self.data = None
        self.nq = 0
        self.nctrl = 0

        # Initialize components
        self.init_glfw()
        self.init_scene()
        self.init_master_viewer()
        self.init_mujoco_rendering()
        self.init_teleop()

        if self.verbose:
            print("=" * 60)
            print("HETEROGENEOUS TELEOPERATION (End-Effector based)")
            print("=" * 60)
            print(
                f"Master robot: {self.master_robot_manager.getRobotName()} (#{master_robot_number})"
            )
            print(
                f"Slave robot:  {self.slave_robot_manager.getRobotName()} (#{slave_robot_number})"
            )
            print("Master: MuJoCo viewer (drag control)")
            print("Slave:  GLFW rendering (OSC + Differential IK control)")
            print("=" * 60)

            # Print keyframes for debugging
            master_qHome = self.master_robot_manager.get_qHome()
            master_qStand = self.master_robot_manager.get_qStand()
            print(f"\nMaster keyframes:")
            print(f"  qHome:  {master_qHome}")
            print(f"  qStand: {master_qStand}")

            print(f"\nSlave keyframes:")
            print(f"  qHome:  {self.q_home}")
            print(f"  qStand: {self.q_stand}")
            print()

    def init_glfw(self):
        """Initialize GLFW window and keyboard callbacks"""
        if not glfw.init():
            raise RuntimeError("Failed to initialize GLFW")

        self.glfw_initialized = True

        # Create window
        self.window = glfw.create_window(
            1200, 900, "Heterogeneous Teleop (EE-based) - Slave Robot", None, None
        )
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Failed to create GLFW window")

        glfw.make_context_current(self.window)
        glfw.swap_interval(1)  # Enable vsync

        # Set keyboard callback
        def key_callback(window, key, scancode, action, mods):
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

        glfw.set_key_callback(self.window, key_callback)

        if self.verbose:
            print("✅ GLFW window initialized")

    def init_scene(self):
        """Load master and slave MuJoCo models"""
        # Load slave model
        slave_scene_path = self.slave_robot_manager.getRobotMJCF()
        if self.verbose:
            print(f"Loading slave model: {slave_scene_path}")

        self.model = mujoco.MjModel.from_xml_path(slave_scene_path)
        self.data = mujoco.MjData(self.model)
        self.nctrl = self.model.nu

        # Initialize slave to qHome (only motion joints, not gripper)
        n_motion_joints = len(self.q_home)
        self.data.qpos[:n_motion_joints] = self.q_home
        mujoco.mj_forward(self.model, self.data)

        if self.verbose:
            print(f"✅ Slave model loaded: {self.model.nq} DOF (MuJoCo), {n_motion_joints} motion joints, {self.nctrl} actuators")

    def init_master_viewer(self):
        """Launch master MuJoCo viewer (passive mode for drag control)"""
        master_scene_path = self.master_robot_manager.getRobotMJCF()
        if self.verbose:
            print(f"Loading master model: {master_scene_path}")

        self.master_model = mujoco.MjModel.from_xml_path(master_scene_path)
        self.master_data = mujoco.MjData(self.master_model)

        # Initialize master to qHome (only motion joints, not gripper)
        master_qHome = self.master_robot_manager.get_qHome()
        n_master_motion_joints = len(master_qHome)
        self.master_data.qpos[:n_master_motion_joints] = master_qHome
        mujoco.mj_forward(self.master_model, self.master_data)

        # Launch passive viewer (allows user to drag joints)
        self.master_viewer = mujoco.viewer.launch_passive(
            self.master_model, self.master_data
        )

        # Compute master gripper range
        self.master_gripper_range = (
            self.master_model.actuator_ctrlrange[-1][1]
            - self.master_model.actuator_ctrlrange[-1][0]
        )

        if self.verbose:
            print(f"✅ Master model loaded: {self.master_model.nq} DOF (MuJoCo), {n_master_motion_joints} motion joints, {self.master_model.nu} actuators")
            print(f"🎮 Master viewer ENABLED (user can drag joints)")
            print(f"Master gripper range: {self.master_gripper_range}")

    def init_mujoco_rendering(self):
        """Initialize MuJoCo rendering context for slave GLFW window"""
        # Create rendering context
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_150
        )

        # Camera setup
        self.camera = mujoco.MjvCamera()
        self.camera.azimuth = 90
        self.camera.elevation = -20
        self.camera.distance = 2.0
        self.camera.lookat[:] = [0, 0, 0.5]

        # Visualization options
        self.option = mujoco.MjvOption()

        if self.verbose:
            print("✅ MuJoCo rendering context initialized")

    def init_teleop(self):
        """Initialize PinKine, PoseMapper, OSC controller for EE-based teleop"""
        master_pin_mjcf = self.master_robot_manager.getPinoMJCF()
        slave_pin_mjcf = self.slave_robot_manager.getPinoMJCF()

        self.master_pinkine = PinKine(master_pin_mjcf, self.site_names)
        self.slave_pinkine = PinKine(slave_pin_mjcf, self.site_names)

        # Compute arm lengths
        master_qStand = self.master_robot_manager.get_qStand()
        self.master_armlength_config = self.load_arm_length(
            self.master_pinkine, master_qStand
        )

        if self.verbose:
            print(f"\nMaster robot arm lengths (at qStand):")
            for key in ["l0", "l1", "l2", "l3"]:
                vec = self.master_armlength_config[key]
                length = np.linalg.norm(vec)
                print(
                    f"  {key}: [{vec[0]:.4f}, {vec[1]:.4f}, {vec[2]:.4f}] (|{key}|={length:.4f}m)"
                )

        slave_qStand = self.slave_robot_manager.get_qStand()
        self.slave_armlength_config = self.load_arm_length(
            self.slave_pinkine, slave_qStand
        )

        if self.verbose:
            print(f"\nSlave robot arm lengths (at qStand):")
            for key in ["l0", "l1", "l2", "l3"]:
                vec = self.slave_armlength_config[key]
                length = np.linalg.norm(vec)
                print(
                    f"  {key}: [{vec[0]:.4f}, {vec[1]:.4f}, {vec[2]:.4f}] (|{key}|={length:.4f}m)"
                )

        # Initialize PoseMapper and OSC
        self.pose_mapper = PoseMapper(
            self.master_armlength_config, self.slave_armlength_config
        )

        self.osc_ctrl = OSCtrl(dt=self.ctrl_dt)

        # Debug: Print EE pose at qStand
        self.master_pinkine.update(master_qStand)
        master_ee_pose = self.master_pinkine.getSiteSE3ByName("EE")
        master_ee_pos = master_ee_pose.translation
        master_ee_quat = self.master_pinkine.getSiteQuatByName("EE")

        self.slave_pinkine.update(slave_qStand)
        slave_ee_pose = self.slave_pinkine.getSiteSE3ByName("EE")
        slave_ee_pos = slave_ee_pose.translation
        slave_ee_quat = self.slave_pinkine.getSiteQuatByName("EE")

        if self.verbose:
            print(f"\n=== EE Pose at qStand ===")
            print(
                f"Master EE: pos=[{master_ee_pos[0]:.4f}, {master_ee_pos[1]:.4f}, {master_ee_pos[2]:.4f}], quat(xyzw)=[{master_ee_quat[0]:.4f}, {master_ee_quat[1]:.4f}, {master_ee_quat[2]:.4f}, {master_ee_quat[3]:.4f}]"
            )
            print(
                f"Slave EE:  pos=[{slave_ee_pos[0]:.4f}, {slave_ee_pos[1]:.4f}, {slave_ee_pos[2]:.4f}], quat(xyzw)=[{slave_ee_quat[0]:.4f}, {slave_ee_quat[1]:.4f}, {slave_ee_quat[2]:.4f}, {slave_ee_quat[3]:.4f}]"
            )

        if self.verbose:
            print("✅ Teleoperation components initialized")

    def load_arm_length(self, pinkine: PinKine, q_config: np.ndarray) -> dict:
        """
        Compute arm segment lengths from site positions

        Computes vectors between consecutive arm sites:
        - l0: base (origin) to shoulderSite
        - l1: shoulderSite to elbowSite
        - l2: elbowSite to wristSite
        - l3: wristSite to EE

        Args:
            pinkine: PinKine instance
            q_config: Joint configuration (e.g., qStand)

        Returns:
            Dictionary with l0, l1, l2, l3 as 3x1 numpy arrays
        """
        pinkine.update(q_config)

        base_pos = np.zeros(3)
        shoulder_pos = pinkine.getSitePosByName("shoulderSite")
        elbow_pos = pinkine.getSitePosByName("elbowSite")
        wrist_pos = pinkine.getSitePosByName("wristSite")
        ee_pos = pinkine.getSitePosByName("EE")

        l0 = shoulder_pos - base_pos
        l1 = elbow_pos - shoulder_pos
        l2 = wrist_pos - elbow_pos
        l3 = ee_pos - wrist_pos

        return {
            "l0": l0,
            "l1": l1,
            "l2": l2,
            "l3": l3,
        }

    def goStandby1(self):
        """Handle GoStandby1 state: LERP from qHome to qStand for both master and slave"""
        elapsed = self.pyfsm.get_elapsed_time()

        # LERP for slave robot
        slave_q_lerp = lerp(self.q_home, self.q_stand, elapsed, self.lerp_duration)

        # LERP for master robot
        master_qHome = self.master_robot_manager.get_qHome()
        master_qStand = self.master_robot_manager.get_qStand()
        master_q_lerp = lerp(master_qHome, master_qStand, elapsed, self.lerp_duration)

        with self.simulation_mutex:
            # Apply to slave (motion joints only, not gripper)
            self.data.ctrl[:len(slave_q_lerp)] = slave_q_lerp

            # Apply to master (motion joints only, not gripper)
            self.master_data.ctrl[:len(master_q_lerp)] = master_q_lerp

        if self.pyfsm.should_auto_transition(self.lerp_duration):
            if self.verbose:
                print("✅ LERP complete, entering DoAction1")
            self.pyfsm.to_do_action1()

    def run_slave_teleop(self):
        """Run EE-based teleoperation: FK on master, pose mapping, OSC + Diff IK"""
        # Read master joint positions (direct access, no LCM)
        with self.simulation_mutex:
            master_joint_positions = self.master_data.qpos.copy()
            master_gripper_cmd = self.master_data.ctrl[-1]
            slave_joint_positions = self.data.qpos.copy()

        # Step 1: Get master EE pose (SE3)
        m_dof = self.master_pinkine.pin_model.nq
        self.master_pinkine.update(master_joint_positions[:m_dof])
        m_eePose = self.master_pinkine.getSiteSE3ByName("EE")

        # Step 2: Get slave EE pose feedback (SE3)
        s_dof = self.slave_pinkine.pin_model.nq
        self.slave_pinkine.update(slave_joint_positions[:s_dof])
        s_eePoseFb = self.slave_pinkine.getSiteSE3ByName("EE")

        # Step 3: Pose mapping (master → slave workspace)
        s_eePoseRef = self.pose_mapper.pose_mapping(
            m_eePose, self.master_pinkine, self.slave_pinkine
        )

        # Step 4: OSC controller (get desired EE velocity)
        self.osc_ctrl.update(s_eePoseFb, s_eePoseRef)
        eeVel = self.osc_ctrl.get_eeVel()

        # Step 5: Differential IK (eeVel → joint velocities)
        s_jaco = self.slave_pinkine.getSiteJacoByName("EE")
        inverse_jaco = (
            np.linalg.pinv(s_jaco.T @ s_jaco + 0.01 * np.eye(s_jaco.shape[1]))
            @ s_jaco.T
        )
        qdot = inverse_jaco @ eeVel

        # Step 6: Integrate to get joint commands
        slave_joint_command = np.zeros(self.nctrl)
        slave_joint_command[:s_dof] = (
            slave_joint_positions[:s_dof] + qdot * self.ctrl_dt
        )

        # Step 7: Map gripper command
        slave_gripper_idx = self.nctrl - 1
        slave_gripper_ctrlrange_min = self.model.actuator_ctrlrange[slave_gripper_idx][
            0
        ]
        slave_gripper_ctrlrange_max = self.model.actuator_ctrlrange[slave_gripper_idx][
            1
        ]
        slave_gripper_cmd = (
            slave_gripper_ctrlrange_min
            + slave_gripper_ctrlrange_max
            * master_gripper_cmd
            / self.master_gripper_range
        )
        if self.slave_robot_name in ["lerobot", "so101"]:
            slave_gripper_cmd *= 180 / 3.14
        slave_joint_command[slave_gripper_idx] = slave_gripper_cmd

        # Step 8: Apply commands (thread-safe)
        with self.simulation_mutex:
            self.data.ctrl[:] = slave_joint_command

    def simulation_thread_func(self):
        """Background simulation thread (control + physics)"""
        last_control_time = time.time()

        while not self.exit_request.is_set():
            loop_start = time.time()

            # FSM state handling
            if self.pyfsm.is_Initr():
                self.pyfsm.to_go_standby1()

            elif self.pyfsm.is_GoStandby1():
                self.goStandby1()

            elif self.pyfsm.is_DoAction1():
                # Control at 100Hz
                if time.time() - last_control_time >= self.ctrl_dt:
                    last_control_time = time.time()

                    # Run EE-based teleoperation
                    self.run_slave_teleop()

            elif self.pyfsm.is_Reset():
                self.goStandby1()
                if self.pyfsm.should_auto_transition(self.lerp_duration):
                    if self.verbose:
                        print("✅ Reset complete")
                    self.pyfsm.finish_reset()

            # Physics step for both master and slave
            with self.simulation_mutex:
                mujoco.mj_step(self.master_model, self.master_data)
                mujoco.mj_step(self.model, self.data)

            # Sync master viewer
            self.master_viewer.sync()

            # Maintain 500Hz physics frequency
            elapsed = time.time() - loop_start
            sleep_time = self.mjstep_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if self.verbose:
            print("🛑 Simulation thread exiting")

    def render_slave(self):
        """Render slave robot to GLFW window"""
        width, height = glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)

        with self.simulation_mutex:
            # Update scene
            mujoco.mjv_updateScene(
                self.model,
                self.data,
                self.option,
                None,
                self.camera,
                mujoco.mjtCatBit.mjCAT_ALL,
                self.scene,
            )

        # Clear background
        mujoco.mjr_rectangle(viewport, 0.1, 0.1, 0.1, 1.0)

        # Render scene
        mujoco.mjr_render(viewport, self.scene, self.context)

    def update_display(self):
        """Main rendering loop (60Hz)"""
        glfw.make_context_current(self.window)

        # Render slave
        self.render_slave()

        # Swap buffers
        glfw.swap_buffers(self.window)
        glfw.poll_events()

    def run(self):
        """Main entry point"""
        try:
            # Start simulation thread
            sim_thread = threading.Thread(
                target=self.simulation_thread_func, daemon=True
            )
            sim_thread.start()

            if self.verbose:
                print("\n🚀 Starting unified teleoperation")
                print("Controls:")
                print("  - ENTER: Start teleoperation (from GoStandby1)")
                print("  - R:     Reset to standby")
                print("  - ESC:   Exit\n")

            # Main thread: rendering loop at 60Hz
            last_render_time = time.time()
            while not self.exit_request.is_set() and not glfw.window_should_close(
                self.window
            ):
                current_time = time.time()

                if current_time - last_render_time >= self.render_dt:
                    self.update_display()
                    last_render_time = current_time
                else:
                    # Small sleep to avoid busy waiting
                    time.sleep(0.001)

            # Signal exit and wait for simulation thread
            self.exit_request.set()
            sim_thread.join(timeout=2.0)

            if self.verbose:
                print("✅ Main loop exited cleanly")

        except KeyboardInterrupt:
            print("\n🛑 Keyboard interrupt")
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback

            traceback.print_exc()
        finally:
            # Cleanup
            if self.verbose:
                print("🧹 Cleaning up resources...")

            if self.master_viewer:
                self.master_viewer.close()

            if self.window:
                glfw.destroy_window(self.window)

            if self.glfw_initialized:
                glfw.terminate()

            if self.verbose:
                print("🧹 Cleanup complete")


def main():
    parser = argparse.ArgumentParser(
        description="Heterogeneous Teleoperation (End-Effector based)",
        epilog="Uses PoseMapper + OSC + Differential IK for EE-based control. Works with any robot combination."
    )
    parser.add_argument(
        "--master_number", "-mn", type=int, required=True,
        help="Master robot ID (e.g., 24 for SO101)"
    )
    parser.add_argument(
        "--slave_number", "-sn", type=int, required=True,
        help="Slave robot ID (e.g., 20=FR3, 21=UR5e, 22=Piper, 23=RM75)"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable verbose output (shows arm lengths, EE poses, etc.)"
    )

    args = parser.parse_args()

    app = EEHTOApp(
        master_robot_number=args.master_number,
        slave_robot_number=args.slave_number,
        verbose=args.verbose,
    )

    app.run()


if __name__ == "__main__":
    main()
