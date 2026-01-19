#!/usr/bin/env python3
"""Generic robot arm IK example with mink.

This example works with any robot arm model from robot_descriptions.
It automatically finds the end-effector and runs a simple IK demo.

Usage:
    python examples/mink/robots/generic_arm.py [robot_name]

Available robots (from robot_descriptions):
    panda_mj_description, ur5e_mj_description, iiwa_mj_description,
    ur10e_mj_description, gen3_mj_description, etc.

Examples:
    python examples/mink/robots/generic_arm.py panda_mj_description
    python examples/mink/robots/generic_arm.py ur5e_mj_description

Requirements:
    pip install robot_descriptions loop-rate-limiters
"""

import argparse

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

import mink

try:
    from robot_descriptions.loaders.mujoco import load_robot_description
except ImportError:
    print("Please install robot_descriptions: pip install robot_descriptions")
    raise


# IK parameters
SOLVER = "daqp"
MAX_ITERS = 10


def find_end_effector(model):
    """Automatically find the end-effector frame."""
    # Priority 1: Look for common site names
    site_keywords = ["attach", "ee", "tool", "effector", "tcp", "flange", "end"]
    for i in range(model.nsite):
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        if site_name and any(kw in site_name.lower() for kw in site_keywords):
            return site_name, "site"

    # Priority 2: Look for common body names
    body_keywords = ["hand", "gripper", "tool", "ee", "link_7", "link7", "wrist"]
    for i in range(model.nbody - 1, 0, -1):  # Search from end
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if body_name and any(kw in body_name.lower() for kw in body_keywords):
            return body_name, "body"

    # Fallback: Use the last non-world body
    if model.nbody > 1:
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody - 1)
        return body_name, "body"

    raise RuntimeError("Could not find end-effector frame")


def print_model_info(model):
    """Print useful model information."""
    print(f"\nModel info:")
    print(f"  Bodies: {model.nbody}")
    print(f"  Joints: {model.njnt}")
    print(f"  DOF: {model.nv}")
    print(f"  Actuators: {model.nu}")
    print(f"  Sites: {model.nsite}")

    print(f"\n  Joint names:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            print(f"    {i}: {name}")

    print(f"\n  Site names:")
    for i in range(model.nsite):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        if name:
            print(f"    {i}: {name}")


def main():
    parser = argparse.ArgumentParser(description="Generic robot arm IK example")
    parser.add_argument(
        "robot",
        nargs="?",
        default="panda_mj_description",
        help="Robot description name (default: panda_mj_description)"
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Print model information and exit"
    )
    args = parser.parse_args()

    # Load model
    print(f"Loading robot: {args.robot}")
    try:
        model = load_robot_description(args.robot)
    except Exception as e:
        print(f"Failed to load {args.robot}: {e}")
        print("\nAvailable MuJoCo robots in robot_descriptions:")
        print("  panda_mj_description, ur5e_mj_description, ur10e_mj_description")
        print("  iiwa_mj_description, gen3_mj_description, etc.")
        return

    data = mujoco.MjData(model)

    if args.info:
        print_model_info(model)
        return

    # Find end-effector
    eef_name, eef_type = find_end_effector(model)
    print(f"Using end-effector: {eef_name} ({eef_type})")

    # Create mink configuration and tasks
    configuration = mink.Configuration(model)

    end_effector_task = mink.FrameTask(
        frame_name=eef_name,
        frame_type=eef_type,
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )
    posture_task = mink.PostureTask(model=model, cost=1e-2)
    tasks = [end_effector_task, posture_task]

    # Limits
    limits = [mink.ConfigurationLimit(model=model)]

    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)

        # Get initial end-effector position
        initial_target = configuration.get_transform_frame_to_world(
            eef_name, eef_type
        ).translation()

        # Trajectory parameters (adaptive based on workspace)
        # Use smaller amplitude if near origin
        dist = np.linalg.norm(initial_target)
        amp = min(0.15, dist * 0.2) if dist > 0.1 else 0.08
        freq = 0.2

        rate = RateLimiter(frequency=200.0, warn=False)

        print(f"\nRunning IK demo with trajectory amplitude: {amp:.3f}m")
        print("Press Ctrl+C to exit.")

        local_time = 0.0
        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Circular trajectory
            offset = np.array([
                amp * np.cos(2 * np.pi * freq * local_time),
                amp * np.sin(2 * np.pi * freq * local_time),
                amp * 0.3 * np.sin(4 * np.pi * freq * local_time),
            ])

            target_pos = initial_target + offset
            T_target = mink.SE3.from_rotation_and_translation(
                mink.SO3.identity(),
                target_pos,
            )
            end_effector_task.set_target(T_target)

            # Solve IK
            for _ in range(MAX_ITERS):
                vel = mink.solve_ik(
                    configuration, tasks, dt, SOLVER,
                    damping=1e-3, limits=limits
                )
                configuration.integrate_inplace(vel, dt)

            # Apply to simulation
            nctrl = min(len(configuration.q), model.nu)
            data.ctrl[:nctrl] = configuration.q[:nctrl]
            mujoco.mj_step(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
