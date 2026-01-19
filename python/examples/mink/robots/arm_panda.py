#!/usr/bin/env python3
"""Franka Emika Panda arm IK example with mink.

This example demonstrates mink inverse kinematics with the Panda robot arm,
using robot_descriptions to load the model.

Usage:
    python examples/mink/robots/arm_panda.py

Requirements:
    pip install robot_descriptions loop-rate-limiters
"""

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
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20


def converge_ik(configuration, tasks, dt, solver, pos_threshold, ori_threshold, max_iters):
    """Run IK iterations until convergence."""
    for _ in range(max_iters):
        vel = mink.solve_ik(configuration, tasks.values(), dt, solver, damping=1e-3)
        configuration.integrate_inplace(vel, dt)

        err = tasks["eef"].compute_error(configuration)
        pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
        ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold

        if pos_achieved and ori_achieved:
            return True
    return False


def main():
    # Load Panda model from robot_descriptions
    model = load_robot_description("panda_mj_description")
    data = mujoco.MjData(model)

    configuration = mink.Configuration(model)

    # Find end-effector site name (may vary by model)
    # Common names: "attachment_site", "end_effector", "ee_site", "hand"
    eef_site = None
    for i in range(model.nsite):
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        if site_name and any(x in site_name.lower() for x in ["attach", "ee", "hand", "effector"]):
            eef_site = site_name
            break

    if eef_site is None:
        # Use last link body as fallback
        eef_site = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody - 1)
        frame_type = "body"
        print(f"No site found, using body: {eef_site}")
    else:
        frame_type = "site"
        print(f"Using end-effector site: {eef_site}")

    end_effector_task = mink.FrameTask(
        frame_name=eef_site,
        frame_type=frame_type,
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )
    posture_task = mink.PostureTask(model=model, cost=1e-2)
    tasks = {"eef": end_effector_task, "posture": posture_task}

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
            eef_site, frame_type
        ).translation()

        # Circular trajectory
        amp = 0.10
        freq = 0.2

        local_time = 0.0
        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running Panda arm IK example...")
        print("Press Ctrl+C to exit.")

        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Circular offset
            offset = np.array([
                amp * np.cos(2 * np.pi * freq * local_time),
                amp * np.sin(2 * np.pi * freq * local_time),
                0.0,
            ])

            target_pos = initial_target + offset
            T_target = mink.SE3.from_rotation_and_translation(
                mink.SO3.identity(),
                target_pos,
            )
            end_effector_task.set_target(T_target)

            converge_ik(
                configuration, tasks, dt, SOLVER,
                POS_THRESHOLD, ORI_THRESHOLD, MAX_ITERS,
            )

            # Apply to simulation
            nctrl = min(len(configuration.q), model.nu)
            data.ctrl[:nctrl] = configuration.q[:nctrl]
            mujoco.mj_step(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
