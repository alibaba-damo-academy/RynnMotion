#!/usr/bin/env python3
"""KUKA iiwa arm IK example with mink.

This example demonstrates mink inverse kinematics with the KUKA iiwa14
7-DOF robot arm.

Usage:
    python examples/mink/robots/arm_iiwa.py

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


def find_end_effector(model):
    """Find the end-effector frame in the model."""
    for i in range(model.nsite):
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        if site_name and any(x in site_name.lower() for x in ["attach", "ee", "tool", "effector"]):
            return site_name, "site"
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody - 1)
    return body_name, "body"


def main():
    # Load iiwa model from robot_descriptions
    model = load_robot_description("iiwa_mj_description")
    data = mujoco.MjData(model)

    configuration = mink.Configuration(model)

    # Find end-effector
    eef_name, eef_type = find_end_effector(model)
    print(f"Using end-effector: {eef_name} ({eef_type})")

    # Create tasks
    end_effector_task = mink.FrameTask(
        frame_name=eef_name,
        frame_type=eef_type,
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )
    posture_task = mink.PostureTask(model=model, cost=1e-2)
    tasks = [end_effector_task, posture_task]

    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)

        # Get initial position
        initial_target = configuration.get_transform_frame_to_world(
            eef_name, eef_type
        ).translation()

        rate = RateLimiter(frequency=500.0, warn=False)

        print("Running KUKA iiwa IK example...")
        print("Press Ctrl+C to exit.")

        local_time = 0.0
        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Circular trajectory
            amp = 0.12
            freq = 0.2
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

            # IK iterations
            for _ in range(MAX_ITERS):
                vel = mink.solve_ik(configuration, tasks, dt, SOLVER, damping=1e-3)
                configuration.integrate_inplace(vel, dt)

                err = end_effector_task.compute_error(configuration)
                if (np.linalg.norm(err[:3]) <= POS_THRESHOLD and
                    np.linalg.norm(err[3:]) <= ORI_THRESHOLD):
                    break

            # Apply to simulation
            nctrl = min(len(configuration.q), model.nu)
            data.ctrl[:nctrl] = configuration.q[:nctrl]
            mujoco.mj_step(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
