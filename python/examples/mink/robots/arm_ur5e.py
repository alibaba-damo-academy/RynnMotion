#!/usr/bin/env python3
"""Universal Robots UR5e arm IK example with mink.

This example demonstrates mink inverse kinematics with collision avoidance
using the UR5e robot arm.

Usage:
    python examples/mink/robots/arm_ur5e.py

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


def find_end_effector(model):
    """Find the end-effector frame in the model."""
    # Try common site names
    for i in range(model.nsite):
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        if site_name and any(x in site_name.lower() for x in ["attach", "ee", "tool", "effector"]):
            return site_name, "site"

    # Fallback to last body
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody - 1)
    return body_name, "body"


def main():
    # Load UR5e model from robot_descriptions
    model = load_robot_description("ur5e_mj_description")
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
        lm_damping=1e-6,
    )
    posture_task = mink.PostureTask(model, cost=1e-3)
    tasks = [end_effector_task, posture_task]

    # Create limits
    limits = [
        mink.ConfigurationLimit(model=model),
    ]

    # Add velocity limits
    velocity_limits = {}
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name:
            velocity_limits[joint_name] = np.pi  # 180 deg/s
    if velocity_limits:
        limits.append(mink.VelocityLimit(model, velocity_limits))

    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)
        posture_task.set_target(configuration.q)

        # Get initial position
        initial_target = configuration.get_transform_frame_to_world(
            eef_name, eef_type
        ).translation()

        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running UR5e arm IK example with velocity limits...")
        print("Press Ctrl+C to exit.")

        local_time = 0.0
        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Figure-8 trajectory
            offset = np.array([
                0.15 * np.sin(2 * np.pi * 0.15 * local_time),
                0.10 * np.sin(4 * np.pi * 0.15 * local_time),
                0.05 * np.cos(2 * np.pi * 0.15 * local_time),
            ])

            target_pos = initial_target + offset
            T_target = mink.SE3.from_rotation_and_translation(
                mink.SO3.identity(),
                target_pos,
            )
            end_effector_task.set_target(T_target)

            # Solve IK with limits
            vel = mink.solve_ik(configuration, tasks, dt, SOLVER, limits=limits)
            configuration.integrate_inplace(vel, dt)

            # Apply to simulation
            nctrl = min(len(configuration.q), model.nu)
            data.ctrl[:nctrl] = configuration.q[:nctrl]
            mujoco.mj_step(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
