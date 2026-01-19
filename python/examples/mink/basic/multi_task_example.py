#!/usr/bin/env python3
"""Combining multiple mink tasks: FrameTask + PostureTask.

This example demonstrates how to use multiple mink tasks together:
- FrameTask for end-effector tracking (high priority)
- PostureTask for joint regularization (low priority)

Usage:
    python examples/mink/basic/multi_task_example.py
"""

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

import mink

from RynnMotion.utils.path_config import get_models_root

# Robot scene path
ROBOT_SCENE = "3.robot_arm/20.fr3/scene/scene.xml"

# IK parameters
SOLVER = "daqp"
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20


def converge_ik(
    configuration: mink.Configuration,
    tasks: list,
    dt: float,
    solver: str,
    max_iters: int = 10,
) -> bool:
    """Run IK iterations."""
    for _ in range(max_iters):
        vel = mink.solve_ik(configuration, tasks, dt, solver, damping=1e-3)
        configuration.integrate_inplace(vel, dt)
    return True


def main() -> None:
    """Run the multi-task example."""
    # Load robot model
    models_root = get_models_root()
    scene_path = str(models_root / ROBOT_SCENE)

    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    # Create mink Configuration
    configuration = mink.Configuration(model)

    # Find end-effector site (FR3 uses "EE_0")
    eef_site = "EE_0"
    for i in range(model.nsite):
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        if site_name and any(x in site_name.lower() for x in ["ee", "attach", "effector"]):
            eef_site = site_name
            break
    print(f"Using end-effector site: {eef_site}")

    # Create tasks with different priorities via cost weights
    # Higher cost = higher priority

    # Primary task: end-effector tracking (high cost)
    end_effector_task = mink.FrameTask(
        frame_name=eef_site,
        frame_type="site",
        position_cost=1.0,       # High priority for position
        orientation_cost=1.0,    # High priority for orientation
        lm_damping=1.0,
    )

    # Secondary task: posture regularization (low cost)
    # This keeps the robot near a comfortable configuration
    # when the primary task doesn't fully constrain the motion
    posture_task = mink.PostureTask(model=model, cost=1e-2)

    # Damping task for stability (even lower cost)
    damping_task = mink.DampingTask(model=model, cost=1e-4)

    tasks = [end_effector_task, posture_task, damping_task]

    # Initialize viewer
    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize configuration
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)

        # Set posture target to current configuration
        posture_task.set_target_from_configuration(configuration)

        # Get initial end-effector position
        initial_target = configuration.get_transform_frame_to_world(
            eef_site, "site"
        ).translation()

        # Figure-8 trajectory parameters
        amp_x = 0.15
        amp_y = 0.10
        freq = 0.15

        local_time = 0.0
        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running multi-task example...")
        print("Tasks:")
        print("  1. FrameTask (cost=1.0): End-effector follows figure-8 trajectory")
        print("  2. PostureTask (cost=0.01): Regularizes to initial configuration")
        print("  3. DampingTask (cost=0.0001): Velocity damping for stability")
        print("Press Ctrl+C to exit.")

        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Compute figure-8 trajectory
            offset = np.array([
                amp_x * np.sin(2 * np.pi * freq * local_time),
                amp_y * np.sin(4 * np.pi * freq * local_time),  # 2x frequency for figure-8
                0.0,
            ])

            # Create target transform
            target_pos = initial_target + offset
            T_target = mink.SE3.from_rotation_and_translation(
                mink.SO3.identity(),
                target_pos,
            )
            end_effector_task.set_target(T_target)

            # Solve IK with all tasks
            converge_ik(configuration, tasks, dt, SOLVER, MAX_ITERS)

            # Apply configuration to MuJoCo data
            data.qpos[:] = configuration.q
            mujoco.mj_forward(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
