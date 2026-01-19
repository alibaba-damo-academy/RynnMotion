#!/usr/bin/env python3
"""Single end-effector tracking with mink FrameTask.

This example demonstrates basic mink usage for tracking a target pose
with an end-effector using a FrameTask.

Usage:
    python examples/mink/basic/frame_task_example.py
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
    tasks: dict,
    dt: float,
    solver: str,
    pos_threshold: float,
    ori_threshold: float,
    max_iters: int,
) -> bool:
    """Run IK iterations until convergence or max iterations reached.

    Args:
        configuration: mink Configuration object
        tasks: Dictionary of mink tasks
        dt: Time step
        solver: QP solver name
        pos_threshold: Position error threshold
        ori_threshold: Orientation error threshold
        max_iters: Maximum number of iterations

    Returns:
        True if converged, False otherwise
    """
    for _ in range(max_iters):
        vel = mink.solve_ik(configuration, tasks.values(), dt, solver, damping=1e-3)
        configuration.integrate_inplace(vel, dt)

        # Check error on end-effector task
        err = tasks["eef"].compute_error(configuration)
        pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
        ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold

        if pos_achieved and ori_achieved:
            return True
    return False


def main() -> None:
    """Run the frame task example."""
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

    # Create end-effector tracking task
    end_effector_task = mink.FrameTask(
        frame_name=eef_site,
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )

    tasks = {"eef": end_effector_task}

    # Initialize viewer
    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize configuration from current state
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)

        # Get initial end-effector position for circular trajectory
        initial_target = configuration.get_transform_frame_to_world(
            eef_site, "site"
        ).translation()

        # Circular trajectory parameters
        amp = 0.10
        freq = 0.2

        local_time = 0.0
        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running frame task example...")
        print("The end-effector will follow a circular trajectory.")
        print("Press Ctrl+C to exit.")

        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Compute circular offset
            offset = np.array([
                amp * np.cos(2 * np.pi * freq * local_time),
                amp * np.sin(2 * np.pi * freq * local_time),
                0.0,
            ])

            # Create target transform
            target_pos = initial_target + offset
            T_target = mink.SE3.from_rotation_and_translation(
                mink.SO3.identity(),
                target_pos,
            )
            end_effector_task.set_target(T_target)

            # Solve IK and integrate
            converge_ik(
                configuration,
                tasks,
                dt,
                SOLVER,
                POS_THRESHOLD,
                ORI_THRESHOLD,
                MAX_ITERS,
            )

            # Apply configuration to MuJoCo data
            data.qpos[:] = configuration.q
            mujoco.mj_forward(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
