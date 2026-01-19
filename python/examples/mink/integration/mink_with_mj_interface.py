#!/usr/bin/env python3
"""Using mink IK with RynnMotion's MujocoRobotInterface.

This example demonstrates how to integrate mink's differential IK solver
with RynnMotion's MujocoRobotInterface for robot control.

Usage:
    python examples/mink/integration/mink_with_mj_interface.py
"""

import numpy as np
from loop_rate_limiters import RateLimiter

import mink

from RynnMotion.algorithms.mink_bridge import (
    configuration_from_model,
    sync_mj_data_to_config,
    get_joint_positions_from_config,
)
from RynnMotion.utils.path_config import get_models_root

import mujoco
import mujoco.viewer


# Robot scene path
ROBOT_SCENE = "3.robot_arm/20.fr3/scene/scene.xml"

# IK parameters
SOLVER = "daqp"
MAX_ITERS = 10


def main() -> None:
    """Run the MujocoRobotInterface integration example."""
    # Load robot model directly (simpler for this example)
    models_root = get_models_root()
    scene_path = str(models_root / ROBOT_SCENE)

    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    # Create mink Configuration from model
    configuration = configuration_from_model(model)

    # Find end-effector site (FR3 uses "EE_0")
    eef_site = "EE_0"
    for i in range(model.nsite):
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        if site_name and any(x in site_name.lower() for x in ["ee", "attach", "effector"]):
            eef_site = site_name
            break
    print(f"Using end-effector site: {eef_site}")

    # Create mink tasks
    end_effector_task = mink.FrameTask(
        frame_name=eef_site,
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )

    posture_task = mink.PostureTask(model=model, cost=1e-2)

    tasks = [end_effector_task, posture_task]

    # Create limits for safe operation
    limits = [
        mink.ConfigurationLimit(model=model),
    ]

    # Initialize viewer
    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize configuration from MuJoCo data
        mujoco.mj_forward(model, data)
        sync_mj_data_to_config(data, configuration)

        # Set initial posture target
        posture_task.set_target_from_configuration(configuration)

        # Get initial end-effector position
        initial_target = configuration.get_transform_frame_to_world(
            eef_site, "site"
        ).translation()

        # Circular trajectory parameters
        amp = 0.12
        freq = 0.2

        local_time = 0.0
        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running MujocoRobotInterface integration example...")
        print("The robot uses mink IK to track a circular trajectory.")
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

            # Solve IK with limits
            for _ in range(MAX_ITERS):
                vel = mink.solve_ik(
                    configuration, tasks, dt, SOLVER,
                    damping=1e-3, limits=limits
                )
                configuration.integrate_inplace(vel, dt)

            # Get joint positions from configuration
            joint_positions = get_joint_positions_from_config(configuration, model.nq)

            # Apply to MuJoCo (simulating MujocoRobotInterface.set_joint_positions)
            data.qpos[:] = joint_positions
            mujoco.mj_forward(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
