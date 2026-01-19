#!/usr/bin/env python3
"""Joint configuration tracking with mink PostureTask.

This example demonstrates using a PostureTask to track a reference
joint configuration while allowing the end-effector to move.

Usage:
    python examples/mink/basic/posture_task_example.py
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


def main() -> None:
    """Run the posture task example."""
    # Load robot model
    models_root = get_models_root()
    scene_path = str(models_root / ROBOT_SCENE)

    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    # Create mink Configuration
    configuration = mink.Configuration(model)

    # Create posture task to maintain joint configuration
    posture_task = mink.PostureTask(model=model, cost=1.0)

    tasks = [posture_task]

    # Define target configurations to cycle through
    # These are example configurations - adjust based on your robot's joint limits
    nq = model.nq
    target_configs = [
        np.zeros(nq),  # Home position
        np.array([0.5, -0.3, 0.2, -1.5, 0.1, 1.2, 0.0] + [0.0] * (nq - 7))[:nq],
        np.array([-0.5, 0.3, -0.2, -1.0, -0.1, 0.8, 0.0] + [0.0] * (nq - 7))[:nq],
    ]
    current_target_idx = 0
    time_since_switch = 0.0
    switch_interval = 3.0  # Switch target every 3 seconds

    # Initialize viewer
    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize configuration
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)

        # Set initial posture target
        posture_task.set_target(target_configs[current_target_idx])

        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running posture task example...")
        print("The robot will cycle through predefined joint configurations.")
        print("Press Ctrl+C to exit.")

        while viewer.is_running():
            dt = rate.dt
            time_since_switch += dt

            # Switch target periodically
            if time_since_switch >= switch_interval:
                current_target_idx = (current_target_idx + 1) % len(target_configs)
                posture_task.set_target(target_configs[current_target_idx])
                time_since_switch = 0.0
                print(f"Switching to configuration {current_target_idx + 1}")

            # Solve IK
            vel = mink.solve_ik(configuration, tasks, dt, SOLVER, damping=1e-2)
            configuration.integrate_inplace(vel, dt)

            # Apply configuration to MuJoCo data
            data.qpos[:] = configuration.q
            mujoco.mj_forward(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
