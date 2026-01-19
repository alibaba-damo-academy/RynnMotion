#!/usr/bin/env python3
"""Teleoperation using mink IK.

This example demonstrates how to use mink for teleoperation by tracking
a movable target (mocap body) in the MuJoCo viewer.

Drag the target sphere in the viewer to control the robot end-effector.

Usage:
    python examples/mink/integration/teleop_with_mink.py
"""

import numpy as np
from loop_rate_limiters import RateLimiter

import mink
import mujoco
import mujoco.viewer

from RynnMotion.utils.path_config import get_models_root


# Robot scene path (needs a scene with mocap body for target)
ROBOT_SCENE = "3.robot_arm/20.fr3/scene/scene.xml"

# IK parameters
SOLVER = "daqp"


def add_mocap_target(model: mujoco.MjModel, data: mujoco.MjData) -> int:
    """Check for and initialize mocap target body.

    Args:
        model: MuJoCo model
        data: MuJoCo data

    Returns:
        Mocap body ID if found, -1 otherwise
    """
    # Look for a body named "target" that has mocapid
    try:
        target_body = model.body("target")
        mocap_id = target_body.mocapid[0]
        if mocap_id >= 0:
            return mocap_id
    except Exception:
        pass

    # No mocap target found
    return -1


def main() -> None:
    """Run the teleoperation example."""
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

    # Create tasks
    end_effector_task = mink.FrameTask(
        frame_name=eef_site,
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )

    posture_task = mink.PostureTask(model=model, cost=1e-2)

    tasks = [end_effector_task, posture_task]

    # Create limits
    limits = [
        mink.ConfigurationLimit(model=model),
    ]

    # Check for mocap target
    mocap_id = add_mocap_target(model, data)
    has_mocap = mocap_id >= 0

    # Initialize viewer
    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize configuration
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)

        # Set initial posture target
        posture_task.set_target_from_configuration(configuration)

        # Get initial end-effector transform
        T_init = configuration.get_transform_frame_to_world(
            eef_site, "site"
        )

        # Initialize mocap target at end-effector if available
        if has_mocap:
            data.mocap_pos[mocap_id] = T_init.translation()
            data.mocap_quat[mocap_id] = T_init.rotation().wxyz
            print("Mocap target found! Drag the target in the viewer to teleop.")
        else:
            print("No mocap target found. Using simulated teleoperation.")
            print("The target will move in a preset pattern.")

        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running teleoperation example...")
        print("Press Ctrl+C to exit.")

        local_time = 0.0

        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            if has_mocap:
                # Get target from mocap body (user can drag this in viewer)
                T_target = mink.SE3.from_mocap_id(data, mocap_id)
            else:
                # Simulated teleoperation with moving target
                offset = np.array([
                    0.1 * np.sin(2 * np.pi * 0.2 * local_time),
                    0.1 * np.cos(2 * np.pi * 0.2 * local_time),
                    0.05 * np.sin(4 * np.pi * 0.2 * local_time),
                ])
                target_pos = T_init.translation() + offset
                T_target = mink.SE3.from_rotation_and_translation(
                    T_init.rotation(),
                    target_pos,
                )

            end_effector_task.set_target(T_target)

            # Solve IK
            vel = mink.solve_ik(
                configuration, tasks, dt, SOLVER,
                damping=1e-3, limits=limits
            )
            configuration.integrate_inplace(vel, dt)

            # Apply to MuJoCo
            data.qpos[:] = configuration.q
            mujoco.mj_forward(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
