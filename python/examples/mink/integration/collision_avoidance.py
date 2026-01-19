#!/usr/bin/env python3
"""Collision avoidance demonstration with mink.

This example demonstrates mink's CollisionAvoidanceLimit to prevent
self-collision and collision with the environment.

Usage:
    python examples/mink/integration/collision_avoidance.py
"""

import numpy as np
from loop_rate_limiters import RateLimiter

import mink
import mujoco
import mujoco.viewer

from RynnMotion.utils.path_config import get_models_root


# Robot scene path
ROBOT_SCENE = "3.robot_arm/20.fr3/scene/scene.xml"

# IK parameters
SOLVER = "daqp"


def main() -> None:
    """Run the collision avoidance example."""
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
        lm_damping=1e-6,
    )

    posture_task = mink.PostureTask(model=model, cost=1e-3)

    tasks = [end_effector_task, posture_task]

    # Define collision pairs to avoid
    # Format: (geom_group_1, geom_group_2)
    # The solver will try to maintain minimum distance between these pairs
    collision_pairs = [
        # Add collision pairs based on your robot's geometry
        # Example: prevent end-effector from hitting the base or floor
        # (["hand_collision"], ["floor", "base_collision"]),
    ]

    # Create limits
    limits = [
        mink.ConfigurationLimit(model=model),
    ]

    # Add collision avoidance if pairs are defined
    if collision_pairs:
        collision_limit = mink.CollisionAvoidanceLimit(
            model=model,
            geom_pairs=collision_pairs,
            minimum_distance_from_collisions=0.01,  # 1cm minimum distance
            collision_detection_distance=0.1,       # Start avoiding at 10cm
        )
        limits.append(collision_limit)

    # Add velocity limits for smooth motion
    # Get joint names from model
    velocity_limits = {}
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name:
            velocity_limits[joint_name] = np.pi  # 180 deg/s max

    if velocity_limits:
        velocity_limit = mink.VelocityLimit(model, velocity_limits)
        limits.append(velocity_limit)

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

        # Get initial end-effector position
        initial_target = configuration.get_transform_frame_to_world(
            eef_site, "site"
        ).translation()

        # Larger amplitude trajectory to potentially trigger collision avoidance
        amp = 0.20
        freq = 0.15

        local_time = 0.0
        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running collision avoidance example...")
        print(f"Active limits: {len(limits)}")
        print("  - ConfigurationLimit: Joint position limits")
        if collision_pairs:
            print("  - CollisionAvoidanceLimit: Collision avoidance")
        if velocity_limits:
            print("  - VelocityLimit: Joint velocity limits")
        print("Press Ctrl+C to exit.")

        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Large circular trajectory that may approach limits
            offset = np.array([
                amp * np.cos(2 * np.pi * freq * local_time),
                amp * np.sin(2 * np.pi * freq * local_time),
                0.1 * np.sin(4 * np.pi * freq * local_time),  # Vertical motion
            ])

            # Create target transform
            target_pos = initial_target + offset
            T_target = mink.SE3.from_rotation_and_translation(
                mink.SO3.identity(),
                target_pos,
            )
            end_effector_task.set_target(T_target)

            # Solve IK with all limits
            vel = mink.solve_ik(
                configuration, tasks, dt, SOLVER,
                damping=1e-3, limits=limits
            )
            configuration.integrate_inplace(vel, dt)

            # Update MuJoCo for visualization
            data.qpos[:] = configuration.q
            mujoco.mj_camlight(model, data)
            mujoco.mj_fwdPosition(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
