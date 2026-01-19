#!/usr/bin/env python3
"""Unitree G1 humanoid IK example with mink.

This example demonstrates mink inverse kinematics with the Unitree G1
humanoid robot, tracking multiple end-effectors (hands and feet).

Usage:
    python examples/mink/robots/humanoid_g1.py

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


def find_frame(model, keywords, frame_type="site"):
    """Find a frame by keyword matching."""
    if frame_type == "site":
        n = model.nsite
        obj_type = mujoco.mjtObj.mjOBJ_SITE
    else:
        n = model.nbody
        obj_type = mujoco.mjtObj.mjOBJ_BODY

    for i in range(n):
        name = mujoco.mj_id2name(model, obj_type, i)
        if name and any(kw in name.lower() for kw in keywords):
            return name
    return None


def main():
    # Load G1 model from robot_descriptions
    model = load_robot_description("g1_mj_description")
    data = mujoco.MjData(model)

    configuration = mink.Configuration(model)

    # Find relevant frames
    pelvis = find_frame(model, ["pelvis", "torso", "base"], "body")
    left_foot = find_frame(model, ["left_foot", "l_foot", "lfoot"], "site") or \
                find_frame(model, ["left_foot", "l_foot", "lfoot"], "body")
    right_foot = find_frame(model, ["right_foot", "r_foot", "rfoot"], "site") or \
                 find_frame(model, ["right_foot", "r_foot", "rfoot"], "body")
    left_hand = find_frame(model, ["left_hand", "l_hand", "lhand", "left_palm"], "site") or \
                find_frame(model, ["left_hand", "l_hand", "lhand"], "body")
    right_hand = find_frame(model, ["right_hand", "r_hand", "rhand", "right_palm"], "site") or \
                 find_frame(model, ["right_hand", "r_hand", "rhand"], "body")

    print(f"Found frames:")
    print(f"  Pelvis: {pelvis}")
    print(f"  Left foot: {left_foot}")
    print(f"  Right foot: {right_foot}")
    print(f"  Left hand: {left_hand}")
    print(f"  Right hand: {right_hand}")

    # Create tasks
    tasks = []

    # Pelvis orientation task (keep upright)
    if pelvis:
        pelvis_task = mink.FrameTask(
            frame_name=pelvis,
            frame_type="body",
            position_cost=0.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        tasks.append(pelvis_task)

    # Posture task
    posture_task = mink.PostureTask(model, cost=1e-1)
    tasks.append(posture_task)

    # Center of mass task
    com_task = mink.ComTask(cost=10.0)
    tasks.append(com_task)

    # Foot tasks (high priority to maintain balance)
    foot_tasks = []
    feet = [left_foot, right_foot]
    for foot in feet:
        if foot:
            task = mink.FrameTask(
                frame_name=foot,
                frame_type="site" if find_frame(model, [foot], "site") else "body",
                position_cost=10.0,
                orientation_cost=1.0,
                lm_damping=1.0,
            )
            foot_tasks.append(task)
            tasks.append(task)

    # Hand tasks (lower priority)
    hand_tasks = []
    hands = [left_hand, right_hand]
    for hand in hands:
        if hand:
            task = mink.FrameTask(
                frame_name=hand,
                frame_type="site" if find_frame(model, [hand], "site") else "body",
                position_cost=2.0,
                orientation_cost=1.0,
                lm_damping=1.0,
            )
            hand_tasks.append(task)
            tasks.append(task)

    # Limits
    limits = [mink.ConfigurationLimit(model)]

    model = configuration.model
    data = configuration.data

    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Initialize
        mujoco.mj_forward(model, data)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)

        if pelvis:
            pelvis_task.set_target_from_configuration(configuration)

        # Store initial foot positions
        initial_foot_transforms = []
        for foot_task in foot_tasks:
            T = configuration.get_transform_frame_to_world(
                foot_task.frame_name, foot_task.frame_type
            )
            initial_foot_transforms.append(T)
            foot_task.set_target(T)

        # Store initial hand positions
        initial_hand_transforms = []
        for hand_task in hand_tasks:
            T = configuration.get_transform_frame_to_world(
                hand_task.frame_name, hand_task.frame_type
            )
            initial_hand_transforms.append(T)

        # Initial COM
        initial_com = data.subtree_com[1].copy()
        com_task.set_target(initial_com)

        rate = RateLimiter(frequency=200.0, warn=False)

        print("Running Unitree G1 humanoid IK example...")
        print("Hands will follow a wave motion while feet stay planted.")
        print("Press Ctrl+C to exit.")

        local_time = 0.0
        while viewer.is_running():
            dt = rate.dt
            local_time += dt

            # Keep feet planted
            for i, foot_task in enumerate(foot_tasks):
                foot_task.set_target(initial_foot_transforms[i])

            # Animate hands (wave motion)
            for i, hand_task in enumerate(hand_tasks):
                phase = i * np.pi  # Opposite phase for left/right
                offset = np.array([
                    0.05 * np.sin(2 * np.pi * 0.3 * local_time + phase),
                    0.0,
                    0.08 * np.sin(2 * np.pi * 0.3 * local_time + phase),
                ])
                T_init = initial_hand_transforms[i]
                target_pos = T_init.translation() + offset
                T_target = mink.SE3.from_rotation_and_translation(
                    T_init.rotation(),
                    target_pos,
                )
                hand_task.set_target(T_target)

            # Keep COM centered
            com_task.set_target(initial_com)

            # Solve IK
            vel = mink.solve_ik(
                configuration, tasks, dt, SOLVER,
                damping=1e-1, limits=limits
            )
            configuration.integrate_inplace(vel, dt)
            mujoco.mj_camlight(model, data)
            mujoco.mj_fwdPosition(model, data)

            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
