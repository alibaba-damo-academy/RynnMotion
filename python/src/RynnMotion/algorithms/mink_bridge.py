"""
Bridge utilities for integrating mink IK with RynnMotion interfaces.

This module provides helper functions to:
- Create mink Configuration objects from MuJoCo models/interfaces
- Sync state between MujocoRobotInterface and mink Configuration
- Apply mink IK solutions back to robot interfaces

Example:
    from RynnMotion.algorithms.mink_bridge import (
        configuration_from_interface,
        sync_interface_to_config,
        apply_config_to_interface,
    )

    # Create mink configuration from interface
    config = configuration_from_interface(interface)

    # Sync current state
    sync_interface_to_config(interface, config)

    # Solve IK...

    # Apply solution
    apply_config_to_interface(config, interface)
"""

from typing import TYPE_CHECKING, Optional
import numpy as np
import mujoco

if TYPE_CHECKING:
    from RynnMotion.core.mj_interface import MujocoRobotInterface
    import mink as _mink_type

# Lazy import for mink - only load when actually used
_mink = None


def _ensure_mink():
    """Lazily import mink, raising a clear error if not installed."""
    global _mink
    if _mink is None:
        try:
            import mink as _mink_module
            _mink = _mink_module
        except ImportError:
            raise ImportError(
                "mink is required for IK bridge functions. "
                "Install with: pip install RynnMotion[mink]"
            )
    return _mink


def configuration_from_model(model: mujoco.MjModel) -> "_mink_type.Configuration":
    """Create a mink Configuration from a MuJoCo model.

    Args:
        model: MuJoCo model object

    Returns:
        mink Configuration object initialized with the model
    """
    mink = _ensure_mink()
    return mink.Configuration(model)


def configuration_from_interface(
    interface: "MujocoRobotInterface",
) -> "_mink_type.Configuration":
    """Create a mink Configuration from a MujocoRobotInterface.

    Args:
        interface: RynnMotion MujocoRobotInterface object

    Returns:
        mink Configuration object initialized with the interface's model

    Example:
        interface = MujocoRobotInterface(robot_model, robot_config, logger)
        config = configuration_from_interface(interface)
    """
    mink = _ensure_mink()
    return mink.Configuration(interface.mjModel)


def sync_interface_to_config(
    interface: "MujocoRobotInterface",
    configuration: "_mink_type.Configuration",
) -> None:
    """Sync state from MujocoRobotInterface to mink Configuration.

    This copies the current joint positions from the interface to the
    mink configuration, ensuring the IK solver starts from the current state.

    Args:
        interface: RynnMotion MujocoRobotInterface object
        configuration: mink Configuration to update

    Example:
        sync_interface_to_config(interface, config)
        # Now solve IK with current robot state
        vel = mink.solve_ik(config, tasks, dt, solver)
    """
    # Get current joint positions from interface
    q = interface.get_joint_positions()

    # Update mink configuration
    # Note: mink Configuration expects full qpos array
    qpos = np.zeros(interface.mjModel.nq)
    qpos[:len(q)] = q
    configuration.update(qpos)


def sync_mj_data_to_config(
    data: mujoco.MjData,
    configuration: "_mink_type.Configuration",
) -> None:
    """Sync state from MjData to mink Configuration.

    Args:
        data: MuJoCo data object
        configuration: mink Configuration to update
    """
    configuration.update(data.qpos)


def apply_config_to_interface(
    configuration: "_mink_type.Configuration",
    interface: "MujocoRobotInterface",
) -> None:
    """Apply mink Configuration solution to MujocoRobotInterface.

    This copies the joint positions from the mink configuration to the
    interface's command buffer.

    Args:
        configuration: mink Configuration with IK solution
        interface: RynnMotion MujocoRobotInterface to update

    Example:
        # After solving IK
        apply_config_to_interface(config, interface)
        interface.step()  # Apply the command
    """
    # Get joint positions from mink configuration
    q = configuration.q[:interface.mdof]

    # Set joint commands on interface
    interface.set_joint_positions(q)


def apply_config_to_mj_data(
    configuration: "_mink_type.Configuration",
    data: mujoco.MjData,
) -> None:
    """Apply mink Configuration solution to MjData.

    Args:
        configuration: mink Configuration with IK solution
        data: MuJoCo data to update
    """
    data.qpos[:] = configuration.q


def get_joint_positions_from_config(
    configuration: "_mink_type.Configuration",
    ndof: Optional[int] = None,
) -> np.ndarray:
    """Get joint positions from mink Configuration.

    Args:
        configuration: mink Configuration object
        ndof: Number of DOFs to return (default: all)

    Returns:
        Joint positions array
    """
    if ndof is None:
        return configuration.q.copy()
    return configuration.q[:ndof].copy()


def create_frame_task(
    frame_name: str,
    frame_type: str = "site",
    position_cost: float = 1.0,
    orientation_cost: float = 1.0,
    lm_damping: float = 1.0,
) -> "_mink_type.FrameTask":
    """Create a mink FrameTask with common defaults.

    Args:
        frame_name: Name of the frame (body, site, or geom)
        frame_type: Type of frame ("body", "site", or "geom")
        position_cost: Cost weight for position error
        orientation_cost: Cost weight for orientation error
        lm_damping: Levenberg-Marquardt damping factor

    Returns:
        Configured mink FrameTask
    """
    mink = _ensure_mink()
    return mink.FrameTask(
        frame_name=frame_name,
        frame_type=frame_type,
        position_cost=position_cost,
        orientation_cost=orientation_cost,
        lm_damping=lm_damping,
    )


def create_posture_task(
    model: mujoco.MjModel,
    cost: float = 1e-2,
) -> "_mink_type.PostureTask":
    """Create a mink PostureTask with common defaults.

    Args:
        model: MuJoCo model
        cost: Cost weight for posture deviation

    Returns:
        Configured mink PostureTask
    """
    mink = _ensure_mink()
    return mink.PostureTask(model=model, cost=cost)


def create_standard_limits(
    model: mujoco.MjModel,
    velocity_limits: Optional[dict] = None,
    collision_pairs: Optional[list] = None,
) -> list:
    """Create standard mink limits for safe operation.

    Args:
        model: MuJoCo model
        velocity_limits: Dict mapping joint names to max velocities (rad/s)
        collision_pairs: List of collision pairs for avoidance

    Returns:
        List of mink Limit objects

    Example:
        limits = create_standard_limits(
            model,
            velocity_limits={"joint1": 1.0, "joint2": 1.5},
            collision_pairs=[
                (["hand"], ["table", "floor"]),
            ],
        )
    """
    mink = _ensure_mink()
    limits = [mink.ConfigurationLimit(model=model)]

    if velocity_limits:
        limits.append(mink.VelocityLimit(model, velocity_limits))

    if collision_pairs:
        limits.append(
            mink.CollisionAvoidanceLimit(
                model=model,
                geom_pairs=collision_pairs,
                minimum_distance_from_collisions=0.01,
                collision_detection_distance=0.1,
            )
        )

    return limits


def solve_ik_step(
    configuration: "_mink_type.Configuration",
    tasks: list,
    dt: float,
    solver: str = "daqp",
    damping: float = 1e-3,
    limits: Optional[list] = None,
    max_iters: int = 1,
) -> np.ndarray:
    """Solve IK and integrate in one step.

    This is a convenience function that calls solve_ik and integrates
    the result.

    Args:
        configuration: mink Configuration object (modified in-place)
        tasks: List of mink tasks
        dt: Time step
        solver: QP solver name
        damping: Damping factor
        limits: Optional list of mink limits
        max_iters: Number of IK iterations

    Returns:
        Final velocity solution

    Example:
        vel = solve_ik_step(config, tasks, 0.005)
        data.qpos[:] = config.q
    """
    mink = _ensure_mink()
    vel = np.zeros(configuration.model.nv)

    for _ in range(max_iters):
        vel = mink.solve_ik(
            configuration, tasks, dt, solver,
            damping=damping, limits=limits
        )
        configuration.integrate_inplace(vel, dt)

    return vel
