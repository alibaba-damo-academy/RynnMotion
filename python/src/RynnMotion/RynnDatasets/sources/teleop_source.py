"""TeleOp sensor source factory for RynnLeRobot recording.

Creates ISensorSource instances wrapping a TeleOperator + cameras.
EE and camera pose features are computed by PinocchioKinematicsProcessor
in the recording pipeline (not by sensor sources).
"""

from .joint_source import DirectJointSensorSource
from .camera_source import DirectCameraSensorSource
from . import ISensorSource


def create_teleop_sources(
    teleop,
    robot_context,
    cameras: dict,
    cameras_config: dict,
) -> list[ISensorSource]:
    """Create ISensorSource instances wrapping a TeleOperator + cameras.

    Joint and camera sources only. EE poses and camera poses are
    computed by PinocchioKinematicsProcessor in the recording pipeline.

    Args:
        teleop: TeleOperator instance (must already be running).
        robot_context: RobotContext with auto-detected joint/EE names and DOF.
        cameras: Dict mapping camera display name to connected camera object.
        cameras_config: Dict mapping camera display name to (height, width, channels).

    Returns:
        List of ISensorSource instances ready for RynnRecorder.
    """
    sources: list[ISensorSource] = []
    mdof = robot_context.mdof
    joint_names = robot_context.joint_names
    ee_names = robot_context.ee_names

    # --- Joint sources ---

    sources.append(DirectJointSensorSource(
        name="obs_joints",
        joint_names=joint_names,
        reader_fn=lambda: teleop.get_observation()[:mdof],
        prefix="observation",
    ))

    if ee_names:
        sources.append(DirectJointSensorSource(
            name="obs_gripper",
            joint_names=ee_names,
            reader_fn=lambda: teleop.get_observation()[mdof:],
            prefix="observation",
            suffix="gripper",
        ))

    action_names = list(joint_names) + list(ee_names)
    sources.append(DirectJointSensorSource(
        name="action",
        joint_names=action_names,
        reader_fn=teleop.get_action,
        prefix="action",
    ))

    # --- Camera image sources ---

    for cam_name, camera in cameras.items():
        if camera is not None and hasattr(camera, "is_connected") and camera.is_connected:
            resolution = cameras_config.get(cam_name, (480, 640, 3))
            from RynnLeRobot.hardware.cameras.configs import ColorMode

            def _make_reader(c):
                return lambda: c.read(color_mode=ColorMode.RGB)

            sources.append(DirectCameraSensorSource(
                name=cam_name,
                camera=_make_reader(camera),
                resolution=resolution,
            ))

    return sources
