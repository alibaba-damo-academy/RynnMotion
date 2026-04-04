#!/usr/bin/env python3
"""MJCF-to-Dataset pipeline.

Auto-generates empty LeRobot v3.0 datasets or ROS2 recording configs from MJCF scene files.
All robot info (joints, grippers, sensors, cameras) is auto-detected from the MJCF XML.

Usage:
    rynn-create-dataset --mjcf scene.xml --output record/
    rynn-create-dataset --mjcf scene.xml --format lerobot --output /tmp/my_dataset
    rynn-create-dataset --mjcf scene.xml --format ros2 --output /tmp/config.yaml
"""

import argparse
import logging
from datetime import datetime
from pathlib import Path

import pandas as pd
import yaml

from RynnMotion.utils.path_config import get_rynnmotion_root
from RynnMotion.RynnDatasets.robot_context import RobotContext
from RynnMotion.RynnDatasets.utils import (
    create_empty_dataset_info,
    write_info,
    write_json,
    write_tasks,
)

logger = logging.getLogger(__name__)

CODEBASE_VERSION = "v3.0"
DEFAULT_RECORD_DIR = "record"


def _detect_robot_name_from_path(mjcf_path: Path) -> str:
    """Detect robot name from MJCF path like .../24.so101/scene/scene.xml → so101."""
    for parent in mjcf_path.parents:
        name = parent.name
        # Match pattern like "24.so101", "20.fr3", "42.dual_piper"
        if "." in name:
            parts = name.split(".", 1)
            if parts[0].isdigit() and len(parts) > 1:
                return parts[1]
    return ""


def _resolve_mjcf_path(mjcf_path: str) -> Path:
    """Resolve MJCF path: try as-is, then relative to project root."""
    p = Path(mjcf_path)
    if p.exists():
        return p.resolve()
    # Try relative to project root (e.g., "models/3.robot_arm/.../scene.xml")
    root = get_rynnmotion_root()
    candidate = root / mjcf_path
    if candidate.exists():
        return candidate.resolve()
    raise FileNotFoundError(
        f"MJCF file not found: {mjcf_path}\n"
        f"  Searched: {p.resolve()}\n"
        f"  Searched: {candidate}\n"
        f"  Example: rynn-create-dataset --mjcf models/3.robot_arm/24.so101/scene/scene.xml"
    )


def create_empty_lerobot_dataset(
    mjcf_path: str,
    pino_path: str | None = None,
    output_dir: str = "./record",
    fps: int = 30,
    robot_name: str = "",
    camera_resolution: tuple[int, int, int] = (480, 640, 3),
    use_videos: bool = True,
) -> Path:
    """Create an empty LeRobot v3.0 dataset from MJCF scene file.

    Args:
        mjcf_path: Path to scene XML (must include actuators, sensors, cameras).
        pino_path: Optional path to Pinocchio MJCF (kinematic tree only).
        output_dir: Output directory. If it looks like a base dir (e.g. 'record/'),
            a timestamped subdirectory is created automatically.
        fps: Recording FPS.
        robot_name: Robot identifier. Auto-detected from joint names if empty.
        camera_resolution: Default (H, W, C) for cameras.
        use_videos: Whether camera features use "video" dtype.

    Returns:
        Path to the created dataset directory.
    """
    mjcf = _resolve_mjcf_path(mjcf_path)

    ctx = RobotContext.from_mjcf(str(mjcf), pino_path, robot_name=robot_name)

    camera_configs = {cam: camera_resolution for cam in ctx.camera_names}
    features = ctx.get_recording_features(camera_configs, use_videos)

    # Auto-detect robot name from MJCF path (e.g., .../24.so101/scene/scene.xml → so101)
    if not robot_name:
        robot_name = _detect_robot_name_from_path(mjcf)

    # Default output to {project_root}/record/
    out = Path(output_dir)
    if not out.is_absolute():
        out = get_rynnmotion_root() / out

    # Auto-generate timestamped subdirectory
    if out.is_dir() or out.name == DEFAULT_RECORD_DIR or not out.suffix:
        ts = datetime.now().strftime("%Y%m%d_%H%M")
        name = f"{robot_name}_{ts}" if robot_name else ts
        out = out / name

    out.mkdir(parents=True, exist_ok=True)
    (out / "meta" / "episodes").mkdir(parents=True, exist_ok=True)
    (out / "data").mkdir(exist_ok=True)
    if use_videos and ctx.camera_names:
        (out / "videos").mkdir(exist_ok=True)

    # Write info.json
    info = create_empty_dataset_info(
        CODEBASE_VERSION, fps, features, use_videos, robot_type=robot_name or None
    )
    write_info(info, out)

    # Write stats.json (empty)
    write_json({}, out / "meta" / "stats.json")

    # Write tasks.parquet (empty)
    empty_tasks = pd.DataFrame({"task_index": pd.Series(dtype="int64")})
    empty_tasks.index.name = "task"
    write_tasks(empty_tasks, out)

    return out


def generate_ros2_config(
    mjcf_path: str,
    pino_path: str | None = None,
    output_path: str = "./config.yaml",
    robot_name: str = "",
    topic_prefix: str = "",
) -> Path:
    """Generate ROS2 recording config YAML from MJCF scene file.

    Args:
        mjcf_path: Path to scene XML.
        pino_path: Optional Pinocchio MJCF path.
        output_path: Output YAML file path.
        robot_name: Robot identifier for topic naming.
        topic_prefix: ROS2 topic prefix (default: /{robot_name}_driver).

    Returns:
        Path to the created YAML file.
    """
    mjcf = _resolve_mjcf_path(mjcf_path)

    ctx = RobotContext.from_mjcf(str(mjcf), pino_path, robot_name=robot_name)

    if not robot_name:
        robot_name = _detect_robot_name_from_path(mjcf)

    prefix = topic_prefix or (f"/{robot_name}_driver" if robot_name else "/robot_driver")

    topics = []

    # Joint states
    if ctx.joint_names:
        topics.append({
            "topic": f"{prefix}/joint_states",
            "type": "sensor_msgs.msg.JointState",
            "adapter": "GenericAdapter",
            "mappings": [{"field": "position", "out_key": "observation.state"}],
        })
        topics.append({
            "topic": f"{prefix}/joint_cmd",
            "type": "sensor_msgs.msg.JointState",
            "adapter": "GenericAdapter",
            "mappings": [{"field": "position", "out_key": "action"}],
        })

    # Gripper
    if ctx.ee_names:
        topics.append({
            "topic": f"{prefix}/gripper_states",
            "type": "sensor_msgs.msg.JointState",
            "adapter": "GripperStateAdapter",
            "mappings": [{"field": "position", "out_key": "observation.state.gripper"}],
        })
        topics.append({
            "topic": f"{prefix}/gripper_cmd",
            "type": "sensor_msgs.msg.JointState",
            "adapter": "GenericAdapter",
            "mappings": [{"field": "position", "out_key": "action.gripper"}],
        })

    # EE pose
    if ctx.num_ee > 0:
        pose_fields = [
            "pose.position.x", "pose.position.y", "pose.position.z",
            "pose.orientation.x", "pose.orientation.y",
            "pose.orientation.z", "pose.orientation.w",
        ]
        topics.append({
            "topic": f"{prefix}/ee_pose",
            "type": "geometry_msgs.msg.PoseStamped",
            "adapter": "GenericAdapter",
            "mappings": [
                {"field": f, "out_key": "observation.end_effector_pose"}
                for f in pose_fields
            ],
        })

    # Force/torque sensors
    for sensor_info in ctx.sensors.get("force", []):
        topics.append({
            "topic": f"{prefix}/{sensor_info.name}",
            "type": "geometry_msgs.msg.WrenchStamped",
            "adapter": "GenericAdapter",
            "mappings": [
                {"field": f, "out_key": f"observation.sensor.force.{sensor_info.name}"}
                for f in ["wrench.force.x", "wrench.force.y", "wrench.force.z"]
            ],
        })

    for sensor_info in ctx.sensors.get("torque", []):
        topics.append({
            "topic": f"{prefix}/{sensor_info.name}",
            "type": "geometry_msgs.msg.WrenchStamped",
            "adapter": "GenericAdapter",
            "mappings": [
                {"field": f, "out_key": f"observation.sensor.torque.{sensor_info.name}"}
                for f in ["wrench.torque.x", "wrench.torque.y", "wrench.torque.z"]
            ],
        })

    # IMU sensors
    for sensor_info in ctx.sensors.get("gyro", []):
        topics.append({
            "topic": f"{prefix}/{sensor_info.name}",
            "type": "sensor_msgs.msg.Imu",
            "adapter": "GenericAdapter",
            "mappings": [
                {"field": f, "out_key": f"observation.sensor.gyro.{sensor_info.name}"}
                for f in ["angular_velocity.x", "angular_velocity.y", "angular_velocity.z"]
            ],
        })

    for sensor_info in ctx.sensors.get("accel", []):
        topics.append({
            "topic": f"{prefix}/{sensor_info.name}",
            "type": "sensor_msgs.msg.Imu",
            "adapter": "GenericAdapter",
            "mappings": [
                {"field": f, "out_key": f"observation.sensor.accel.{sensor_info.name}"}
                for f in [
                    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"
                ]
            ],
        })

    # Cameras
    for cam_name in ctx.camera_names:
        topics.append({
            "topic": f"/camera/{cam_name}/color/image_raw",
            "type": "sensor_msgs.msg.Image",
            "adapter": "CameraAdapter",
            "mappings": [{"field": "data", "out_key": f"observation.images.{cam_name}"}],
        })

    config = {"protocol": "ros2", "topics": topics}
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        yaml.safe_dump(config, f, default_flow_style=False, sort_keys=False)

    return out


def _resolve_robot_to_mjcf(robot: str, scene: str | None) -> tuple[str, str, str]:
    """Resolve robot name/number/alias + scene to (mjcf_path, pino_path, robot_name)."""
    from RynnMotion.manager.discovery import get_discovery

    d = get_discovery()
    info = d.get_robot_info(robot)
    scene_info = d.get_scene(info, scene)
    return scene_info.full_path, info.pino_mjcf, info.canonical_name


def main():
    parser = argparse.ArgumentParser(
        description="Create empty datasets from MJCF robot definitions",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  rynn-create-dataset so101                           # default scene, both formats
  rynn-create-dataset fr3 --scene pickplace           # specific scene
  rynn-create-dataset 24 --format lerobot             # by robot number
  rynn-create-dataset --mjcf path/to/scene.xml        # direct MJCF path
  rynn-create-dataset --list                          # list all robots""",
    )
    parser.add_argument("robot", nargs="?", default=None,
                        help="Robot name, alias, or number (e.g., so101, fr3, 24)")
    parser.add_argument("--scene", default=None, help="Scene name (default: first available)")
    parser.add_argument("--mjcf", default=None, help="Direct path to MJCF scene XML (overrides robot)")
    parser.add_argument("--pino", default=None, help="Path to Pinocchio MJCF (optional)")
    parser.add_argument("--list", action="store_true", help="List all available robots and scenes")
    parser.add_argument(
        "--format", choices=["lerobot", "ros2", "both"], default="both",
        help="Output format (default: both)"
    )
    parser.add_argument("--output", "-o", default="record",
                        help="Output dir or file (default: record/ under project root)")
    parser.add_argument("--fps", type=int, default=30, help="Recording FPS (default: 30)")
    parser.add_argument("--robot-name", default="", help="Override robot name in output")
    parser.add_argument("--camera-resolution", default="480x640", help="HxW (default: 480x640)")
    parser.add_argument("--topic-prefix", default="", help="ROS2 topic prefix")
    parser.add_argument("--no-videos", action="store_true", help="Use image dtype instead of video")

    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")

    # List mode
    if args.list:
        from RynnMotion.manager.discovery import get_discovery
        d = get_discovery()
        for r in d.get_all_robots():
            scenes = ", ".join(s.name for s in r.scenes)
            print(f"  {r.number:3d}. {r.canonical_name:<20s} scenes: {scenes}")
        return

    # Resolve MJCF path
    if args.mjcf:
        mjcf_path = str(_resolve_mjcf_path(args.mjcf))
        pino_path = args.pino
        robot_name = args.robot_name
    elif args.robot:
        mjcf_path, pino_path, robot_name = _resolve_robot_to_mjcf(args.robot, args.scene)
        if args.robot_name:
            robot_name = args.robot_name
        if args.pino:
            pino_path = args.pino
    else:
        parser.print_help()
        print("\n--- Available Robots ---")
        from RynnMotion.manager.discovery import get_discovery
        d = get_discovery()
        for r in d.get_all_robots():
            scenes = ", ".join(s.name for s in r.scenes)
            print(f"  {r.number:3d}. {r.canonical_name:<20s} scenes: {scenes}")
        return

    h, w = map(int, args.camera_resolution.split("x"))
    camera_resolution = (h, w, 3)

    if args.format in ("lerobot", "both"):
        dataset_dir = create_empty_lerobot_dataset(
            mjcf_path=mjcf_path,
            pino_path=pino_path or None,
            output_dir=args.output,
            fps=args.fps,
            robot_name=robot_name,
            camera_resolution=camera_resolution,
            use_videos=not args.no_videos,
        )
        if args.format == "both":
            generate_ros2_config(
                mjcf_path=mjcf_path,
                pino_path=pino_path or None,
                output_path=str(dataset_dir / "ros2_config.yaml"),
                robot_name=robot_name,
                topic_prefix=args.topic_prefix,
            )
        print(f"Dataset: {dataset_dir}")

    elif args.format == "ros2":
        ros2_path = generate_ros2_config(
            mjcf_path=mjcf_path,
            pino_path=pino_path or None,
            output_path=args.output,
            robot_name=robot_name,
            topic_prefix=args.topic_prefix,
        )
        print(f"ROS2 config: {ros2_path}")



if __name__ == "__main__":
    main()
