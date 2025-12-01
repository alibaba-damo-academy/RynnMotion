"""Debug frame lookup in a Pinocchio model.

Checks if specific site names exist and are accessible in oMf.

Usage:
    python debug_frames.py [model] [site_names...]
    python debug_frames.py                      # uses default (so101)
    python debug_frames.py fr3                  # shorthand
    python debug_frames.py so101 EE camera_front
"""

from __future__ import annotations

import sys
from pathlib import Path

import pinocchio as pin

_HERE = Path(__file__).parent
_MODELS = _HERE / "../../models/3.robot_arm"

MODELS = {
    "fr3": _MODELS / "20.fr3/mjcf/fr3_pinocchio.xml",
    "piper": _MODELS / "22.piper/mjcf/piper_pinocchio.xml",
    "so101": _MODELS / "24.so101/mjcf/so101_pinocchio.xml",
}

DEFAULT_SITES = ["shoulderSite", "elbowSite", "wristSite", "EE", "camera_wrist", "camera_front"]


def debug_frames(model_path: str, site_names: list[str] | None = None) -> None:
    """Load model and debug frame access for specified sites."""
    if site_names is None:
        site_names = DEFAULT_SITES

    pin_model, _, _ = pin.buildModelsFromMJCF(model_path)
    pin_data = pin_model.createData()

    print(f"\nModel: {model_path}")
    print(f"Frames: {pin_model.nframes}, oMf size: {len(pin_data.oMf)}")
    print(f"Joints: {pin_model.njoints}, DOF (nq): {pin_model.nq}")
    print()

    print("Checking sites:")
    for site_name in site_names:
        try:
            frame_id = pin_model.getFrameId(site_name)
            frame = pin_model.frames[frame_id]
            status = "OK" if frame_id < len(pin_data.oMf) else "WARNING: out of bounds"
            print(f"  {site_name:20s} -> id={frame_id:2d} type={frame.type} [{status}]")
        except Exception as e:
            print(f"  {site_name:20s} -> NOT FOUND ({e})")

    print()
    print("All frames:")
    for i in range(pin_model.nframes):
        frame = pin_model.frames[i]
        print(f"  [{i:2d}] {frame.name:30s} type={frame.type} parentJoint={frame.parentJoint}")


def main() -> None:
    if len(sys.argv) < 2:
        model_path = MODELS["so101"]
        site_names = None
    else:
        arg = sys.argv[1]
        model_path = MODELS.get(arg, Path(arg))
        site_names = sys.argv[2:] if len(sys.argv) > 2 else None

    debug_frames(str(model_path.resolve()), site_names)


if __name__ == "__main__":
    main()
