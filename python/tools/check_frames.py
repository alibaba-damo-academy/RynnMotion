"""List all frames in a Pinocchio model.

Usage:
    python check_frames.py [model_path]
    python check_frames.py                          # uses default (piper)
    python check_frames.py fr3                      # shorthand for fr3_pinocchio.xml
    python check_frames.py ../../models/.../xx.xml  # custom path
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


def list_all_frames(model_path: str) -> None:
    """Load model and print all frames with their properties."""
    pin_model, _, _ = pin.buildModelsFromMJCF(model_path)

    print(f"\nModel: {model_path}")
    print(f"Joints: {pin_model.njoints}, DOF: {pin_model.nq}, Frames: {pin_model.nframes}")
    print()
    print(f"{'ID':<4} {'Name':<30} {'Type':<20} {'ParentJoint':<15}")
    print("=" * 70)

    for i in range(pin_model.nframes):
        frame = pin_model.frames[i]
        print(f"{i:<4} {frame.name:<30} {str(frame.type):<20} {frame.parentJoint:<15}")


def main() -> None:
    if len(sys.argv) < 2:
        model_path = MODELS["piper"]
    else:
        arg = sys.argv[1]
        model_path = MODELS.get(arg, Path(arg))

    list_all_frames(str(model_path.resolve()))


if __name__ == "__main__":
    main()
