#!/usr/bin/env python3
"""
Robot Package to RynnMotion MJCF Converter

Convert ROS URDF packages or MuJoCo Menagerie robots to RynnMotion's
modular MJCF layout.

Usage:
    python convert_robot.py --urdf /path/to/package --output ../3.robot_arm/30.myrobot
    python convert_robot.py --menagerie /path/to/menagerie/robot --output ../3.robot_arm/31.robot
    python convert_robot.py --urdf /path/to/robot.urdf --output ../3.robot_arm/32.robot --dual
"""

import argparse
import sys
from pathlib import Path

from robot_data import RobotData
from urdf_converter import UrdfConverter
from menagerie_parser import MenagerieParser
from mjcf_generator import MjcfGenerator
from dual_arm_generator import DualArmGenerator


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert robot packages to RynnMotion MJCF layout',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Convert ROS URDF package
  python convert_robot.py --urdf ~/catkin_ws/src/myrobot_description \\
      --output ../3.robot_arm/30.myrobot --robot-name myrobot

  # Convert MuJoCo Menagerie robot
  python convert_robot.py --menagerie ~/mujoco_menagerie/universal_robots_ur5e \\
      --output ../3.robot_arm/31.ur5e

  # With dual-arm generation
  python convert_robot.py --urdf ./robot.urdf --output ../3.robot_arm/32.robot --dual

  # Override auto-detection
  python convert_robot.py --urdf ./robot.urdf --output ../3.robot_arm/33.robot \\
      --ee-link tool0 --gripper-joints finger_joint1 finger_joint2
''')

    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        '--urdf', type=Path, metavar='PATH',
        help='Path to ROS URDF package directory or .urdf file'
    )
    input_group.add_argument(
        '--menagerie', type=Path, metavar='PATH',
        help='Path to MuJoCo Menagerie robot directory or .xml file'
    )

    parser.add_argument(
        '--output', '-o', type=Path, required=True, metavar='PATH',
        help='Output directory (e.g., ../3.robot_arm/30.myrobot)'
    )

    parser.add_argument(
        '--robot-name', type=str, metavar='NAME',
        help='Override robot name (default: from URDF/MJCF)'
    )

    parser.add_argument(
        '--ee-link', type=str, metavar='LINK',
        help='End-effector link name (default: auto-detect)'
    )

    parser.add_argument(
        '--gripper-joints', nargs='+', metavar='JOINT',
        help='Gripper joint names (default: auto-detect by name pattern)'
    )

    parser.add_argument(
        '--home-qpos', nargs='+', type=float, metavar='Q',
        help='Home position joint values'
    )

    parser.add_argument(
        '--standby-qpos', nargs='+', type=float, metavar='Q',
        help='Standby position joint values'
    )

    parser.add_argument(
        '--dual', action='store_true',
        help='Also generate dual-arm variant'
    )

    parser.add_argument(
        '--dual-separation', type=float, default=0.5, metavar='M',
        help='Arm separation distance for dual-arm (default: 0.5m)'
    )

    parser.add_argument(
        '--dual-output', type=Path, metavar='PATH',
        help='Dual-arm output directory (default: ../4.dual_arm/4X.dual_<name>)'
    )

    parser.add_argument(
        '--validate', action='store_true',
        help='Validate generated files with MuJoCo after conversion'
    )

    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='Verbose output'
    )

    return parser.parse_args()


def convert_urdf(args) -> RobotData:
    """Convert URDF to RobotData."""
    converter = UrdfConverter(
        urdf_path=args.urdf,
        robot_name=args.robot_name,
        ee_link=args.ee_link,
        gripper_joints=args.gripper_joints,
        home_qpos=args.home_qpos,
        standby_qpos=args.standby_qpos,
    )
    return converter.convert()


def convert_menagerie(args) -> RobotData:
    """Convert Menagerie to RobotData."""
    parser = MenagerieParser(
        menagerie_path=args.menagerie,
        robot_name=args.robot_name,
        ee_link=args.ee_link,
        gripper_joints=args.gripper_joints,
        home_qpos=args.home_qpos,
        standby_qpos=args.standby_qpos,
    )
    return parser.convert()


def validate_output(output_dir: Path, robot_name: str) -> bool:
    """Validate generated MJCF files with MuJoCo."""
    try:
        import mujoco
    except ImportError:
        print("Warning: mujoco not installed, skipping validation")
        return True

    mjcf_dir = output_dir / 'mjcf'
    scene_dir = output_dir / 'scene'

    files_to_check = [
        mjcf_dir / f'{robot_name}_robot.xml',
        mjcf_dir / f'{robot_name}_pinocchio.xml',
        scene_dir / 'scene.xml',
    ]

    all_valid = True
    for xml_file in files_to_check:
        if not xml_file.exists():
            print(f"  Warning: {xml_file.name} not found")
            continue

        try:
            model = mujoco.MjModel.from_xml_path(str(xml_file))
            print(f"  {xml_file.name}: OK (nq={model.nq}, nu={model.nu})")
        except Exception as e:
            print(f"  {xml_file.name}: FAILED - {e}")
            all_valid = False

    return all_valid


def main():
    args = parse_args()

    print(f"Converting robot to RynnMotion MJCF format...")
    print(f"  Output: {args.output}")

    if args.urdf:
        print(f"  Input: URDF from {args.urdf}")
        robot_data = convert_urdf(args)
    else:
        print(f"  Input: Menagerie from {args.menagerie}")
        robot_data = convert_menagerie(args)

    if args.verbose:
        print(f"\nRobot: {robot_data.name}")
        print(f"  Motion joints ({robot_data.motion_dof}): {robot_data.motion_joints}")
        print(f"  Gripper joints ({robot_data.gripper_dof}): {robot_data.gripper_joints}")
        print(f"  Meshes: {len(robot_data.meshes)}")
        if robot_data.ee_site:
            print(f"  EE site: {robot_data.ee_site.name} on {robot_data.ee_site.parent_link}")

    generator = MjcfGenerator(robot_data, args.output)
    generator.generate_all()

    if args.dual:
        print(f"\nGenerating dual-arm variant...")

        if args.dual_output:
            dual_output = args.dual_output
        else:
            output_parts = str(args.output).split('/')
            for i, part in enumerate(output_parts):
                if part.startswith('3.robot_arm'):
                    output_parts[i] = '4.dual_arm'
                elif '.' in part and part[0].isdigit():
                    num = int(part.split('.')[0])
                    name = part.split('.', 1)[1]
                    output_parts[i] = f'{num + 20}.dual_{name}'
            dual_output = Path('/'.join(output_parts))

        dual_gen = DualArmGenerator(
            robot_data=robot_data,
            single_arm_dir=args.output,
            dual_arm_dir=dual_output,
            arm_separation=args.dual_separation,
        )
        dual_gen.generate()

    if args.validate:
        print(f"\nValidating generated files...")
        if validate_output(args.output, robot_data.name):
            print("Validation passed!")
        else:
            print("Validation failed!")
            sys.exit(1)

    print(f"\nConversion complete!")
    print(f"\nTo use the robot:")
    print(f"  1. Rebuild CMake to trigger auto-discovery")
    print(f"  2. Run: ./mujocoExe {robot_data.name} 1")


if __name__ == '__main__':
    main()
