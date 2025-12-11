"""Test MjcfParser robot configuration extraction.

Usage:
    python test_mjcf_parser.py fr3
    python test_mjcf_parser.py dual_fr3 --verbose
    python test_mjcf_parser.py piper -v
    python test_mjcf_parser.py --all
"""

import argparse
import numpy as np

from RynnMotion.manager.mjcf_parser import MjcfParser, ActuatorMode
from RynnMotion.utils.path_config import get_models_root


ROBOT_PATHS = {
    "fr3": "3.robot_arm/20.fr3/mjcf/fr3_robot.xml",
    "ur5e": "3.robot_arm/21.ur5e/mjcf/ur5e_robot.xml",
    "piper": "3.robot_arm/22.piper/mjcf/piper_robot.xml",
    "rm75": "3.robot_arm/23.rm75/mjcf/rm75_robot.xml",
    "so101": "3.robot_arm/24.so101/mjcf/so101_robot.xml",
    "dual_fr3": "4.dual_arm/40.dual_fr3/mjcf/dual_fr3_robot.xml",
    "dual_ur5e": "4.dual_arm/41.dual_ur5e/mjcf/dual_ur5e_robot.xml",
    "dual_piper": "4.dual_arm/42.dual_piper/mjcf/dual_piper_robot.xml",
    "dual_rm75": "4.dual_arm/43.dual_rm75/mjcf/dual_rm75_robot.xml",
    "dual_so101": "4.dual_arm/44.dual_so101/mjcf/dual_so101_robot.xml",
}

ROBOT_ALIASES = {
    "fr3": ["FR3", "franka", "panda"],
    "ur5e": ["UR5E", "ur5-e", "ur5_e"],
    "piper": ["PIPER", "Piper"],
    "rm75": ["RM75", "realman75", "realman-rm75"],
    "so101": ["SO101", "so-101", "so_101", "soarm101"],
    "dual_fr3": ["dual-fr3", "dualfr3", "dual_franka"],
    "dual_ur5e": ["dual-ur5e", "dualur5e"],
    "dual_piper": ["dual-piper", "dualpiper"],
    "dual_rm75": ["dual-rm75", "dualrm75"],
    "dual_so101": ["dual-so101", "dualso101"],
}


def resolve_robot_name(name: str) -> str:
    """Resolve robot alias to canonical name."""
    lower = name.lower()
    if lower in ROBOT_PATHS:
        return lower

    for canonical, aliases in ROBOT_ALIASES.items():
        if lower in [a.lower() for a in aliases]:
            return canonical

    raise ValueError(f"Unknown robot: {name}")


def get_mjcf_path(robot_name: str) -> str:
    """Get full MJCF path for robot."""
    canonical = resolve_robot_name(robot_name)
    rel_path = ROBOT_PATHS[canonical]
    return str(get_models_root() / rel_path)


def test_robot(robot_name: str, verbose: bool = False) -> None:
    """Test MjcfParser for a robot."""
    canonical = resolve_robot_name(robot_name)
    mjcf_path = get_mjcf_path(canonical)

    print(f"\nRobot: {canonical}")
    print(f"  MJCF: {mjcf_path}")

    state = MjcfParser.extract_robot_state_dim(mjcf_path)

    print(f"  mdof: {state['mdof']}")
    print(f"  adof: {state['adof']}")
    print(f"  num_ee: {state['num_ee']}")
    print(f"  joint_indices: {state['joint_indices']}")
    print(f"  ee_indices: {state['ee_indices']}")
    print(f"  ee_joint_indices: {state['ee_joint_indices']}")

    if verbose:
        print(f"  ee_ranges: {state['ee_ranges']}")
        print(f"  ft_sensor_num: {state['ft_sensor_num']}")
        print(f"  site_num: {state['site_num']}")
        print(f"  ee_site_idx: {state['ee_site_idx']}")
        print(f"  ee_site_name: {state['ee_site_name']}")

        limits = MjcfParser.extract_joint_limits(mjcf_path, state["joint_indices"])
        print("\n  Joint Limits:")
        np.set_printoptions(precision=4, suppress=True)
        print(f"    q_pos_min: {limits['q_pos_min']}")
        print(f"    q_pos_max: {limits['q_pos_max']}")
        print(f"    q_vel_max: {limits['q_vel_max']}")
        print(f"    q_torque_max: {limits['q_torque_max']}")

        modes = MjcfParser.detect_actuator_modes(mjcf_path)
        mode_names = [m.name for m in modes]
        print(f"\n  Actuator Modes: {mode_names}")

        gains = MjcfParser.extract_actuator_gains(mjcf_path)
        print(f"\n  Gains:")
        print(f"    sim_kp: {gains['sim_kp']}")
        print(f"    sim_kd: {gains['sim_kd']}")


def main():
    parser = argparse.ArgumentParser(description="Test MjcfParser robot configuration extraction")
    parser.add_argument("robot", nargs="?", help="Robot name or alias (e.g., fr3, dual_fr3)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Show joint limits, modes, gains")
    parser.add_argument("--all", action="store_true", help="Test all robots")
    args = parser.parse_args()

    if args.all:
        for robot in ROBOT_PATHS.keys():
            try:
                test_robot(robot, args.verbose)
            except Exception as e:
                print(f"\nRobot: {robot}")
                print(f"  ERROR: {e}")
    elif args.robot:
        test_robot(args.robot, args.verbose)
    else:
        print("Available robots:")
        for name in ROBOT_PATHS.keys():
            aliases = ROBOT_ALIASES.get(name, [])
            alias_str = f" (aliases: {', '.join(aliases)})" if aliases else ""
            print(f"  {name}{alias_str}")
        print("\nUsage: python test_mjcf_parser.py <robot> [-v]")


if __name__ == "__main__":
    main()
