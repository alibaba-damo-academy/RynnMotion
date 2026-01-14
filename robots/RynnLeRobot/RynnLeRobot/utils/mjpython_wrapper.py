#!/usr/bin/env python3
"""Platform-aware wrapper for MuJoCo scripts.

Handles mjpython requirement on macOS for MuJoCo viewer scripts.
"""

import os
import sys
import platform
import subprocess


def get_platform_python_command():
    """Get the appropriate Python command for the current platform."""
    if platform.system() == "Darwin":  # macOS
        return "mjpython"
    else:  # Linux/Ubuntu
        return sys.executable  # Use current Python


def run_with_platform_python(script_module, script_function="main"):
    """
    Run a script with the appropriate Python interpreter for the platform.

    Args:
        script_module: Module path (e.g., "RynnLeRobot.controller.joint_teleop")
        script_function: Function name to call (default: "main")
    """
    current_platform = platform.system()

    if current_platform == "Darwin":  # macOS
        # Check if we're already running under mjpython
        is_mjpython = "mjpython" in sys.executable or os.path.basename(sys.executable) == "mjpython"

        if not is_mjpython:
            # Re-execute with mjpython to ensure main thread GUI execution
            try:
                subprocess.run(["which", "mjpython"], check=True, capture_output=True)
            except subprocess.CalledProcessError:
                print("❌ Error: mjpython not found!")
                print("MuJoCo viewer requires mjpython on macOS.")
                print("Make sure MuJoCo is properly installed and mjpython is in PATH.")
                sys.exit(1)

            # Re-execute this script with mjpython, preserving sys.argv arguments
            # IMPORTANT: Must pass sys.argv to preserve command-line arguments like --robot ur5e
            cmd = ["mjpython", "-c",
                   f"import sys; sys.argv = {repr(sys.argv)}; "
                   f"from {script_module} import {script_function}; {script_function}()"]
            try:
                subprocess.run(cmd, check=True)
                sys.exit(0)
            except subprocess.CalledProcessError as e:
                print(f"❌ Error running script with mjpython: {e}")
                sys.exit(e.returncode)
            except KeyboardInterrupt:
                print("\n🛑 Script interrupted by user")
                sys.exit(130)
        else:
            # Already running under mjpython, import and run directly
            try:
                module = __import__(script_module, fromlist=[script_function])
            except ImportError as e:
                print(f"❌ Error importing {script_module}: {e}")
                sys.exit(1)

            try:
                main_func = getattr(module, script_function)
            except AttributeError:
                print(f"❌ Error: {script_function} function not found in {script_module}")
                sys.exit(1)

            try:
                main_func()
            except Exception as e:
                print(f"❌ Error running script: {e}")
                import traceback
                traceback.print_exc()
                sys.exit(1)

    else:  # Linux/Ubuntu
        # Import and run directly
        try:
            module = __import__(script_module, fromlist=[script_function])
        except ImportError as e:
            print(f"❌ Error importing {script_module}: {e}")
            sys.exit(1)

        try:
            main_func = getattr(module, script_function)
        except AttributeError:
            print(f"❌ Error: {script_function} function not found in {script_module}")
            sys.exit(1)

        try:
            main_func()
        except Exception as e:
            print(f"❌ Error running script: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)


def create_mujoco_wrapper(script_module):
    """
    Create a wrapper function for MuJoCo-based scripts.

    Args:
        script_module: Module path (e.g., "RynnLeRobot.controller.joint_teleop")

    Returns:
        A wrapper function that can be used as a main entry point
    """
    def wrapper_main():
        run_with_platform_python(script_module)

    return wrapper_main


def run_with_regular_python(script_module, script_function="main"):
    """
    Run a script with regular Python interpreter (no mjpython).
    Use this for scripts that handle macOS main thread requirements internally.

    Args:
        script_module: Module path (e.g., "RynnLeRobot.teleop.multi_teleop_mac")
        script_function: Function name to call (default: "main")
    """
    try:
        module = __import__(script_module, fromlist=[script_function])
    except ImportError as e:
        print(f"❌ Error importing {script_module}: {e}")
        sys.exit(1)

    try:
        main_func = getattr(module, script_function)
    except AttributeError:
        print(f"❌ Error: {script_function} function not found in {script_module}")
        sys.exit(1)

    try:
        main_func()
    except Exception as e:
        print(f"❌ Error running script: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def create_direct_wrapper(script_module):
    """
    Create a wrapper function that runs directly with regular Python.
    Use for scripts that don't need mjpython (handle main thread internally).

    Args:
        script_module: Module path (e.g., "RynnLeRobot.teleop.multi_teleop_mac")

    Returns:
        A wrapper function that can be used as a main entry point
    """
    def wrapper_main():
        run_with_regular_python(script_module)

    return wrapper_main


def is_mujoco_script(script_path):
    """
    Check if a script uses MuJoCo viewer functionality.

    Args:
        script_path: Path to the script file

    Returns:
        True if script contains MuJoCo viewer calls
    """
    try:
        with open(script_path, 'r') as f:
            content = f.read()
            return any(keyword in content for keyword in [
                'mujoco.viewer',
                'launch_passive',
                'MuJoCoInterface',
                'viewer.launch'
            ])
    except FileNotFoundError:
        return False


# =============================================================================
# Entry Point Wrapper Functions
# =============================================================================

joint_teleop_main = create_mujoco_wrapper("RynnLeRobot.controller.joint_teleop")
so101_motion_main = create_mujoco_wrapper("RynnLeRobot.controller.so101_motion")
lekiwi_motion_main = create_mujoco_wrapper("RynnLeRobot.controller.lekiwi_motion")
multi_teleop_main = create_mujoco_wrapper("RynnLeRobot.teleop.multi_teleop")
# multi_teleop_mac uses regular python (not mjpython) - handles main thread internally
multi_teleop_mac_main = create_direct_wrapper("RynnLeRobot.teleop.multi_teleop_mac")
lerobot_ui_main = create_mujoco_wrapper("RynnLeRobot.scripts.lerobot_UI")
lekiwi_ui_main = create_mujoco_wrapper("RynnLeRobot.scripts.lekiwi_UI")
so101_teleop_main = create_mujoco_wrapper("RynnLeRobot.teleop.so101_teleop")
so101_pickplace_main = create_mujoco_wrapper("RynnLeRobot.controller.so101_pickplace")
so101_floating_ee_main = create_mujoco_wrapper("RynnLeRobot.teleop.so101_floating_ee")


# Entry point functions for pyproject.toml
def joint_teleop():
    """Entry point for joint-teleop command."""
    joint_teleop_main()


def so101_motion():
    """Entry point for so101-motion command."""
    so101_motion_main()


def lekiwi_motion():
    """Entry point for lekiwi-motion command."""
    lekiwi_motion_main()


def multi_teleop():
    """Entry point for multi-teleop command."""
    multi_teleop_main()


def multi_teleop_mac():
    """Entry point for multi-teleop-mac command (optimized for macOS)."""
    multi_teleop_mac_main()


def lerobot_ui():
    """Entry point for lerobot-ui command."""
    lerobot_ui_main()


def lekiwi_ui():
    """Entry point for lekiwi-ui command."""
    lekiwi_ui_main()


def so101_teleop():
    """Entry point for unified so101-teleop command."""
    so101_teleop_main()


def so101_pickplace():
    """Entry point for so101-pickplace command."""
    so101_pickplace_main()


def so101_floating_ee():
    """Entry point for so101-floating-ee command."""
    so101_floating_ee_main()


# For direct execution testing
if __name__ == "__main__":
    if len(sys.argv) > 1:
        script_name = sys.argv[1]
        if script_name == "joint-teleop":
            joint_teleop()
        elif script_name == "multi-teleop":
            multi_teleop()
        elif script_name == "so101-teleop":
            so101_teleop()
        elif script_name == "ui":
            lerobot_ui()
        elif script_name == "lekiwi-ui":
            lekiwi_ui()
        else:
            print(f"Unknown script: {script_name}")
            sys.exit(1)
    else:
        print("Usage: python mjpython_wrapper.py [joint-teleop|multi-teleop|so101-teleop|ui|lekiwi-ui]")
        sys.exit(1)
