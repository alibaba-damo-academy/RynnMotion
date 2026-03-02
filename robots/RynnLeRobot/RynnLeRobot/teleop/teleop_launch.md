# SO101 Teleoperation Launch Guide

## Quick Start

The SO101 teleoperation system provides a unified command-line interface for controlling dual-arm robots (FR3, UR5e, Piper, RM75) using the SO101 master robot.

## Supported Robots

| Name | Number | Robot | DOF |
|------|--------|-------|-----|
| `fr3` | `20` | Franka FR3 | 7+1 |
| `ur5e` | `21` | Universal Robots UR5e | 6+1 |
| `piper` | `22` | Piper | 6+1 |
| `rm75` | `23` | RealMan RM75 | 7+1 |

## Installation

```bash
cd /path/to/RynnMotion/robots/RynnLeRobot
source venv/bin/activate
pip install -e .
```

## Launch Commands

### Basic Usage

```bash
# Launch with robot name (recommended)
so101-teleop --robot fr3
so101-teleop --robot ur5e
so101-teleop --robot piper
so101-teleop --robot rm75

# Launch with robot number
so101-teleop --robot 20  # FR3
so101-teleop --robot 21  # UR5e
so101-teleop --robot 22  # Piper
so101-teleop --robot 23  # RM75
```

### Platform-Specific Launch

The system automatically detects your platform:
- **macOS**: Uses `mujoco.viewer` (passive viewer mode)
- **Linux**: Uses GLFW with manual rendering

#### macOS

```bash
# Auto-detected (default)
so101-teleop --robot fr3

# Explicitly force viewer mode
so101-teleop --robot fr3 --mode viewer
```

#### Linux

```bash
# Auto-detected (default)
so101-teleop --robot fr3

# Explicitly force GLFW mode
so101-teleop --robot fr3 --mode glfw
```

### Advanced Options

```bash
# Force specific rendering mode
so101-teleop --robot fr3 --mode viewer  # Force mujoco.viewer
so101-teleop --robot fr3 --mode glfw    # Force GLFW
so101-teleop --robot fr3 --mode auto    # Auto-detect (default)

# Custom configuration file
so101-teleop --robot fr3 --config /path/to/custom/so101.yaml

# Show help
so101-teleop --help
```

## Runtime Controls

### During Execution

- **R** - Reset simulation to standby position
- **ESC** - Exit application (Linux GLFW mode)
- **Close window** - Exit application (macOS viewer mode)

### Automatic Behavior

- Teleoperation starts automatically after initialization
- System lerps from current position to standby in 1 second
- Master-slave control begins immediately after standby reached

## Configuration

Default configuration file: `configs/so101.yaml`

Key configuration parameters:
- Master robot ports: `/dev/ttyACM1`, `/dev/ttyACM2`
- Calibration directories for dual arms
- Control frequency: 100 Hz
- Rendering frequency: 60 Hz (Linux GLFW)

## Examples

```bash
# Most common: Launch FR3 teleoperation
so101-teleop --robot fr3

# Launch UR5e with custom config
so101-teleop --robot ur5e --config ~/my_config.yaml

# Launch Piper in GLFW mode (testing on macOS)
so101-teleop --robot piper --mode glfw

# Launch RM75 by number
so101-teleop --robot 23
```

## Troubleshooting

### Command not found

```bash
# Reinstall package
cd /path/to/RynnMotion/robots/RynnLeRobot
source venv/bin/activate
pip install -e .
```

### Hardware connection issues

Check master robot connections:
- Arm 1: `/dev/ttyACM1` (default)
- Arm 2: `/dev/ttyACM2` (default)

Update ports in `configs/so101.yaml` if needed.

### Platform detection issues

Force specific mode if auto-detection fails:
```bash
so101-teleop --robot fr3 --mode viewer  # macOS
so101-teleop --robot fr3 --mode glfw    # Linux
```

## System Architecture

### Rendering Modes

**Viewer Mode (macOS)**:
- Uses `mujoco.viewer.launch_passive()`
- Main-thread GUI compliance
- Interactive camera control
- Right UI panel enabled

**GLFW Mode (Linux)**:
- Manual GLFW window management
- Multi-threaded (physics + rendering)
- Custom camera rendering
- Better performance control

### Control Loop

1. Initialize robot interfaces (master + slave)
2. Load MuJoCo scene for selected robot
3. Lerp to standby position (1 second)
4. Enter teleoperation loop at 100 Hz:
   - Read master joint positions
   - Compute slave commands via mapping
   - Apply commands to slave robots
   - Step simulation
   - Render visualization

---

## SO101 Floating End-Effector Mode

Control floating grippers in MuJoCo using SO101 master arm end-effector pose. This mode reads the EE position and orientation from two SO101 master arms and maps them directly to floating grippers in simulation.

### Usage

```bash
# Use different gripper types
so101-floating-ee --gripper robotiq   # Robotiq 2F85 gripper (default)
so101-floating-ee --gripper umi       # UMI gripper
so101-floating-ee --gripper openarm   # OpenArm gripper

# Custom configuration
so101-floating-ee --gripper robotiq --config /path/to/so101.yaml
```

### Supported Grippers

| Gripper | Description |
|---------|-------------|
| `robotiq` | Robotiq 2F85 parallel gripper |
| `umi` | UMI gripper with GoPro mount |
| `openarm` | OpenArm crank-slider gripper |

### Control Mapping

| Master SO101 | Floating Gripper |
|--------------|------------------|
| EE delta position | Gripper position (slide joints) |
| EE delta orientation | Gripper orientation (hinge joints) |
| Gripper (0-1) | Gripper actuator (0-1) |

### How It Works

1. **Home Pose Capture**: On startup, the system captures the current EE pose of both SO101 arms as the "home" reference
2. **Relative Motion**: During teleoperation, the system tracks the delta (change) from home position/orientation
3. **Workspace Mapping**: Delta motion is applied to gripper scene centers:
   - Arm 0 (left): centered at (0, 0, 0.3)
   - Arm 1 (right): centered at (0.25, 0, 0.3)
4. **Rotation**: Relative rotation from home is converted to euler angles (xyz intrinsic) for hinge joint control

### Notes

- **Gripper control**: Standardized 0=closed, 1=open
- **Dual-arm**: Two SO101 master arms control two floating grippers
- **FK computation**: Uses PinKine (Pinocchio-based) for forward kinematics
- **Quaternion format**: (x, y, z, w) - same as scipy and Eigen conventions
- **Position is relative**: Move arms from their current position to see gripper motion

---

**Project**: RynnMotion RynnLeRobot
**Version**: 0.1.0
**Last Updated**: 2025-12-21
