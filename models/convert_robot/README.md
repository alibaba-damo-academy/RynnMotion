# Robot Package to RynnMotion MJCF Converter

Self-contained tool to convert ROS URDF packages or MuJoCo Menagerie robots to RynnMotion's modular MJCF layout.

## Quick Start

```bash
cd models/convert_robot

# Convert ROS URDF package
./convert.sh --urdf ~/catkin_ws/src/myrobot_description \
    --output ../3.robot_arm/30.myrobot

# Convert MuJoCo Menagerie robot
./convert.sh --menagerie ~/mujoco_menagerie/universal_robots_ur5e \
    --output ../3.robot_arm/31.ur5e

# With dual-arm variant
./convert.sh --urdf ./robot.urdf --output ../3.robot_arm/32.robot --dual
```

## Installation

```bash
pip install -r requirements.txt
```

Required:
- `mujoco>=3.0.0` - MuJoCo Python bindings

Optional:
- `xacrodoc>=0.1.0` - For xacro to URDF conversion (only needed for .xacro files)

## Usage

```
python convert_robot.py --help

usage: convert_robot.py [-h] (--urdf PATH | --menagerie PATH) --output PATH
                       [--robot-name NAME] [--ee-link LINK]
                       [--gripper-joints JOINT [JOINT ...]]
                       [--home-qpos Q [Q ...]] [--standby-qpos Q [Q ...]]
                       [--dual] [--dual-separation M] [--dual-output PATH]
                       [--validate] [--verbose]
```

### Options

| Option | Description |
|--------|-------------|
| `--urdf PATH` | Path to ROS URDF package or .urdf file |
| `--menagerie PATH` | Path to MuJoCo Menagerie robot directory |
| `--output PATH` | Output directory (e.g., `../3.robot_arm/30.myrobot`) |
| `--robot-name NAME` | Override robot name |
| `--ee-link LINK` | End-effector link name (default: auto-detect) |
| `--gripper-joints` | Gripper joint names (default: auto-detect) |
| `--home-qpos` | Home position joint values |
| `--standby-qpos` | Standby position joint values |
| `--dual` | Also generate dual-arm variant |
| `--dual-separation M` | Arm separation for dual-arm (default: 0.5m) |
| `--validate` | Validate output with MuJoCo |

## Output Structure

The converter generates RynnMotion's modular MJCF layout:

```
output_dir/
├── assets/           # Mesh files (OBJ/STL)
├── mjcf/
│   ├── robot_robot.xml      # Main model (includes others)
│   ├── robot_pinocchio.xml  # Kinematics-only (no meshes/grippers)
│   ├── robot_defaults.xml   # Class definitions
│   ├── robot_assets.xml     # Mesh + material registry
│   ├── robot_actuators.xml  # Position controllers + gripper
│   ├── robot_sensors.xml    # EE + joint feedback sensors
│   ├── robot_links.xml      # Full kinematic tree
│   └── robot_contacts.xml   # Self-collision exclusions
└── scene/
    └── scene.xml            # Basic simulation scene
```

## Auto-Detection

### Gripper Joints
Detected by name patterns: `finger`, `gripper`, `grip`, `hand`, `jaw`, `claw`

### End-Effector
Detected by link name patterns: `ee`, `tool`, `flange`, `tcp`, `end_effector`

If auto-detection fails, use `--ee-link` and `--gripper-joints` to specify manually.

## Examples

### Convert UR5e from Menagerie
```bash
./convert.sh --menagerie ~/mujoco_menagerie/universal_robots_ur5e \
    --output ../3.robot_arm/31.ur5e \
    --robot-name ur5e
```

### Convert custom URDF with manual config
```bash
./convert.sh --urdf ./my_arm.urdf \
    --output ../3.robot_arm/35.myarm \
    --robot-name myarm \
    --ee-link tool0 \
    --gripper-joints finger_left finger_right \
    --standby-qpos 0 -1.57 1.57 0 1.57 0
```

### Generate dual-arm variant
```bash
./convert.sh --urdf ~/robot_ws/src/fr3_description \
    --output ../3.robot_arm/36.fr3_custom \
    --dual \
    --dual-separation 0.6
```

## After Conversion

1. Rebuild CMake to trigger auto-discovery:
   ```bash
   cd ../../build && cmake .. && make
   ```

2. Run simulation:
   ```bash
   ./mujocoExe robot_name 1
   ```
