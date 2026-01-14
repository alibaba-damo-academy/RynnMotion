# RynnLeRobot

RynnMotion LeRobot Motion Control and Communication package.

## Overview

RynnLeRobot provides robot control interfaces, teleoperation scripts, and utilities for the LeRobot platform integrated with RynnMotion.

## Features

- Robot control interfaces for various robot arms (SO101, Piper, FR3, UR5e, RM75)
- Teleoperation applications with MuJoCo simulation
- Joint-level and end-effector based control strategies
- Multi-platform support (Linux/macOS with mjpython wrapper)
- CLI tools for calibration, recording, and testing

## Installation

Install in editable mode:

```bash
cd robots/RynnLeRobot
pip install -e .
```

## Teleoperation Commands

After installation, the following teleoperation commands are available:

### SO101 Teleoperation (Recommended)

```bash
# Launch by robot name (recommended)
so101-teleop --robot fr3
so101-teleop --robot ur5e
so101-teleop --robot piper
so101-teleop --robot rm75

# Launch by robot number
so101-teleop --robot 20  # FR3
so101-teleop --robot 21  # UR5e
so101-teleop --robot 22  # Piper
so101-teleop --robot 23  # RM75
```

### Multi-Robot Teleoperation

```bash
# Control multiple slave robots simultaneously
multi-teleop --dual --slave_numbers 20,21,22,23
```

### Other Commands

- `joint-teleop` - Generic joint-level teleoperation
- `lerobot-ui` - LeRobot UI application
- `lekiwi-ui` - LeKiwi UI application
- `so101-pickplace` - SO101 pick-and-place controller

## Supported Robots

| Name | Number | Robot | DOF |
|------|--------|-------|-----|
| `fr3` | `20` | Franka FR3 | 7+1 |
| `ur5e` | `21` | Universal Robots UR5e | 6+1 |
| `piper` | `22` | Piper | 6+1 |
| `rm75` | `23` | RealMan RM75 | 7+1 |
| `so101` | `24` | SO101 (Master) | 5+1 |

## Configuration

Default configuration: `configs/so101.yaml`

Key settings:
- Master robot ports: `/dev/ttyACM1`, `/dev/ttyACM2`
- Control frequency: 100 Hz
- Rendering frequency: 60 Hz

## Development

The package uses setuptools for building and follows the standard Python package structure.

## License

See the main RynnMotion repository for license information.
