# MuJoCo Simulation

MuJoCo physics simulation interface for RynnMotion.

## Architecture

```
InterfaceBase (abstract)
    │
    └── MujocoInterface
            │
            ├── MujocoActuator   # Joint command/feedback
            ├── MujocoSensor     # Sensor data collection
            ├── MujocoScene      # Scene & object management
            └── MujocoUI         # Keyboard/mouse control
            │
            └── RosMujocoInterface (ROS 2 wrapper)
```

## Components

| Component | Responsibility |
|-----------|----------------|
| **MujocoInterface** | Main simulation loop, physics stepping, rendering |
| **MujocoActuator** | qCmd/qFb ↔ mjData->ctrl mapping, PID control |
| **MujocoSensor** | Read sensors → RuntimeData (EE pose, F/T, IMU) |
| **MujocoScene** | Object tracking, camera updates, keyframe init |
| **MujocoUI** | Keyboard shortcuts, mouse control, overlay text |
| **RosMujocoInterface** | ROS 2 topics: /joint_states, /clock, /cmd_vel |

## Control Flow

```
getJointFeedbacks()          # mjActuator/Sensor/Scene → RuntimeData
    ↓
moduleManager->updateAll()   # Planner → OSC → Commands
    ↓
setJointCommands()           # RuntimeData → mjData->ctrl
    ↓
mj_step()                    # Physics integration
```

## Usage

```bash
./mujocoExe <robot> <scene>

# Examples
./mujocoExe fr3 ui           # FR3 with tracking UI
./mujocoExe piper pickplace  # Piper pick-and-place
./mujocoExe --help           # List all robots
```

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Space | Pause/Resume |
| Backspace | Reset |
| Tab | Toggle left panel |
| [ ] | Cycle cameras |
| T | Transparent mode |
| F1-F7 | Debug overlays |

## Files

| File | Description |
|------|-------------|
| `mj_interface.cpp` | Main MuJoCo implementation |
| `mj_actuator.cpp` | Actuator mapping & control |
| `mj_sensor.cpp` | Sensor reading |
| `mj_scene.cpp` | Scene management |
| `mj_ui.cpp` | UI & keyboard handling |
| `ros_mj_interface.cpp` | ROS 2 integration |
