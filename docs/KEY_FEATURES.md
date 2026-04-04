# RynnMotion: Key Features

> A framework bridging fast robot prototyping, motion primitives, heterogeneous teleoperation, and data collection across diverse robot platforms.

---

## Highlights

| # | Feature | One-Line Summary |
|---|---------|------------------|
| 1 | **Unified Multi-Morphology Control Interface** | Single-arm, dual-arm, wheeled, quadruped, humanoid, dexterous hand — all morphologies share one control API |
| 2 | **Core-Satellite Architecture** | Algorithms and simulation decoupled from real hardware; seamless sim-to-real transfer with zero ROS dependency |
| 3 | **Native Multi-Modal Data Integration** | One-click C++/Python generation of Parquet/HDF5 multi-modal datasets, natively compatible with LeRobot training pipeline |
| 4 | **Unified Heterogeneous Teleoperation Framework** | Low-cost master arm controls diverse slave arms and end-effectors with pose scaling, action mapping, and real-robot deployment |

---

## Feature 1: Unified Multi-Morphology Control Interface

**One API for all robot morphologies** — single-arm, dual-arm, wheeled, quadruped, humanoid, and dexterous hands share the same control interface.

### Supported Robots

| Robot | Type | DOF | Use Case |
|-------|------|-----|----------|
| Franka FR3 | Industrial | 7 | Research, precision tasks |
| Universal UR5e | Industrial | 6 | Manufacturing, automation |
| Piper | Dual-arm | 6×2 | Bimanual manipulation |
| RealMan RM75 | Collaborative | 7 | Human-robot interaction |
| SO101 (LeRobot) | Desktop | 6 | Education, teleoperation |
| + Rizon4S, ECO65, DM6-DOF | Various | - | Custom applications |

### Usage Example

```python
from RynnMotion.manager import PRobotManager

# Initialize any robot by name or ID
robot = PRobotManager("fr3")      # or PRobotManager(20)
robot = PRobotManager("ur5e")     # or PRobotManager(21)
robot = PRobotManager("so101")    # or PRobotManager(24)

# Same API for all robots
robot.get_joint_positions()
robot.set_joint_positions(target_q)
robot.get_ee_pose()
```

```bash
# Run simulation with any robot
./build/mujocoExe fr3 ui       # Franka FR3
./build/mujocoExe ur5e ui      # UR5e
./build/mujocoExe piper ui     # Piper dual-arm
```

### Why It Matters
- **Add new robot in minutes**: Drop MJCF model + register in `config/robot.yaml`
- **Auto-extracts**: Joint limits, actuator gains, keyframes from MJCF
- **Dual-arm ready**: `dual_fr3`, `dual_ur5e`, `dual_piper` variants included

---

## Feature 2: Core-Satellite Architecture

**Algorithms and simulation decoupled from real hardware** — seamless sim-to-real transfer with zero ROS dependency, enabling lightweight algorithm development and rapid real-robot deployment.

### Architecture

```
RynnMotion/
├── python/                    # CORE: Algorithms, simulation, datasets
│   └── src/RynnMotion/       # Hardware-agnostic, minimal dependencies
│
└── robots/                    # SATELLITES: Robot-specific implementations
    ├── RynnLeRobot/          # LeRobot hardware (own venv + SDK)
    ├── franka/               # Franka hardware (own venv + SDK)
    ├── piper/                # Piper hardware (own venv + SDK)
    └── realman/              # RealMan hardware (own venv + SDK)
```

### Key Principles

1. **Unidirectional dependency**: Satellites → Core (never reverse)
2. **Isolated environments**: Each robot has its own venv with hardware SDKs
3. **Editable install**: Satellites install core via `pip install -e ../../python/`

### Usage Example

```bash
# Development in simulation (Core only)
cd python/
pip install -e ".[dev]"
python scripts/pinkine_viewer.py --robot fr3

# Deployment to real hardware (Satellite)
cd robots/RynnLeRobot/
./setup_env.sh && source venv/bin/activate
python scripts/run_lerobot.py --robot so101 --mode record
```

### Why It Matters
- **No dependency conflicts**: PyTorch CPU vs CUDA, different SDK versions
- **Deploy without touching core**: Hardware changes stay in satellites
- **Scale to 50+ robots**: Each isolated, independently versioned

---

## Feature 3: Native Multi-Modal Data Integration

**One-click C++/Python generation of Parquet/HDF5 multi-modal datasets** — physics states, sensor data, and video natively compatible with LeRobot training pipeline.

### Supported Formats

| Format | Best For | Dependency |
|--------|----------|------------|
| **Parquet** (default) | ML training, LeRobot | `libarrow-dev` |
| **HDF5** | Scientific computing | `libhdf5-dev` |
| **None** | Video-only recording | - |

### Configuration

```yaml
# config/mujoco.yaml
recorder:
  data_format: "parquet"    # auto-detect available format
  record_video: true
  video_codec: "h264"
  crf: 23                   # video quality (lower = better)
```

### Usage Example

```bash
# Run simulation and record
./build/mujocoExe fr3 ui

# Keyboard controls:
#   R - Start/Stop recording
#   N - New episode (auto-saves current)
#   ESC - Exit (auto-saves)
```

### Recorded Data

| Feature | Shape | Description |
|---------|-------|-------------|
| `observation.state` | (mdof,) | Joint positions |
| `observation.velocity` | (mdof,) | Joint velocities |
| `observation.ee_pos` | (num_ee×3,) | End-effector positions |
| `observation.ee_quat` | (num_ee×4,) | End-effector orientations |
| `action` | (mdof,) | Commanded positions |
| Video | Multi-camera | H.264/AV1 encoded |

### Output Structure

```
record/mj_20241209_1430/
├── meta/
│   ├── info.json           # Dataset metadata
│   └── episodes.jsonl      # Episode index
├── data/chunk-000/
│   └── episode_000000.parquet
└── videos/chunk-000/
    └── episode_000000/
        └── camera_0.mp4
```

### Load in Python

```python
from RynnMotion.RynnDatasets import RynnDataset

dataset = RynnDataset(repo_id="mj_20241209_1430", root="./record")
sample = dataset[0]
print(sample["observation.state"])  # Joint positions
```

### Why It Matters
- **No ROS required**: Standalone C++/Python, zero middleware overhead
- **LeRobot-compatible**: Train directly without format conversion
- **Multi-modal**: Synchronized joints + EE poses + multi-camera video

---

## Feature 4: Unified Heterogeneous Teleoperation Framework

**Low-cost master arm controls diverse slave arms and end-effectors** — with end-effector pose scaling, action mapping, and seamless real-robot deployment.

### Capability

```
SO101 Master (6-DOF desktop arm)
    │
    ├──► FR3 Slave (7-DOF industrial)
    ├──► UR5e Slave (6-DOF industrial)
    ├──► Piper Slave (6-DOF dual-arm)
    └──► RM75 Slave (7-DOF collaborative)
```

### Usage Example

```bash
cd robots/RynnLeRobot
./setup_env.sh && source venv/bin/activate

# Launch multi-robot teleoperation
multi-teleop -d  # Default: SO101 master → 4 slaves
```

### How It Works

**PoseMapper** handles heterogeneous kinematics:

```python
from RynnMotion.algorithms import PoseMapper

# Automatic workspace scaling between different arm lengths
mapper = PoseMapper(
    arm1L=[master_shoulder, master_elbow, master_wrist],
    arm2L=[slave_shoulder, slave_elbow, slave_wrist]
)

# Map master pose to slave (handles orientation bias too)
slave_pose = mapper.pose_mapping(master_ee_pose, master_kin, slave_kin)
```

### Key Components

1. **Workspace Scaling**: Adapts commands based on arm length ratios
2. **Orientation Compensation**: Handles different robot base orientations
3. **Real-time Control**: 100 Hz control loop, 500 Hz physics
4. **Multi-camera Rendering**: 60 Hz visualization

### Why It Matters
- **True heterogeneous control**: Not just multi-instance of same robot
- **Intelligent mapping**: Automatically handles kinematic differences
- **Scalable**: Add more slaves without code changes

---

## Summary: What Makes RynnMotion Different

| Feature | Traditional Approach | RynnMotion |
|---------|---------------------|------------|
| Multi-robot | Separate codebases | Unified interface |
| Architecture | Monolithic | Core-Satellite (plugin-style) |
| Data recording | ROS bags | Standalone Parquet/HDF5 |
| Teleoperation | Same robot type | Heterogeneous (different robots) |

**Target users**: Robotics researchers, embodied AI developers, and anyone who needs fast prototyping across multiple robot platforms without the ROS learning curve.

---

## Learn More

- [Quick Start](../README.md#-quick-start)
- [Architecture Details](./core-satellite-architecture.md)
- [Data Recording Guide](./how_to_record_datasets_c++.md)
- [Installation](./installation.md)
