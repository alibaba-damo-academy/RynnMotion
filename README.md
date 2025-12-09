# RynnMotion

> A framework aiming to bridge fast robot prototyping, predefined motion promitives, heterogeneous teleoperation, data collection, and flexible deployment across diverse robot platforms.

<p align="center">
  <img src="https://img.shields.io/badge/license-Apache--2.0-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/C%2B%2B-20-blue" alt="C++20">
  <img src="https://img.shields.io/badge/Python-3.13%2B-blue" alt="Python 3.13+">
  <img src="https://img.shields.io/badge/version-0.9.0--pre-orange" alt="Version">
  <img src="https://img.shields.io/badge/MuJoCo-3.4.0-green" alt="MuJoCo 3.4.0">
  <img src="https://img.shields.io/badge/Pinocchio-3.7.0-green" alt="Pinocchio 3.7.0">
</p>

<p align="center">
  <a href="#-quick-start">Quick Start</a> •
  <a href="#-key-features">Features</a> •
  <a href="#-examples--tutorials">Examples</a> •
  <a href="docs/ARCHITECTURE.md">Architecture</a> •
  <a href="#-documentation">Docs</a> •
  <a href="#-community">Community</a>
</p>

---

## 🚀 Quick Start

### Option 1: Native Installation (Recommended - Production Ready)

```bash
# 1. Install dependencies (one-time, 30-60 min)
sudo ./scripts/setup_dependencies.sh  # Linux
# OR
./scripts/setup_dependencies.sh       # macOS (no sudo)

# 2. Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# 3. Run your first simulation
./mujocoExe fr3 ui
./mujocoExe ur5e pickplace
```

**See detailed guide:** [Installation Documentation](docs/installation.md)

### Option 2: Docker (Experimental - Untested)

> ⚠️ **Note:** Docker setup has not been fully tested. For production use, please use native installation above.

```bash
# Clone repository
git clone https://github.com/alibaba-damo-academy/RynnMotion.git
cd RynnMotion

# Build and run Docker container
./scripts/docker-build.sh
./scripts/docker-run.sh

# Inside container: build
mkdir build && cd build
cmake .. && make -j$(nproc)
./mujocoExe fr3 ui
```

**See detailed guide:** [Docker Setup](docs/DOCKER-SETUP.md) (experimental)

---

#### Prerequisites

- Ubuntu 22.04+ or macOS 13+
- Internet connection

#### Quick Setup

```bash
# 1. Clone repository
git clone https://github.com/alibaba-damo-academy/RynnMotion.git
cd RynnMotion

# 2. Install dependencies
sudo ./scripts/setup_dependencies.sh  # Linux
# OR
./scripts/setup_dependencies.sh       # macOS (no sudo)

# 3. Build RynnMotion
mkdir build && cd build
cmake ..
make -j$(nproc)

# 4. Install Python package (optional)
cd ../python
pip install -e .

# 5. Test it works!
./build/mujocoExe fr3 1
```

#### What Gets Installed

**Dependencies:** Eigen (≥3.3.0), Boost (≥1.65), Pinocchio (v3.7.0), MuJoCo (3.3.5), yaml-cpp, qpOASES, LCM, FCL, Ruckig (v0.15.3), OpenCV, nlohmann/json

**Installation locations:** `/usr/local/` (Linux/macOS)

#### Troubleshooting

See comprehensive [Installation Guide](docs/installation.md) for:

- Platform-specific instructions
- Manual dependency installation
- Common errors and solutions
- Verification steps

<summary><b>🐳 Docker Installation (Experimental)</b></summary>

> ⚠️ **Experimental Status:** Docker setup has not been fully tested. For production use, please use native installation.

#### Quick Setup

```bash
# 1. Build Docker image
./scripts/docker-build.sh

# 2. Run container
./scripts/docker-run.sh

# 3. Build inside container
mkdir build && cd build
cmake .. && make -j$(nproc)

# 4. Test
./mujocoExe fr3 ui
```

## 🎬 Demos & Features

### 1. Single-Arm Robot Control

**OSC-based trajectory tracking** with real-time visualization:

<p align="center">
  <img src="docs/images/singlearm_UI.webp" alt="Single-Arm Robot Control" width="100%">
</p>

High-performance operational space control with MuJoCo simulation. Track complex trajectories with sub-millimeter precision.

```bash
# Run single-arm demo
cd build
./mujocoExe fr3 ui       # FR3 robot with interactive UI
./mujocoExe ur5e ui      # UR5e robot with interactive UI
```

---

### 2. Dual-Arm Coordination

**Synchronized dual-arm control** with independent OSC controllers:

<p align="center">
  <img src="docs/images/dualarm_UI.webp" alt="Dual-Arm Coordination" width="100%">
</p>

Coordinate two robot arms seamlessly. Each arm runs independent OSC with nullspace optimization for redundancy resolution.

```bash
# Run dual-arm demo
./mujocoExe dual_fr3 ui      # Dual FR3 with interactive UI
./mujocoExe dual_ur5e ui     # Dual UR5e with interactive UI
./mujocoExe piper ui         # Piper with interactive UI
```

---

### 3. Pick-and-Place Automation

**State machine-based object manipulation** with dual-arm robots:

<p align="center">
  <img src="docs/images/pickplace.webp" alt="Pick-and-Place Demo" width="100%">
</p>

Fully automated pick-and-place pipeline with visual feedback. FSM handles approach, grasp, transfer, and release phases.

```bash
# Run pick-and-place demo
./mujocoExe dual_fr3 pickplace
```

---

### 4. Multi-Robot Teleoperation

**1 Master → 4 Slave Robots** in real-time teleoperation:

<p align="center">
  <img src="docs/images/hto.webp" alt="Multi-Robot Teleoperation" width="100%">
</p>

One SO101 master arm controlling 4 different slave robots (FR3, UR5e, Piper, RM75) simultaneously.

```bash
cd robots/RynnLeRobot
./setup_env.sh && source .venv/bin/activate
multi-teleop -d
```

---

### 5. Data Recording

**Record simulation data** for imitation learning and analysis:

Record joint states, end-effector poses, actions, and video during simulation. Data is saved in LeRobot-compatible format for training.

#### Configuration

Edit `config/mujoco.yaml`:

```yaml
recorder:
  data_format: "auto"      # "auto", "parquet", "hdf5", or "none"
  record_video: true       # Record camera video
  video_codec: "h264"      # "h264" or "av1"
  crf: 23                  # Video quality (lower = better, 18-28 typical)
  chunks_size: 1000        # Frames per data chunk
```

| Format | Description | Dependency |
|--------|-------------|------------|
| `auto` | Auto-detect best available (Parquet → HDF5 → None) | - |
| `parquet` | Apache Parquet (recommended for ML) | libarrow-dev, libparquet-dev |
| `hdf5` | HDF5 format | libhdf5-dev |
| `none` | Disable data recording (video only if enabled) | - |

#### Recording Data

```bash
# Run simulation - press 'R' to start/stop recording
./build/mujocoExe fr3 ui

# Keyboard controls:
#   R - Start/Stop recording episode
#   N - Start new episode (auto-saves current)
#   ESC - Exit (auto-saves if recording)
```

#### Output Location

Data is saved to `record/` directory:

```
record/
└── mj_YYYYMMDD_HHMM/           # Session folder (auto-generated)
    ├── meta/
    │   ├── info.json           # Dataset metadata
    │   ├── episodes.jsonl      # Episode index
    │   └── tasks.jsonl         # Task descriptions
    ├── data/
    │   ├── chunk-000/
    │   │   └── episode_000000.parquet  # Joint states, actions, timestamps
    │   └── chunk-001/
    │       └── ...
    └── videos/
        ├── chunk-000/
        │   └── episode_000000/
        │       └── camera_0.mp4        # Camera recordings
        └── ...
```

#### Recorded Features

| Feature | Shape | Description |
|---------|-------|-------------|
| `observation.state` | (mdof,) | Joint positions |
| `observation.velocity` | (mdof,) | Joint velocities |
| `action` | (mdof,) | Commanded joint positions |
| `observation.ee_pos` | (num_ee * 3,) | End-effector positions |
| `observation.ee_quat` | (num_ee * 4,) | End-effector orientations |
| `timestamp` | scalar | Simulation time |
| `frame_index` | scalar | Frame number in episode |

#### Optional Dependencies

Parquet support requires Apache Arrow (auto-detected at build time):

```bash
# Ubuntu/Debian
wget -q https://apache.jfrog.io/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb -O /tmp/arrow.deb
sudo apt install -y /tmp/arrow.deb && sudo apt update
sudo apt install -y libarrow-dev libparquet-dev

# macOS
brew install apache-arrow

# Verify detection
cd build && cmake .. | grep -i parquet
# Output: -- Parquet recording support: ENABLED (auto-detected)
```

---
