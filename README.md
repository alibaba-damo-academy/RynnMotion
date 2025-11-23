# RynnMotion

> A modern C++20 framework for robot manipulation with MuJoCo simulation and Pinocchio dynamics

<p align="center">
  <img src="https://img.shields.io/badge/license-Apache--2.0-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/C%2B%2B-20-blue" alt="C++20">
  <img src="https://img.shields.io/badge/Python-3.13%2B-blue" alt="Python 3.13+">
  <img src="https://img.shields.io/badge/version-0.9.0--pre-orange" alt="Version">
  <img src="https://img.shields.io/badge/MuJoCo-3.3.5-green" alt="MuJoCo 3.3.5">
  <img src="https://img.shields.io/badge/Pinocchio-3.7.0-green" alt="Pinocchio 3.7.0">
  <img src="https://img.shields.io/badge/ROS-optional-green" alt="ROS Optional">
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

## Why RynnMotion?

🚀 **Fast Robot Prototyping** - MJCF to deployable physics simulation in minutes
🤖 **Modern C++20 & Python** - High-performance C++ core with full Python bindings
📊 **Data Collection & Policy Training** - Built-in LeRobot integration for imitation learning
🏗️ **Core-Satellite Architecture** - Clean separation: shared algorithms vs robot-specific drivers
⚡ **ROS-Independent** - Zero ROS/MoveIt dependency, optional integration available
🎯 **Latest Stack** - MuJoCo 3.3.5 + Pinocchio 3.7.0 for best-in-class physics & dynamics
💻 **Cross-Platform** - Native support for Linux & macOS
🔧 **8+ Pre-configured Robots** - FR3, UR5e, Piper, dual-arm variants, drop in MJCF for more

---

## 🚀 Quick Start

### Option 1: Native Installation (Recommended - Production Ready)

```bash
# 1. Install dependencies (one-time, 30-60 min)
sudo ./scripts/setup_dependencies.sh  # Linux
# OR
./scripts/setup_dependencies.sh       # macOS (no sudo)

# 2. Build (3-5 min)
mkdir build && cd build
cmake ..
make -j$(nproc)

# 3. Run your first simulation
./mujocoExe fr3 1  # FR3 robot tracking demo 🎉
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
./mujocoExe fr3 1
```

**See detailed guide:** [Docker Setup](docs/DOCKER-SETUP.md) (experimental)

### Option 3: Python Package (Coming Dec 2025)

```bash
pip install rynnmotion  # Coming soon!
python -c "from RynnMotion import RynnMuJoCo; sim = RynnMuJoCo('fr3', 1); sim.run()"
```

---

## 📦 Installation

### Choose Your Installation Method

<table>
<tr>
<th>Method</th>
<th>Setup Time</th>
<th>Status</th>
<th>Best For</th>
</tr>
<tr>
<td><b>Native</b></td>
<td>30-60 min</td>
<td>✅ Tested & Production-Ready</td>
<td>Development, hardware deployment, performance</td>
</tr>
<tr>
<td><b>Docker</b></td>
<td>15-30 min</td>
<td>⚠️ Experimental (Untested)</td>
<td>Quick evaluation, consistent environments</td>
</tr>
<tr>
<td><b>Python Package</b></td>
<td>2 min</td>
<td>📅 Coming Q2 2025</td>
<td>Scripting, prototyping, Python-only workflows</td>
</tr>
</table>

<details>
<summary><b>💻 Native Installation (Recommended)</b></summary>

#### Prerequisites

- Ubuntu 22.04+ or macOS 13+
- 10 GB free disk space
- Internet connection

#### Quick Setup

```bash
# 1. Clone repository
git clone https://github.com/alibaba-damo-academy/RynnMotion.git
cd RynnMotion

# 2. Install dependencies (one-time, 30-60 min)
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

</details>

<details>
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
./mujocoExe fr3 1
```

#### Full Documentation

See [Docker Setup Guide](docs/DOCKER-SETUP.md) for:

- GUI setup (X11 forwarding)
- Hardware access (USB devices)
- GPU support (NVIDIA)
- Advanced configuration
- Troubleshooting

</details>

<details>
<summary><b>🐍 Python Package (Coming Soon)</b></summary>

```bash
# Future release (Q2 2025)
pip install rynnmotion
```

Python example:

```python
from RynnMotion import RynnMuJoCo

sim = RynnMuJoCo(robot="fr3", scene=1)
for _ in range(1000):
    sim.step()
    q, qd = sim.get_state()
    sim.set_command(q_des, qd_des)
```

</details>

---

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
./mujocoExe fr3 1  # FR3 robot figure-8 tracking
./mujocoExe ur5e 1  # UR5e robot tracking
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
./mujocoExe dual_fr3 1      # Dual FR3 tracking
./mujocoExe dual_ur5e 1     # Dual UR5e tracking
./mujocoExe piper 1         # Piper dual-arm robot
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

One SO101 master arm controlling 4 different slave robots (FR3, UR5e, Piper, RM75) simultaneously via universal joint mapping.

```bash
# Run multi-robot teleoperation
cd robots/RynnLeRobot
./setup_lerobot.sh && source .venv/bin/activate
multi-teleop -d
```

---

> 📺 **Video Tutorials:** Full walkthrough videos coming to our [YouTube channel](https://youtube.com/@RynnMotion) (coming soon)

---

## 📚 Examples & Tutorials

### 🚀 Getting Started

Start here if you're new to RynnMotion:

- **[Quick Start Guide](tutorials/quickstart.md)** - Get RynnMotion running in 5 minutes
  - Installation walkthrough
  - First simulation
  - Basic concepts
  - Troubleshooting

### 📖 Example Projects

Hands-on examples to learn RynnMotion features:

| Example                                                    | Description                     | Difficulty        | Time   |
| ---------------------------------------------------------- | ------------------------------- | ----------------- | ------ |
| [01_basic_simulation](examples/01_basic_simulation/)       | Joint position control with FR3 | ⭐ Beginner       | 5 min  |
| [02_operational_space](examples/02_operational_space/)     | End-effector tracking with OSC  | ⭐⭐ Intermediate | 10 min |
| [03_dual_arm_pickplace](examples/03_dual_arm_pickplace/)   | Dual FR3 pick-and-place         | ⭐⭐⭐ Advanced   | 15 min |
| [04_custom_robot](examples/04_custom_robot/)               | Add your own robot              | ⭐⭐ Intermediate | 20 min |
| [05_lerobot_integration](examples/05_lerobot_integration/) | Dataset collection for IL       | ⭐⭐⭐ Advanced   | 30 min |

**Each example includes:**

- Self-contained code (copy-paste runnable)
- Detailed explanations and comments
- Expected output screenshots/GIFs
- Learning objectives and next steps

### 📚 More Tutorials

Browse all tutorials in the [tutorials/](tutorials/) directory:

- **[Tutorial Index](tutorials/README.md)** - Complete list of guides
- Robot control tutorials (coming soon)
- Data collection workflows (coming soon)
- Advanced topics (coming soon)

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────┐
│         RuntimeData (Shared State)      │
│  qFb, bodyStates, jacobians, bodyPlans  │
└────────┬──────────┬──────────┬──────────┘
         │          │          │
    ┌────▼────┐ ┌───▼────┐ ┌──▼─────┐
    │Estimator│ │Planner │ │  OSC   │
    │   FK    │ │TrajGen │ │  IK    │
    └─────────┘ └────────┘ └────────┘
         Sequential Update: 1 → 2 → 3

    Estimator:  qFb         → bodyStates[], jacobians[]
    Planner:    bodyStates  → bodyPlans[], gripperCmd[]
    OSC:        bodyPlans   → qCmd, qdCmd, qtauCmd
```

**Key Concepts:**

- **Shared RuntimeData**: All modules read/write to single state struct
- **Sequential Processing**: Estimator → Planner → OSC (clear data flow)
- **Dependency Injection**: Managers passed as references (no globals)
- **initModule() Pattern**: One-time initialization after dependencies set

[Full Architecture Documentation →](docs/ARCHITECTURE.md)

---

## 📖 Documentation

### Getting Started

- **[Quick Start Guide](tutorials/quickstart.md)** - 5-minute setup and first simulation
- **[Tutorial Index](tutorials/README.md)** - Browse all tutorials
- **[Installation Guide](docs/installation.md)** - Comprehensive installation instructions
- **[Examples Directory](examples/)** - Hands-on example projects

### Technical Documentation

- **[Architecture Overview](docs/ARCHITECTURE.md)** - System design and module architecture
- **[Open Source Roadmap](docs/OPEN_SOURCE_ROADMAP.md)** - Development plans and milestones
- **[C++ Coding Style](docs/c++_coding_style.md)** - Code conventions (v2.2, Nov 2025)
- **[Adding Robots](models/how_to_add_new_robot_scene.md)** - Auto-discovery system guide
- **[Docker Setup](docs/DOCKER-SETUP.md)** - Container development environment (experimental)

### API Reference

- **[Python API](https://rynnmotion.readthedocs.io)** - Python bindings documentation
- **[C++ API](https://rynnmotion.readthedocs.io/cpp)** - C++ core library reference

### For Contributors

- **[Contributing Guide](CONTRIBUTING.md)** - How to contribute to RynnMotion
- **[AI Assistant Context](CLAUDE.md)** - Developer reference for Claude Code

---

## 📊 Project Status

**Current Version:** v0.9.0 (pre-release)

**Stability:**

- ✅ C++ Core: Stable (recent refactoring Nov 2025)
- ✅ Python Bindings: Stable
- ⚠️ ROS2 Integration: Experimental
- 🚧 Documentation: In progress

**Tested Platforms:**

- Ubuntu 22.04/24.04 (primary)
- macOS 13+ (experimental)

**Tested Robots:**

- Franka FR3 (sim + hardware)
- Universal Robots UR5e (sim + hardware)
- Piper (sim + hardware)
- RealMan RM75, SO101 (sim)
- Dual-arm variants (sim)

---

## 🗺️ Roadmap

### v1.0 (Target: Q2 2025)

- ✅ C++ core refactoring (Nov 2025)
- ✅ Docker development environment (Nov 2025)
- 🚧 Documentation suite
- 🚧 5 tutorial examples
- 📅 PyPI package

### v1.1 (Target: Q3 2025)

- ROS2 minimal bridge
- Model zoo (community robots)
- Benchmark suite (vs MoveIt/Drake)

### v2.0 (Target: 2026)

- Plugin system for custom modules
- GPU acceleration (batch FK/IK)
- Isaac Sim integration

[Full Roadmap →](docs/OPEN_SOURCE_ROADMAP.md)

---

## 🤝 Community

### Get Help

- [GitHub Discussions](https://github.com/alibaba-damo-academy/RynnMotion/discussions)
- [Discord Server](https://discord.gg/rynnmotion)
- [Issue Tracker](https://github.com/alibaba-damo-academy/RynnMotion/issues)

### Stay Updated

- [Mailing List](https://groups.google.com/g/rynnmotion)
- [Twitter/X](https://twitter.com/RynnMotion)
- [Blog](https://rynnmotion.dev/blog)

### Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for:

- Code style guidelines
- PR workflow
- Issue templates
- Development setup

**Good first issues** tagged on GitHub for newcomers.

---

## 📜 Citation

If you use RynnMotion in your research, please cite:

```bibtex
@software{rynnmotion2025,
  title = {RynnMotion: A Modern C++20 Framework for Robot Manipulation},
  author = {RynnMotion Team},
  year = {2025},
  url = {https://github.com/alibaba-damo-academy/RynnMotion},
  note = {Open-source robotics manipulation framework}
}
```

**Academic papers using RynnMotion:** (list updated quarterly)

- (Your paper could be here!)

---

## 📄 License

RynnMotion is released under the [Apache License 2.0](LICENSE).

```
Apache License 2.0 - Copyright (c) 2025 RynnMotion Team
Licensed under the Apache License, Version 2.0.
See LICENSE file for full details.
```

**Third-party licenses:**

- MuJoCo: Apache 2.0
- Pinocchio: BSD 2-Clause
- Eigen: MPL2
- Ruckig: MIT

---

## 🙏 Acknowledgments

RynnMotion is built with these excellent open-source projects:

- [**MuJoCo**](https://mujoco.org/) - Physics simulation engine
- [**Pinocchio**](https://github.com/stack-of-tasks/pinocchio) - Rigid body dynamics library
- [**Ruckig**](https://github.com/pantor/ruckig) - Trajectory generation with constraints
- [**LeRobot**](https://github.com/huggingface/lerobot) - Imitation learning toolkit
- [**Eigen**](https://eigen.tuxfamily.org/) - Linear algebra library
- [**yaml-cpp**](https://github.com/jbeder/yaml-cpp) - YAML parser

**Special thanks** to:

- Researchers providing feedback during development
- Contributors improving documentation and code
- Academic labs testing with real robots
- Open-source community for inspiration

---

## ❓ Frequently Asked Questions

### Why RynnMotion instead of MoveIt?

**MoveIt** is excellent for ROS-centric workflows and broad hardware support.

**RynnMotion** is better for:

- MuJoCo-based research (sim-to-real)
- Dual-arm manipulation
- Performance-critical applications (no ROS overhead)
- Custom control algorithms (clean C++ codebase)

**Use both!** RynnMotion + MoveIt can complement each other.

### Why RynnMotion instead of Drake?

**Drake** is a comprehensive robotics toolkit with optimization, planning, and control.

**RynnMotion** is better for:

- Lower learning curve (smaller, focused codebase)
- MuJoCo-first design (vs Drake's MultibodyPlant abstraction)
- Imitation learning pipelines (LeRobot integration)

**Use Drake if:** You need trajectory optimization, contact-implicit planning, or prefer Bazel.

### Does RynnMotion support ROS2?

**Currently:** Minimal ROS2 bridge (experimental)

**Roadmap:** Full ROS2 integration in v1.1 (Q3 2025)

**Philosophy:** ROS-optional design. RynnMotion works standalone, but can interop with ROS if needed.

### What robots are supported?

**Pre-configured (sim + some with hardware drivers):**

- Franka FR3
- Universal Robots UR5e
- Piper (松灵)
- RealMan RM75, SO101
- Rizon4s, ECO65, KHI
- Dual-arm variants for all

**Any robot with MJCF model** can be added in minutes via auto-discovery.

### Can I use this for real robots?

**Yes!** Hardware interfaces exist for:

- Franka (via libfranka)
- UR (via UR RTDE)
- Piper (via SDK)

**Sim-to-real workflow:**

1. Develop in MuJoCo simulation
2. Test control algorithms
3. Deploy to hardware with same codebase

**Note:** Always test carefully and use safety limits on real hardware.

---

## Star History

<!--
[![Star History Chart](https://api.star-history.com/svg?repos=alibaba-damo-academy/RynnMotion&type=Date)](https://star-history.com/#alibaba-damo-academy/RynnMotion&Date)
-->

**Help us grow!** If you find RynnMotion useful:

- ⭐ Star the repository
- 🐛 Report issues
- 💡 Suggest features
- 📝 Improve docs
- 🔧 Submit PRs

**Every contribution matters.** Thank you!

---

<p align="center">
  <sub>Built with ❤️ by the RynnMotion Team</sub><br>
  <sub>Making robot manipulation research accessible to everyone</sub>
</p>

---

## Related Projects

**Robotics Frameworks:**

- [MoveIt2](https://moveit.ros.org/) - ROS-based manipulation
- [Drake](https://drake.mit.edu/) - Model-based design and control
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) - Rigid body dynamics

**Simulation:**

- [MuJoCo](https://mujoco.org/) - Physics simulation
- [Isaac Sim](https://developer.nvidia.com/isaac-sim) - NVIDIA robotics platform
- [PyBullet](https://pybullet.org/) - Physics simulation

**Machine Learning:**

- [LeRobot](https://github.com/huggingface/lerobot) - Imitation learning
- [RoboSuite](https://robosuite.ai/) - Manipulation benchmarks
- [Robomimic](https://robomimic.github.io/) - Imitation learning framework

---

**RynnMotion**: Modern tools for the next generation of robotics research. 🚀
