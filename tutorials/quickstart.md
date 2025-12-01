# Quick Start Guide

Get RynnMotion running in 5 minutes! This guide walks you through installation, building, and running your first robot simulation.

## Prerequisites

- **Linux**: Ubuntu 22.04+ or similar
- **macOS**: macOS 13+ (Ventura or later)
- **Tools**: Git, CMake (≥3.16), GCC/Clang with C++20 support
- **Disk Space**: ~3GB for dependencies + build artifacts

## Step 1: Clone the Repository

```bash
git clone https://github.com/alibaba-damo-academy/RynnMotion.git
cd RynnMotion
```

## Step 2: Install Dependencies

This one-time setup installs all required libraries (Eigen, Pinocchio, MuJoCo, etc.):

### Linux (Ubuntu/Debian)

```bash
# Run with sudo (takes 30-60 minutes)
sudo ./scripts/setup_dependencies.sh
```

### macOS

```bash
# Run WITHOUT sudo (takes 30-60 minutes)
./scripts/setup_dependencies.sh
```

**What gets installed?**
- Eigen 3.3.0+ (linear algebra)
- Pinocchio 3.7.0 (rigid body dynamics)
- MuJoCo 3.3.5 (physics simulation)
- Boost, yaml-cpp, qpOASES, Ruckig, and more

**Troubleshooting:** If the script fails, check the [Installation Guide](../docs/installation.md) for manual steps.

## Step 3: Build RynnMotion

```bash
# Create build directory
mkdir -p build && cd build

# Configure (generates Makefiles)
cmake ..

# Build (3-5 minutes on modern hardware)
make -j$(nproc)
```

**Build options:**
- `cmake -DCMAKE_BUILD_TYPE=Release ..` - Optimized build (default)
- `cmake -DCMAKE_BUILD_TYPE=Debug ..` - Debug symbols
- `cmake -DBUILD_TESTS=ON ..` - Include unit tests

## Step 4: Run Your First Simulation

```bash
# From the build/ directory
./mujocoExe fr3 ui
```

**What you'll see:**
- MuJoCo viewer window opens
- FR3 robot arm loads in a tracking scene
- Robot smoothly follows a figure-8 trajectory
- Press `Space` to pause, `Tab` to cycle cameras

**Try other robots:**
```bash
./mujocoExe ur5e ui             # Universal Robots UR5e
./mujocoExe piper ui            # AgileX Piper
./mujocoExe dual_fr3 pickplace  # Dual-arm pick-and-place
```

## Step 5: Explore the Python Interface

```bash
# Install Python package (from repo root, not build/)
cd ../python
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"

# Test it out
python -c "from RynnMotion import RynnMuJoCo; sim = RynnMuJoCo('fr3', 1); print('✓ Python interface working!')"
```

## Understanding the Basics

### Available Robots

RynnMotion includes 8+ pre-configured robots:

| Robot | Description | DOF | Command |
|-------|-------------|-----|---------|
| `fr3` | Franka Emika FR3 | 7 | `./mujocoExe fr3 ui` |
| `ur5e` | Universal Robots UR5e | 6 | `./mujocoExe ur5e ui` |
| `piper` | AgileX Piper | 6 | `./mujocoExe piper ui` |
| `rm75` | RealMan RM75 | 7 | `./mujocoExe rm75 ui` |
| `so101` | SoArm SO101 | 5 | `./mujocoExe so101 ui` |
| `dual_fr3` | Dual FR3 arms | 14 | `./mujocoExe dual_fr3 ui` |
| `dual_ur5e` | Dual UR5e arms | 12 | `./mujocoExe dual_ur5e ui` |

### Scene Names

Each robot supports multiple demo scenes (use names, not numbers):

| # | Scene | Aliases | Description |
|---|-------|---------|-------------|
| 1 | `joint` | default | Joint-space control (JointMove) |
| 2 | `keyframe` | wobble, cycle | Keyframe cycling demo |
| 3 | `ui` | tracking | Interactive UI with OSC tracking |
| 4 | `predefined` | workspace | Predefined workspace motions |
| 5 | `pickplace` | pick | Pick-and-place automation |

### Keyboard Controls

While the simulation is running:

- `Space` - Pause/resume
- `Tab` - Cycle through cameras
- `Ctrl+R` - Reset simulation
- `Ctrl+Q` - Quit
- `F5` - Toggle wireframe
- `F6` - Toggle contact forces

### Modifying Control Parameters

All control parameters are in YAML files - no recompilation needed!

```bash
# Edit OSC gains, IK solver settings, constraints
nano ../config/motion.yaml

# Edit robot-specific overrides
nano ../config/robot.yaml

# Edit simulation timestep, rendering options
nano ../config/mujoco.yaml
```

Changes take effect when you restart the simulation.

## Next Steps

### 🤖 Add Your Own Robot

1. Drop MJCF files into `models/3.robot_arm/{NUMBER}.{robot_name}/mjcf/`
2. Rebuild: `cmake ..` (auto-generates robot enums)
3. Run: `./mujocoExe {robot_name} ui`

See [Adding New Robots](../docs/how_to_add_new_robot_scene.md) for details.

### 📊 Collect Datasets for Imitation Learning

```bash
# Install LeRobot integration
cd ../robots/RynnLeRobot
python3 -m venv venv
source venv/bin/activate
pip install -e .

# Record teleoperation data
python scripts/run_lerobot.py --robot so101 --mode record
```

See [LeRobot Integration Guide](../robots/RynnLeRobot/README.md).

### 🎮 Write Custom Controllers

Create a new module in `motion/module/mycontroller/`:

```cpp
#include "motion/module/module_base.hpp"

class CMyController : public rynn::CModuleBase {
public:
    void initModule() override {
        // One-time initialization
        control_vec_.resize(robotManager_->getMotionDOF());
    }

    void update() override {
        // Read from runtimeData_
        const auto& q = runtimeData_->qFb;

        // Your control logic here
        control_vec_ = computeControl(q);

        // Write to runtimeData_
        runtimeData_->qCmd = control_vec_;
    }
};
```

Register in `ModuleManager::createModuleByType()` and add to `config/motion.yaml`.

### 📚 Dive Deeper

- **[Architecture Guide](../docs/ARCHITECTURE.md)** - Understand the shared RuntimeData pattern
- **[C++ Coding Style](../docs/c++_coding_style.md)** - Code conventions (v2.2, Nov 2025)
- **[CLAUDE.md](../CLAUDE.md)** - Complete developer reference

## Troubleshooting

### Build fails with "Eigen not found"

```bash
# Verify Eigen installation
pkg-config --modversion eigen3

# If missing, reinstall dependencies
sudo ./scripts/setup_dependencies.sh  # Linux
./scripts/setup_dependencies.sh       # macOS
```

### MuJoCo viewer doesn't open

**Linux:**
```bash
# Install OpenGL libraries
sudo apt install libglew-dev libglfw3-dev

# Check DISPLAY variable
echo $DISPLAY  # Should show :0 or :1
```

**macOS:**
```bash
# Ensure XQuartz is NOT running (native OpenGL preferred)
pkill XQuartz
```

### Python import fails

```bash
# Rebuild and reinstall Python package
cd python
pip install -e ".[dev]" --force-reinstall

# Verify installation
python -c "import RynnMotion; print(RynnMotion.__version__)"
```

### Still stuck?

- Check [Installation Guide](../docs/installation.md) for detailed troubleshooting
- Open an issue on [GitHub](https://github.com/alibaba-damo-academy/RynnMotion/issues)
- See [CLAUDE.md](../CLAUDE.md) for common development issues

## Summary

You've successfully:
- ✅ Installed RynnMotion and all dependencies
- ✅ Built the C++ libraries
- ✅ Run your first robot simulation
- ✅ Explored different robots and scenes
- ✅ Installed the Python interface

**Ready to build amazing robot applications!** 🚀

Check out the [tutorials index](README.md) for more guides on advanced topics like custom controllers, multi-robot coordination, and imitation learning.

## License

Apache License 2.0 - See [LICENSE](../LICENSE) for details.
