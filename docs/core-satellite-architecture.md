# Core-Satellite Architecture Pattern for RynnMotion

## Overview

RynnMotion uses a **Core-Satellite Pattern** (Hub-and-Spoke) for managing a shared algorithmic core with multiple robot-specific implementations. This architecture enables:

- **Hardware SDK Isolation**: Each robot's proprietary SDK lives in its own environment
- **Dependency Management**: Avoid conflicts between different robot frameworks
- **Unified Interface**: Same API for simulation and real hardware
- **Sim-to-Real Transfer**: Develop in simulation, deploy to hardware without code changes

### Dependency Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         CORE PACKAGE                            │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  python/src/RynnMotion/     motion/                     │   │
│  │  ├── core/                  ├── interface/              │   │
│  │  │   ├── interface_base.py  │   └── interface_base.hpp  │   │
│  │  │   └── mj_interface.py    ├── robot_manager/          │   │
│  │  ├── manager/               └── module_manager/         │   │
│  │  ├── utils/                                             │   │
│  │  └── algorithms/                                        │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              ↑                                  │
│                    (unidirectional)                             │
│                              │                                  │
└──────────────────────────────┼──────────────────────────────────┘
                               │
       ┌───────────────────────┼───────────────────────┐
       │                       │                       │
       ▼                       ▼                       ▼
┌─────────────┐        ┌─────────────┐        ┌─────────────┐
│ Satellite A │        │ Satellite B │        │ Satellite C │
│  (Python)   │        │   (C++)     │        │  (Hybrid)   │
│             │        │             │        │             │
│ pip install │        │ #include    │        │ Both Python │
│ -e core/    │        │ interface_  │        │ and C++     │
│             │        │ base.hpp    │        │             │
│ Hardware    │        │             │        │             │
│ SDK A       │        │ Hardware    │        │ Hardware    │
│             │        │ SDK B       │        │ SDK C       │
└─────────────┘        └─────────────┘        └─────────────┘
```

**Key Principle**: Satellites depend on Core. Core NEVER depends on satellites.

---

## Core Package Structure

### Python Core (`python/src/RynnMotion/`)

```
python/src/RynnMotion/
├── core/                    # Base interfaces and factories
│   ├── interface_base.py    # RobotInterfaceBase (ABC)
│   ├── mj_interface.py      # MuJoCo simulation interface
│   └── base_controller.py   # ControllerBase for applications
├── manager/                 # Robot & scene management
│   ├── robot_manager.py     # Robot model configuration
│   └── scene_manager.py     # MuJoCo scene setup
├── common/                  # Shared data structures
│   └── data/
│       └── robot_state.py   # RobotState dataclass
├── utils/                   # Utilities
│   ├── path_config.py       # Path resolution
│   └── pin_kine.py          # Pinocchio kinematics
└── algorithms/              # Shared algorithms
    └── ...
```

### C++ Core (`motion/`)

```
motion/
├── interface/
│   └── interface_base.hpp   # InterfaceBase (abstract class)
├── robot_manager/
│   └── robot_manager.hpp    # Robot configuration
├── module_manager/
│   └── module_manager.hpp   # Controller modules
├── fsm_manager/
│   └── fsm_manager.hpp      # State machine
└── runtime_data/
    └── runtime_data.hpp     # Shared runtime data
```

---

## Interface Contracts

### Python Interface (`RobotInterfaceBase`)

The base class that all Python robot interfaces must implement:

```python
from abc import ABC, abstractmethod
from collections import OrderedDict
from RynnMotion.common.data.robot_state import RobotState

REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS = OrderedDict()

def register_robotinterface_factory_func(robot_type):
    """Decorator to register interface factories for runtime selection."""
    def decorator(factory_func):
        REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS[robot_type] = factory_func
    return decorator

def robotinterface_factory(robot_type, robot_model, robot_config, logger=None):
    """Create interface instance by type string."""
    factory_func = REGISTERED_ROBOT_INTERFACE_FACTORY_FUNCS[robot_type]
    return factory_func(robot_model=robot_model, robot_config=robot_config, logger=logger)


class RobotInterfaceBase(ABC):
    """Abstract base class for all robot interfaces (simulation and hardware)."""

    def __init__(self, robot_model, robot_config, logger):
        self.logger = logger
        self.robot_model = robot_model
        self.connected = False
        self._load_robot_config(robot_config)

    def _load_robot_config(self, robot_config):
        """Initialize robot dimensions from model."""
        self.mdof = self.robot_model.get_actuator_num()
        self.ee_num = self.robot_model.get_ee_num()
        self.gripper_num = self.robot_model.get_gripper_num()
        # ... additional configuration

    @abstractmethod
    def connect(self) -> None:
        """Connect to robot hardware or initialize simulation."""
        pass

    @abstractmethod
    def get_robot_state_feedbacks(self) -> RobotState:
        """Read current robot state from sensors or simulation."""
        pass

    @abstractmethod
    def set_robot_command(self, command: RobotState) -> None:
        """Send control commands to robot."""
        pass

    @abstractmethod
    def step(self) -> None:
        """Advance one timestep (simulation) or sync (hardware)."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Clean shutdown of robot connection."""
        pass

    def is_connected(self) -> bool:
        """Check connection status."""
        return self.connected
```

### C++ Interface (`InterfaceBase`)

The base class that all C++ robot interfaces must implement:

```cpp
#pragma once

#include <yaml-cpp/yaml.h>
#include <memory>
#include "robot_manager.hpp"
#include "module_manager.hpp"
#include "runtime_data.hpp"

class InterfaceBase {
public:
    /**
     * @brief Constructor for InterfaceBase
     * @param motionYaml Motion configuration YAML node
     * @param robotNumber Robot number to initialize
     * @param sceneNumber Scene configuration number
     */
    InterfaceBase(const YAML::Node &motionYaml,
                  int robotNumber,
                  int sceneNumber = 1);

    virtual ~InterfaceBase() = default;

    /**
     * @brief Run the main control application loop
     */
    virtual void runApplication() = 0;

protected:
    /**
     * @brief Load parameters from YAML configuration
     */
    virtual void loadYaml() = 0;

    /**
     * @brief Get feedback from sensors or simulation
     */
    virtual void getFeedbacks();
    virtual void getJointFeedbacks() = 0;
    virtual void getEEFeedbacks();  // Optional, default empty

    /**
     * @brief Send commands to actuators or simulation
     */
    virtual void setActuatorCommands();
    virtual void setJointCommands() = 0;
    virtual void setEECommands();  // Optional, default empty

    /**
     * @brief Advance physics simulation by one step
     */
    virtual void step() = 0;

    /**
     * @brief Controller management
     */
    virtual void callController();
    virtual void resetController();

    // Core managers (initialized by base class)
    std::unique_ptr<rynn::RobotManager> robotManager;
    std::unique_ptr<rynn::ModuleManager> moduleManager;
    data::RuntimeData runtimeData_;

    YAML::Node _motionYaml;
    int _robotNumber;
    double _controlFreq{1000.0};
};
```

### Method Mapping (Python ↔ C++)

| Python Method | C++ Method | Purpose |
|---------------|------------|---------|
| `connect()` | Constructor + `initInterface()` | Initialize robot connection |
| `get_robot_state_feedbacks()` | `getFeedbacks()` → `getJointFeedbacks()` | Read sensor data |
| `set_robot_command()` | `setActuatorCommands()` → `setJointCommands()` | Send control commands |
| `step()` | `step()` | Advance simulation / sync hardware |
| `disconnect()` | Destructor | Clean shutdown |
| `is_connected()` | N/A (managed internally) | Check connection status |

---

## Satellite Implementation Guide

### Python Satellite Example

A complete example of implementing a robot interface in Python:

```python
#!/usr/bin/env python3
"""Example robot interface implementation."""

from RynnMotion.core.interface_base import (
    RobotInterfaceBase,
    register_robotinterface_factory_func,
)
from RynnMotion.common.data.robot_state import RobotState

# Import your hardware SDK
# from my_robot_sdk import RobotDriver


@register_robotinterface_factory_func("my_robot_real")
def my_robot_factory(robot_model, robot_config, logger=None):
    """Factory function registered for runtime selection."""
    return MyRobotInterface(robot_model, robot_config, logger)


class MyRobotInterface(RobotInterfaceBase):
    """Hardware interface for MyRobot."""

    def __init__(self, robot_model, robot_config, logger):
        super().__init__(robot_model, robot_config, logger)
        self.driver = None
        self.ip_address = robot_config.get("ip_address", "192.168.1.100")
        self.port = robot_config.get("port", 8080)

    def connect(self) -> None:
        """Connect to robot hardware."""
        self.logger.info(f"Connecting to robot at {self.ip_address}:{self.port}")
        # self.driver = RobotDriver(self.ip_address, self.port)
        # self.driver.connect()
        self.connected = True
        self.logger.info("Robot connected successfully")

    def get_robot_state_feedbacks(self) -> RobotState:
        """Read current joint positions and velocities from hardware."""
        # positions = self.driver.get_joint_positions()
        # velocities = self.driver.get_joint_velocities()

        self.robot_feedback.joint_pos[:] = [0.0] * self.mdof  # Replace with actual
        self.robot_feedback.joint_vel[:] = [0.0] * self.mdof
        return self.robot_feedback.copy()

    def set_robot_command(self, command: RobotState) -> None:
        """Send joint position commands to hardware."""
        positions = command.joint_pos[:self.mdof]
        # self.driver.set_joint_positions(positions)
        self.robot_command = command

    def step(self) -> None:
        """For real hardware, this is typically a no-op or sync point."""
        pass

    def disconnect(self) -> None:
        """Disconnect from robot hardware."""
        if self.connected:
            # self.driver.disconnect()
            self.connected = False
            self.logger.info("Robot disconnected")
```

### C++ Satellite Example

A complete example of implementing a robot interface in C++:

```cpp
// my_robot_interface.hpp
#pragma once

#include "interface_base.hpp"
#include <memory>
#include <vector>

// #include "my_robot_sdk/driver.hpp"  // Your hardware SDK

class MyRobotInterface : public InterfaceBase {
public:
    MyRobotInterface(const YAML::Node &robotYaml,
                     const YAML::Node &motionYaml,
                     int robotNumber,
                     int sceneNumber = 1);

    ~MyRobotInterface() override;

    void runApplication() override;

protected:
    void loadYaml() override;
    void getJointFeedbacks() override;
    void setJointCommands() override;
    void step() override;

private:
    YAML::Node _robotYaml;
    std::string _ipAddress;
    int _port;
    // std::unique_ptr<MyRobotDriver> _driver;

    std::vector<double> _jointPositions;
    std::vector<double> _jointVelocities;
};
```

```cpp
// my_robot_interface.cpp
#include "my_robot_interface.hpp"
#include <chrono>
#include <thread>
#include <iostream>

MyRobotInterface::MyRobotInterface(const YAML::Node &robotYaml,
                                   const YAML::Node &motionYaml,
                                   int robotNumber,
                                   int sceneNumber)
    : InterfaceBase(motionYaml, robotNumber, sceneNumber),
      _robotYaml(robotYaml) {
    loadYaml();
    initHardware();
}

MyRobotInterface::~MyRobotInterface() {
    // Clean shutdown
    // _driver->disconnect();
}

void MyRobotInterface::loadYaml() {
    auto config = _robotYaml["my_robot"];
    _ipAddress = config["ip_address"].as<std::string>("192.168.1.100");
    _port = config["port"].as<int>(8080);
    _controlFreq = config["control_freq"].as<double>(1000.0);

    // Initialize storage
    int numJoints = robotManager->getNumJoints();
    _jointPositions.resize(numJoints, 0.0);
    _jointVelocities.resize(numJoints, 0.0);
}

void MyRobotInterface::getJointFeedbacks() {
    // Read from hardware
    // _jointPositions = _driver->getJointPositions();
    // _jointVelocities = _driver->getJointVelocities();

    for (size_t i = 0; i < _jointPositions.size(); ++i) {
        runtimeData_.qFb(i) = _jointPositions[i];
        runtimeData_.qdFb(i) = _jointVelocities[i];
    }
}

void MyRobotInterface::setJointCommands() {
    std::vector<double> commands(_jointPositions.size());
    for (size_t i = 0; i < commands.size(); ++i) {
        commands[i] = runtimeData_.qCmd(i);
    }
    // _driver->setJointPositions(commands);
}

void MyRobotInterface::step() {
    // For real hardware: sync point or no-op
    // For simulation: would call physics step
}

void MyRobotInterface::runApplication() {
    std::cout << "Starting control loop at " << _controlFreq << " Hz" << std::endl;

    const auto loopDuration = std::chrono::microseconds(
        static_cast<int>(1000000.0 / _controlFreq));
    auto nextTime = std::chrono::steady_clock::now();

    while (true) {
        nextTime += loopDuration;

        getFeedbacks();      // Read sensors
        callController();    // Update control
        setActuatorCommands(); // Send commands

        std::this_thread::sleep_until(nextTime);
    }
}
```

### Setup Script Template

Each satellite should have a setup script that installs the core package:

```bash
#!/bin/bash
# setup_env.sh - Satellite environment setup

set -e

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install core package WITHOUT its dependencies (to avoid conflicts)
pip install -e ../../python/ --no-deps

# Install core dependencies manually (allows version flexibility)
pip install numpy scipy PyYAML matplotlib mujoco pinocchio

# Install robot-specific SDK
pip install my-robot-sdk>=1.0.0

# Set PYTHONPATH if needed for development
export PYTHONPATH="$(pwd)/../../:$PYTHONPATH"

echo "Environment setup complete"
```

---

## Controller Layer (Application)

For higher-level applications, RynnMotion provides `ControllerBase` which abstracts mode switching:

### ControllerBase

```python
from abc import ABC, abstractmethod
from RynnMotion.core.interface_base import robotinterface_factory
from RynnMotion.manager.robot_manager import RobotManager

class ControllerBase(ABC):
    """Application-level controller with sim/real mode switching."""

    def __init__(self, mode="sim", frequency=100, config_path="config.yaml"):
        self.mode = mode
        self.frequency = frequency
        self.robot_model = RobotManager(self.get_robot_name())

        # Factory selects interface based on mode
        interface_type = "mujoco_sim_robot" if mode == "sim" else f"{self.get_robot_name()}_real"
        self.interface = robotinterface_factory(
            robot_type=interface_type,
            robot_model=self.robot_model,
            robot_config=self.load_config(config_path)
        )

    @abstractmethod
    def get_robot_name(self) -> str:
        """Return robot identifier (e.g., 'so101', 'fr3')."""
        pass

    @abstractmethod
    def motion_planner(self, current_time: float) -> np.ndarray:
        """Compute target joint positions for current timestep."""
        pass

    def run(self):
        """Main control loop."""
        self.interface.connect()
        current_time = 0.0
        dt = 1.0 / self.frequency

        try:
            while self.is_running():
                # Get current state
                state = self.interface.get_robot_state_feedbacks()

                # Compute commands
                target_positions = self.motion_planner(current_time)

                # Send commands
                command = RobotState()
                command.joint_pos[:] = target_positions
                self.interface.set_robot_command(command)

                # Step
                self.interface.step()
                current_time += dt
        finally:
            self.interface.disconnect()
```

### Controller Example

```python
class MyMotionController(ControllerBase):
    """Example controller for predefined motion patterns."""

    def __init__(self, mode="sim", motion_type=1):
        self.motion_type = motion_type
        super().__init__(mode=mode, frequency=100, config_path="configs/my_robot.yaml")

    def get_robot_name(self) -> str:
        return "my_robot"

    def motion_planner(self, current_time: float) -> np.ndarray:
        """Generate sinusoidal motion pattern."""
        amplitude = 0.5
        freq = 0.2
        num_joints = self.robot_model.get_actuator_num()

        target = np.zeros(num_joints)
        target[0] = amplitude * np.sin(2 * np.pi * freq * current_time)
        return target


# Usage:
# Simulation mode
controller = MyMotionController(mode="sim", motion_type=1)
controller.run()

# Real hardware mode
controller = MyMotionController(mode="real", motion_type=1)
controller.run()
```

---

## Best Practices

### Do's

1. **Keep core dependencies minimal** - Only essential packages (numpy, scipy, yaml)
2. **Use editable installs for development** - `pip install -e` for real-time updates
3. **Document interfaces clearly** - Type hints and docstrings
4. **Isolate hardware SDKs** - Keep in satellite venvs only
5. **Use the factory pattern** - Enables runtime mode switching
6. **Test in simulation first** - Use `MujocoRobotInterface` before hardware

### Don'ts

1. **Don't create circular dependencies** - Core should NEVER import satellites
2. **Don't hardcode paths** - Use `path_config.py` utilities
3. **Don't mix hardware SDKs** - Keep them isolated per satellite
4. **Don't skip the interface contract** - Implement all abstract methods
5. **Don't use global Python packages** - Always use virtual environments

### Dependency Isolation Strategy

```
Core Package (minimal deps)
├── numpy>=1.20.0
├── scipy>=1.7.0
├── pyyaml>=5.4
└── [optional] mujoco, pinocchio

Satellite A (servo motors)
├── Core (via pip install -e --no-deps)
├── servo-sdk>=2.0.0
└── pyserial>=3.5

Satellite B (industrial arm)
├── Core (via pip install -e --no-deps)
├── industrial-robot-sdk>=1.5.0
└── protobuf>=3.20.0
```

---

## Adding a New Robot

### Step-by-Step Guide

1. **Create satellite repository**
   ```
   my-robot-satellite/
   ├── src/
   │   └── my_robot_interface.py  # or .cpp
   ├── configs/
   │   └── my_robot.yaml
   ├── setup.py  # or CMakeLists.txt
   └── setup_env.sh
   ```

2. **Implement the interface**
   - Inherit from `RobotInterfaceBase` (Python) or `InterfaceBase` (C++)
   - Implement all abstract methods
   - Add hardware SDK integration

3. **Register with factory** (Python only)
   ```python
   @register_robotinterface_factory_func("my_robot_real")
   def factory(robot_model, robot_config, logger=None):
       return MyRobotInterface(robot_model, robot_config, logger)
   ```

4. **Create setup script**
   - Install core with `--no-deps`
   - Install hardware SDK
   - Configure PYTHONPATH if needed

5. **Test in simulation**
   - Use `MujocoRobotInterface` with your robot model
   - Verify control logic before hardware

6. **Deploy to hardware**
   - Switch mode from "sim" to "real"
   - Same controller code, different interface

### Checklist

- [ ] Interface inherits from base class
- [ ] All abstract methods implemented
- [ ] Factory function registered (Python)
- [ ] Setup script creates isolated environment
- [ ] Core installed with `--no-deps`
- [ ] Hardware SDK installed
- [ ] YAML configuration created
- [ ] Tested in simulation
- [ ] Tested on hardware

---

## Conclusion

The Core-Satellite pattern provides clean separation between shared algorithms and robot-specific implementations. This enables:

- **Rapid prototyping** in simulation
- **Safe deployment** to hardware
- **Easy addition** of new robots
- **Isolated dependencies** per robot
- **Consistent API** across all robots

The unidirectional dependency flow ensures the core remains stable while satellites can evolve independently with their respective hardware SDKs.
