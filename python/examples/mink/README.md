# mink IK Examples

Examples demonstrating mink differential inverse kinematics integration with RynnMotion.

## Installation

```bash
# Install RynnMotion (includes mink and dependencies)
uv pip install -e .
```

## Basic Examples

Simple examples demonstrating core mink functionality.

| Script | Scenario |
|--------|----------|
| `basic/frame_task_example.py` | Single end-effector tracking a circular trajectory |
| `basic/posture_task_example.py` | Joint configuration tracking between predefined poses |
| `basic/multi_task_example.py` | Combining FrameTask + PostureTask + DampingTask |

```bash
# End-effector follows circular path
python examples/mink/basic/frame_task_example.py

# Robot cycles through joint configurations
python examples/mink/basic/posture_task_example.py

# Multi-task: end-effector tracks figure-8 with posture regularization
python examples/mink/basic/multi_task_example.py
```

## Integration Examples

Examples showing mink integration with RynnMotion interfaces.

| Script | Scenario |
|--------|----------|
| `integration/mink_with_mj_interface.py` | Using mink with RynnMotion's MujocoRobotInterface |
| `integration/collision_avoidance.py` | ConfigurationLimit + VelocityLimit constraints |
| `integration/teleop_with_mink.py` | Teleoperation by dragging mocap target in viewer |

```bash
# mink IK with RynnMotion bridge utilities
python examples/mink/integration/mink_with_mj_interface.py

# IK with joint position and velocity limits
python examples/mink/integration/collision_avoidance.py

# Interactive teleoperation (drag target in viewer)
python examples/mink/integration/teleop_with_mink.py
```

## Robot Examples

Robot-specific examples using robot_descriptions package.

| Script | Scenario |
|--------|----------|
| `robots/arm_panda.py` | Franka Emika Panda arm IK |
| `robots/arm_ur5e.py` | Universal Robots UR5e with velocity limits |
| `robots/arm_iiwa.py` | KUKA iiwa14 7-DOF arm |
| `robots/humanoid_g1.py` | Unitree G1 humanoid with multi-effector tracking |
| `robots/generic_arm.py` | Generic loader for any robot_descriptions model |

```bash
# Panda arm circular trajectory
python examples/mink/robots/arm_panda.py

# UR5e with velocity limits
python examples/mink/robots/arm_ur5e.py

# KUKA iiwa arm
python examples/mink/robots/arm_iiwa.py

# G1 humanoid with hand wave motion
python examples/mink/robots/humanoid_g1.py

# Generic: works with any robot_descriptions model
python examples/mink/robots/generic_arm.py panda_mj_description
python examples/mink/robots/generic_arm.py ur5e_mj_description
python examples/mink/robots/generic_arm.py --info panda_mj_description  # Print model info
```

## Bridge Utilities

Import mink bridge utilities from RynnMotion:

```python
from RynnMotion.algorithms import (
    configuration_from_model,
    configuration_from_interface,
    sync_mj_data_to_config,
    apply_config_to_mj_data,
    create_frame_task,
    create_posture_task,
    create_standard_limits,
    solve_ik_step,
)
```
