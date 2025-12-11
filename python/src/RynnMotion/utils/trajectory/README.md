# Trajectory Generation Module

This directory contains Python implementations of trajectory generation algorithms based on the Ruckig library, converted from C++ counterparts.

## Files

- `trajgen.py`: Main trajectory generation module with JointTrajGen and EEPoseTrajGen classes
- `test_joint.py`: Joint space trajectory generation tests with various constraint scenarios
- `test_ee.py`: End-effector pose trajectory generation tests with TCP constraints
- `plot_jointtraj.py`: Visualization tools for joint trajectory data
- `plot_eetraj.py`: Visualization tools for end-effector trajectory data

## Usage

### Joint Space Trajectory Generation

```python
from utils.trajectory.trajgen import JointTrajGen
import numpy as np

# Create trajectory generator for a 6-DOF robot
joint_traj = JointTrajGen(6, 0.001)  # 6 DOF, 1ms time step

# Set start and target states
q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
q_target = np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
joint_traj.set_start_state(q_start)
joint_traj.set_target_state(q_target)

# Set joint limits
q_upper = np.ones(6) * 2.0
q_lower = np.ones(6) * -2.0
joint_traj.set_joint_pos_limits(q_upper, q_lower)

# Set velocity and acceleration limits
qd_max = np.ones(6) * 1.0
qdd_max = np.ones(6) * 2.0
joint_traj.set_joint_motion_limits(qd_max, qdd_max)

# Generate trajectory
result = joint_traj.update()
```

### End-Effector Pose Trajectory Generation

```python
from utils.trajectory.trajgen import EEPoseTrajGen
from scipy.spatial.transform import Rotation as R
import numpy as np

# Create trajectory generator
ee_traj = EEPoseTrajGen(0.002)  # 2ms time step

# Set start and target poses
start_pos = np.array([0.5, 0.2, 0.8])
start_rpy = np.array([-0.5, -0.8, 0.2])
start_quat = R.from_euler('xyz', start_rpy)

target_pos = np.array([1.2, -2.3, 2.1])
target_rpy = np.array([0.5, 0.8, 1.5])
target_quat = R.from_euler('xyz', target_rpy)

ee_traj.set_start_state(start_pos, start_quat)
ee_traj.set_target_state(target_pos, target_quat)

# Set TCP limits
ee_traj.set_tcp_limits(2.0, 5.0)  # max speed: 2 m/s, max acceleration: 5 m/s²

# Generate trajectory
result = ee_traj.update()
```

## Running Tests

```bash
# Run joint trajectory tests
python test_joint.py

# Run end-effector trajectory tests
python test_ee.py
```

## Key Features

1. **Joint Space Trajectory Generation**:
   - Position, velocity, and acceleration limits
   - Support for different constraint modes
   - Boundary condition handling

2. **End-Effector Pose Trajectory Generation**:
   - TCP (Tool Center Point) speed and acceleration limits
   - Quaternion-based orientation handling
   - 4-DOF trajectory generation (x, y, z, rotation angle)

3. **Constraint Handling**:
   - Automatic clamping of positions exceeding limits
   - Velocity and acceleration limit enforcement
   - Smooth trajectory generation with Ruckig library

## Dependencies

- numpy
- scipy
- ruckig