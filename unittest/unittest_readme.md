# Unittest Quick Reference

## Build Tests

```bash
# From project root - Configure with tests enabled
cmake --preset=tests

# Build all unittest/
cmake --build build -j8

# Build only unittest/fsm/
cd build/unittest/fsm && make -j8

# Build specific test target
make simpleTick
make motionTick
make pickplaceTest
```

## Run Tests

```bash
# From project root - Run all tests
ctest --test-dir build --output-on-failure

# From build/ directory - Run all tests
cd build && ctest --output-on-failure

# Run specific test category (from project root)
ctest --test-dir build -R kinematics --output-on-failure
ctest --test-dir build -R fsm --output-on-failure

# Run test executable directly
./build/unittest/kinematics/pinkine/franka_kine
./build/unittest/kinematics/pinkine/lerobot_kine
./build/unittest/kinematics/pinkine_mj/hybrid_main
./build/unittest/trajectory/test_ee
./build/unittest/rcp_motion/rcpNode
./build/unittest/fsm/simpleTick
./build/unittest/fsm/motionTick
./build/unittest/fsm/pickplaceTest
```

## Standalone Build

```bash
# Build kinematics tests standalone
cd unittest/kinematics/pinkine
mkdir build && cd build
cmake .. && make -j8
./franka_kine

# Build rcp_motion tests standalone
cd unittest/rcp_motion
mkdir build && cd build
cmake .. && make -j8
./rcpNode

# Build FSM tests standalone
cd unittest/fsm
mkdir build && cd build
cmake .. && make -j8
./simpleTick
./motionTick
./pickplaceTest
```

## Clean & Rebuild

```bash
# Clean build directory
rm -rf build

# Rebuild from scratch
cmake --preset=tests
cmake --build build -j8
```

## Test Organization

```
unittest/
├── kinematics/
│   ├── pinkine/        # Pinocchio kinematics tests (5 robots)
│   └── pinkine_mj/     # Pinocchio+MuJoCo hybrid test
├── trajectory/         # Trajectory generation tests
├── rcp_motion/         # LCM bidirectional communication tests
├── collision/          # Collision detection tests
├── utility/            # Utility function tests
├── sketch/             # Path sketch tests
├── message/            # Message passing tests
└── fsm/                # FSM state machine tests
```

## Available Test Executables

**Kinematics (Pinocchio):**
- `franka_kine` - Franka FR3 kinematics test
- `lerobot_kine` - LeRobot kinematics test
- `rm75_kine` - RM75 kinematics test
- `ur5e_kine` - UR5e kinematics test
- `piper_kine` - Piper kinematics test

**Kinematics (Pinocchio+MuJoCo):**
- `hybrid_main` - Compare Pinocchio vs MuJoCo kinematics

**Trajectory:**
- `test_ee` - End-effector trajectory test
- `test_joint` - Joint trajectory test

**RCP Motion:**
- `rcpNode` - RCP communication node
- `motionNode` - Motion control node

**Collision:**
- `collision_test_fcl` - FCL collision detection
- `collision_test_primitives` - Primitive collision detection

**Utility:**
- `utility_test` - Utility functions test

**Sketch:**
- `sketch_test` - Path sketch test

**Message:**
- Various message type tests

**FSM:**
- `simpleTick` - Basic FSM state transitions
- `motionTick` - FsmManager integration test
- `pickplaceTest` - PickPlace FSM sequence test
