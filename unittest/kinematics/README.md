# Kinematics Tests

Pinocchio + MuJoCo FK comparison tests. Verifies forward kinematics computations match between the two libraries.

## Build

### Standalone

```bash
cd unittest/kinematics
./build.sh

cd build
./pinMjTest --help
```

### With Project

```bash
cd build
cmake .. -DBUILD_TESTS=ON
make pinMjTest -j4

./unittest/pinMjTest --help
```

## Usage

```
./pinMjTest [options]

Options:
  --robot, -R <num>  Robot number (default: 20)
                     20=FR3, 21=UR5E, 22=Piper, 23=RM75, 24=SO101
  --scenario, -s <n> Test scenario (default: 0)
                     0=zero angles (5 steps), 1=lerp motion (10s)
  --multi, -m        Multi-site mode (shoulder, elbow, wrist, EE)
  --jaco, -j         Compare Jacobians (single-site mode only)
  --render, -r       Enable MuJoCo rendering
  --help, -h         Show help
```

## Examples

```bash
./pinMjTest                   # Single EE test on FR3
./pinMjTest --multi           # Multi-site test (shoulder, elbow, wrist, EE)
./pinMjTest -R 22 --multi     # Multi-site test on Piper
./pinMjTest --jaco            # Jacobian comparison
./pinMjTest -s 1 --render     # Lerp motion with rendering
```

## Test Modes

| Mode | Flag | Sites Tested | Tolerances |
|------|------|--------------|------------|
| Single | (default) | EE only | pos: 1e-5, ori: 1e-5 |
| Multi | `--multi` | shoulder, elbow, wrist, EE | pos: 1e-3, ori: 1e-2 |
| Jacobian | `--jaco` | EE Jacobian matrix | - |

## Expected Output

```
=== Single-Site Kinematics Test ===
Robot: FR3 (20)
Scenario: 0 (zero)

qRef: 0 0 0 0 0 0 0
--- t=0.00s ---
[MuJoCo   ] Pos: [ 0.088113  0.000000  0.823555 ]
[MuJoCo   ] Quat: [ 0.923879  -0.000090  -0.000218  0.382684 ]
[Pinocchio] Pos: [ 0.088113  0.000000  0.823555 ]
[Pinocchio] Quat: [ 0.923879  -0.000090  -0.000218  0.382684 ]
Pin-MJ diff: 5.493e-09 m, 7.885e-08 rad

✓ All tests passed
```

Both position (~1e-9 m) and orientation (~1e-8 rad) errors should be near zero.
Quaternions are displayed in Eigen standard format (x, y, z, w).

## Dependencies

- Pinocchio 3.7+
- MuJoCo 3.3+
- Eigen3, Boost, yaml-cpp
