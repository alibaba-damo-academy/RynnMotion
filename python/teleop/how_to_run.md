# Teleoperation Module

Heterogeneous teleoperation from SO101 (master) to various slave robots.

## Usage

### Joint-Level Teleoperation

Direct joint mapping with robot-specific direction vectors.

```bash
cd python/
python -m teleop.so101_jointhto fr3
python -m teleop.so101_jointhto piper -v
python -m teleop.so101_jointhto ur5e --side right
python -m teleop.so101_jointhto rm75 --side left -v
```

### End-Effector Teleoperation

FK + PoseMapper + OSC + Differential IK control.

```bash
cd python/
python -m teleop.so101_eehto fr3
python -m teleop.so101_eehto piper -v
python -m teleop.so101_eehto rm75 --side right
```

## Supported Robots

| Alias | Robot | Number |
|-------|-------|--------|
| `fr3` | Franka FR3 | 20 |
| `ur5e` | Universal Robots UR5e | 21 |
| `piper` | Piper | 22 |
| `rm75` | RealMan RM75 | 23 |

## Controls

| Key | Action |
|-----|--------|
| `ENTER` | Start teleoperation (from standby) |
| `R` | Reset to standby position |
| `ESC` | Exit |

## Options

| Flag | Description |
|------|-------------|
| `-v, --verbose` | Enable verbose output |
| `--side {left,right}` | Arm side for mapping (default: left) |