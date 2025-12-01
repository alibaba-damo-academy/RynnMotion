# Pinocchio Debug Tools

Utilities for debugging Pinocchio model frames and sites.

## Tools

### check_frames.py
List all frames in a Pinocchio model.

```bash
python check_frames.py           # default: piper
python check_frames.py fr3       # shorthand
python check_frames.py so101
```

### debug_frames.py
Debug frame lookup for specific sites.

```bash
python debug_frames.py                       # default: so101
python debug_frames.py fr3
python debug_frames.py piper EE shoulderSite
```

## Available Models

Shorthand names resolve to `models/3.robot_arm/`:
- `fr3` - Franka FR3 (7 DOF)
- `piper` - Piper (6 DOF)
- `so101` - SO-101 (5 DOF)

Custom paths also work:
```bash
python check_frames.py ../../models/3.robot_arm/21.ur5e/mjcf/ur5e_pinocchio.xml
```
