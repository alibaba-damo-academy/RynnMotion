# How to Add Robots and Scenes

**Key Principle:** The `models/` directory IS the source of truth. Just add files - no code changes needed.

---

## Adding a New Robot

### Directory Structure
```
models/{category}/{NUMBER}.{robot_name}/
  mjcf/
    {robot}_pinocchio.xml    # Required: kinematics/dynamics
    {robot}_robot.xml        # Required: full simulation model
  scene/
    scene.xml                # Required: default scene
```

### Example: Add robot #28 "myrobot"
```bash
mkdir -p models/3.robot_arm/28.myrobot/mjcf
mkdir -p models/3.robot_arm/28.myrobot/scene

# Add MJCF files (must end with _pinocchio.xml and _robot.xml)
cp myrobot_pinocchio.xml models/3.robot_arm/28.myrobot/mjcf/
cp myrobot_robot.xml models/3.robot_arm/28.myrobot/mjcf/

# Add default scene
cp scene.xml models/3.robot_arm/28.myrobot/scene/

# Rebuild to generate RobotType enum
cmake --build build

# Launch (prefer using names over numbers)
./mujocoExe myrobot default    # By robot name + scene name
./mujocoExe myrobot ui         # Interactive UI scene
```

---

## Adding Scenes

Just drop `.xml` files in the `scene/` directory. Numbering is alphabetical.

```bash
cd models/3.robot_arm/20.fr3/scene/
cp scene_tracking.xml .
cp scene_pickplace.xml .

# Auto-discovered as:
# scene.xml              → Scene 1 "default"
# scene_pickplace.xml    → Scene 2 "pickplace"
# scene_tracking.xml     → Scene 3 "tracking"

./mujocoExe fr3 tracking   # By name (recommended)
./mujocoExe fr3 pickplace  # Another scene by name
```

### Scene Aliases (1-5)

| # | Name | Aliases | Module Chain |
|---|------|---------|--------------|
| 1 | `joint` | default, jointmove | JointMove |
| 2 | `keyframe` | wobble, cycle | JointMove |
| 3 | `ui` | tracking | Estimator → Planner → OSC |
| 4 | `predefined` | workspace | Estimator → Planner → OSC |
| 5 | `pickplace` | pick | Estimator → Planner → OSC |

### Scene Naming Patterns

| Filename | Parsed Name |
|----------|-------------|
| `scene.xml` | `default` (→ joint) |
| `scene_pickplace.xml` | `pickplace` |
| `scene_tracking.xml` | `tracking` (→ ui) |

---

## RobotType Constants (Auto-Generated)

When you add a robot directory, CMake auto-generates an enum at build time:

```
Directory: models/3.robot_arm/28.myrobot/
Generated: build/robotDiscovery/robot_types.hpp
```

```cpp
// Auto-generated enum
enum class RobotType {
  ...
  myrobot = 28,
  ...
};
```

Use in code:
```cpp
#include "robot_types.hpp"

switch (robotManager->getRobotType()) {
case rynn::RobotType::myrobot:
  // Custom handling
  break;
}
```

**No manual edits needed** - constants sync automatically with `models/`.

---

## Optional: Custom Module Chains

Create `robot_config.yaml` in robot directory:

```yaml
module_chains:
  default: [JointMove]
  tracking: [Estimator, SingleEEPlanner, OSC]
  custom_scene: [Estimator, Planner, OSC]
```

---

## Optional: Aliases

Edit `models/discovery.hpp`:

```cpp
const std::map<std::string, std::vector<std::string>> ROBOT_ALIASES = {
  {"myrobot", {"MyRobot", "my-robot", "robot28"}},
  // ... existing aliases
};
```

Now: `./mujocoExe my-robot ui` works.

---

## What Happens Automatically?

- Robot discovery (scans `models/` at startup)
- RobotType enum (generated at build time via `cmake/GenerateRobotTypes.cmake`)
- Case-insensitive lookup
- Scene numbering (alphabetical)
- Module chain selection (smart defaults)
- Path resolution (finds pinocchio/robot MJCF)

---

## Troubleshooting

**"Robot not found"**
- Check naming: `{number}.{robot_name}` (e.g., `28.myrobot`)
- Rebuild: `cmake --build build`

**"No pinocchio file found"**
- Check filename ends with `_pinocchio.xml`

**"No robot MJCF found"**
- Check filename ends with `_robot.xml`

**"No scenes found"**
- Create at least `scene.xml` in `scene/` directory

---

**Last Updated:** 2025-12-01
