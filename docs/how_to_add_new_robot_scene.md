# How to Add Robots and Scenes

**Key Principle:** The `models/` directory IS the source of truth. Just add files - no code changes needed.

---

## Adding a New Robot

### Directory Structure
```
models/{category}/{NUMBER}.{robot_name}/
  mjcf/
    {robot}_pinocchio.xml    # Required: kinematics
    {robot}_robot.xml         # Required: full model
  scene/
    scene.xml                 # Required: default scene
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

# Done! Launch immediately
./mujocoExe myrobot 1
./mujocoExe 28 1
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

./mujocoExe fr3 3          # By number
./mujocoExe fr3 tracking   # By name
```

### Scene Naming Patterns
| Filename | Parsed Name | Auto Modules |
|----------|-------------|--------------|
| `scene.xml` | `default` | JointMove |
| `scene_tracking.xml` | `tracking` | Estimator → SingleEEPlanner → OSC |
| `scene_pickplace.xml` | `pickplace` | Estimator → SingleEEPlanner → OSC |

---

## RobotType Constants (Auto-Generated)

When you add a robot directory, CMake auto-generates constants at build time:

```
Directory: models/3.robot_arm/28.myrobot/
Generated: constexpr RobotType myrobot = 28;
```

Use in code:
```cpp
switch (robotManager->getRobotType()) {
case Robot::myrobot:  // Type-safe, auto-generated
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

Edit `motion/manager/robot_discovery.cpp`:

```cpp
const std::map<std::string, std::vector<std::string>> ROBOT_ALIASES = {
  {"myrobot", {"MyRobot", "my-robot", "robot28"}},
};
```

Now: `./mujocoExe my-robot 1` works.

---

## What Happens Automatically?

✅ Robot discovery (scans `models/` at startup)
✅ RobotType constants (generated at build time)
✅ Case-insensitive lookup
✅ Scene numbering (alphabetical)
✅ Module chain selection (smart defaults)
✅ Path resolution (finds pinocchio/robot MJCF)

---

## Troubleshooting

**"Robot not found"**
- Check naming: `{number}.{robot_name}` (e.g., `28.myrobot`)

**"No pinocchio file found"**
- Check filename ends with `_pinocchio.xml`

**"No robot MJCF found"**
- Check filename ends with `_robot.xml`

**"No scenes found"**
- Create at least `scene.xml` in `scene/` directory

---

**Last Updated:** 2025-11-17 (Phase 5.5 Complete)
