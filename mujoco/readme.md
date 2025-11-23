# MuJoCo

Mujoco的C++ version规划控制仿真

## Directory Structure

The MuJoCo component is organized as follows:

C++ implementation of MuJoCo simulation
- `include/`: Header files used by C++ implementation
- `src/`: Source files used by C++ implementation
- Built using CMake from the root CMakeLists.txt

### Note about Python Linter Errors

The linter may show errors for MuJoCo-specific attributes like `MjModel`, `MjData`, `mj_step`, and `mjtVisFlag` when they are correctly used. This is because these are dynamically created at runtime by the MuJoCo Python binding's C extension.

The code is correct and will run despite these linter warnings.

### MuJoCo Viewer Keyboard Shortcuts

一些mujoco viewer的快捷键:
- T: Transparent model
- I: Show inertia
- A: Show kinematic tree
- M: Show COM position
- U: Show rotation axis
- F1 ~ F7: Show various data, sensor data, etc.
- Tab: Close left panel
- Shift + Tab: Close right panel
- Backspace: Reset simulation
- Ctrl+L: Reload model
- Space: Play & Pause
- Speed controls: - and +
- Zoom: Right double click the point + mouse wheel
- Pan Camera: Shift + right drag
- Camera cycle: [ and ]
- Free camera: ESC
- Tracking camera: Ctrl + right double click

# move and totate object
- x/a + up arrow: x axis pose increase
- y/s + up arrow: y axis pose increase
- z/d + up arrow: z axis pose increase
- x/a + down arrow: x axis pose decrease
- y/s + down arrow: y axis pose decrease
- z/d + down arrow: z axis pose decrease
- x/a + right arrow: x axis rotation increase
- y/s + right arrow: y axis rotation increase
- z/d + right arrow: z axis rotation increase
- x/a + left arrow: x axis rotation decrease
- y/s + left arrow: y axis rotation decrease
- z/d + left arrow: z axis rotation decrease

# select object
- shift + left click: select object

# if not dynamically tracking object, you can use the following command to send object pose info for once:
- ctrl + enter

## C++ Implementation

### Installation

To get started with MuJoCo for C++:

1. Download MuJoCo from the official website: [MuJoCo Website](https://github.com/google-deepmind/mujoco/releases)
2. Extract the downloaded files to a desired location on your machine.

### Building

mujoco/ is cmake subdirectory in **robotMotion**, run:

```bash
mkdir -p build
cd build
cmake ..
make
```

### Components

- `mujocoExe`: Standalone MuJoCo simulation executable
- `rosMujocoExe`: ROS-enabled MuJoCo simulation executable (only built if `BUILD_ROS_MUJOCO` is enabled)

### Usage

After building, the executables will be placed in the `build/` directory at the project root.

To run:
```bash
.mujocoExe [robotNumber] [sceneNumber]
```

If ROS integration is enabled:
```bash
./build/rosMujocoExe [robotNumber]
```

### URDF to MuJoCo XML Conversion

To convert URDF files to MuJoCo XML format:
- Open a terminal, navigate to your mujoco-3.3.0/bin/
- ```./compile ./path_to_ur_urdf/xxx.urdf ./path_to_ur_xml/xxx.xml ```

## Resources

Here are some additional resources to help you dive deeper into MuJoCo:

- [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/overview.html): Official documentation for MuJoCo, providing detailed explanations and examples.
- [MuJoCo Github](https://github.com/google-deepmind/mujoco): mujoco official github repo

Happy exploring in the MuJoCo Playground!
