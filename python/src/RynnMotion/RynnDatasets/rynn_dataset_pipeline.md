# RynnDataset Pipeline Documentation

## Overview

This document provides a comprehensive call graph and data flow analysis of the RynnDataset recording and storage pipeline. The pipeline processes robot teleoperation data including joint positions, end-effector poses, camera poses, and RGB video streams into structured datasets.

## Complete Pipeline Architecture

```mermaid
graph TD
    %% Initialization Phase
    A[🎬 run_record.sh] --> B[RynnLeRobotRecorder.__init__]
    B --> C[get_robot_features]
    C --> D[hw_to_dataset_features]
    D --> E[RynnDataset.create]

    %% Feature Processing Detail
    C --> C1[Joint Features: float]
    C --> C2[Camera Features: tuple]
    C --> C3[Pose Features: list]

    C1 --> D1[joint_fts: shoulder_pan.pos, etc.]
    C2 --> D2[cam_fts: front, wrist camera shapes]
    C3 --> D3[pose_fts: end_effector_pose, camera_poses]

    D1 --> D4[action/observation.state arrays]
    D2 --> D5[observation.images.front/wrist]
    D3 --> D6[action.end_effector_pose, observation.camera_poses.*]

    %% Recording Loop
    E --> F[🔄 record_episode]
    F --> G[Recording Loop: 150 frames @ 30fps]

    %% Data Collection per Frame
    G --> H[get_robot_observation]
    G --> I[get_robot_action]

    %% Observation Collection
    H --> H1[📊 Joint Data]
    H --> H2[📹 Camera Data]
    H --> H3[📍 Pose Data]

    H1 --> H1a[teleop.get_observation<br/>→ 6 joint positions]
    H2 --> H2a[camera.read<br/>→ RGB frames]
    H2 --> H2b[camera.get_optical_center_pose<br/>→ 7D pose arrays]
    H3 --> H3a[teleop.get_eePose_observation<br/>→ 7D pose array]

    %% Action Collection
    I --> I1[📊 Joint Actions]
    I --> I2[📍 Pose Actions]

    I1 --> I1a[teleop.get_action<br/>→ 6 joint commands]
    I2 --> I2a[teleop.get_eePose_action<br/>→ 7D pose array]

    %% Data Processing
    H1a --> J1[observation: shoulder_pan.pos, etc.]
    H2a --> J2[observation: front, wrist images]
    H2b --> J3[observation: camera_poses.front/wrist]
    H3a --> J4[observation: end_effector_pose]

    I1a --> K1[action: shoulder_pan.pos, etc.]
    I2a --> K2[action: end_effector_pose]

    %% Frame Building
    J1 --> L[build_dataset_frame: observation]
    J2 --> L
    J3 --> L
    J4 --> L

    K1 --> M[build_dataset_frame: action]
    K2 --> M

    %% Dataset Frame Construction
    L --> L1[observation.state: joint array]
    L --> L2[observation.images.front/wrist: video]
    L --> L3[observation.end_effector_pose: pose array]
    L --> L4[observation.camera_poses.*: pose arrays]

    M --> M1[action: joint array]
    M --> M2[action.end_effector_pose: pose array]

    %% Storage
    L1 --> N[RynnDataset.add_frame]
    L2 --> N
    L3 --> N
    L4 --> N
    M1 --> N
    M2 --> N

    N --> O[📦 Parquet Files]
    N --> P[🎥 MP4 Videos]
    N --> Q[📄 JSON Metadata]

    %% Output Structure
    O --> O1["episode_000000.parquet<br/>- action: 6 joints<br/>- action.end_effector_pose: pose array<br/>- observation.state: 6 joints<br/>- observation.end_effector_pose: pose array<br/>- observation.camera_poses.front: pose array<br/>- observation.camera_poses.wrist: pose array"]

    P --> P1[observation.images.front/<br/>episode_000000.mp4]
    P --> P2[observation.images.wrist/<br/>episode_000000.mp4]

    Q --> Q1["info.json: Dataset metadata<br/>with meaningful field names"]

    %% Styling
    classDef initPhase fill:#e1f5fe
    classDef dataCollection fill:#f3e5f5
    classDef processing fill:#e8f5e8
    classDef storage fill:#fff3e0

    class A,B,C,D,E initPhase
    class H,I,H1,H2,H3,I1,I2 dataCollection
    class L,M,L1,L2,L3,L4,M1,M2 processing
    class N,O,P,Q,O1,P1,P2,Q1 storage
```

## Phase-by-Phase Breakdown

### 1. Initialization Phase

```mermaid
sequenceDiagram
    participant Script as run_record.sh
    participant Recorder as RynnLeRobotRecorder
    participant Features as get_robot_features
    participant HW2DS as hw_to_dataset_features
    participant Dataset as RynnDataset

    Script->>Recorder: Initialize
    Recorder->>Features: Define robot features
    Features-->>Features: joint_features: {shoulder_pan.pos: float, ...}
    Features-->>Features: camera_features: {front: (480,640,3), ...}
    Features-->>Features: pose_features: {end_effector_pose: [7], ...}
    Features->>HW2DS: Process features
    HW2DS-->>HW2DS: joint_fts → action/observation.state
    HW2DS-->>HW2DS: cam_fts → observation.images.*
    HW2DS-->>HW2DS: pose_fts → action.end_effector_pose, etc.
    HW2DS->>Dataset: Create dataset with features
```

**Key Components:**

- **`get_robot_features()`**: Defines data types for joints (float), cameras (tuple), poses (list)
- **`hw_to_dataset_features()`**: Converts hardware features to dataset format
- **Feature Type Processing**: Creates appropriate field structures for each data type

### 2. Recording Loop

```mermaid
sequenceDiagram
    participant Recorder as RynnLeRobotRecorder
    participant Teleop as TeleOperator
    participant Camera as Camera Systems
    participant Builder as build_dataset_frame
    participant Dataset as RynnDataset

    loop Every Frame (30fps)
        Recorder->>Teleop: get_observation()
        Teleop-->>Recorder: [6 joint positions]

        Recorder->>Teleop: get_eePose_observation()
        Teleop-->>Recorder: [x,y,z,qx,qy,qz,qw].astype(float32)

        Recorder->>Camera: read() + get_optical_center_pose()
        Camera-->>Recorder: RGB frame + [x,y,z,qx,qy,qz,qw].astype(float32)

        Recorder->>Teleop: get_action() + get_eePose_action()
        Teleop-->>Recorder: [6 joint commands] + [x,y,z,qx,qy,qz,qw].astype(float32)

        Recorder->>Builder: build_dataset_frame(observation)
        Builder-->>Builder: Joint features → observation.state array
        Builder-->>Builder: Pose features → separate pose arrays
        Builder-->>Builder: Camera features → observation.images.*

        Recorder->>Builder: build_dataset_frame(action)
        Builder-->>Builder: Joint features → action array
        Builder-->>Builder: Pose features → action.end_effector_pose

        Recorder->>Dataset: add_frame(observation_frame, action_frame)
    end
```

**Data Collection Points:**

- **Joint Data**: 6 DOF robot joint positions and commands
- **End-Effector Poses**: 7D pose arrays (position + quaternion)
- **Camera Data**: RGB frames and optical center poses
- **Type Safety**: All poses converted to float32 for consistency

### 3. Feature Type Processing

```mermaid
graph LR
    %% Input Types
    A[Input Features] --> A1[float: joint positions]
    A --> A2[tuple: camera shapes]
    A --> A3[list: pose dimensions]

    %% Processing Logic
    A1 --> B1[hw_to_dataset_features<br/>joint_fts processing]
    A2 --> B2[hw_to_dataset_features<br/>cam_fts processing]
    A3 --> B3[hw_to_dataset_features<br/>pose_fts processing]

    %% Feature Definitions
    B1 --> C1["action: dtype float32, shape 6, joint names"]
    B1 --> C2["observation.state: dtype float32, shape 6, joint names"]

    B2 --> C3["observation.images.front: dtype video, shape 480x640x3"]
    B2 --> C4["observation.images.wrist: dtype video, shape 480x640x3"]

    B3 --> C5["action.end_effector_pose: dtype float32, shape 7, names x,y,z,qx,qy,qz,qw"]
    B3 --> C6["observation.end_effector_pose: dtype float32, shape 7, names x,y,z,qx,qy,qz,qw"]
    B3 --> C7["observation.camera_poses.*: dtype float32, shape 7, names x,y,z,qx,qy,qz,qw"]

    %% Frame Building
    C1 --> D1[build_dataset_frame: Joint array from individual values]
    C2 --> D1
    C3 --> D2[build_dataset_frame: Direct image assignment]
    C4 --> D2
    C5 --> D3[build_dataset_frame: Direct numpy array assignment]
    C6 --> D3
    C7 --> D3
```

## Core Implementation Details

### Feature Detection Logic

The pipeline uses type-based feature detection:

```python
# In hw_to_dataset_features()
joint_fts = {key: ftype for key, ftype in hw_features.items() if ftype is float}
cam_fts = {key: shape for key, shape in hw_features.items() if isinstance(shape, tuple)}
pose_fts = {key: shape for key, shape in hw_features.items() if isinstance(shape, list)}
```

### Data Type Processing

**Joint Features (float type):**

- Individual float values grouped into arrays
- Names preserve semantic meaning (e.g., "shoulder_pan.pos")
- Built using: `np.array([values[name] for name in ft["names"]])`

**Camera Features (tuple type):**

- RGB frames stored as video files
- Shape information: `(height, width, channels)`
- Processed as: `observation.images.{camera_name}`

**Pose Features (list type):**

- 7D pose arrays with meaningful names
- Consistent float32 dtype
- Direct numpy array assignment

### Meaningful Field Names

Pose features now use semantic names instead of generic dimensions:

```python
# 7D poses get meaningful names
if shape[0] == 7:
    pose_names = ["x", "y", "z", "qx", "qy", "qz", "qw"]
else:
    pose_names = [f"dim_{i}" for i in range(shape[0])]
```

## Output Structure

### Directory Layout

```
outputs/MMDD_HHMM/
├── data/
│   └── chunk-000/
│       └── episode_000000.parquet  # All numeric data
├── videos/
│   └── chunk-000/
│       ├── observation.images.front/
│       │   └── episode_000000.mp4
│       └── observation.images.wrist/
│           └── episode_000000.mp4
└── meta/
    ├── info.json        # Dataset metadata
    ├── tasks.jsonl      # Task definitions
    ├── episodes.jsonl   # Episode metadata
    └── stats/           # Dataset statistics
```

### Parquet Schema

The final parquet files contain clean, separate fields:

```json
{
  "action": [joint1, joint2, joint3, joint4, joint5, gripper],
  "action.end_effector_pose": [x, y, z, qx, qy, qz, qw],

  "observation.state": [joint1, joint2, joint3, joint4, joint5, gripper],
  "observation.end_effector_pose": [x, y, z, qx, qy, qz, qw],
  "observation.camera_poses.front": [x, y, z, qx, qy, qz, qw],
  "observation.camera_poses.wrist": [x, y, z, qx, qy, qz, qw],

  "timestamp": 0.033,
  "frame_index": 1,
  "episode_index": 0,
  "task_index": 0
}
```

### Metadata Schema

The `info.json` contains feature definitions with meaningful names:

```json
{
  "features": {
    "action.end_effector_pose": {
      "dtype": "float32",
      "shape": [7],
      "names": ["x", "y", "z", "qx", "qy", "qz", "qw"]
    },
    "observation.camera_poses.front": {
      "dtype": "float32",
      "shape": [7],
      "names": ["x", "y", "z", "qx", "qy", "qz", "qw"]
    }
  }
}
```

## Key Architectural Benefits

### 🎯 **Clean Data Separation**

- Joint data grouped logically
- Pose data as separate accessible arrays
- Video data in efficient MP4 format

### 🏗️ **Extensible Design**

- Type-based feature detection
- Easy to add new pose types
- Flexible field naming system

### 🔧 **Type Safety**

- Consistent float32 dtype for poses
- Proper error handling and fallbacks
- Validation at each processing step

### 📊 **Meaningful Metadata**

- Semantic field names (x,y,z,qx,qy,qz,qw)
- Complete feature documentation
- Easy data analysis and visualization

### 🚀 **Performance Optimized**

- Direct numpy array assignment for poses
- Efficient video encoding
- Minimal data copying

## Usage Examples

### Loading Dataset

```python
from RynnMotion.RynnDatasets import RynnDataset

# Load dataset
dataset = RynnDataset("path/to/outputs/MMDD_HHMM")

# Access different data types
joint_actions = dataset["action"]  # Joint commands only
ee_poses = dataset["action.end_effector_pose"]  # End-effector poses
camera_poses = dataset["observation.camera_poses.front"]  # Camera poses
```

### Data Analysis

```python
import pandas as pd

# Load parquet directly
df = pd.read_parquet("episode_000000.parquet")

# Analyze end-effector trajectory
ee_trajectory = df["observation.end_effector_pose"]
positions = ee_trajectory.apply(lambda x: x[:3])  # Extract x,y,z
orientations = ee_trajectory.apply(lambda x: x[3:])  # Extract qx,qy,qz,qw
```

## Recent Improvements

### ✅ **Enhanced Pose Support**

- Added list-type feature detection in `hw_to_dataset_features`
- Implemented pose array handling in `build_dataset_frame`
- Clean separation from joint and camera processing

### ✅ **Meaningful Field Names**

- Changed from generic `dim_0, dim_1, ...` to semantic `x, y, z, qx, qy, qz, qw`
- Improved readability and usability
- Maintained backward compatibility

### ✅ **Type Safety**

- All pose arrays converted to consistent float32 dtype
- Eliminated dtype mismatch errors
- Robust error handling throughout pipeline

### ✅ **Architecture Cleanup**

- Elegant three-way feature detection (float/tuple/list)
- Extensible design for future pose types
- Clean code organization and documentation

This pipeline now provides a robust, extensible, and user-friendly framework for robot dataset collection and storage.
