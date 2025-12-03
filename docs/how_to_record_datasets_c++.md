# How to Record Datasets in C++ (MuJoCo)

Record robot manipulation datasets from MuJoCo simulation, compatible with Python `RynnDataset`.

---

## Launch Simulation

```bash
cd build
./mujocoExe <robotAlias> <sceneAlias>
```

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| **R** | Toggle recording (start/stop) |
| **N** | End current episode, start new one |

---

## Output Location

```
RynnMotion/record/mj_<date_time>/
├── meta/
│   ├── info.json
│   └── episodes.jsonl
├── data/chunk-000/
│   └── episode_000000.hdf5
└── videos/chunk-000/
    └── observation.images.{camera}/
        └── episode_000000.mp4
```

---

## Recorded Data

| Data | Source |
|------|--------|
| Joint positions, velocities, torques | `qFb`, `qdFb`, `qtauFb` |
| Joint commands | `qCmd`, `qdCmd`, `qtauCmd` |
| EE poses | Position + quaternion per arm |
| Video | All MJCF-defined cameras (RGB) |

---

## Loading in Python

```python
from RynnMotion.RynnDatasets import RynnDataset

dataset = RynnDataset(repo_id="mj_<date_time>", root="./record")
sample = dataset[0]
```
