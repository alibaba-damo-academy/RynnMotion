"""Robot Discovery — Python port of C++ RobotDiscovery from models/discovery.hpp.

Scans the models/ directory to find all robots and scenes using the NUMBER.name convention.
Supports name, alias, and number lookup.

Example:
    from RynnMotion.manager.discovery import get_discovery
    d = get_discovery()
    info = d.get_robot_info("so101")
    scene = d.get_scene(info, "default")
    print(scene.full_path)  # /path/to/models/3.robot_arm/24.so101/scene/scene.xml
"""

import re
from dataclasses import dataclass, field
from pathlib import Path

from RynnMotion.utils.path_config import get_models_root

# Mirrors C++ ROBOT_ALIASES from models/discovery.hpp
ROBOT_ALIASES: dict[str, list[str]] = {
    "fr3": ["franka", "panda"],
    "ur5e": ["ur5-e", "ur5_e"],
    "piper": [],
    "rm75": ["realman75"],
    "so101": ["so-101", "so_101", "soarm101"],
    "rizon4s": ["rizon"],
    "eco65": ["eco-65"],
    "dm6dof": ["dm-6dof"],
    "openarm": ["open_arm"],
    "dual_fr3": ["dual-fr3", "dual_franka"],
    "dual_ur5e": ["dual-ur5e"],
    "dual_piper": ["dual-piper"],
    "dual_rm75": ["dual-rm75"],
    "dual_so101": ["dual-so101"],
    "so101_6dof": ["so-101-6dof", "soarm101_6dof"],
    "wuji": ["wuji_hand"],
    "sharpa_wave": ["sharpa", "sharpa-wave"],
    "unitree_g1": ["g1"],
    "agibot_x2": ["x2"],
}

SCENE_ALIASES: dict[str, list[str]] = {
    "default": ["joint", "jointmove"],
    "keyframe": ["wobble", "cycle"],
    "ui": ["tracking"],
    "pickplace": ["pick", "pick-place"],
}


@dataclass
class SceneInfo:
    name: str
    filename: str
    full_path: str


@dataclass
class RobotInfo:
    number: int
    canonical_name: str
    base_path: str
    robot_mjcf: str = ""
    pino_mjcf: str = ""
    scenes: list[SceneInfo] = field(default_factory=list)


class RobotDiscovery:
    def __init__(self):
        self._robots: dict[int, RobotInfo] = {}
        self._name_to_number: dict[str, int] = {}
        self._alias_to_number: dict[str, int] = {}
        self._scanned = False

    def scan(self, models_root: Path | None = None) -> None:
        if self._scanned:
            return
        if models_root is None:
            models_root = get_models_root()
        models_root = Path(models_root)

        for category in sorted(models_root.iterdir()):
            if not category.is_dir() or category.name.startswith("."):
                continue
            for robot_dir in sorted(category.iterdir()):
                if not robot_dir.is_dir() or robot_dir.name.startswith("."):
                    continue
                if robot_dir.name in ("meshes", "assets", "human", "urdf", "textures", "environments"):
                    continue
                m = re.match(r"^(\d+)\.(.+)$", robot_dir.name)
                if not m:
                    continue

                number = int(m.group(1))
                name = m.group(2)

                info = RobotInfo(
                    number=number,
                    canonical_name=name,
                    base_path=str(robot_dir),
                )
                info.robot_mjcf = self._find_robot_mjcf(robot_dir)
                info.pino_mjcf = self._find_pino_mjcf(robot_dir)
                info.scenes = self._scan_scenes(robot_dir)
                self._register(info)

        self._scanned = True

    def get_robot_info(self, name_or_number: str) -> RobotInfo:
        self.scan()
        # Try as number
        try:
            num = int(name_or_number)
            if num in self._robots:
                return self._robots[num]
        except ValueError:
            pass

        lower = name_or_number.lower()
        if lower in self._name_to_number:
            return self._robots[self._name_to_number[lower]]
        if lower in self._alias_to_number:
            return self._robots[self._alias_to_number[lower]]

        available = ", ".join(sorted(self._name_to_number.keys()))
        raise ValueError(f"Robot not found: '{name_or_number}'. Available: {available}")

    def get_scene(self, robot: RobotInfo, scene_name: str | None = None) -> SceneInfo:
        if not robot.scenes:
            raise ValueError(f"No scenes found for robot {robot.canonical_name}")
        if scene_name is None:
            return robot.scenes[0]

        lower = scene_name.lower()
        # Direct match
        for s in robot.scenes:
            if s.name.lower() == lower:
                return s
        # Alias match
        for canonical, aliases in SCENE_ALIASES.items():
            if lower == canonical or lower in [a.lower() for a in aliases]:
                for s in robot.scenes:
                    if s.name.lower() == canonical:
                        return s

        names = [s.name for s in robot.scenes]
        raise ValueError(f"Scene '{scene_name}' not found for {robot.canonical_name}. Available: {names}")

    def get_all_robots(self) -> list[RobotInfo]:
        self.scan()
        return sorted(self._robots.values(), key=lambda r: r.number)

    def _register(self, info: RobotInfo) -> None:
        self._robots[info.number] = info
        self._name_to_number[info.canonical_name.lower()] = info.number
        for alias in ROBOT_ALIASES.get(info.canonical_name, []):
            self._alias_to_number[alias.lower()] = info.number

    @staticmethod
    def _find_robot_mjcf(robot_dir: Path) -> str:
        for d in [robot_dir / "mjcf", robot_dir]:
            if not d.exists():
                continue
            for f in d.iterdir():
                if f.is_file() and ("_robot.xml" in f.name or f.name == "robot.xml"):
                    return str(f)
        return ""

    @staticmethod
    def _find_pino_mjcf(robot_dir: Path) -> str:
        for d in [robot_dir / "mjcf", robot_dir]:
            if not d.exists():
                continue
            for f in d.iterdir():
                if f.is_file() and "_pinocchio.xml" in f.name:
                    return str(f)
        return ""

    @staticmethod
    def _scan_scenes(robot_dir: Path) -> list[SceneInfo]:
        scene_dir = robot_dir / "scene"
        if not scene_dir.exists():
            scene_dir = robot_dir

        scenes = []
        for f in sorted(scene_dir.glob("*.xml")):
            if "_robot.xml" in f.name or "_pinocchio.xml" in f.name:
                continue
            name = f.stem
            if name == "scene":
                name = "default"
            elif name.startswith("scene_"):
                name = name[6:]
            scenes.append(SceneInfo(name=name, filename=f.name, full_path=str(f)))
        return scenes


_global_discovery: RobotDiscovery | None = None


def get_discovery() -> RobotDiscovery:
    global _global_discovery
    if _global_discovery is None:
        _global_discovery = RobotDiscovery()
    return _global_discovery
