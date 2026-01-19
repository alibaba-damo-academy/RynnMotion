"""
Intermediate data representation for robot conversion.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict, Tuple, Optional


@dataclass
class InertialInfo:
    mass: float = 1.0
    pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    diaginertia: Tuple[float, float, float] = (0.01, 0.01, 0.01)


@dataclass
class JointInfo:
    name: str
    type: str  # revolute, prismatic, continuous, fixed
    axis: Tuple[float, float, float] = (0, 0, 1)
    range: Tuple[float, float] = (-3.14, 3.14)
    parent_link: str = ""
    child_link: str = ""
    pos: Tuple[float, float, float] = (0, 0, 0)
    quat: Tuple[float, float, float, float] = (1, 0, 0, 0)
    armature: float = 0.1
    damping: float = 1.0
    frictionloss: float = 0.0
    force_range: Tuple[float, float] = (-87, 87)


@dataclass
class GeomInfo:
    name: str = ""
    type: str = "mesh"  # mesh, box, sphere, cylinder, capsule
    mesh: str = ""
    material: str = ""
    size: Tuple[float, ...] = ()
    pos: Tuple[float, float, float] = (0, 0, 0)
    quat: Tuple[float, float, float, float] = (1, 0, 0, 0)
    is_visual: bool = True
    rgba: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)


@dataclass
class LinkInfo:
    name: str
    inertial: Optional[InertialInfo] = None
    visual_geoms: List[GeomInfo] = field(default_factory=list)
    collision_geoms: List[GeomInfo] = field(default_factory=list)
    pos: Tuple[float, float, float] = (0, 0, 0)
    quat: Tuple[float, float, float, float] = (1, 0, 0, 0)


@dataclass
class MeshInfo:
    name: str
    file_path: Path
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass
class MaterialInfo:
    name: str
    rgba: Tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1.0)
    texture: str = ""
    specular: float = 0.5
    shininess: float = 0.5


@dataclass
class ActuatorInfo:
    name: str
    joint: str
    type: str = "position"  # position, velocity, motor, general
    kp: float = 2000.0
    kv: float = 200.0
    ctrl_range: Tuple[float, float] = (-1, 1)
    force_range: Tuple[float, float] = (-100, 100)
    tendon: str = ""


@dataclass
class SiteInfo:
    name: str
    pos: Tuple[float, float, float] = (0, 0, 0)
    euler: Tuple[float, float, float] = (0, 0, 0)
    quat: Optional[Tuple[float, float, float, float]] = None
    parent_link: str = ""
    size: float = 0.001
    rgba: Tuple[float, float, float, float] = (1, 0, 0, 1)


@dataclass
class TendonInfo:
    name: str
    joints: List[str] = field(default_factory=list)
    coefs: List[float] = field(default_factory=list)


@dataclass
class ContactExclude:
    body1: str
    body2: str


@dataclass
class RobotData:
    """Intermediate representation of a robot for conversion."""

    name: str
    links: List[LinkInfo] = field(default_factory=list)
    joints: List[JointInfo] = field(default_factory=list)

    motion_joints: List[str] = field(default_factory=list)
    gripper_joints: List[str] = field(default_factory=list)

    meshes: Dict[str, MeshInfo] = field(default_factory=dict)
    materials: Dict[str, MaterialInfo] = field(default_factory=dict)

    actuators: List[ActuatorInfo] = field(default_factory=list)
    tendons: List[TendonInfo] = field(default_factory=list)

    ee_site: Optional[SiteInfo] = None
    extra_sites: List[SiteInfo] = field(default_factory=list)

    contact_excludes: List[ContactExclude] = field(default_factory=list)

    home_qpos: List[float] = field(default_factory=list)
    standby_qpos: List[float] = field(default_factory=list)

    source_path: Optional[Path] = None

    @property
    def motion_dof(self) -> int:
        return len(self.motion_joints)

    @property
    def gripper_dof(self) -> int:
        return len(self.gripper_joints)

    @property
    def total_dof(self) -> int:
        return self.motion_dof + self.gripper_dof

    def get_joint(self, name: str) -> Optional[JointInfo]:
        for j in self.joints:
            if j.name == name:
                return j
        return None

    def get_link(self, name: str) -> Optional[LinkInfo]:
        for link in self.links:
            if link.name == name:
                return link
        return None

    def get_kinematic_chain(self) -> List[str]:
        """Get ordered list of links in kinematic chain."""
        chain = []
        for jname in self.motion_joints:
            joint = self.get_joint(jname)
            if joint and joint.child_link not in chain:
                chain.append(joint.child_link)
        return chain


GRIPPER_PATTERNS = ['finger', 'gripper', 'grip', 'hand', 'jaw', 'claw']
EE_PATTERNS = ['ee', 'tool', 'flange', 'tcp', 'end_effector', 'endpoint', 'ee_link']


def is_gripper_joint(name: str) -> bool:
    name_lower = name.lower()
    return any(p in name_lower for p in GRIPPER_PATTERNS)


def is_ee_link(name: str) -> bool:
    name_lower = name.lower()
    return any(p in name_lower for p in EE_PATTERNS)
