"""
URDF to RobotData converter.

Parses ROS URDF packages and converts them to RobotData intermediate representation.
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Dict, Tuple, Optional
import shutil
import re
import math

from robot_data import (
    RobotData, JointInfo, LinkInfo, MeshInfo, MaterialInfo, GeomInfo,
    InertialInfo, ActuatorInfo, SiteInfo, ContactExclude,
    is_gripper_joint, is_ee_link, GRIPPER_PATTERNS, EE_PATTERNS
)


class UrdfConverter:
    """Convert ROS URDF packages to RobotData."""

    def __init__(
        self,
        urdf_path: Path,
        robot_name: Optional[str] = None,
        ee_link: Optional[str] = None,
        gripper_joints: Optional[List[str]] = None,
        home_qpos: Optional[List[float]] = None,
        standby_qpos: Optional[List[float]] = None,
    ):
        self.urdf_path = Path(urdf_path)
        self.robot_name_override = robot_name
        self.ee_link_override = ee_link
        self.gripper_joints_override = gripper_joints
        self.home_qpos_override = home_qpos
        self.standby_qpos_override = standby_qpos

        self.urdf_file: Optional[Path] = None
        self.mesh_dir: Optional[Path] = None

    def convert(self) -> RobotData:
        """Parse URDF and return RobotData."""
        self.urdf_file = self._find_urdf_file()
        self.mesh_dir = self._find_mesh_dir()

        tree = ET.parse(self.urdf_file)
        root = tree.getroot()

        robot_name = self.robot_name_override or root.get('name', 'robot')

        links = self._parse_links(root)
        joints = self._parse_joints(root)
        meshes = self._collect_meshes(root)
        materials = self._parse_materials(root)

        motion_joints, gripper_joints = self._classify_joints(joints)

        ee_site = self._detect_ee_site(joints, links, motion_joints)

        actuators = self._create_actuators(motion_joints, gripper_joints, joints)

        contact_excludes = self._generate_contact_excludes(joints, gripper_joints)

        home_qpos = self.home_qpos_override or self._compute_home_position(joints, motion_joints + gripper_joints)
        standby_qpos = self.standby_qpos_override or self._compute_standby_position(joints, motion_joints)

        return RobotData(
            name=robot_name,
            links=links,
            joints=joints,
            motion_joints=motion_joints,
            gripper_joints=gripper_joints,
            meshes=meshes,
            materials=materials,
            actuators=actuators,
            ee_site=ee_site,
            contact_excludes=contact_excludes,
            home_qpos=home_qpos,
            standby_qpos=standby_qpos,
            source_path=self.urdf_path,
        )

    def _find_urdf_file(self) -> Path:
        """Find URDF file in package."""
        if self.urdf_path.suffix == '.urdf':
            return self.urdf_path

        search_patterns = [
            'urdf/*.urdf',
            '*.urdf',
            'robot/*.urdf',
            'description/*.urdf',
        ]

        for pattern in search_patterns:
            files = list(self.urdf_path.glob(pattern))
            if files:
                urdf_files = [f for f in files if not f.name.endswith('.xacro')]
                if urdf_files:
                    return urdf_files[0]

        xacro_patterns = ['urdf/*.urdf.xacro', '*.xacro']
        for pattern in xacro_patterns:
            files = list(self.urdf_path.glob(pattern))
            if files:
                return self._process_xacro(files[0])

        raise FileNotFoundError(f"No URDF found in {self.urdf_path}")

    def _process_xacro(self, xacro_file: Path) -> Path:
        """Process xacro to URDF."""
        try:
            from xacrodoc import XacroDoc
            doc = XacroDoc.from_file(str(xacro_file))
            output_file = xacro_file.parent / f"{xacro_file.stem.replace('.urdf', '')}_processed.urdf"
            doc.to_urdf_file(str(output_file))
            return output_file
        except ImportError:
            raise ImportError("xacrodoc required for xacro processing: pip install xacrodoc")

    def _find_mesh_dir(self) -> Optional[Path]:
        """Find mesh directory in package."""
        if self.urdf_path.is_file():
            base = self.urdf_path.parent
        else:
            base = self.urdf_path

        for name in ['meshes', 'mesh', 'visual', 'collision', 'assets']:
            mesh_dir = base / name
            if mesh_dir.exists():
                return mesh_dir

        return base

    def _parse_links(self, root: ET.Element) -> List[LinkInfo]:
        """Parse link elements from URDF."""
        links = []
        for link_elem in root.findall('link'):
            name = link_elem.get('name', '')

            inertial = None
            inertial_elem = link_elem.find('inertial')
            if inertial_elem is not None:
                inertial = self._parse_inertial(inertial_elem)

            visual_geoms = []
            for visual in link_elem.findall('visual'):
                geom = self._parse_geometry(visual, is_visual=True)
                if geom:
                    visual_geoms.append(geom)

            collision_geoms = []
            for collision in link_elem.findall('collision'):
                geom = self._parse_geometry(collision, is_visual=False)
                if geom:
                    collision_geoms.append(geom)

            links.append(LinkInfo(
                name=name,
                inertial=inertial,
                visual_geoms=visual_geoms,
                collision_geoms=collision_geoms,
            ))

        return links

    def _parse_inertial(self, elem: ET.Element) -> InertialInfo:
        """Parse inertial element."""
        mass_elem = elem.find('mass')
        mass = float(mass_elem.get('value', 1.0)) if mass_elem is not None else 1.0

        origin_elem = elem.find('origin')
        pos = (0.0, 0.0, 0.0)
        quat = (1.0, 0.0, 0.0, 0.0)
        if origin_elem is not None:
            xyz = origin_elem.get('xyz', '0 0 0')
            pos = tuple(float(x) for x in xyz.split())
            rpy = origin_elem.get('rpy', '0 0 0')
            quat = self._rpy_to_quat(rpy)

        inertia_elem = elem.find('inertia')
        diaginertia = (0.01, 0.01, 0.01)
        if inertia_elem is not None:
            ixx = float(inertia_elem.get('ixx', 0.01))
            iyy = float(inertia_elem.get('iyy', 0.01))
            izz = float(inertia_elem.get('izz', 0.01))
            diaginertia = (ixx, iyy, izz)

        return InertialInfo(mass=mass, pos=pos, quat=quat, diaginertia=diaginertia)

    def _parse_geometry(self, elem: ET.Element, is_visual: bool) -> Optional[GeomInfo]:
        """Parse visual/collision geometry."""
        geom_elem = elem.find('geometry')
        if geom_elem is None:
            return None

        origin_elem = elem.find('origin')
        pos = (0.0, 0.0, 0.0)
        quat = (1.0, 0.0, 0.0, 0.0)
        if origin_elem is not None:
            xyz = origin_elem.get('xyz', '0 0 0')
            pos = tuple(float(x) for x in xyz.split())
            rpy = origin_elem.get('rpy', '0 0 0')
            quat = self._rpy_to_quat(rpy)

        mesh_elem = geom_elem.find('mesh')
        if mesh_elem is not None:
            filename = mesh_elem.get('filename', '')
            mesh_name = self._extract_mesh_name(filename)
            return GeomInfo(
                type='mesh',
                mesh=mesh_name,
                pos=pos,
                quat=quat,
                is_visual=is_visual,
            )

        box_elem = geom_elem.find('box')
        if box_elem is not None:
            size_str = box_elem.get('size', '0.1 0.1 0.1')
            size = tuple(float(x) / 2 for x in size_str.split())  # URDF full size -> MuJoCo half-size
            return GeomInfo(type='box', size=size, pos=pos, quat=quat, is_visual=is_visual)

        cylinder_elem = geom_elem.find('cylinder')
        if cylinder_elem is not None:
            radius = float(cylinder_elem.get('radius', 0.05))
            length = float(cylinder_elem.get('length', 0.1))
            return GeomInfo(type='cylinder', size=(radius, length / 2), pos=pos, quat=quat, is_visual=is_visual)

        sphere_elem = geom_elem.find('sphere')
        if sphere_elem is not None:
            radius = float(sphere_elem.get('radius', 0.05))
            return GeomInfo(type='sphere', size=(radius,), pos=pos, quat=quat, is_visual=is_visual)

        return None

    def _parse_joints(self, root: ET.Element) -> List[JointInfo]:
        """Parse joint elements from URDF."""
        joints = []
        for joint_elem in root.findall('joint'):
            name = joint_elem.get('name', '')
            jtype = joint_elem.get('type', 'fixed')

            if jtype == 'continuous':
                jtype = 'revolute'

            parent_elem = joint_elem.find('parent')
            child_elem = joint_elem.find('child')
            parent_link = parent_elem.get('link', '') if parent_elem is not None else ''
            child_link = child_elem.get('link', '') if child_elem is not None else ''

            axis_elem = joint_elem.find('axis')
            axis = (0, 0, 1)
            if axis_elem is not None:
                xyz = axis_elem.get('xyz', '0 0 1')
                axis = tuple(float(x) for x in xyz.split())

            limit_elem = joint_elem.find('limit')
            joint_range = (-3.14159, 3.14159)
            force_range = (-87, 87)
            if limit_elem is not None:
                lower = float(limit_elem.get('lower', -3.14159))
                upper = float(limit_elem.get('upper', 3.14159))
                joint_range = (lower, upper)
                effort = float(limit_elem.get('effort', 87))
                force_range = (-effort, effort)

            origin_elem = joint_elem.find('origin')
            pos = (0.0, 0.0, 0.0)
            quat = (1.0, 0.0, 0.0, 0.0)
            if origin_elem is not None:
                xyz = origin_elem.get('xyz', '0 0 0')
                pos = tuple(float(x) for x in xyz.split())
                rpy = origin_elem.get('rpy', '0 0 0')
                quat = self._rpy_to_quat(rpy)

            dynamics_elem = joint_elem.find('dynamics')
            damping = 1.0
            if dynamics_elem is not None:
                damping = float(dynamics_elem.get('damping', 1.0))

            joints.append(JointInfo(
                name=name,
                type=jtype,
                axis=axis,
                range=joint_range,
                parent_link=parent_link,
                child_link=child_link,
                pos=pos,
                quat=quat,
                damping=damping,
                force_range=force_range,
            ))

        return joints

    def _parse_materials(self, root: ET.Element) -> Dict[str, MaterialInfo]:
        """Parse material definitions."""
        materials = {}

        for material_elem in root.findall('.//material'):
            name = material_elem.get('name', '')
            if not name:
                continue

            color_elem = material_elem.find('color')
            rgba = (0.5, 0.5, 0.5, 1.0)
            if color_elem is not None:
                rgba_str = color_elem.get('rgba', '0.5 0.5 0.5 1.0')
                rgba = tuple(float(x) for x in rgba_str.split())

            materials[name] = MaterialInfo(name=name, rgba=rgba)

        return materials

    def _collect_meshes(self, root: ET.Element) -> Dict[str, MeshInfo]:
        """Collect all mesh references."""
        meshes = {}

        for mesh_elem in root.findall('.//mesh'):
            filename = mesh_elem.get('filename', '')
            if not filename:
                continue

            mesh_name = self._extract_mesh_name(filename)
            mesh_path = self._resolve_mesh_path(filename)

            if mesh_path and mesh_path.exists():
                scale_str = mesh_elem.get('scale', '1 1 1')
                scale = tuple(float(x) for x in scale_str.split())
                meshes[mesh_name] = MeshInfo(name=mesh_name, file_path=mesh_path, scale=scale)

        return meshes

    def _extract_mesh_name(self, filename: str) -> str:
        """Extract mesh name from filename."""
        filename = filename.replace('package://', '')
        parts = filename.split('/')
        name = parts[-1]
        name = re.sub(r'\.(stl|obj|dae|STL|OBJ|DAE)$', '', name)
        return name

    def _resolve_mesh_path(self, filename: str) -> Optional[Path]:
        """Resolve mesh file path."""
        filename = filename.replace('package://', '')

        if self.mesh_dir:
            mesh_file = self.mesh_dir / Path(filename).name
            if mesh_file.exists():
                return mesh_file

            for pattern in ['**/' + Path(filename).name, '**/*.stl', '**/*.obj']:
                matches = list(self.mesh_dir.glob(pattern))
                if matches:
                    return matches[0]

        if self.urdf_file:
            base = self.urdf_file.parent
            mesh_file = base / filename
            if mesh_file.exists():
                return mesh_file

            mesh_file = base / Path(filename).name
            if mesh_file.exists():
                return mesh_file

        return None

    def _classify_joints(self, joints: List[JointInfo]) -> Tuple[List[str], List[str]]:
        """Classify joints as motion or gripper."""
        if self.gripper_joints_override:
            gripper = self.gripper_joints_override
            motion = [j.name for j in joints if j.name not in gripper and j.type != 'fixed']
        else:
            motion = []
            gripper = []
            for joint in joints:
                if joint.type == 'fixed':
                    continue
                if is_gripper_joint(joint.name):
                    gripper.append(joint.name)
                else:
                    motion.append(joint.name)

        return motion, gripper

    def _detect_ee_site(
        self,
        joints: List[JointInfo],
        links: List[LinkInfo],
        motion_joints: List[str]
    ) -> SiteInfo:
        """Detect end-effector site."""
        ee_link_name = self.ee_link_override

        if not ee_link_name:
            for link in links:
                if is_ee_link(link.name):
                    ee_link_name = link.name
                    break

        if not ee_link_name and motion_joints:
            last_joint_name = motion_joints[-1]
            for joint in joints:
                if joint.name == last_joint_name:
                    ee_link_name = joint.child_link
                    break

        if not ee_link_name and links:
            ee_link_name = links[-1].name

        return SiteInfo(
            name='EE',
            pos=(0, 0, 0.1),
            euler=(3.14159, 0, 0),
            parent_link=ee_link_name or '',
            rgba=(1, 0, 0, 1),
        )

    def _create_actuators(
        self,
        motion_joints: List[str],
        gripper_joints: List[str],
        joints: List[JointInfo]
    ) -> List[ActuatorInfo]:
        """Create actuator definitions."""
        actuators = []

        gains_by_index = [
            (4500, 450),  # Joint 1-2: high stiffness
            (4500, 450),
            (3500, 350),  # Joint 3-4: medium
            (3500, 350),
            (2000, 200),  # Joint 5+: lower for wrist
            (2000, 200),
            (2000, 200),
        ]

        for i, jname in enumerate(motion_joints):
            joint = next((j for j in joints if j.name == jname), None)
            kp, kv = gains_by_index[i] if i < len(gains_by_index) else (2000, 200)

            actuators.append(ActuatorInfo(
                name=f'lj{i+1}',
                joint=jname,
                type='position',
                kp=kp,
                kv=kv,
                force_range=joint.force_range if joint else (-87, 87),
            ))

        for i, jname in enumerate(gripper_joints):
            actuators.append(ActuatorInfo(
                name=f'grip{i}' if len(gripper_joints) > 1 else 'grip',
                joint=jname,
                type='position',
                kp=100,
                kv=10,
            ))

        return actuators

    def _generate_contact_excludes(
        self,
        joints: List[JointInfo],
        gripper_joints: List[str]
    ) -> List[ContactExclude]:
        """Generate contact exclusion pairs."""
        excludes = []

        joint_map = {j.name: j for j in joints}
        gripper_links = set()
        for gj in gripper_joints:
            if gj in joint_map:
                gripper_links.add(joint_map[gj].child_link)
                gripper_links.add(joint_map[gj].parent_link)

        gripper_links_list = list(gripper_links)
        for i, link1 in enumerate(gripper_links_list):
            for link2 in gripper_links_list[i+1:]:
                excludes.append(ContactExclude(body1=link1, body2=link2))

        return excludes

    def _compute_home_position(
        self,
        joints: List[JointInfo],
        joint_names: List[str]
    ) -> List[float]:
        """Compute home position."""
        home = []
        joint_map = {j.name: j for j in joints}

        for jname in joint_names:
            joint = joint_map.get(jname)
            if joint:
                lower, upper = joint.range
                if lower <= 0 <= upper:
                    home.append(0.0)
                else:
                    home.append((lower + upper) / 2)
            else:
                home.append(0.0)

        return home

    def _compute_standby_position(
        self,
        joints: List[JointInfo],
        motion_joints: List[str]
    ) -> List[float]:
        """Compute standby position based on DOF count."""
        ndof = len(motion_joints)

        common_standby = {
            7: [0, 0, 0, -1.5708, 0, 1.5708, 0.7854],  # 7-DOF (FR3, Panda style)
            6: [0, -1.5708, 1.5708, 0, 1.5708, 0],     # 6-DOF (UR style)
            5: [0, 0, 0, 0, 0],
            4: [0, 0, 0, 0],
        }

        if ndof in common_standby:
            return common_standby[ndof]

        return self._compute_home_position(joints, motion_joints)

    def _rpy_to_quat(self, rpy_str: str) -> Tuple[float, float, float, float]:
        """Convert RPY string to quaternion (w, x, y, z)."""
        rpy = [float(x) for x in rpy_str.split()]
        if len(rpy) != 3:
            return (1.0, 0.0, 0.0, 0.0)

        roll, pitch, yaw = rpy

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (w, x, y, z)
