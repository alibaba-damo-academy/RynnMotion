"""
MuJoCo Menagerie to RobotData parser.

Parses existing MuJoCo Menagerie robots and converts them to RobotData.
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Dict, Tuple, Optional, Set
import re
import math

from robot_data import (
    RobotData, JointInfo, LinkInfo, MeshInfo, MaterialInfo, GeomInfo,
    InertialInfo, ActuatorInfo, SiteInfo, TendonInfo, ContactExclude,
    is_gripper_joint, is_ee_link
)


class MenagerieParser:
    """Parse MuJoCo Menagerie robots to RobotData."""

    def __init__(
        self,
        menagerie_path: Path,
        robot_name: Optional[str] = None,
        ee_link: Optional[str] = None,
        gripper_joints: Optional[List[str]] = None,
        home_qpos: Optional[List[float]] = None,
        standby_qpos: Optional[List[float]] = None,
    ):
        self.menagerie_path = Path(menagerie_path)
        self.robot_name_override = robot_name
        self.ee_link_override = ee_link
        self.gripper_joints_override = gripper_joints
        self.home_qpos_override = home_qpos
        self.standby_qpos_override = standby_qpos

        self.main_file: Optional[Path] = None
        self.resolved_tree: Optional[ET.Element] = None

    def convert(self) -> RobotData:
        """Parse Menagerie MJCF and return RobotData."""
        self.main_file = self._find_main_file()
        self.resolved_tree = self._resolve_includes(self.main_file)

        robot_name = self.robot_name_override or self.resolved_tree.get('model', 'robot')

        links = self._parse_bodies(self.resolved_tree)
        joints = self._parse_joints(self.resolved_tree)
        meshes = self._collect_meshes(self.resolved_tree)
        materials = self._parse_materials(self.resolved_tree)

        actuators = self._parse_actuators(self.resolved_tree)
        tendons = self._parse_tendons(self.resolved_tree)

        motion_joints, gripper_joints = self._classify_actuators(actuators, tendons)

        ee_site = self._find_ee_site(self.resolved_tree, joints, motion_joints)

        contact_excludes = self._parse_contacts(self.resolved_tree)

        keyframes = self._parse_keyframes(self.resolved_tree)
        home_qpos = self.home_qpos_override or keyframes.get('home', [])
        standby_qpos = self.standby_qpos_override or keyframes.get('standby1', keyframes.get('ready', []))

        return RobotData(
            name=robot_name,
            links=links,
            joints=joints,
            motion_joints=motion_joints,
            gripper_joints=gripper_joints,
            meshes=meshes,
            materials=materials,
            actuators=actuators,
            tendons=tendons,
            ee_site=ee_site,
            contact_excludes=contact_excludes,
            home_qpos=home_qpos,
            standby_qpos=standby_qpos,
            source_path=self.menagerie_path,
        )

    def _find_main_file(self) -> Path:
        """Find main MJCF file in menagerie directory."""
        if self.menagerie_path.suffix == '.xml':
            return self.menagerie_path

        xml_files = list(self.menagerie_path.glob('*.xml'))

        for f in xml_files:
            name = f.name.lower()
            if 'scene' not in name:
                return f

        for f in xml_files:
            name = f.name.lower()
            if 'scene' in name:
                return f

        if xml_files:
            return xml_files[0]

        raise FileNotFoundError(f"No XML found in {self.menagerie_path}")

    def _resolve_includes(self, xml_file: Path) -> ET.Element:
        """Resolve all include elements recursively."""
        tree = ET.parse(xml_file)
        root = tree.getroot()

        self._resolve_includes_recursive(root, xml_file.parent)

        return root

    def _resolve_includes_recursive(self, elem: ET.Element, base_dir: Path):
        """Recursively resolve include elements."""
        includes_to_process = []
        for child in list(elem):
            if child.tag == 'include':
                includes_to_process.append(child)
            else:
                self._resolve_includes_recursive(child, base_dir)

        for include in includes_to_process:
            file_attr = include.get('file', '')
            if not file_attr:
                continue

            include_path = base_dir / file_attr
            if not include_path.exists():
                continue

            try:
                include_tree = ET.parse(include_path)
                include_root = include_tree.getroot()

                self._resolve_includes_recursive(include_root, include_path.parent)

                idx = list(elem).index(include)
                elem.remove(include)

                for i, child in enumerate(include_root):
                    elem.insert(idx + i, child)

            except ET.ParseError:
                continue

    def _parse_bodies(self, root: ET.Element) -> List[LinkInfo]:
        """Parse body elements from MJCF."""
        links = []
        worldbody = root.find('worldbody')
        if worldbody is None:
            return links

        self._parse_body_recursive(worldbody, links)
        return links

    def _parse_body_recursive(self, elem: ET.Element, links: List[LinkInfo], depth: int = 0):
        """Recursively parse body elements."""
        for body in elem.findall('body'):
            name = body.get('name', f'body_{len(links)}')

            inertial = None
            inertial_elem = body.find('inertial')
            if inertial_elem is not None:
                inertial = self._parse_inertial_mjcf(inertial_elem)

            pos = self._parse_vec3(body.get('pos', '0 0 0'))
            quat = self._parse_quat(body.get('quat', '1 0 0 0'))

            visual_geoms = []
            collision_geoms = []
            for geom in body.findall('geom'):
                geom_info = self._parse_geom_mjcf(geom)
                if geom_info:
                    geom_class = geom.get('class', '')
                    if 'collision' in geom_class.lower() or geom.get('contype', '1') != '0':
                        collision_geoms.append(geom_info)
                    if 'visual' in geom_class.lower() or geom.get('contype', '1') == '0':
                        visual_geoms.append(geom_info)
                    if not geom_class:
                        visual_geoms.append(geom_info)

            links.append(LinkInfo(
                name=name,
                inertial=inertial,
                visual_geoms=visual_geoms,
                collision_geoms=collision_geoms,
                pos=pos,
                quat=quat,
            ))

            self._parse_body_recursive(body, links, depth + 1)

    def _parse_inertial_mjcf(self, elem: ET.Element) -> InertialInfo:
        """Parse MJCF inertial element."""
        mass = float(elem.get('mass', 1.0))
        pos = self._parse_vec3(elem.get('pos', '0 0 0'))
        quat = self._parse_quat(elem.get('quat', '1 0 0 0'))
        diaginertia = self._parse_vec3(elem.get('diaginertia', '0.01 0.01 0.01'))

        return InertialInfo(mass=mass, pos=pos, quat=quat, diaginertia=diaginertia)

    def _parse_geom_mjcf(self, elem: ET.Element) -> Optional[GeomInfo]:
        """Parse MJCF geom element."""
        geom_type = elem.get('type', 'sphere')
        mesh = elem.get('mesh', '')
        material = elem.get('material', '')
        pos = self._parse_vec3(elem.get('pos', '0 0 0'))
        quat = self._parse_quat(elem.get('quat', '1 0 0 0'))

        size_str = elem.get('size', '')
        size = tuple(float(x) for x in size_str.split()) if size_str else ()

        rgba_str = elem.get('rgba', '')
        rgba = tuple(float(x) for x in rgba_str.split()) if rgba_str else (0.5, 0.5, 0.5, 1.0)

        is_visual = elem.get('contype', '1') == '0'

        return GeomInfo(
            name=elem.get('name', ''),
            type=geom_type if not mesh else 'mesh',
            mesh=mesh,
            material=material,
            size=size,
            pos=pos,
            quat=quat,
            is_visual=is_visual,
            rgba=rgba,
        )

    def _parse_joints(self, root: ET.Element) -> List[JointInfo]:
        """Parse joint elements from MJCF."""
        joints = []
        worldbody = root.find('worldbody')
        if worldbody is None:
            return joints

        self._parse_joints_recursive(worldbody, joints, '')
        return joints

    def _parse_joints_recursive(self, elem: ET.Element, joints: List[JointInfo], parent_link: str):
        """Recursively parse joints."""
        for body in elem.findall('body'):
            body_name = body.get('name', '')

            for joint in body.findall('joint'):
                jname = joint.get('name', '')
                jtype = joint.get('type', 'hinge')

                if jtype == 'hinge':
                    jtype = 'revolute'
                elif jtype == 'slide':
                    jtype = 'prismatic'

                axis = self._parse_vec3(joint.get('axis', '0 0 1'))
                range_str = joint.get('range', '-3.14159 3.14159')
                joint_range = tuple(float(x) for x in range_str.split())

                pos = self._parse_vec3(body.get('pos', '0 0 0'))
                quat = self._parse_quat(body.get('quat', '1 0 0 0'))

                armature = float(joint.get('armature', 0.1))
                damping = float(joint.get('damping', 1.0))

                frc_range_str = joint.get('actuatorfrcrange', '-87 87')
                force_range = tuple(float(x) for x in frc_range_str.split())

                joints.append(JointInfo(
                    name=jname,
                    type=jtype,
                    axis=axis,
                    range=joint_range,
                    parent_link=parent_link,
                    child_link=body_name,
                    pos=pos,
                    quat=quat,
                    armature=armature,
                    damping=damping,
                    force_range=force_range,
                ))

            self._parse_joints_recursive(body, joints, body_name)

    def _collect_meshes(self, root: ET.Element) -> Dict[str, MeshInfo]:
        """Collect mesh definitions."""
        meshes = {}
        asset = root.find('asset')
        if asset is None:
            return meshes

        for mesh in asset.findall('mesh'):
            name = mesh.get('name', '')
            file_attr = mesh.get('file', '')
            if not name and file_attr:
                name = Path(file_attr).stem

            if file_attr:
                mesh_path = self.menagerie_path / file_attr
                if not mesh_path.exists():
                    mesh_path = self.menagerie_path.parent / file_attr
                if not mesh_path.exists():
                    for pattern in ['**/' + Path(file_attr).name, 'assets/**/*.obj', 'assets/**/*.stl']:
                        matches = list(self.menagerie_path.glob(pattern))
                        if matches:
                            mesh_path = matches[0]
                            break

                scale_str = mesh.get('scale', '1 1 1')
                scale = tuple(float(x) for x in scale_str.split())

                meshes[name] = MeshInfo(name=name, file_path=mesh_path, scale=scale)

        return meshes

    def _parse_materials(self, root: ET.Element) -> Dict[str, MaterialInfo]:
        """Parse material definitions."""
        materials = {}
        asset = root.find('asset')
        if asset is None:
            return materials

        for material in asset.findall('material'):
            name = material.get('name', '')
            if not name:
                continue

            rgba_str = material.get('rgba', '0.5 0.5 0.5 1.0')
            rgba = tuple(float(x) for x in rgba_str.split())
            specular = float(material.get('specular', 0.5))
            shininess = float(material.get('shininess', 0.5))

            materials[name] = MaterialInfo(
                name=name,
                rgba=rgba,
                specular=specular,
                shininess=shininess,
            )

        return materials

    def _parse_actuators(self, root: ET.Element) -> List[ActuatorInfo]:
        """Parse actuator definitions."""
        actuators = []
        actuator_elem = root.find('actuator')
        if actuator_elem is None:
            return actuators

        for act in actuator_elem:
            act_type = act.tag
            name = act.get('name', '')
            joint = act.get('joint', '')
            tendon = act.get('tendon', '')

            kp = float(act.get('kp', 2000))
            kv = float(act.get('kv', act.get('dampratio', 200)))

            ctrlrange_str = act.get('ctrlrange', '-1 1')
            ctrl_range = tuple(float(x) for x in ctrlrange_str.split())

            forcerange_str = act.get('forcerange', '-100 100')
            force_range = tuple(float(x) for x in forcerange_str.split())

            actuators.append(ActuatorInfo(
                name=name,
                joint=joint,
                type=act_type,
                kp=kp,
                kv=kv,
                ctrl_range=ctrl_range,
                force_range=force_range,
                tendon=tendon,
            ))

        return actuators

    def _parse_tendons(self, root: ET.Element) -> List[TendonInfo]:
        """Parse tendon definitions."""
        tendons = []
        tendon_elem = root.find('tendon')
        if tendon_elem is None:
            return tendons

        for fixed in tendon_elem.findall('fixed'):
            name = fixed.get('name', '')
            joints = []
            coefs = []

            for joint_ref in fixed.findall('joint'):
                joints.append(joint_ref.get('joint', ''))
                coefs.append(float(joint_ref.get('coef', 1.0)))

            tendons.append(TendonInfo(name=name, joints=joints, coefs=coefs))

        return tendons

    def _classify_actuators(
        self,
        actuators: List[ActuatorInfo],
        tendons: List[TendonInfo]
    ) -> Tuple[List[str], List[str]]:
        """Classify actuators as motion or gripper."""
        if self.gripper_joints_override:
            gripper_set = set(self.gripper_joints_override)
            motion = [a.joint for a in actuators if a.joint and a.joint not in gripper_set]
            gripper = list(gripper_set)
        else:
            motion = []
            gripper = []

            tendon_joints = set()
            for t in tendons:
                tendon_joints.update(t.joints)

            for act in actuators:
                if act.tendon:
                    gripper.append(act.name)
                elif act.joint and is_gripper_joint(act.joint):
                    gripper.append(act.joint)
                elif act.joint in tendon_joints:
                    continue
                elif act.joint:
                    motion.append(act.joint)

        return motion, gripper

    def _find_ee_site(
        self,
        root: ET.Element,
        joints: List[JointInfo],
        motion_joints: List[str]
    ) -> SiteInfo:
        """Find or create EE site."""
        if self.ee_link_override:
            return SiteInfo(
                name='EE',
                pos=(0, 0, 0.1),
                euler=(3.14159, 0, 0),
                parent_link=self.ee_link_override,
            )

        for site in root.iter('site'):
            site_name = site.get('name', '').lower()
            if 'ee' in site_name or 'tcp' in site_name or 'tool' in site_name:
                pos = self._parse_vec3(site.get('pos', '0 0 0'))
                euler_str = site.get('euler', '')
                euler = tuple(float(x) for x in euler_str.split()) if euler_str else (0, 0, 0)

                parent = site.getparent() if hasattr(site, 'getparent') else None
                parent_link = ''
                if parent is not None and parent.tag == 'body':
                    parent_link = parent.get('name', '')

                return SiteInfo(
                    name='EE',
                    pos=pos,
                    euler=euler,
                    parent_link=parent_link,
                )

        if motion_joints:
            last_joint = motion_joints[-1]
            for j in joints:
                if j.name == last_joint:
                    return SiteInfo(
                        name='EE',
                        pos=(0, 0, 0.1),
                        euler=(3.14159, 0, 0),
                        parent_link=j.child_link,
                    )

        return SiteInfo(name='EE', pos=(0, 0, 0.1), euler=(3.14159, 0, 0))

    def _parse_contacts(self, root: ET.Element) -> List[ContactExclude]:
        """Parse contact exclusions."""
        excludes = []
        contact = root.find('contact')
        if contact is None:
            return excludes

        for exclude in contact.findall('exclude'):
            body1 = exclude.get('body1', '')
            body2 = exclude.get('body2', '')
            if body1 and body2:
                excludes.append(ContactExclude(body1=body1, body2=body2))

        return excludes

    def _parse_keyframes(self, root: ET.Element) -> Dict[str, List[float]]:
        """Parse keyframe definitions."""
        keyframes = {}
        keyframe_elem = root.find('keyframe')
        if keyframe_elem is None:
            return keyframes

        for key in keyframe_elem.findall('key'):
            name = key.get('name', '')
            qpos_str = key.get('qpos', '')
            if name and qpos_str:
                qpos = [float(x) for x in qpos_str.split()]
                keyframes[name] = qpos

        return keyframes

    def _parse_vec3(self, s: str) -> Tuple[float, float, float]:
        """Parse 3D vector string."""
        parts = s.split()
        if len(parts) >= 3:
            return (float(parts[0]), float(parts[1]), float(parts[2]))
        return (0.0, 0.0, 0.0)

    def _parse_quat(self, s: str) -> Tuple[float, float, float, float]:
        """Parse quaternion string (w, x, y, z)."""
        parts = s.split()
        if len(parts) >= 4:
            return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
        return (1.0, 0.0, 0.0, 0.0)
