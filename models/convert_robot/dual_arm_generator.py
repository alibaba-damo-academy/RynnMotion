"""
Dual-arm generator for RynnMotion.

Generates dual-arm robot configurations from single-arm robots.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path
from typing import Optional

from robot_data import RobotData


class DualArmGenerator:
    """Generate dual-arm variant from single-arm robot."""

    def __init__(
        self,
        robot_data: RobotData,
        single_arm_dir: Path,
        dual_arm_dir: Path,
        arm_separation: float = 0.5,
    ):
        self.data = robot_data
        self.single_arm_dir = Path(single_arm_dir)
        self.dual_arm_dir = Path(dual_arm_dir)
        self.arm_separation = arm_separation

        self.single_arm_name = robot_data.name
        self.dual_arm_name = f'dual_{robot_data.name}'

    def generate(self):
        """Generate dual-arm MJCF files."""
        self.mjcf_dir = self.dual_arm_dir / 'mjcf'
        self.scene_dir = self.dual_arm_dir / 'scene'

        self.mjcf_dir.mkdir(parents=True, exist_ok=True)
        self.scene_dir.mkdir(parents=True, exist_ok=True)

        rel_path = self._compute_relative_path()

        self._generate_dual_robot_xml(rel_path)
        self._generate_dual_pinocchio_xml(rel_path)
        self._generate_dual_scene_xml(rel_path)

        print(f"Generated dual-arm variant at: {self.dual_arm_dir}")

    def _compute_relative_path(self) -> str:
        """Compute relative path from dual-arm mjcf to single-arm mjcf."""
        try:
            rel = self.single_arm_dir.relative_to(self.dual_arm_dir.parent.parent)
            return f'../../{rel}'
        except ValueError:
            return str(self.single_arm_dir)

    def _generate_dual_robot_xml(self, rel_path: str):
        """Generate dual_robot.xml that references single-arm files."""
        root = ET.Element('mujoco', model=self.dual_arm_name)

        ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.80665', integrator='implicitfast')
        ET.SubElement(root, 'compiler', angle='radian', meshdir=f'{rel_path}/assets', autolimits='true')

        asset = ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'texture', type='skybox', builtin='gradient',
                     rgb1='0.3 0.5 0.7', rgb2='0.2 0.2 0.2', width='512', height='3072')

        ET.SubElement(root, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_defaults.xml')
        ET.SubElement(root, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_assets.xml')

        contact = ET.SubElement(root, 'contact')
        ET.SubElement(contact, 'exclude', body1='base_0', body2='base_1')

        worldbody = ET.SubElement(root, 'worldbody')

        left_offset = -self.arm_separation / 2
        right_offset = self.arm_separation / 2

        left_body = ET.SubElement(worldbody, 'body', name='left_arm_base', pos=f'{left_offset} 0 0')
        ET.SubElement(left_body, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_links.xml')

        right_body = ET.SubElement(worldbody, 'body', name='right_arm_base', pos=f'{right_offset} 0 0')
        ET.SubElement(right_body, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_links.xml')

        actuator = ET.SubElement(root, 'actuator')

        for arm_idx, prefix in enumerate(['left', 'right']):
            for i in range(self.data.motion_dof):
                act_info = next((a for a in self.data.actuators if a.joint == self.data.motion_joints[i]), None)
                kp = act_info.kp if act_info else 2000
                kv = act_info.kv if act_info else 200

                act = ET.SubElement(actuator, 'position')
                act.set('class', self.single_arm_name)
                act.set('name', f'{prefix}_lj{i+1}')
                act.set('joint', f'{self.data.motion_joints[i]}_{arm_idx}')
                act.set('kp', str(int(kp)))
                act.set('kv', str(int(kv)))

            if self.data.gripper_joints:
                grip = ET.SubElement(actuator, 'general')
                grip.set('class', self.single_arm_name)
                grip.set('name', f'{prefix}_grip')
                grip.set('tendon', f'split_{arm_idx}')
                grip.set('forcerange', '-100 100')
                grip.set('ctrlrange', '0 255')

        keyframe = ET.SubElement(root, 'keyframe')

        motion_dof = self.data.motion_dof
        single_standby = self.data.standby_qpos[:motion_dof] if self.data.standby_qpos else [0.0] * motion_dof

        dual_home = [0.0] * (motion_dof * 2)
        dual_standby = single_standby + single_standby

        if self.data.gripper_joints:
            dual_home.extend([0.04, 0.04, 0.04, 0.04])
            dual_standby.extend([0.04, 0.04, 0.04, 0.04])

        ET.SubElement(keyframe, 'key', name='home',
                     qpos=' '.join(f'{x:.6g}' for x in dual_home))
        ET.SubElement(keyframe, 'key', name='standby1',
                     qpos=' '.join(f'{x:.6g}' for x in dual_standby))

        self._write_xml(root, self.mjcf_dir / f'{self.dual_arm_name}_robot.xml')

    def _generate_dual_pinocchio_xml(self, rel_path: str):
        """Generate dual_pinocchio.xml for kinematics."""
        root = ET.Element('mujoco', model=self.dual_arm_name)
        ET.SubElement(root, 'compiler', angle='radian')

        default = ET.SubElement(root, 'default')
        robot_class = ET.SubElement(default, 'default', {'class': self.single_arm_name})
        ET.SubElement(robot_class, 'joint', armature='0.1', damping='1')
        ET.SubElement(robot_class, 'position', inheritrange='1')

        asset = ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'material', name='white', rgba='1 1 1 1')

        worldbody = ET.SubElement(root, 'worldbody')

        left_offset = -self.arm_separation / 2
        right_offset = self.arm_separation / 2

        for arm_idx, (prefix, offset) in enumerate([('left', left_offset), ('right', right_offset)]):
            arm_base = ET.SubElement(worldbody, 'body', name=f'{prefix}_base', pos=f'{offset} 0 0')
            self._generate_arm_chain(arm_base, arm_idx)

        actuator = ET.SubElement(root, 'actuator')
        for arm_idx, prefix in enumerate(['left', 'right']):
            for i in range(self.data.motion_dof):
                act = ET.SubElement(actuator, 'position')
                act.set('class', self.single_arm_name)
                act.set('name', f'{prefix}_lj{i+1}')
                act.set('joint', f'{self.data.motion_joints[i]}_{arm_idx}')
                act.set('kp', '2000')
                act.set('kv', '200')

        keyframe = ET.SubElement(root, 'keyframe')
        motion_dof = self.data.motion_dof
        dual_home = [0.0] * (motion_dof * 2)
        ET.SubElement(keyframe, 'key', name='home', qpos=' '.join(f'{x:.6g}' for x in dual_home))

        self._write_xml(root, self.mjcf_dir / f'{self.dual_arm_name}_pinocchio.xml')

    def _generate_arm_chain(self, parent: ET.Element, arm_idx: int):
        """Generate kinematic chain for one arm with indexed joint names."""
        joint_map = {j.name: j for j in self.data.joints}
        link_map = {l.name: l for l in self.data.links}

        current = parent
        for i, jname in enumerate(self.data.motion_joints):
            joint = joint_map.get(jname)
            if not joint:
                continue

            link = link_map.get(joint.child_link)

            body = ET.SubElement(current, 'body', name=f'{joint.child_link}_{arm_idx}')
            if joint.pos != (0, 0, 0):
                body.set('pos', ' '.join(f'{x:.6g}' for x in joint.pos))

            if link and link.inertial:
                inertial = ET.SubElement(body, 'inertial')
                inertial.set('mass', str(link.inertial.mass))
                inertial.set('pos', ' '.join(f'{x:.6g}' for x in link.inertial.pos))
                inertial.set('diaginertia', ' '.join(f'{x:.6g}' for x in link.inertial.diaginertia))

            joint_elem = ET.SubElement(body, 'joint', name=f'{joint.name}_{arm_idx}')
            joint_elem.set('axis', ' '.join(str(int(x)) if x in (0, 1, -1) else f'{x:.6g}' for x in joint.axis))
            joint_elem.set('range', f'{joint.range[0]:.6g} {joint.range[1]:.6g}')

            current = body

        if self.data.ee_site:
            ee = self.data.ee_site
            prefix = 'left' if arm_idx == 0 else 'right'
            ee_elem = ET.SubElement(current, 'site', name=f'{prefix}_EE')
            ee_elem.set('pos', ' '.join(f'{x:.6g}' for x in ee.pos))
            ee_elem.set('euler', ' '.join(f'{x:.6g}' for x in ee.euler))

    def _generate_dual_scene_xml(self, rel_path: str):
        """Generate scene.xml for dual-arm."""
        root = ET.Element('mujoco', model=f'{self.dual_arm_name} scene')

        ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.80665', integrator='implicitfast')
        ET.SubElement(root, 'compiler', angle='radian', meshdir=f'{rel_path}/assets')

        ET.SubElement(root, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_defaults.xml')
        ET.SubElement(root, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_assets.xml')

        ET.SubElement(root, 'statistic', center='0 0 0.5', extent='1.5')

        visual = ET.SubElement(root, 'visual')
        ET.SubElement(visual, 'headlight', diffuse='0.6 0.6 0.6', ambient='0.1 0.1 0.1')

        asset = ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'texture', type='skybox', builtin='gradient',
                     rgb1='0.3 0.5 0.7', rgb2='0.2 0.2 0.2')
        ET.SubElement(asset, 'texture', type='2d', name='groundplane', builtin='flat',
                     mark='edge', rgb1='0.1 0.1 0.1', rgb2='0.3 0.5 0.3')
        ET.SubElement(asset, 'material', name='groundplane', texture='groundplane',
                     texuniform='true', texrepeat='5 5')

        worldbody = ET.SubElement(root, 'worldbody')
        ET.SubElement(worldbody, 'light', pos='0 0 1.5', dir='0 0 -1', directional='true')
        ET.SubElement(worldbody, 'geom', name='floor', size='0 0 0.05', type='plane', material='groundplane')

        left_offset = -self.arm_separation / 2
        right_offset = self.arm_separation / 2

        left_body = ET.SubElement(worldbody, 'body', name='left_arm_mount', pos=f'{left_offset} 0 0')
        ET.SubElement(left_body, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_links.xml')

        right_body = ET.SubElement(worldbody, 'body', name='right_arm_mount', pos=f'{right_offset} 0 0')
        ET.SubElement(right_body, 'include', file=f'{rel_path}/mjcf/{self.single_arm_name}_links.xml')

        self._write_xml(root, self.scene_dir / f'{self.dual_arm_name}_scene.xml')

    def _write_xml(self, root: ET.Element, path: Path):
        """Write formatted XML to file."""
        rough_string = ET.tostring(root, encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        pretty = reparsed.toprettyxml(indent='  ')

        lines = []
        for line in pretty.split('\n'):
            if line.strip() and not line.strip().startswith('<?xml'):
                lines.append(line)

        with open(path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
