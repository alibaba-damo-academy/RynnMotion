"""
MJCF Generator for RynnMotion.

Generates modular MJCF files from RobotData intermediate representation.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path
from typing import List, Dict, Optional
import shutil

from robot_data import RobotData, JointInfo, LinkInfo, ActuatorInfo, SiteInfo


class MjcfGenerator:
    """Generate RynnMotion modular MJCF structure."""

    def __init__(self, robot_data: RobotData, output_dir: Path):
        self.data = robot_data
        self.output_dir = Path(output_dir)
        self.mjcf_dir = self.output_dir / 'mjcf'
        self.assets_dir = self.output_dir / 'assets'
        self.scene_dir = self.output_dir / 'scene'

    def generate_all(self):
        """Generate all MJCF files."""
        self.mjcf_dir.mkdir(parents=True, exist_ok=True)
        self.assets_dir.mkdir(parents=True, exist_ok=True)
        self.scene_dir.mkdir(parents=True, exist_ok=True)

        self._copy_meshes()

        self._generate_defaults()
        self._generate_assets()
        self._generate_links()
        self._generate_actuators()
        self._generate_sensors()
        self._generate_contacts()

        self._generate_robot_xml()
        self._generate_pinocchio_xml()

        self._generate_scene()

        print(f"Generated RynnMotion MJCF at: {self.output_dir}")
        print(f"  - Motion DOF: {self.data.motion_dof}")
        print(f"  - Gripper DOF: {self.data.gripper_dof}")
        print(f"  - Total DOF: {self.data.total_dof}")

    def _copy_meshes(self):
        """Copy mesh files to assets directory."""
        for mesh_name, mesh_info in self.data.meshes.items():
            if mesh_info.file_path and mesh_info.file_path.exists():
                dest = self.assets_dir / mesh_info.file_path.name
                if not dest.exists():
                    shutil.copy2(mesh_info.file_path, dest)

    def _generate_defaults(self):
        """Generate {robot}_defaults.xml."""
        root = ET.Element('mujoco', model=f'{self.data.name}_defaults')

        default = ET.SubElement(root, 'default')
        robot_class = ET.SubElement(default, 'default', {'class': self.data.name})

        ET.SubElement(robot_class, 'joint', armature='0.1', damping='1')
        ET.SubElement(robot_class, 'position', inheritrange='1')

        visual = ET.SubElement(robot_class, 'default', {'class': 'visual'})
        ET.SubElement(visual, 'geom', type='mesh', group='2', contype='0', conaffinity='0')

        collision = ET.SubElement(robot_class, 'default', {'class': 'collision'})
        ET.SubElement(collision, 'geom', type='mesh', group='3')

        ET.SubElement(robot_class, 'site', size='0.001', rgba='0.5 0.5 0.5 0.3', group='4')

        if self.data.gripper_joints:
            finger = ET.SubElement(robot_class, 'default', {'class': 'finger'})
            ET.SubElement(finger, 'joint', axis='0 1 0', type='slide', range='0 0.04')

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_defaults.xml')

    def _generate_assets(self):
        """Generate {robot}_assets.xml."""
        root = ET.Element('mujoco', model=f'{self.data.name}_assets')
        asset = ET.SubElement(root, 'asset')

        standard_materials = [
            ('white', '1 1 1 1'),
            ('off_white', '0.9 0.92 0.93 1'),
            ('black', '0.2 0.2 0.2 1'),
            ('gray', '0.5 0.5 0.5 1'),
            ('metal', '0.58 0.58 0.58 1'),
            ('red', '1 0.07 0.04 1'),
            ('green', '0 1 0 1'),
            ('blue', '0 0 1 1'),
            ('orange', '1 0.5 0 1'),
            ('yellow', '1 1 0 1'),
        ]
        for name, rgba in standard_materials:
            ET.SubElement(asset, 'material', name=name, rgba=rgba)

        for name, mat_info in self.data.materials.items():
            if name not in [m[0] for m in standard_materials]:
                rgba_str = ' '.join(str(x) for x in mat_info.rgba)
                ET.SubElement(asset, 'material', name=name, rgba=rgba_str)

        for mesh_name, mesh_info in sorted(self.data.meshes.items()):
            if mesh_info.file_path:
                ET.SubElement(asset, 'mesh', name=mesh_name, file=mesh_info.file_path.name)

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_assets.xml')

    def _generate_links(self):
        """Generate {robot}_links.xml."""
        root = ET.Element('mujoco', model=f'{self.data.name}_links')

        joint_map = {j.name: j for j in self.data.joints}
        link_map = {l.name: l for l in self.data.links}

        base = ET.SubElement(root, 'body', name='base', childclass=self.data.name)

        all_joints = self.data.motion_joints + self.data.gripper_joints
        processed_links = set()

        current_elem = base
        for i, jname in enumerate(all_joints):
            joint = joint_map.get(jname)
            if not joint:
                continue

            link = link_map.get(joint.child_link)
            if not link or link.name in processed_links:
                continue

            body = ET.SubElement(current_elem, 'body', name=link.name)
            processed_links.add(link.name)

            if joint.pos != (0, 0, 0):
                body.set('pos', ' '.join(f'{x:.6g}' for x in joint.pos))
            if joint.quat != (1, 0, 0, 0):
                body.set('quat', ' '.join(f'{x:.6g}' for x in joint.quat))

            if link.inertial:
                inertial = ET.SubElement(body, 'inertial')
                inertial.set('mass', str(link.inertial.mass))
                inertial.set('pos', ' '.join(f'{x:.6g}' for x in link.inertial.pos))
                inertial.set('diaginertia', ' '.join(f'{x:.6g}' for x in link.inertial.diaginertia))

            joint_elem = ET.SubElement(body, 'joint', name=joint.name)
            joint_elem.set('axis', ' '.join(str(int(x)) if x in (0, 1, -1) else f'{x:.6g}' for x in joint.axis))
            joint_elem.set('range', f'{joint.range[0]:.6g} {joint.range[1]:.6g}')
            if joint.force_range != (-87, 87):
                joint_elem.set('actuatorfrcrange', f'{joint.force_range[0]} {joint.force_range[1]}')

            if i == 0:
                ET.SubElement(body, 'site', name='shoulderSite', pos='0 0 0', rgba='1 0 0 1')
            elif i == len(self.data.motion_joints) // 2:
                ET.SubElement(body, 'site', name='elbowSite', pos='0 0 0', rgba='1 0 0 1')
            elif i == len(self.data.motion_joints) - 2:
                ET.SubElement(body, 'site', name='wristSite', pos='0 0 0', rgba='1 0 0 1')

            for geom in link.visual_geoms:
                geom_elem = ET.SubElement(body, 'geom')
                geom_elem.set('class', 'visual')
                if geom.mesh:
                    geom_elem.set('mesh', geom.mesh)
                if geom.material:
                    geom_elem.set('material', geom.material)

            for geom in link.collision_geoms:
                geom_elem = ET.SubElement(body, 'geom')
                geom_elem.set('class', 'collision')
                if geom.mesh:
                    geom_elem.set('mesh', geom.mesh)
                elif geom.type != 'mesh':
                    geom_elem.set('type', geom.type)
                    if geom.size:
                        geom_elem.set('size', ' '.join(f'{x:.6g}' for x in geom.size))

            if jname in self.data.motion_joints:
                current_elem = body

        if self.data.ee_site:
            ee = self.data.ee_site
            ee_elem = ET.SubElement(current_elem, 'site', name=ee.name)
            ee_elem.set('pos', ' '.join(f'{x:.6g}' for x in ee.pos))
            ee_elem.set('euler', ' '.join(f'{x:.6g}' for x in ee.euler))
            ee_elem.set('rgba', '1 0 0 1')

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_links.xml')

    def _generate_actuators(self):
        """Generate {robot}_actuators.xml."""
        root = ET.Element('mujoco', model=f'{self.data.name}_actuators')

        if len(self.data.gripper_joints) >= 2:
            tendon = ET.SubElement(root, 'tendon')
            fixed = ET.SubElement(tendon, 'fixed', name='split')
            for gj in self.data.gripper_joints[:2]:
                ET.SubElement(fixed, 'joint', joint=gj, coef='0.5')

            equality = ET.SubElement(root, 'equality')
            ET.SubElement(equality, 'joint',
                         joint1=self.data.gripper_joints[0],
                         joint2=self.data.gripper_joints[1],
                         solimp='0.95 0.99 0.001',
                         solref='0.005 1')

        actuator = ET.SubElement(root, 'actuator')

        for i, jname in enumerate(self.data.motion_joints):
            act_info = next((a for a in self.data.actuators if a.joint == jname), None)
            kp = act_info.kp if act_info else 2000
            kv = act_info.kv if act_info else 200

            act_elem = ET.SubElement(actuator, 'position')
            act_elem.set('class', self.data.name)
            act_elem.set('name', f'lj{i+1}')
            act_elem.set('joint', jname)
            act_elem.set('kp', str(int(kp)))
            act_elem.set('kv', str(int(kv)))

        if self.data.gripper_joints:
            if len(self.data.gripper_joints) >= 2:
                grip = ET.SubElement(actuator, 'general')
                grip.set('class', self.data.name)
                grip.set('name', 'grip')
                grip.set('tendon', 'split')
                grip.set('forcerange', '-100 100')
                grip.set('ctrlrange', '0 255')
                grip.set('gainprm', '0.01568627451 0 0')
                grip.set('biasprm', '0 -100 -10')
            else:
                for i, gj in enumerate(self.data.gripper_joints):
                    grip = ET.SubElement(actuator, 'position')
                    grip.set('class', self.data.name)
                    grip.set('name', f'grip{i}' if len(self.data.gripper_joints) > 1 else 'grip')
                    grip.set('joint', gj)
                    grip.set('kp', '100')
                    grip.set('kv', '10')

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_actuators.xml')

    def _generate_sensors(self):
        """Generate {robot}_sensors.xml."""
        root = ET.Element('mujoco', model=f'{self.data.name}_sensors')
        sensor = ET.SubElement(root, 'sensor')

        ee_name = self.data.ee_site.name if self.data.ee_site else 'EE'

        ET.SubElement(sensor, 'framepos', name='eePos', objtype='site', objname=ee_name)
        ET.SubElement(sensor, 'framequat', name='eeQuat', objtype='site', objname=ee_name)
        ET.SubElement(sensor, 'framelinvel', name='eeLinVel', objtype='site', objname=ee_name)
        ET.SubElement(sensor, 'frameangvel', name='eeAngVel', objtype='site', objname=ee_name)
        ET.SubElement(sensor, 'gyro', name='eeGyro', site=ee_name)
        ET.SubElement(sensor, 'accelerometer', name='eeAcc', site=ee_name)
        ET.SubElement(sensor, 'force', name='eeForce', site=ee_name)
        ET.SubElement(sensor, 'torque', name='eeTorque', site=ee_name)

        for jname in self.data.motion_joints:
            ET.SubElement(sensor, 'jointpos', name=f'{jname}_pos', joint=jname, noise='0.001')

        for jname in self.data.motion_joints:
            ET.SubElement(sensor, 'jointvel', name=f'{jname}_vel', joint=jname, noise='0.001')

        for i in range(len(self.data.motion_joints)):
            ET.SubElement(sensor, 'actuatorfrc', name=f'lj{i+1}_frc', actuator=f'lj{i+1}')

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_sensors.xml')

    def _generate_contacts(self):
        """Generate {robot}_contacts.xml."""
        root = ET.Element('mujoco', model=f'{self.data.name}_contacts')
        contact = ET.SubElement(root, 'contact')

        for exclude in self.data.contact_excludes:
            ET.SubElement(contact, 'exclude', body1=exclude.body1, body2=exclude.body2)

        joint_map = {j.name: j for j in self.data.joints}
        all_joints = self.data.motion_joints + self.data.gripper_joints

        for i in range(len(all_joints) - 1):
            for j in range(i + 2, min(i + 4, len(all_joints))):
                j1 = joint_map.get(all_joints[i])
                j2 = joint_map.get(all_joints[j])
                if j1 and j2:
                    existing = any(
                        (e.body1 == j1.child_link and e.body2 == j2.child_link) or
                        (e.body1 == j2.child_link and e.body2 == j1.child_link)
                        for e in self.data.contact_excludes
                    )
                    if not existing:
                        ET.SubElement(contact, 'exclude', body1=j1.child_link, body2=j2.child_link)

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_contacts.xml')

    def _generate_robot_xml(self):
        """Generate main {robot}_robot.xml with includes."""
        root = ET.Element('mujoco', model=self.data.name)

        ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.80665', integrator='implicitfast')
        ET.SubElement(root, 'compiler', angle='radian', meshdir='../assets', autolimits='true')

        asset = ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'texture', type='skybox', builtin='gradient',
                     rgb1='0.3 0.5 0.7', rgb2='0.2 0.2 0.2', width='512', height='3072')

        ET.SubElement(root, 'include', file=f'{self.data.name}_defaults.xml')
        ET.SubElement(root, 'include', file=f'{self.data.name}_contacts.xml')
        ET.SubElement(root, 'include', file=f'{self.data.name}_assets.xml')
        ET.SubElement(root, 'include', file=f'{self.data.name}_actuators.xml')
        ET.SubElement(root, 'include', file=f'{self.data.name}_sensors.xml')

        worldbody = ET.SubElement(root, 'worldbody')
        ET.SubElement(worldbody, 'include', file=f'{self.data.name}_links.xml')

        keyframe = ET.SubElement(root, 'keyframe')

        total_dof = self.data.total_dof
        motion_dof = self.data.motion_dof

        home_qpos = self.data.home_qpos[:total_dof] if self.data.home_qpos else [0.0] * total_dof
        while len(home_qpos) < total_dof:
            home_qpos.append(0.0)

        standby_qpos = self.data.standby_qpos[:total_dof] if self.data.standby_qpos else home_qpos[:]
        while len(standby_qpos) < total_dof:
            standby_qpos.append(0.04 if len(standby_qpos) >= motion_dof else 0.0)

        home_ctrl = home_qpos[:motion_dof] + ([0] if self.data.gripper_joints else [])
        standby_ctrl = standby_qpos[:motion_dof] + ([255] if self.data.gripper_joints else [])

        ET.SubElement(keyframe, 'key', name='home',
                     qpos=' '.join(f'{x:.6g}' for x in home_qpos),
                     ctrl=' '.join(f'{x:.6g}' for x in home_ctrl))
        ET.SubElement(keyframe, 'key', name='standby1',
                     qpos=' '.join(f'{x:.6g}' for x in standby_qpos),
                     ctrl=' '.join(f'{x:.6g}' for x in standby_ctrl))
        ET.SubElement(keyframe, 'key', name='standby2',
                     qpos=' '.join(f'{x:.6g}' for x in standby_qpos),
                     ctrl=' '.join(f'{x:.6g}' for x in standby_ctrl))

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_robot.xml')

    def _generate_pinocchio_xml(self):
        """Generate {robot}_pinocchio.xml - kinematics only, self-contained."""
        root = ET.Element('mujoco', model=self.data.name)
        ET.SubElement(root, 'compiler', angle='radian')

        default = ET.SubElement(root, 'default')
        robot_class = ET.SubElement(default, 'default', {'class': self.data.name})
        ET.SubElement(robot_class, 'joint', armature='0.1', damping='1')
        ET.SubElement(robot_class, 'position', inheritrange='1')
        visual = ET.SubElement(robot_class, 'default', {'class': 'visual'})
        ET.SubElement(visual, 'geom', type='mesh', group='2', contype='0', conaffinity='0')
        ET.SubElement(robot_class, 'site', size='0.001', rgba='0.5 0.5 0.5 0.3', group='4')

        asset = ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'material', name='white', rgba='1 1 1 1')
        ET.SubElement(asset, 'material', name='black', rgba='0.2 0.2 0.2 1')

        worldbody = ET.SubElement(root, 'worldbody')

        joint_map = {j.name: j for j in self.data.joints}
        link_map = {l.name: l for l in self.data.links}

        base = ET.SubElement(worldbody, 'body', name='base', childclass=self.data.name)

        current_elem = base
        for i, jname in enumerate(self.data.motion_joints):
            joint = joint_map.get(jname)
            if not joint:
                continue

            link = link_map.get(joint.child_link)

            body = ET.SubElement(current_elem, 'body', name=joint.child_link)

            if joint.pos != (0, 0, 0):
                body.set('pos', ' '.join(f'{x:.6g}' for x in joint.pos))
            if joint.quat != (1, 0, 0, 0):
                body.set('quat', ' '.join(f'{x:.6g}' for x in joint.quat))

            if link and link.inertial:
                inertial = ET.SubElement(body, 'inertial')
                inertial.set('mass', str(link.inertial.mass))
                inertial.set('pos', ' '.join(f'{x:.6g}' for x in link.inertial.pos))
                inertial.set('diaginertia', ' '.join(f'{x:.6g}' for x in link.inertial.diaginertia))

            joint_elem = ET.SubElement(body, 'joint', name=joint.name)
            joint_elem.set('axis', ' '.join(str(int(x)) if x in (0, 1, -1) else f'{x:.6g}' for x in joint.axis))
            joint_elem.set('range', f'{joint.range[0]:.6g} {joint.range[1]:.6g}')
            if joint.force_range != (-87, 87):
                joint_elem.set('actuatorfrcrange', f'{joint.force_range[0]} {joint.force_range[1]}')

            if i == 0:
                ET.SubElement(body, 'site', name='shoulderSite', pos='0 0 0', rgba='1 0 0 1')
            elif i == len(self.data.motion_joints) // 2:
                ET.SubElement(body, 'site', name='elbowSite', pos='0 0 0', rgba='1 0 0 1')
            elif i == len(self.data.motion_joints) - 2:
                ET.SubElement(body, 'site', name='wristSite', pos='0 0 0', rgba='1 0 0 1')

            current_elem = body

        if self.data.ee_site:
            ee = self.data.ee_site
            ee_elem = ET.SubElement(current_elem, 'site', name=ee.name)
            ee_elem.set('pos', ' '.join(f'{x:.6g}' for x in ee.pos))
            ee_elem.set('euler', ' '.join(f'{x:.6g}' for x in ee.euler))
            ee_elem.set('rgba', '1 0 0 1')

        actuator = ET.SubElement(root, 'actuator')
        for i, jname in enumerate(self.data.motion_joints):
            act_info = next((a for a in self.data.actuators if a.joint == jname), None)
            kp = act_info.kp if act_info else 2000
            kv = act_info.kv if act_info else 200

            act_elem = ET.SubElement(actuator, 'position')
            act_elem.set('class', self.data.name)
            act_elem.set('name', f'lj{i+1}')
            act_elem.set('joint', jname)
            act_elem.set('kp', str(int(kp)))
            act_elem.set('kv', str(int(kv)))

        keyframe = ET.SubElement(root, 'keyframe')
        motion_dof = self.data.motion_dof

        home_qpos = [0.0] * motion_dof
        standby_qpos = self.data.standby_qpos[:motion_dof] if self.data.standby_qpos else home_qpos[:]
        while len(standby_qpos) < motion_dof:
            standby_qpos.append(0.0)

        ET.SubElement(keyframe, 'key', name='home',
                     qpos=' '.join(f'{x:.6g}' for x in home_qpos))
        ET.SubElement(keyframe, 'key', name='standby1',
                     qpos=' '.join(f'{x:.6g}' for x in standby_qpos))
        ET.SubElement(keyframe, 'key', name='standby2',
                     qpos=' '.join(f'{x:.6g}' for x in standby_qpos))

        self._write_xml(root, self.mjcf_dir / f'{self.data.name}_pinocchio.xml')

    def _generate_scene(self):
        """Generate scene/scene.xml."""
        root = ET.Element('mujoco', model=f'{self.data.name} scene')

        ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.80665', integrator='implicitfast')
        ET.SubElement(root, 'compiler', angle='radian', meshdir='../assets')

        ET.SubElement(root, 'include', file=f'../mjcf/{self.data.name}_defaults.xml')
        ET.SubElement(root, 'include', file=f'../mjcf/{self.data.name}_assets.xml')
        ET.SubElement(root, 'include', file=f'../mjcf/{self.data.name}_contacts.xml')

        ET.SubElement(root, 'statistic', center='0 0 0.5', extent='1.0')

        visual = ET.SubElement(root, 'visual')
        ET.SubElement(visual, 'headlight', diffuse='0.6 0.6 0.6', ambient='0.1 0.1 0.1', specular='0 0 0')
        ET.SubElement(visual, 'rgba', haze='0.15 0.25 0.35 1')
        ET.SubElement(visual, 'global', azimuth='135', elevation='-20')

        asset = ET.SubElement(root, 'asset')
        ET.SubElement(asset, 'texture', type='skybox', builtin='gradient',
                     rgb1='0.3 0.5 0.7', rgb2='0.2 0.2 0.2', width='512', height='3072')
        ET.SubElement(asset, 'texture', type='2d', name='groundplane', builtin='flat',
                     mark='edge', rgb1='0.1 0.1 0.1', rgb2='0.3 0.5 0.3',
                     markrgb='0.8 0.8 0.8', width='300', height='300')
        ET.SubElement(asset, 'material', name='groundplane', texture='groundplane',
                     texuniform='true', texrepeat='5 5', reflectance='0.5')

        worldbody = ET.SubElement(root, 'worldbody')
        ET.SubElement(worldbody, 'light', pos='0 0 1.5', dir='0 0 -1', directional='true')
        ET.SubElement(worldbody, 'geom', name='floor', size='0 0 0.05', type='plane', material='groundplane')

        ET.SubElement(worldbody, 'include', file=f'../mjcf/{self.data.name}_links.xml')

        ET.SubElement(root, 'include', file=f'../mjcf/{self.data.name}_actuators.xml')
        ET.SubElement(root, 'include', file=f'../mjcf/{self.data.name}_sensors.xml')

        self._write_xml(root, self.scene_dir / 'scene.xml')

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
