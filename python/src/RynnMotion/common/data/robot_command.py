"""
Common utilities and shared components for the robot motion project.
"""

from typing import List, Optional, Dict, Any
from dataclasses import is_dataclass, asdict
from copy import deepcopy
from RynnMotion.common.data.robot_state import RobotState
from RynnMotion.common.data.basic_data import Pose, Wrench, Twist


class RobotCommand:
    """
    robot command structure for sending commands to the robot.
    Designed for use in control and communication.

    Supports template usage: keep an immutable template (dress) and call
    instantiate()/copy() to produce runtime-modifiable instances.
    """

    def __init__(self):
        # squence number
        self.seq: int = 0
        # private chunk counter
        self._chunk_size: int = 0
        # trajectory: list of RobotState points
        self.trajectory: List[RobotState] = []

        self.work_mode: int = 0  # robot workmode

        # template counts used when creating/resizing trajectory
        self._template_counts: Dict[str, int] = {
            "num_joints": 0,
            "num_grippers": 0,
            "num_end_effectors": 0,
            "num_ft_sensors": 0,
            "num_sites": 0,
        }

    # helper resize used by setters (same logic as RobotState._resize)
    def _resize(self, lst: Optional[List[Any]], new_len: int, fill: Any):
        if lst is None:
            lst = []
        cur = len(lst)
        if cur < new_len:
            if callable(fill):
                lst.extend([fill() for _ in range(new_len - cur)])
            else:
                lst.extend([fill] * (new_len - cur))
        elif cur > new_len:
            del lst[new_len:]
        return lst

    @property
    def chunk_size(self) -> int:
        return int(self._chunk_size)

    @chunk_size.setter
    def chunk_size(self, n: int):
        """Set chunk size and resize trajectory list (filled with RobotState)."""
        n = int(n)
        if n < 0:
            raise ValueError("chunk_size must be >= 0")
        self._chunk_size = n
        # prefer creating based on template counts when available
        template = None
        if any(v > 0 for v in self._template_counts.values()):
            template = self._make_template_state()
        if template:
            # fill with deep copies of template
            self.trajectory = [deepcopy(template) for _ in range(n)]
        else:
            self.trajectory = self._resize(
                getattr(self, "trajectory", []), n, RobotState
            )

    def init_with_template(self, template: RobotState):
        """
        Replace each trajectory slot with a deep copy of template.
        Call after setting chunk_size if you want prefilled templates.
        """
        for i in range(len(self.trajectory)):
            self.trajectory[i] = deepcopy(template)

    def set_trajectory_list(self, points: List[RobotState]):
        """
        Replace trajectory with provided list and update chunk_size accordingly.
        """
        self.trajectory = list(points)
        self._chunk_size = len(self.trajectory)

    def _make_template_state(self) -> RobotState:
        """Create a RobotState instance initialized to the stored template counts."""
        st = RobotState()
        # attempt to set via properties, fall back to direct attrs if properties missing
        try:
            st.num_joints = int(self._template_counts.get("num_joints", 0))
        except Exception:
            st._num_joints = int(self._template_counts.get("num_joints", 0))
            st.joint_pos = [0.0] * st._num_joints
            st.joint_vel = [0.0] * st._num_joints
            st.joint_torque = [0.0] * st._num_joints
        try:
            st.num_grippers = int(self._template_counts.get("num_grippers", 0))
        except Exception:
            st._num_grippers = int(self._template_counts.get("num_grippers", 0))
            st.gripper_pos = [0.0] * st._num_grippers
            st.gripper_vel = [0.0] * st._num_grippers
            st.gripper_force = [0.0] * st._num_grippers
        try:
            st.num_end_effectors = int(
                self._template_counts.get("num_end_effectors", 0)
            )
        except Exception:
            st._num_end_effectors = int(
                self._template_counts.get("num_end_effectors", 0)
            )
            st.ee_pose = [Pose() for _ in range(st._num_end_effectors)]
            st.ee_vel = [Twist() for _ in range(st._num_end_effectors)]
        try:
            st.num_ft_sensors = int(self._template_counts.get("num_ft_sensors", 0))
        except Exception:
            st._num_ft_sensors = int(self._template_counts.get("num_ft_sensors", 0))
            st.ft_sensor_wrench = [Wrench() for _ in range(st._num_ft_sensors)]
        try:
            st.num_sites = int(self._template_counts.get("num_sites", 0))
        except Exception:
            st._num_sites = int(self._template_counts.get("num_sites", 0))
            st.site_pose = [Pose() for _ in range(st._num_sites)]
            st.site_vel = [Twist() for _ in range(st._num_sites)]
        return st

    def init_from_dofs(
        self,
        num_joints: int,
        num_grippers: int = 0,
        num_end_effectors: int = 0,
        num_ft_sensors: int = 0,
        num_sites: int = 0,
        chunk_size: int = 0,
        template: Optional[RobotState] = None,
    ):
        """
        Initialize internal template counts and create trajectory of length chunk_size.

        - Stores template counts used later when changing chunk_size.
        - If template is provided, deep-copy it into each trajectory slot.
          Otherwise a fresh RobotState with appropriate counts is created.
        - Ensures each RobotState in trajectory has its counts set (num_joints, num_grippers, ...).

        Returns self for chaining.
        """
        # normalize ints
        num_j = int(num_joints)
        num_g = int(num_grippers)
        num_ee = int(num_end_effectors)
        num_ft = int(num_ft_sensors)
        num_s = int(num_sites)
        chunk_n = int(chunk_size)

        # store template counts
        self._template_counts = {
            "num_joints": num_j,
            "num_grippers": num_g,
            "num_end_effectors": num_ee,
            "num_ft_sensors": num_ft,
            "num_sites": num_s,
        }

        # prepare a base template RobotState with requested sizes
        if template is None:
            base = RobotState()
            # set counts via properties if available, otherwise set attributes directly
            try:
                base.num_joints = num_j
            except Exception:
                base._num_joints = num_j
                base.joint_pos = [0.0] * num_j
                base.joint_vel = [0.0] * num_j
                base.joint_torque = [0.0] * num_j
            try:
                base.num_grippers = num_g
            except Exception:
                base._num_grippers = num_g
                base.gripper_pos = [0.0] * num_g
                base.gripper_vel = [0.0] * num_g
                base.gripper_force = [0.0] * num_g
            try:
                base.num_end_effectors = num_ee
            except Exception:
                base._num_end_effectors = num_ee
                base.ee_pose = [Pose() for _ in range(num_ee)]
                base.ee_vel = [Twist() for _ in range(num_ee)]
            try:
                base.num_ft_sensors = num_ft
            except Exception:
                base._num_ft_sensors = num_ft
                base.ft_sensor_wrench = [Wrench() for _ in range(num_ft)]
            try:
                base.num_sites = num_s
            except Exception:
                base._num_sites = num_s
                base.site_pose = [Pose() for _ in range(num_s)]
                base.site_vel = [Twist() for _ in range(num_s)]
        else:
            # use provided template but ensure counts match requested counts
            base = deepcopy(template)
            try:
                base.num_joints = num_j
            except Exception:
                pass
            try:
                base.num_grippers = num_g
            except Exception:
                pass
            try:
                base.num_end_effectors = num_ee
            except Exception:
                pass
            try:
                base.num_ft_sensors = num_ft
            except Exception:
                pass
            try:
                base.num_sites = num_s
            except Exception:
                pass

        # fill trajectory with deep copies of base template
        self.trajectory = [deepcopy(base) for _ in range(chunk_n)]
        self._chunk_size = len(self.trajectory)
        return self

    def to_dict(self) -> Dict[str, Any]:
        """Serialize RobotCommand (trajectory items serialized via RobotState.to_dict when available)."""
        out: Dict[str, Any] = {}
        out["chunk_size"] = self.chunk_size
        out["work_mode"] = int(self.work_mode)
        out["template_counts"] = dict(self._template_counts)
        traj_list = []
        for item in self.trajectory:
            if hasattr(item, "to_dict") and callable(getattr(item, "to_dict")):
                traj_list.append(item.to_dict())
            elif is_dataclass(item):
                traj_list.append(asdict(item))
            else:
                traj_list.append(item)
        out["trajectory"] = traj_list
        return out

    def from_dict(self, d: Dict[str, Any]):
        """Deserialize RobotCommand. Expects trajectory entries as dicts convertible by RobotState.from_dict."""
        # restore template counts if present
        if "template_counts" in d:
            self._template_counts = dict(d.get("template_counts", {}))
        # set work_mode if present
        if "work_mode" in d:
            self.work_mode = int(d.get("work_mode", 0))
        # build trajectory
        traj = []
        for item in d.get("trajectory", []):
            if isinstance(item, dict):
                traj.append(RobotState().from_dict(item))
            else:
                traj.append(item)
        self.trajectory = traj
        # set chunk size property to keep consistent (will resize if needed)
        if "chunk_size" in d:
            self.chunk_size = int(d.get("chunk_size", len(self.trajectory)))
        else:
            # ensure private counter matches actual list
            self._chunk_size = len(self.trajectory)
        return self

    def instantiate(self) -> "RobotCommand":
        """Return a deep-copy instance for runtime use (template -> runtime instance)."""
        return deepcopy(self)

    @classmethod
    def from_template(cls, template: "RobotCommand") -> "RobotCommand":
        """Create a new instance from an existing template (alias)."""
        return deepcopy(template)

    def validate(self) -> bool:
        """Basic sanity checks. Extend as needed."""
        if not isinstance(self._chunk_size, int) or self._chunk_size < 0:
            return False
        if self.trajectory is None:
            return False
        if self._chunk_size != len(self.trajectory):
            return False
        # further checks could verify RobotState sizes if desired
        return True

    def copy(self):
        """deep copy (alias)"""
        return deepcopy(self)

    def __repr__(self):
        return f"RobotCommand(chunk_size={self.chunk_size}, work_mode={self.work_mode}, traj_len={len(self.trajectory)})"
