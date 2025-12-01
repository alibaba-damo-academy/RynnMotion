"""
Common utilities and shared components for the robot motion project.
"""

from typing import List, Optional, Dict, Any
from dataclasses import dataclass, asdict, field, is_dataclass
from datetime import datetime
from RynnMotion.common.data.basic_data import Pose, Wrench, Twist


class RobotState:
    """
    robot data structure for storing comprehensive robot state.
    Designed for use in control, logging, visualization, and communication.
    """

    def __init__(self):
        # time
        self.sec: int = 0
        self.nanosec: int = 0
        self.utime: int = 0  # microseconds

        # joints
        self._num_joints: int = 0
        self.joint_pos: List[float] = []  # q
        self.joint_vel: List[float] = []  # dq
        self.joint_torque: List[float] = []  # torque

        # End-Effector
        self._num_end_effectors: int = 0
        self.ee_pose: List[Pose] = []
        self.ee_vel: List[Twist] = []

        # grippers
        self._num_grippers: int = 0
        self.gripper_pos: List[float] = []
        self.gripper_vel: List[float] = []
        self.gripper_force: List[float] = []

        # FT Sensor
        self._num_ft_sensors: int = 0
        self.ft_sensor_wrench: List[Wrench] = []

        # Sites, e.g., markers, tools
        self._num_sites: int = 0
        self.site_pose: List[Pose] = []  # pose × N
        self.site_vel: List[Twist] = []  # vel × N

        # additional custom data
        self.custom: Dict[str, Any] = {}

    # helper resize used by setters
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

    # add property after __init__
    @property
    def num_joints(self) -> int:
        return int(self._num_joints)

    @num_joints.setter
    def num_joints(self, n: int):
        """Set number of joints and resize joint lists (pos/vel/torque)."""
        n = int(n)
        if n < 0:
            raise ValueError("num_joints must be >= 0")
        self._num_joints = n
        self.joint_pos = self._resize(getattr(self, "joint_pos", []), n, 0.0)
        self.joint_vel = self._resize(getattr(self, "joint_vel", []), n, 0.0)
        self.joint_torque = self._resize(getattr(self, "joint_torque", []), n, 0.0)

    # end effectors
    @property
    def num_end_effectors(self) -> int:
        return int(self._num_end_effectors)

    @num_end_effectors.setter
    def num_end_effectors(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_end_effectors must be >= 0")
        self._num_end_effectors = n
        self.ee_pose = self._resize(getattr(self, "ee_pose", []), n, Pose)
        self.ee_vel = self._resize(getattr(self, "ee_vel", []), n, Twist)

    # grippers
    @property
    def num_grippers(self) -> int:
        return int(self._num_grippers)

    @num_grippers.setter
    def num_grippers(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_grippers must be >= 0")
        self._num_grippers = n
        self.gripper_pos = self._resize(getattr(self, "gripper_pos", []), n, 0.0)
        self.gripper_vel = self._resize(getattr(self, "gripper_vel", []), n, 0.0)
        self.gripper_force = self._resize(getattr(self, "gripper_force", []), n, 0.0)

    # force/torque sensors
    @property
    def num_ft_sensors(self) -> int:
        return int(self._num_ft_sensors)

    @num_ft_sensors.setter
    def num_ft_sensors(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_ft_sensors must be >= 0")
        self._num_ft_sensors = n
        self.ft_sensor_wrench = self._resize(
            getattr(self, "ft_sensor_wrench", []), n, Wrench
        )

    # sites
    @property
    def num_sites(self) -> int:
        return int(self._num_sites)

    @num_sites.setter
    def num_sites(self, n: int):
        n = int(n)
        if n < 0:
            raise ValueError("num_sites must be >= 0")
        self._num_sites = n
        self.site_pose = self._resize(getattr(self, "site_pose", []), n, Pose)
        self.site_vel = self._resize(getattr(self, "site_vel", []), n, Twist)

    def now(self):
        """Set timestamp to current time (utime in microseconds)"""
        dt = datetime.now()
        ts = dt.timestamp()
        self.sec = int(ts)
        # nanoseconds as integer
        self.nanosec = int((ts - self.sec) * 1e9)
        self.utime = int(ts * 1e6)
        return self

    def from_dict(self, d: Dict[str, Any]):
        """get data from dict"""
        for k, v in d.items():
            if not hasattr(self, k):
                continue
            if k in ("ee_pose", "site_pose") and isinstance(v, list):
                converted = []
                for item in v:
                    if isinstance(item, dict):
                        converted.append(Pose(**item))
                    else:
                        converted.append(item)
                setattr(self, k, converted)
            elif k in ("ee_vel", "site_vel") and isinstance(v, list):
                converted = []
                for item in v:
                    if isinstance(item, dict):
                        converted.append(Twist(**item))
                    else:
                        converted.append(item)
                setattr(self, k, converted)
            elif k in ("ft_sensor_wrench",) and isinstance(v, list):
                converted = []
                for item in v:
                    if isinstance(item, dict):
                        converted.append(Wrench(**item))
                    else:
                        converted.append(item)
                setattr(self, k, converted)
            else:
                setattr(self, k, v)
        try:
            self._update_counts()
        except Exception:
            pass
        return self

    def to_dict(self) -> Dict[str, Any]:
        """transform to dict"""
        out: Dict[str, Any] = {}
        for k, v in self.__dict__.items():
            if isinstance(v, list):
                lst = []
                for item in v:
                    if is_dataclass(item):
                        lst.append(asdict(item))
                    else:
                        lst.append(item)
                out[k] = lst
            elif isinstance(v, dict):
                out[k] = v.copy()
            else:
                if is_dataclass(v):
                    out[k] = asdict(v)
                else:
                    out[k] = v
        out["num_joints"] = self.num_joints
        out["num_end_effectors"] = self.num_end_effectors
        out["num_grippers"] = self.num_grippers
        out["num_ft_sensors"] = self.num_ft_sensors
        out["num_sites"] = self.num_sites
        return out

    def _update_counts(self):
        """update counts"""
        try:
            self._num_joints = len(self.joint_pos) if self.joint_pos is not None else 0
        except Exception:
            self._num_joints = 0
        self._num_end_effectors = len(self.ee_pose) if self.ee_pose is not None else 0
        self._num_grippers = (
            len(self.gripper_pos) if self.gripper_pos is not None else 0
        )
        self._num_ft_sensors = (
            len(self.ft_sensor_wrench) if self.ft_sensor_wrench is not None else 0
        )
        self._num_sites = len(self.site_pose) if self.site_pose is not None else 0

    def copy(self):
        """deep copy"""
        import copy as _copy

        return _copy.deepcopy(self)

    def __repr__(self):
        return (
            f"RobotState(joints={self.num_joints}, "
            f"eef={self.num_end_effectors}, "
            f"grippers={self.num_grippers}, "
            f"ft={self.num_ft_sensors}, "
            f"sites={self.num_sites}, "
            f"utime={self.utime})"
        )
