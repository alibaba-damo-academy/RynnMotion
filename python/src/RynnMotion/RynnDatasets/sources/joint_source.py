"""Direct joint sensor source - reads from any object with joint position access."""

from collections.abc import Callable

import numpy as np

from . import ISensorSource, SensorReading, SensorType


class DirectJointSensorSource(ISensorSource):
    """Reads joint positions from any callable that returns np.ndarray.

    Works with InterfaceBase.get_joint_positions(), TeleOperator SDK,
    or any function returning a numpy array of joint values.

    Args:
        name: Sensor name (e.g. "arm_joints", "gripper").
        joint_names: Ordered list of joint names for dataset features.
        reader_fn: Callable returning np.ndarray of joint positions.
        prefix: Dataset key prefix ("observation" or "action").
        suffix: Optional suffix appended after prefix.state (e.g. "arm", "gripper").
            If None, uses bare "prefix.state" or "prefix" (for action).
    """

    def __init__(
        self,
        name: str,
        joint_names: list[str],
        reader_fn: Callable[[], np.ndarray],
        prefix: str = "observation",
        suffix: str | None = None,
    ):
        super().__init__(name, SensorType.JOINT)
        self.joint_names = joint_names
        self.reader_fn = reader_fn
        self.prefix = prefix
        self.suffix = suffix

    def connect(self) -> None:
        pass

    def read(self) -> SensorReading:
        values = self.reader_fn()
        if not isinstance(values, np.ndarray):
            values = np.asarray(values, dtype=np.float32)
        elif values.dtype != np.float32:
            values = values.astype(np.float32)

        out_key = self._feature_key()
        return SensorReading(
            name=self.name,
            timestamp=self.monotonic_timestamp(),
            data={out_key: values},
        )

    def disconnect(self) -> None:
        pass

    def get_features(self) -> dict[str, dict]:
        key = self._feature_key()
        return {
            key: {
                "dtype": "float32",
                "shape": (len(self.joint_names),),
                "names": list(self.joint_names),
            }
        }

    def _feature_key(self) -> str:
        if self.prefix == "action":
            return "action" if self.suffix is None else f"action.{self.suffix}"
        if self.suffix is not None:
            return f"{self.prefix}.state.{self.suffix}"
        return f"{self.prefix}.state"
