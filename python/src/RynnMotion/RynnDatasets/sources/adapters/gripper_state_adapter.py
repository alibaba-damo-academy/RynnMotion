"""Gripper state adapter for ROS2 messages.

Ported from ros2_data_collector/src/adapters/gripper_state_adapter.py.
"""

import array
from typing import Any

from . import FieldMapping, ITopicAdapter


class GripperStateAdapter(ITopicAdapter):
    """Extracts gripper state from ROS2 messages with optional value mapping.

    Args:
        mappings: List of FieldMapping for gripper fields.
    """

    def __init__(self, mappings: list[FieldMapping]):
        self.mappings = mappings
        self._split_mappings = [(m.field.split("."), m.out_key) for m in mappings]

    def extract_fields(self, msg) -> tuple[dict[str, Any], float]:
        out: dict[str, Any] = {}
        timestamp = msg.header.stamp
        for parts, out_key in self._split_mappings:
            val = self._get_field(msg, parts)
            if out_key == "observation.state.gripper":
                val = val[0]
                val = [self.map_state_gripper(val)]
            if val is not None:
                if out_key not in out:
                    out[out_key] = []
                if isinstance(val, list):
                    out[out_key].extend(val)
                else:
                    out[out_key].append(val)
        return out, timestamp

    @staticmethod
    def _get_field(obj: Any, parts: list[str]) -> Any:
        cur = obj
        for p in parts:
            if isinstance(cur, dict):
                cur = cur.get(p)
            else:
                cur = getattr(cur, p, None)

            if isinstance(cur, array.array):
                cur = list(cur)

            if cur is None:
                return None
        return cur

    @staticmethod
    def map_state_gripper(value: float) -> float:
        """Map physical gripper position to [0, 1] range.

        Default: identity mapping. Override for robot-specific scaling.
        """
        return value
