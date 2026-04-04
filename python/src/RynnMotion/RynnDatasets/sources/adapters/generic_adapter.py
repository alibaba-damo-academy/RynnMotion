"""Generic field-mapping adapter for ROS2 messages.

Ported from ros2_data_collector/src/adapters/generic_adapter.py.
"""

import array
from typing import Any

from . import FieldMapping, ITopicAdapter


class GenericAdapter(ITopicAdapter):
    """Extracts fields from ROS2 messages using configurable field mappings.

    Args:
        mappings: List of FieldMapping defining field paths and output keys.
    """

    def __init__(self, mappings: list[FieldMapping]):
        self.mappings = mappings
        self._split_mappings = [(m.field.split("."), m.out_key) for m in mappings]

    def extract_fields(self, msg) -> tuple[dict[str, Any], float]:
        out: dict[str, Any] = {}
        timestamp = msg.header.stamp
        for parts, out_key in self._split_mappings:
            val = self._get_field(msg, parts)
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
