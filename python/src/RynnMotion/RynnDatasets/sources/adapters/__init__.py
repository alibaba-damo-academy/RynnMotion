"""
Topic adapters for extracting structured data from ROS2 messages.

Ported from ros2_data_collector with local imports.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass
class FieldMapping:
    """Maps a message field path to an output key."""
    field: str
    out_key: str


class ITopicAdapter(ABC):
    """Abstract base for ROS2 topic message adapters."""

    @abstractmethod
    def extract_fields(self, msg) -> tuple[dict[str, Any], float]:
        """Extract structured fields from a ROS2 message.

        Args:
            msg: ROS2 message object.

        Returns:
            Tuple of (extracted_data_dict, timestamp).
        """
        ...
