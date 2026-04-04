"""
Sensor source abstractions for unified data collection.

Provides ISensorSource ABC, SensorReading dataclass, and SensorType enum
for building robot-agnostic data collection pipelines.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import time


class SensorType(Enum):
    """Types of sensor data sources."""
    JOINT = "joint"
    CAMERA = "camera"
    WRENCH = "wrench"
    IMU = "imu"
    DEPTH = "depth"
    TACTILE = "tactile"


@dataclass
class SensorReading:
    """A single reading from a sensor source."""
    name: str
    timestamp: float
    data: dict[str, Any] = field(default_factory=dict)


class ISensorSource(ABC):
    """Abstract base for all sensor data sources.

    Implementations wrap specific hardware, simulation, or network
    interfaces and present a uniform read() API.
    """

    def __init__(self, name: str, sensor_type: SensorType):
        self.name = name
        self.sensor_type = sensor_type

    @abstractmethod
    def connect(self) -> None:
        """Initialize the sensor connection."""
        ...

    @abstractmethod
    def read(self) -> SensorReading:
        """Read the latest sensor data.

        Returns:
            SensorReading with timestamped data dict mapping
            output keys to values (e.g. numpy arrays).
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Release sensor resources."""
        ...

    @abstractmethod
    def get_features(self) -> dict[str, dict]:
        """Return dataset feature dict for this source.

        The returned dict maps feature keys (e.g. "observation.state")
        to feature descriptors with "dtype", "shape", and "names".
        """
        ...

    @staticmethod
    def monotonic_timestamp() -> float:
        """Return a monotonic timestamp in seconds."""
        return time.monotonic()
