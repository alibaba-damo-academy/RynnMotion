"""ROS2 sensor source - wraps ROS2 topic subscriptions as ISensorSource.

Requires rclpy (ROS2 Python client). Falls back gracefully if unavailable.
"""

import importlib
import logging
import queue
from typing import Any

import numpy as np

from . import ISensorSource, SensorReading, SensorType
from .adapters import FieldMapping

logger = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


def _import_message_type(msg_type_str: str):
    """Dynamically import a ROS2 message type from string.

    Args:
        msg_type_str: Fully qualified message type, e.g. "sensor_msgs.msg.JointState".

    Returns:
        The message class.
    """
    parts = msg_type_str.rsplit(".", 1)
    if len(parts) != 2:
        raise ValueError(f"Invalid message type format: '{msg_type_str}'. Expected 'pkg.msg.Type'")
    module_path, class_name = parts
    module = importlib.import_module(module_path)
    return getattr(module, class_name)


def _create_adapter(adapter_name: str, mappings: list[FieldMapping]):
    """Create an adapter instance by name."""
    from .adapters.generic_adapter import GenericAdapter
    from .adapters.camera_adapter import CameraAdapter
    from .adapters.gripper_state_adapter import GripperStateAdapter

    adapters = {
        "GenericAdapter": GenericAdapter,
        "CameraAdapter": CameraAdapter,
        "GripperStateAdapter": GripperStateAdapter,
    }
    cls = adapters.get(adapter_name)
    if cls is None:
        raise ValueError(f"Unknown adapter '{adapter_name}'. Available: {list(adapters)}")
    return cls(mappings)


class Ros2SensorSource(ISensorSource):
    """Wraps a ROS2 topic subscription + adapter as a sensor source.

    Subscribes to a ROS2 topic, uses an adapter to extract structured
    data from messages, and presents it through the ISensorSource API.

    Args:
        name: Source name.
        topic: ROS2 topic to subscribe to.
        msg_type_str: Fully qualified message type string.
        adapter_name: Name of the adapter class to use.
        mappings_raw: List of dicts with "field" and "out_key" for FieldMapping.
        sensor_type: Type of sensor data.
        prefix: Dataset key prefix.
        qos_depth: QoS history depth.
    """

    def __init__(
        self,
        name: str,
        topic: str,
        msg_type_str: str,
        adapter_name: str,
        mappings_raw: list[dict[str, str]],
        sensor_type: SensorType,
        prefix: str = "observation",
        qos_depth: int = 10,
    ):
        if not HAS_ROS2:
            raise ImportError(
                "rclpy not available. Install ROS2 to use Ros2SensorSource."
            )
        super().__init__(name, sensor_type)
        self.topic = topic
        self.msg_type_str = msg_type_str
        self.adapter_name = adapter_name
        self.prefix = prefix
        self.qos_depth = qos_depth

        self._mappings = [
            FieldMapping(field=m["field"], out_key=m["out_key"])
            for m in mappings_raw
        ]

        self._msg_type = _import_message_type(msg_type_str)
        self._adapter = _create_adapter(adapter_name, self._mappings)
        self._data_queue: queue.Queue[SensorReading] = queue.Queue(maxsize=100)
        self._node: Any = None
        self._subscription: Any = None

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        node_name = f"rynnmotion_{self.name}".replace("/", "_").replace(".", "_")
        self._node = rclpy.create_node(node_name)
        self._subscription = self._node.create_subscription(
            self._msg_type,
            self.topic,
            self._msg_callback,
            self.qos_depth,
        )
        logger.info(f"Ros2SensorSource '{self.name}' subscribed to {self.topic}")

    def _msg_callback(self, msg) -> None:
        """ROS2 subscription callback - extract fields and queue."""
        try:
            extracted, timestamp = self._adapter.extract_fields(msg)

            # Convert ROS2 stamp to float seconds
            if hasattr(timestamp, "sec"):
                ts = timestamp.sec + timestamp.nanosec * 1e-9
            else:
                ts = float(timestamp)

            # Convert extracted lists to numpy arrays
            data = {}
            for key, val in extracted.items():
                if isinstance(val, list) and all(isinstance(v, (int, float)) for v in val):
                    data[key] = np.array(val, dtype=np.float32)
                else:
                    data[key] = val

            reading = SensorReading(name=self.name, timestamp=ts, data=data)
            try:
                self._data_queue.put_nowait(reading)
            except queue.Full:
                # Drop oldest, add newest
                try:
                    self._data_queue.get_nowait()
                except queue.Empty:
                    pass
                self._data_queue.put_nowait(reading)
        except Exception as e:
            logger.warning(f"Error in ROS2 callback for '{self.name}': {e}")

    def read(self) -> SensorReading:
        try:
            return self._data_queue.get_nowait()
        except queue.Empty:
            # Return empty reading with current timestamp
            return SensorReading(
                name=self.name,
                timestamp=self.monotonic_timestamp(),
                data={},
            )

    def disconnect(self) -> None:
        if self._node is not None:
            if self._subscription is not None:
                self._node.destroy_subscription(self._subscription)
                self._subscription = None
            self._node.destroy_node()
            self._node = None

        # Close camera adapter if applicable
        if hasattr(self._adapter, "close"):
            self._adapter.close()

    def get_features(self) -> dict[str, dict]:
        # Infer features from adapter mappings and sensor type
        features = {}
        out_keys = {m.out_key for m in self._mappings}

        for out_key in out_keys:
            mapping_count = sum(1 for m in self._mappings if m.out_key == out_key)
            if self.sensor_type == SensorType.CAMERA:
                features[out_key] = {
                    "dtype": "video",
                    "shape": (480, 640, 3),
                    "names": ["height", "width", "channels"],
                }
            else:
                features[out_key] = {
                    "dtype": "float32",
                    "shape": (mapping_count,),
                    "names": [m.field for m in self._mappings if m.out_key == out_key],
                }

        return features

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """Process pending ROS2 callbacks. Call from main loop."""
        if self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=timeout_sec)
