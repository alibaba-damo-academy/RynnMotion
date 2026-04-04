"""Camera adapter for ROS2 Image messages.

Ported from ros2_data_collector/src/adapters/camera_adapter.py.
Handles image decoding, color conversion, and async disk writes.
"""

import array
import logging
import os
import queue
import threading
import time
from typing import Any

import cv2
import numpy as np

from . import FieldMapping, ITopicAdapter

logger = logging.getLogger(__name__)


class CameraAdapter(ITopicAdapter):
    """Extracts image data from ROS2 sensor_msgs/Image messages.

    Decodes raw image data, handles color conversion (rgb8/bgr8),
    and writes images to disk asynchronously.

    Args:
        mappings: List of FieldMapping for this camera topic.
        write_queue_size: Max pending async image writes.
        temp_dir_base: Base directory for temporary image files.
    """

    def __init__(
        self,
        mappings: list[FieldMapping],
        write_queue_size: int = 200,
        temp_dir_base: str | None = None,
    ):
        self.mappings = mappings
        self._split_mappings = [(m.field.split("."), m.out_key) for m in mappings]

        base_dir = temp_dir_base or os.environ.get("RYNNMOTION_TMP_DIR", "/tmp")
        cam_name = mappings[0].out_key if mappings else "unknown"
        safe_name = cam_name.replace("/", "_").replace(".", "_")
        self.temp_dir = os.path.join(base_dir, "ros_images", safe_name)
        os.makedirs(self.temp_dir, exist_ok=True)

        self._write_q: queue.Queue[tuple[str, np.ndarray] | None] = queue.Queue(
            maxsize=write_queue_size
        )
        self._stop_evt = threading.Event()
        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()

    def close(self, flush: bool = True, timeout: float = 10.0) -> None:
        """Stop the async writer thread."""
        if flush:
            t0 = time.time()
            while not self._write_q.empty() and (time.time() - t0) < timeout:
                time.sleep(0.01)

        self._stop_evt.set()
        try:
            self._write_q.put_nowait(None)
        except queue.Full:
            pass
        self._writer.join(timeout=timeout)

    def _writer_loop(self) -> None:
        while not self._stop_evt.is_set():
            item = self._write_q.get()
            if item is None:
                break
            img_path, img_bgr = item
            try:
                cv2.imwrite(img_path, img_bgr)
            except Exception as e:
                logger.warning(f"Failed to write image {img_path}: {e}")

    def extract_fields(self, msg) -> tuple[dict[str, Any], float]:
        out: dict[str, Any] = {}
        timestamp = msg.header.stamp
        image_path = None

        if any(parts == ["data"] for parts, _ in self._split_mappings):
            image_path = self._process_image_data(msg)

        for parts, out_key in self._split_mappings:
            if parts == ["data"] and image_path is not None:
                val = image_path
            else:
                val = self._get_field(msg, parts)

            if val is not None:
                out.setdefault(out_key, [])
                if isinstance(val, list):
                    out[out_key].extend(val)
                else:
                    out[out_key].append(val)

        return out, timestamp

    def _process_image_data(self, msg) -> str | None:
        """Decode image, enqueue async write, return file path."""
        try:
            height = msg.height
            width = msg.width
            step = msg.step
            encoding = msg.encoding.lower()

            img_data = np.frombuffer(msg.data, dtype=np.uint8)

            if encoding in ("bgr8", "rgb8"):
                channels = 3
                if step == width * channels:
                    img = img_data.reshape((height, width, channels))
                else:
                    img = np.zeros((height, width, channels), dtype=np.uint8)
                    row_bytes = width * channels
                    for i in range(height):
                        start = i * step
                        img[i, :, :] = img_data[start : start + row_bytes].reshape(
                            (width, channels)
                        )

                if encoding == "rgb8":
                    img = img[:, :, ::-1]  # RGB -> BGR for cv2
            else:
                raise NotImplementedError(f"Unsupported encoding: {encoding}")

            stamp = msg.header.stamp
            frame_id = msg.header.frame_id.replace("/", "_")
            filename = f"image_{stamp.sec}_{stamp.nanosec}_{frame_id}.jpg"
            img_path = os.path.join(self.temp_dir, filename)

            img_copy = img.copy()
            try:
                self._write_q.put_nowait((img_path, img_copy))
            except queue.Full:
                return None

            return img_path

        except Exception as e:
            logger.warning(f"Failed to process image: {e}")
            return None

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
