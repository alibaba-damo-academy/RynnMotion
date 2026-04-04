"""Direct camera sensor source - reads from OpenCV or any camera object."""

import logging

import numpy as np

from . import ISensorSource, SensorReading, SensorType

logger = logging.getLogger(__name__)


class DirectCameraSensorSource(ISensorSource):
    """Reads camera frames from OpenCV VideoCapture or any object with read().

    Args:
        name: Camera name (e.g. "camera_front", "wrist_cam").
        camera: OpenCV VideoCapture index (int), or object with .read() -> np.ndarray.
        resolution: (height, width, channels) tuple.
        fps: Expected frame rate (informational, not enforced).
        prefix: Dataset key prefix.
        use_video: Whether to store as video (True) or images (False).
    """

    def __init__(
        self,
        name: str,
        camera: int | object,
        resolution: tuple[int, int, int] = (480, 640, 3),
        fps: int = 30,
        prefix: str = "observation",
        use_video: bool = True,
    ):
        super().__init__(name, SensorType.CAMERA)
        self._camera_input = camera
        self._camera = None
        self.resolution = resolution
        self.fps = fps
        self.prefix = prefix
        self.use_video = use_video
        self._last_good_frame: np.ndarray | None = None

    def connect(self) -> None:
        if isinstance(self._camera_input, int):
            import cv2
            self._camera = cv2.VideoCapture(self._camera_input)
            if not self._camera.isOpened():
                raise RuntimeError(f"Failed to open camera at index {self._camera_input}")
        else:
            self._camera = self._camera_input

    def read(self) -> SensorReading:
        if self._camera is None:
            raise RuntimeError("Camera not connected. Call connect() first.")

        try:
            if hasattr(self._camera, "read"):
                result = self._camera.read()
                # OpenCV VideoCapture returns (bool, frame)
                if isinstance(result, tuple):
                    ret, frame = result
                    if not ret:
                        raise RuntimeError(f"Failed to read from camera '{self.name}'")
                else:
                    frame = result
            elif callable(self._camera):
                frame = self._camera()
            else:
                raise TypeError(f"Camera object of type {type(self._camera)} not supported")

            if not isinstance(frame, np.ndarray):
                frame = np.asarray(frame)

            self._last_good_frame = frame
        except Exception as e:
            logger.warning(f"Camera '{self.name}' read failed: {e}")
            if self._last_good_frame is not None:
                frame = self._last_good_frame
            else:
                frame = np.zeros(self.resolution, dtype=np.uint8)

        out_key = f"{self.prefix}.images.{self.name}"
        return SensorReading(
            name=self.name,
            timestamp=self.monotonic_timestamp(),
            data={out_key: frame},
        )

    def disconnect(self) -> None:
        if self._camera is not None and isinstance(self._camera_input, int):
            self._camera.release()
            self._camera = None

    def get_features(self) -> dict[str, dict]:
        key = f"{self.prefix}.images.{self.name}"
        return {
            key: {
                "dtype": "video" if self.use_video else "image",
                "shape": self.resolution,
                "names": ["height", "width", "channels"],
            }
        }
