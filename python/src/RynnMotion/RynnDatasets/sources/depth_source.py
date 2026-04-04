"""Direct depth camera sensor source."""

import numpy as np

from . import ISensorSource, SensorReading, SensorType


class DirectDepthSensorSource(ISensorSource):
    """Reads depth images (single channel float or uint16).

    Args:
        name: Camera name (e.g. "depth_front").
        camera: Object with .read() returning depth np.ndarray, or callable.
        resolution: (height, width, 1) tuple.
        prefix: Dataset key prefix.
        use_video: Whether to store as video or images.
    """

    def __init__(
        self,
        name: str,
        camera: object,
        resolution: tuple[int, int, int] = (480, 640, 1),
        prefix: str = "observation",
        use_video: bool = True,
    ):
        super().__init__(name, SensorType.DEPTH)
        self._camera_input = camera
        self._camera = None
        self.resolution = resolution
        self.prefix = prefix
        self.use_video = use_video

    def connect(self) -> None:
        self._camera = self._camera_input

    def read(self) -> SensorReading:
        if self._camera is None:
            raise RuntimeError("Depth camera not connected. Call connect() first.")

        if hasattr(self._camera, "read"):
            result = self._camera.read()
            if isinstance(result, tuple):
                ret, frame = result
                if not ret:
                    raise RuntimeError(f"Failed to read from depth camera '{self.name}'")
            else:
                frame = result
        elif callable(self._camera):
            frame = self._camera()
        else:
            raise TypeError(f"Camera object of type {type(self._camera)} not supported")

        if not isinstance(frame, np.ndarray):
            frame = np.asarray(frame)

        # Ensure single channel
        if frame.ndim == 2:
            frame = frame[:, :, np.newaxis]

        out_key = f"{self.prefix}.images.depth_{self.name}"
        return SensorReading(
            name=self.name,
            timestamp=self.monotonic_timestamp(),
            data={out_key: frame},
        )

    def disconnect(self) -> None:
        self._camera = None

    def get_features(self) -> dict[str, dict]:
        key = f"{self.prefix}.images.depth_{self.name}"
        return {
            key: {
                "dtype": "video" if self.use_video else "image",
                "shape": self.resolution,
                "names": ["height", "width", "channels"],
            }
        }
