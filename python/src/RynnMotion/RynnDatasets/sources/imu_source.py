"""Direct IMU sensor source."""

from collections.abc import Callable

import numpy as np

from . import ISensorSource, SensorReading, SensorType


class DirectIMUSensorSource(ISensorSource):
    """Reads IMU data (accelerometer, gyroscope, optional quaternion).

    Default 10D: [ax, ay, az, gx, gy, gz, qw, qx, qy, qz]
    Can be configured for 6D (accel+gyro only) or other layouts.

    Args:
        name: Sensor name (e.g. "body_imu").
        reader_fn: Callable returning np.ndarray.
        num_dims: Number of IMU dimensions (6, 9, or 10).
        prefix: Dataset key prefix.
        suffix: Optional key suffix.
    """

    IMU_NAMES_10 = ["ax", "ay", "az", "gx", "gy", "gz", "qw", "qx", "qy", "qz"]
    IMU_NAMES_9 = ["ax", "ay", "az", "gx", "gy", "gz", "qx", "qy", "qz"]
    IMU_NAMES_6 = ["ax", "ay", "az", "gx", "gy", "gz"]

    def __init__(
        self,
        name: str,
        reader_fn: Callable[[], np.ndarray],
        num_dims: int = 10,
        prefix: str = "observation",
        suffix: str | None = None,
    ):
        super().__init__(name, SensorType.IMU)
        self.reader_fn = reader_fn
        self.num_dims = num_dims
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

        key = self._feature_key()
        return SensorReading(
            name=self.name,
            timestamp=self.monotonic_timestamp(),
            data={key: values},
        )

    def disconnect(self) -> None:
        pass

    def get_features(self) -> dict[str, dict]:
        if self.num_dims == 10:
            names = list(self.IMU_NAMES_10)
        elif self.num_dims == 9:
            names = list(self.IMU_NAMES_9)
        elif self.num_dims == 6:
            names = list(self.IMU_NAMES_6)
        else:
            names = [f"dim_{i}" for i in range(self.num_dims)]

        key = self._feature_key()
        return {
            key: {
                "dtype": "float32",
                "shape": (self.num_dims,),
                "names": names,
            }
        }

    def _feature_key(self) -> str:
        suffix_part = f".{self.suffix}" if self.suffix else ""
        return f"{self.prefix}.state.imu{suffix_part}"
