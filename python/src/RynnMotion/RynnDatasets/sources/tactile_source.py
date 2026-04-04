"""Direct tactile sensor source."""

from collections.abc import Callable

import numpy as np

from . import ISensorSource, SensorReading, SensorType


class DirectTactileSensorSource(ISensorSource):
    """Reads tactile sensor arrays.

    Args:
        name: Sensor name (e.g. "fingertip_tactile").
        reader_fn: Callable returning np.ndarray of tactile values.
        num_dims: Number of tactile elements.
        prefix: Dataset key prefix.
        suffix: Optional key suffix.
    """

    def __init__(
        self,
        name: str,
        reader_fn: Callable[[], np.ndarray],
        num_dims: int = 16,
        prefix: str = "observation",
        suffix: str | None = None,
    ):
        super().__init__(name, SensorType.TACTILE)
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
        key = self._feature_key()
        return {
            key: {
                "dtype": "float32",
                "shape": (self.num_dims,),
                "names": [f"taxel_{i}" for i in range(self.num_dims)],
            }
        }

    def _feature_key(self) -> str:
        suffix_part = f".{self.suffix}" if self.suffix else ""
        return f"{self.prefix}.state.tactile{suffix_part}"
