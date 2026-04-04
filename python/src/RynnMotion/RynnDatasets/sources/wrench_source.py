"""Direct wrench (force/torque) sensor source."""

from collections.abc import Callable

import numpy as np

from . import ISensorSource, SensorReading, SensorType


class DirectWrenchSensorSource(ISensorSource):
    """Reads 6D force/torque data [fx, fy, fz, tx, ty, tz].

    Args:
        name: Sensor name (e.g. "wrist_wrench").
        reader_fn: Callable returning np.ndarray of shape (6,).
        prefix: Dataset key prefix.
        suffix: Optional key suffix.
    """

    WRENCH_NAMES = ["fx", "fy", "fz", "tx", "ty", "tz"]

    def __init__(
        self,
        name: str,
        reader_fn: Callable[[], np.ndarray],
        prefix: str = "observation",
        suffix: str | None = None,
    ):
        super().__init__(name, SensorType.WRENCH)
        self.reader_fn = reader_fn
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
                "shape": (6,),
                "names": list(self.WRENCH_NAMES),
            }
        }

    def _feature_key(self) -> str:
        suffix_part = f".{self.suffix}" if self.suffix else ""
        return f"{self.prefix}.state.wrench{suffix_part}"
