"""
Temporal alignment for multi-sensor data collection.

Provides sync and async alignment modes:
- sync: Passthrough, no buffering. For direct hardware reads in lock-step.
- async: Buffer + min-latest-timestamp alignment. For ROS2/network sources.

Async alignment algorithm ported from ros2_data_collector ProcessingModule.
"""

import logging
import threading
from collections import deque

from RynnMotion.RynnDatasets.sources import SensorReading

logger = logging.getLogger(__name__)


class TemporalAligner:
    """Aligns readings from multiple sensor sources.

    Args:
        source_names: List of source names that will push readings.
        mode: "sync" for lock-step direct reads, "async" for buffered alignment.
        fps: Target frame rate (used for timing in async mode).
        buffer_maxlen: Maximum buffer size per source in async mode.
    """

    def __init__(
        self,
        source_names: list[str],
        mode: str = "sync",
        fps: float = 30.0,
        buffer_maxlen: int = 3000,
    ):
        if mode not in ("sync", "async"):
            raise ValueError(f"mode must be 'sync' or 'async', got '{mode}'")

        self.source_names = list(source_names)
        self.mode = mode
        self.fps = fps

        if mode == "async":
            self._lock = threading.Lock()
            self._buffers: dict[str, deque[SensorReading]] = {
                name: deque(maxlen=buffer_maxlen) for name in source_names
            }

    def push(self, reading: SensorReading) -> None:
        """Push a sensor reading into the alignment buffer (async mode).

        In sync mode this is a no-op since readings are assembled directly.

        Args:
            reading: Sensor reading to buffer.
        """
        if self.mode == "sync":
            return

        with self._lock:
            buf = self._buffers.get(reading.name)
            if buf is None:
                logger.warning(f"Unknown source '{reading.name}' pushed to aligner")
                return
            buf.append(reading)

    def get_aligned_frame(self) -> dict[str, SensorReading] | None:
        """Get an aligned frame from buffered readings.

        sync mode: Returns None (caller assembles frame directly from read()).
        async mode: Uses min-latest-timestamp strategy.
            - Finds the minimum of all sources' latest timestamps.
            - For each source, pops readings older than that timestamp
              and returns the first reading at or after the frame time.

        Returns:
            Dict mapping source name to aligned SensorReading, or None
            if not all sources have data (async) or in sync mode.
        """
        if self.mode == "sync":
            return None

        with self._lock:
            # Check all buffers have data
            if not all(buf for buf in self._buffers.values()):
                return None

            # Min-latest-timestamp: the oldest "latest" across all sources
            latest_timestamps = []
            for buf in self._buffers.values():
                if buf:
                    latest_timestamps.append(buf[-1].timestamp)
            frame_time = min(latest_timestamps)

            frame: dict[str, SensorReading] = {}
            for name, buf in self._buffers.items():
                # Pop readings older than frame_time
                while buf and buf[0].timestamp < frame_time:
                    buf.popleft()

                if buf:
                    frame[name] = buf.popleft()
                else:
                    logger.warning(
                        f"No valid data for source '{name}' after alignment "
                        f"(frame_time={frame_time:.4f})"
                    )
                    return None

            return frame

    def reset(self) -> None:
        """Clear all internal buffers."""
        if self.mode == "async":
            with self._lock:
                for buf in self._buffers.values():
                    buf.clear()
