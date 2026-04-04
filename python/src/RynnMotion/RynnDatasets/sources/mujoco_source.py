"""MuJoCo simulation sensor source - reads joints/sensors from mj_data."""

import numpy as np

from . import ISensorSource, SensorReading, SensorType


class MuJoCoSensorSource(ISensorSource):
    """Reads sensor data from a MuJoCo simulation.

    Can read joint positions, sensor values, or other simulation state
    from mujoco.MjData.

    Args:
        name: Source name (e.g. "sim_joints", "sim_force_sensor").
        mj_model: MuJoCo MjModel instance.
        mj_data: MuJoCo MjData instance.
        sensor_names: List of MuJoCo sensor names or joint names to read.
        sensor_type: What type of sensor data this represents.
        read_mode: "joint" to read qpos, "sensor" to read sensordata.
        prefix: Dataset key prefix.
        suffix: Optional key suffix (e.g. "arm").
    """

    def __init__(
        self,
        name: str,
        mj_model,
        mj_data,
        sensor_names: list[str],
        sensor_type: SensorType = SensorType.JOINT,
        read_mode: str = "joint",
        prefix: str = "observation",
        suffix: str | None = None,
    ):
        super().__init__(name, sensor_type)
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.sensor_names = sensor_names
        self.read_mode = read_mode
        self.prefix = prefix
        self.suffix = suffix

        self._joint_ids: list[int] | None = None
        self._sensor_ids: list[tuple[int, int]] | None = None

    def connect(self) -> None:
        import mujoco

        if self.read_mode == "joint":
            self._joint_ids = []
            for jname in self.sensor_names:
                jid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                if jid < 0:
                    raise ValueError(f"Joint '{jname}' not found in MuJoCo model")
                qadr = self.mj_model.jnt_qposadr[jid]
                self._joint_ids.append(qadr)
        elif self.read_mode == "sensor":
            self._sensor_ids = []
            for sname in self.sensor_names:
                sid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
                if sid < 0:
                    raise ValueError(f"Sensor '{sname}' not found in MuJoCo model")
                adr = self.mj_model.sensor_adr[sid]
                dim = self.mj_model.sensor_dim[sid]
                self._sensor_ids.append((adr, dim))

    def read(self) -> SensorReading:
        if self.read_mode == "joint":
            values = np.array(
                [self.mj_data.qpos[qid] for qid in self._joint_ids],
                dtype=np.float32,
            )
        elif self.read_mode == "sensor":
            parts = []
            for adr, dim in self._sensor_ids:
                parts.append(self.mj_data.sensordata[adr : adr + dim])
            values = np.concatenate(parts).astype(np.float32)
        else:
            raise ValueError(f"Unknown read_mode: {self.read_mode}")

        out_key = self._feature_key()
        return SensorReading(
            name=self.name,
            timestamp=self.mj_data.time,
            data={out_key: values},
        )

    def disconnect(self) -> None:
        pass

    def get_features(self) -> dict[str, dict]:
        if self.read_mode == "joint":
            n = len(self.sensor_names)
        else:
            n = sum(dim for _, dim in self._sensor_ids) if self._sensor_ids else len(self.sensor_names)

        key = self._feature_key()
        return {
            key: {
                "dtype": "float32",
                "shape": (n,),
                "names": list(self.sensor_names),
            }
        }

    def _feature_key(self) -> str:
        if self.prefix == "action":
            return "action" if self.suffix is None else f"action.{self.suffix}"
        if self.suffix is not None:
            return f"{self.prefix}.state.{self.suffix}"
        return f"{self.prefix}.state"
