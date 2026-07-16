"""uArm leader RobotClient (Feetech STS3215 serial bus).

Leader contract for the studio ``TeleoperateWorker``: ``read_state()`` returns
``{"j1.pos": norm, ..., "j6.pos": norm}`` in -100..100 (SO101's centered
convention, which the UI viewer treats as degrees). The worker forwards those
same-named values straight into the FR5 follower, so *all* per-joint sign,
permutation and range live in this leader's calibration:

    raw = present_position(motor_id) mod 4096            # single-turn absolute
    raw = (raw - homing_offset) mod 4096
    norm = (raw - range_min) / (range_max - range_min) * 200 - 100   # clamped -100..100
    if drive_mode == 1: norm = -norm                     # inverted joint

Workspace notes captured while tuning the standalone teleop script
(``uarm-xarm-VLAproject``): j1, j2, j4 are inverted, and joint 6 uses a 360°
servo. Encode those as ``drive_mode`` / motor-id order in the calibration passed
by the factory. Without a calibration the leader falls back to the full turn
(0..4095), which is safe to *read* (leader never moves) but not yet aligned to the
FR5 — calibrate before driving.

Imports the Feetech ``scservo_sdk`` vendored at ``robots.drivers.vendor`` — see
that package's docstring for why the PyPI ``feetech-servo-sdk`` pulled in by
``lerobot[feetech]`` cannot be used instead. ``UARM_FEETECH_SDK`` overrides it
with an external directory, but the vendored copy is the supported path.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field

from loguru import logger

from robots.robot_client import RobotClient
from schemas.robot import RobotType

from . import JOINT_NAMES, NUM_JOINTS, POS_FEATURES


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass
class JointCalibration:
    motor_id: int
    range_min: int = 0
    range_max: int = 4095
    drive_mode: int = 0  # 1 -> invert
    homing_offset: int = 0


@dataclass
class UArmLeaderConfig:
    port: str = "/dev/ttyACM0"
    baud: int = 1_000_000
    # Escape hatch only: absolute path to an external Feetech SDK directory to
    # import instead of the vendored copy. Empty -> the vendored SDK, which is
    # the supported path.
    sdk_path: str = field(default_factory=lambda: os.environ.get("UARM_FEETECH_SDK", ""))
    # One entry per joint j1..j6, in order. Defaults: motor ids 1..6, full turn.
    joints: list[JointCalibration] | None = None

    def resolved_joints(self) -> list[JointCalibration]:
        if self.joints is not None:
            if len(self.joints) != NUM_JOINTS:
                raise ValueError(f"uArm calibration must have {NUM_JOINTS} joints, got {len(self.joints)}")
            return self.joints
        return [JointCalibration(motor_id=i) for i in range(1, NUM_JOINTS + 1)]


class UArmLeaderClient(RobotClient):
    name = "UArmLeader"

    def __init__(self, config: UArmLeaderConfig | None = None) -> None:
        self._config = config or UArmLeaderConfig()
        self._joints = self._config.resolved_joints()
        self._port_handler = None
        self._reader = None
        self._addr = None  # SMS_STS_PRESENT_POSITION_L
        self._connected = False
        self._prev_raw: list[float | None] = [None] * NUM_JOINTS

    @property
    def robot_type(self) -> RobotType:
        return RobotType.UARM_LEADER

    @property
    def is_connected(self) -> bool:
        return self._connected

    # -- connection -------------------------------------------------------

    def _import_sdk(self):
        override = self._config.sdk_path
        if not override:
            from .vendor.scservo_sdk import SMS_STS_PRESENT_POSITION_L, GroupSyncRead, PortHandler, sms_sts

            return GroupSyncRead, PortHandler, SMS_STS_PRESENT_POSITION_L, sms_sts
        logger.warning(f"Importing external Feetech SDK from {override} instead of the vendored copy")
        if override not in sys.path:
            sys.path.insert(0, override)
        from scservo_sdk import SMS_STS_PRESENT_POSITION_L, GroupSyncRead, PortHandler, sms_sts  # type: ignore

        return GroupSyncRead, PortHandler, SMS_STS_PRESENT_POSITION_L, sms_sts

    def connect(self) -> None:
        logger.info(f"Connecting uArm leader on {self._config.port}@{self._config.baud}")
        group_sync_read, port_handler_cls, present_pos_l, sms_sts_cls = self._import_sdk()
        self._addr = present_pos_l
        ph = port_handler_cls(self._config.port)
        pk = sms_sts_cls(ph)
        if not ph.openPort():
            raise RuntimeError(f"Failed to open uArm leader port: {self._config.port}")
        ph.setBaudRate(self._config.baud)
        reader = group_sync_read(pk, present_pos_l, 4)
        for jc in self._joints:
            reader.addParam(jc.motor_id)
        self._port_handler = ph
        self._reader = reader
        self._connected = True

    def disconnect(self) -> None:
        logger.info("Disconnecting uArm leader")
        try:
            if self._port_handler is not None:
                self._port_handler.closePort()
        finally:
            self._connected = False

    # -- state ------------------------------------------------------------

    def _read_norm(self) -> dict[str, float]:
        self._reader.txRxPacket()
        state: dict[str, float] = {}
        for j, jc in enumerate(self._joints):
            ok = self._reader.isAvailable(jc.motor_id, self._addr, 2)[0]
            if ok:
                raw = self._reader.getData(jc.motor_id, self._addr, 2)
                self._prev_raw[j] = raw
            elif self._prev_raw[j] is not None:
                raw = self._prev_raw[j]  # tolerate a dropped read
            else:
                raw = jc.range_min
            single = (int(raw) - jc.homing_offset) % 4096
            span = jc.range_max - jc.range_min
            norm = 0.0 if span == 0 else _clamp((single - jc.range_min) / span * 200.0 - 100.0, -100.0, 100.0)
            if jc.drive_mode == 1:
                norm = -norm
            state[f"{JOINT_NAMES[j]}.pos"] = norm
        return state

    def ping(self) -> dict:
        return self._create_event("pong")

    def read_state(self, *, normalize: bool = True) -> dict:  # noqa: ARG002
        return self._create_event("state_was_updated", state=self._read_norm(), is_controlled=False)

    # -- leader is read-only ---------------------------------------------

    def set_joints_state(self, joints: dict, goal_time: float) -> dict:  # noqa: ARG002
        # Leader never receives motion; present for interface completeness.
        return self._create_event("joints_state_was_set", joints=joints)

    def enable_torque(self) -> dict:
        return self._create_event("torque_was_enabled")

    def disable_torque(self) -> dict:
        return self._create_event("torque_was_disabled")

    def read_forces(self) -> dict | None:
        return self._create_event("force_was_updated", state=None, is_controlled=False)

    def set_forces(self, forces: dict) -> dict:
        return forces

    def features(self) -> list[str]:
        return list(POS_FEATURES)
