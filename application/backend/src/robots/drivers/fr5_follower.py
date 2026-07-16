"""Fairino FR5 follower RobotClient (Ethernet / Fairino SDK).

Follower contract for the studio ``TeleoperateWorker``:
- ``read_state()``  -> ``{"j1.pos": deg, ..., "j6.pos": deg}`` in controller degrees.
- ``set_joints_state(joints, goal_time)`` -> read each ``jN.pos`` as degrees, clamp to
  the joint's usable range and stream it with ``ServoJ``.

Joint values are plain controller degrees -- the same numbers the FR5 WebApp shows.
This departs from SO101's normalized -100..100 convention on purpose: the UI viewer
renders observation values as degrees (``degToRad`` in robot-models-context.tsx), so
publishing normalized values draws a pose that does not match the real arm. SO101 gets
away with it because its calibrated range happens to be about +-100 deg and centered;
FR5 does not, and j2/j4 -- whose soft limits are not centered on 0 deg -- were drawn
~88 deg off. Reporting degrees keeps state, actions and the viewer in one unit.

The uArm leader still publishes -100..100, which lands here as -100..100 deg and is
then clamped to the usable range. All sign/permutation lives on the leader (uArm
calibration), so this driver applies no remapping of its own.

Safety: ``set_joints_state`` rate-limits every joint to ``max_step_deg`` per call.
Because absolute mapping makes the FR5 jump to the leader's mapped pose the instant
teleoperation starts, this per-call cap turns that jump into a bounded-speed slew
the operator can still E-stop. Widen it only after the mapping is trusted.

Uses the Fairino SDK vendored at ``robots.drivers.vendor`` (pure XML-RPC over port
20003, so it runs in-process in the py3.12 backend). That copy is pinned to
controller firmware v3.9.1 — see the vendor package docstring. ``FAIRINO_SDK_LINUX``
overrides it with an external directory, but the vendored copy is the supported path.
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
class FR5FollowerConfig:
    ip: str = "192.168.58.2"
    # Escape hatch only: absolute path to an external Fairino SDK directory to
    # import instead of the vendored copy (e.g. while testing a firmware upgrade).
    # Empty -> the vendored SDK, which is the supported path.
    sdk_path: str = field(default_factory=lambda: os.environ.get("FAIRINO_SDK_LINUX", ""))
    # Per-joint degree range that incoming commands are clamped to.
    # None -> derive from the controller's soft joint limits at connect().
    joint_deg_min: list[float] | None = None
    joint_deg_max: list[float] | None = None
    # When ranges are derived, stay this far inside the controller's soft limits (deg).
    limit_margin_deg: float = 5.0
    # Per-call slew cap (deg). Primary guard against the start-of-teleop jump.
    max_step_deg: float = 2.0
    # ServoJ interpolation time (s). 0 -> use the worker's goal_time.
    servo_cmd_t: float = 0.0


class FR5FollowerClient(RobotClient):
    name = "FR5Follower"

    def __init__(self, config: FR5FollowerConfig | None = None) -> None:
        self._config = config or FR5FollowerConfig()
        self._rpc = None
        self._connected = False
        self._servo_started = False
        self._deg_lo: list[float] = []
        self._deg_hi: list[float] = []
        self._last_target: list[float] = []

    @property
    def robot_type(self) -> RobotType:
        return RobotType.FR5_FOLLOWER

    @property
    def is_connected(self) -> bool:
        return self._connected

    # -- connection -------------------------------------------------------

    def _import_sdk(self):
        override = self._config.sdk_path
        if not override:
            from .vendor.fairino import Robot

            return Robot
        logger.warning(f"Importing external Fairino SDK from {override} instead of the vendored copy")
        if override not in sys.path:
            sys.path.insert(0, override)
        from fairino import Robot  # type: ignore

        return Robot

    def _read_joint_deg(self) -> list[float]:
        ret = self._rpc.GetActualJointPosDegree()
        if not (isinstance(ret, list | tuple) and ret[0] == 0):
            raise RuntimeError(f"FR5 GetActualJointPosDegree failed: {ret}")
        return [float(v) for v in ret[1][:NUM_JOINTS]]

    def _resolve_ranges(self) -> None:
        cfg = self._config
        if cfg.joint_deg_min is not None and cfg.joint_deg_max is not None:
            self._deg_lo = list(cfg.joint_deg_min)
            self._deg_hi = list(cfg.joint_deg_max)
            return

        lim = self._fetch_soft_limits(cfg.limit_margin_deg)
        if lim is None:
            raise RuntimeError("FR5 soft joint limits unavailable; set joint_deg_min/joint_deg_max explicitly")
        self._deg_lo, self._deg_hi = lim

    def _fetch_soft_limits(self, margin: float) -> tuple[list[float], list[float]] | None:
        getter = getattr(self._rpc, "GetJointSoftLimitDeg", None)
        if getter is None:
            return None
        ret = getter(1)
        if not (isinstance(ret, list | tuple) and ret[0] == 0 and ret[1]):
            return None
        v = list(ret[1])
        if len(v) < 2 * NUM_JOINTS:
            return None
        a_lo, a_hi = v[0:NUM_JOINTS], v[NUM_JOINTS : 2 * NUM_JOINTS]
        if all(a_lo[j] < a_hi[j] for j in range(NUM_JOINTS)):
            p_lo, p_hi = a_lo, a_hi
        else:  # interleaved layout [neg1,pos1,neg2,pos2,...]
            p_lo = [v[2 * j] for j in range(NUM_JOINTS)]
            p_hi = [v[2 * j + 1] for j in range(NUM_JOINTS)]
        lo = [min(p_lo[j], p_hi[j]) + margin for j in range(NUM_JOINTS)]
        hi = [max(p_lo[j], p_hi[j]) - margin for j in range(NUM_JOINTS)]
        return lo, hi

    def connect(self) -> None:
        logger.info(f"Connecting FR5 follower at {self._config.ip}")
        robot_cls = self._import_sdk()
        self._rpc = robot_cls.RPC(self._config.ip)
        current = self._read_joint_deg()  # validates the link
        self._rpc.RobotEnable(1)
        self._rpc.Mode(0)  # auto mode
        self._resolve_ranges()
        self._last_target = current
        self._connected = True
        logger.info(
            "FR5 connected. usable deg ranges: "
            + ", ".join(f"J{j + 1}[{self._deg_lo[j]:.0f},{self._deg_hi[j]:.0f}]" for j in range(NUM_JOINTS))
        )

    def disconnect(self) -> None:
        logger.info("Disconnecting FR5 follower")
        try:
            if self._servo_started and self._rpc is not None:
                self._rpc.ServoMoveEnd()
        except Exception as e:
            logger.warning(f"FR5 ServoMoveEnd on disconnect failed: {e}")
        finally:
            self._servo_started = False
            self._connected = False
            # Torque is intentionally left enabled; use the pendant to disable.

    # -- state / mapping --------------------------------------------------

    def ping(self) -> dict:
        return self._create_event("pong")

    def set_joints_state(self, joints: dict, goal_time: float) -> dict:
        if not self._servo_started:
            self._rpc.ServoMoveStart()
            self._servo_started = True

        max_step = self._config.max_step_deg
        target: list[float] = []
        for j, name in enumerate(JOINT_NAMES):
            desired = _clamp(float(joints[f"{name}.pos"]), self._deg_lo[j], self._deg_hi[j])
            step = _clamp(desired - self._last_target[j], -max_step, max_step)
            target.append(self._last_target[j] + step)

        cmd_t = self._config.servo_cmd_t or goal_time
        err = self._rpc.ServoJ(joint_pos=target, axisPos=[0, 0, 0, 0], cmdT=cmd_t)
        if err != 0:
            logger.warning(f"FR5 ServoJ err={err}")
        self._last_target = target
        return self._create_event("joints_state_was_set", joints=joints)

    def enable_torque(self) -> dict:
        if self._rpc is not None:
            self._rpc.RobotEnable(1)
        return self._create_event("torque_was_enabled")

    def disable_torque(self) -> dict:
        try:
            if self._servo_started and self._rpc is not None:
                self._rpc.ServoMoveEnd()
                self._servo_started = False
        except Exception as e:
            logger.warning(f"FR5 ServoMoveEnd failed: {e}")
        return self._create_event("torque_was_disabled")

    def read_state(self, *, normalize: bool = True) -> dict:  # noqa: ARG002
        degs = self._read_joint_deg()
        state = {f"{JOINT_NAMES[j]}.pos": degs[j] for j in range(NUM_JOINTS)}
        return self._create_event("state_was_updated", state=state, is_controlled=True)

    def read_forces(self) -> dict | None:
        # FR5 joint torque/force streaming is not wired up yet; report no force state.
        return self._create_event("force_was_updated", state=None, is_controlled=True)

    def set_forces(self, forces: dict) -> dict:
        return forces

    def features(self) -> list[str]:
        return list(POS_FEATURES)
