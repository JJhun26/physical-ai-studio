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

Gripper: the follower carries a DH Robotics PGEA-100-40 parallel gripper, exposed as
the ``gripper.pos`` feature in metres over [0, GRIPPER_STROKE_M] (0 = closed). It is
actuated with ``MoveGripper`` (percent 0..100), which is a separate control path from
``ServoJ``. ``MoveGripper`` blocks in the controller until the previous open/close
finishes and hangs the XML-RPC call if a new command overlaps an in-flight one, so it
runs on a dedicated thread with its own RPC connection (xmlrpc ServerProxy is not
thread-safe) that waits on ``GetGripperMotionDone`` between commands. The teleop loop
only writes the latest target; the thread coalesces to it. This keeps gripper motion
from stalling the arm's ``ServoJ`` streaming.

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
import threading
import time
from dataclasses import dataclass, field

from loguru import logger

from robots.robot_client import RobotClient
from schemas.robot import RobotType

from . import GRIPPER_FEATURE, GRIPPER_STROKE_M, JOINT_NAMES, NUM_JOINTS, POS_FEATURES


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

    # -- PGEA-100-40 gripper --------------------------------------------------
    # Whether the follower drives its parallel gripper via the gripper.pos feature.
    gripper_enabled: bool = True
    # Controller gripper number (Fairino end-effector index; usually 1).
    gripper_index: int = 1
    # MoveGripper velocity / force percentages.
    gripper_vel_pct: int = 100
    gripper_force_pct: int = 30
    # Send ActGripper(index, 1) at connect() to activate the gripper.
    gripper_activate: bool = True
    # Only re-command when the target moves at least this many percent (anti-flood).
    gripper_thresh_pct: float = 2.0
    # Cap on how long the worker waits for one open/close to report done (s).
    gripper_motion_timeout_s: float = 3.0


class _GripperWorker:
    """Serialises MoveGripper on its own thread + RPC connection.

    The teleop loop only ever writes the latest target percent; this worker
    coalesces to it and never lets two open/close motions overlap (which hangs the
    controller's XML-RPC reply). Proven against hardware by the standalone
    uArm->FR5 gripper teleop script; the overlap guard is the load-bearing part.
    """

    def __init__(self, rpc, config: FR5FollowerConfig) -> None:
        self._rpc = rpc
        self._index = config.gripper_index
        self._vel = config.gripper_vel_pct
        self._force = config.gripper_force_pct
        self._thresh = config.gripper_thresh_pct
        self._timeout = config.gripper_motion_timeout_s
        self._target_pct: float | None = None
        self._last_cmd_pct: float | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="FR5GripperWorker", daemon=True)
        self._thread.start()

    def set_target_pct(self, pct: float) -> None:
        with self._lock:
            self._target_pct = _clamp(pct, 0.0, 100.0)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _motion_done(self) -> bool:
        try:
            md = self._rpc.GetGripperMotionDone()
            return isinstance(md, (list, tuple)) and md[0] == 0 and md[1][1] == 1
        except Exception:  # noqa: BLE001 -- unreadable status: allow the next command
            return True

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                target = self._target_pct
            if target is not None and (self._last_cmd_pct is None or abs(target - self._last_cmd_pct) >= self._thresh):
                try:
                    # (index, pos%, vel%, force%, maxtime ms, block=1 non-blocking,
                    #  type=0 parallel, rotNum, rotVel, rotTorque) -- last three unused.
                    err = self._rpc.MoveGripper(self._index, int(round(target)), self._vel, self._force, 3000, 1, 0, 0.0, 0, 0)
                    self._last_cmd_pct = target
                    if err != 0:
                        logger.warning(f"FR5 MoveGripper(pos={int(round(target))}%) err={err}")
                except Exception as e:  # noqa: BLE001 -- keep the worker alive on a transient RPC error
                    logger.warning(f"FR5 MoveGripper failed: {e}")
                    self._stop.wait(0.1)
                    continue
                # Wait out the motion so the next command cannot overlap it.
                self._stop.wait(0.06)
                t0 = time.perf_counter()
                while not self._stop.is_set() and (time.perf_counter() - t0) < self._timeout:
                    if self._motion_done():
                        break
                    self._stop.wait(0.03)
            self._stop.wait(0.02)


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
        self._gripper: _GripperWorker | None = None
        # Last commanded gripper opening in metres [0, GRIPPER_STROKE_M]; the controller
        # exposes no position getter, so read_state echoes the last command. Starts at
        # fully open, the pose the gripper is modelled and usually powers on in.
        self._last_gripper_m: float = GRIPPER_STROKE_M

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
        if self._config.gripper_enabled:
            self._start_gripper(robot_cls)

    def _start_gripper(self, robot_cls) -> None:
        # Dedicated RPC connection: xmlrpc ServerProxy is not thread-safe, so the
        # gripper worker must not share the arm's _rpc.
        try:
            gripper_rpc = robot_cls.RPC(self._config.ip)
            if self._config.gripper_activate:
                err = gripper_rpc.ActGripper(self._config.gripper_index, 1)
                if err != 0:
                    logger.warning(f"FR5 ActGripper({self._config.gripper_index},1) err={err}")
            worker = _GripperWorker(gripper_rpc, self._config)
            worker.start()
            self._gripper = worker
            logger.info(f"FR5 gripper worker started (index={self._config.gripper_index})")
        except Exception as e:  # noqa: BLE001 -- a missing/faulty gripper must not fail arm teleop
            logger.warning(f"FR5 gripper unavailable, continuing without it: {e}")
            self._gripper = None

    def disconnect(self) -> None:
        logger.info("Disconnecting FR5 follower")
        if self._gripper is not None:
            self._gripper.stop()
            self._gripper = None
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

        # Gripper is a separate control path: hand the latest opening to the worker
        # thread (metres -> percent) instead of streaming it with the arm.
        if self._gripper is not None and GRIPPER_FEATURE in joints:
            opening_m = _clamp(float(joints[GRIPPER_FEATURE]), 0.0, GRIPPER_STROKE_M)
            self._last_gripper_m = opening_m
            self._gripper.set_target_pct(opening_m / GRIPPER_STROKE_M * 100.0)

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
        if self._config.gripper_enabled:
            # No controller-side gripper position getter, so echo the last command;
            # this is what drives the viewer's carriage joints.
            state[GRIPPER_FEATURE] = self._last_gripper_m
        return self._create_event("state_was_updated", state=state, is_controlled=True)

    def read_forces(self) -> dict | None:
        # FR5 joint torque/force streaming is not wired up yet; report no force state.
        return self._create_event("force_was_updated", state=None, is_controlled=True)

    def set_forces(self, forces: dict) -> dict:
        return forces

    def features(self) -> list[str]:
        feats = list(POS_FEATURES)
        if self._config.gripper_enabled:
            feats.append(GRIPPER_FEATURE)
        return feats
