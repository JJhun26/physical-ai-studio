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

The uArm leader publishes the same degrees (it used to saturate at +-100, which put j2's
habitual -150 deg working pose out of reach; see ``uarm_leader.out_limit_deg``). Values
land here as degrees and are clamped to the usable range -- this clamp, not anything on
the leader, is what bounds the arm. All sign/permutation lives on the leader (uArm
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
import xmlrpc.client
from contextlib import suppress
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from loguru import logger

from robots.robot_client import RobotClient
from schemas.robot import RobotType

from . import GRIPPER_FEATURE, GRIPPER_STROKE_M, JOINT_NAMES, NUM_JOINTS, POS_FEATURES

if TYPE_CHECKING:
    import http.client


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
    # ServoJ interpolation time (s): how long the controller takes to consume one point.
    # It must not exceed the interval points arrive at, or the controller's lookahead
    # buffer grows by the difference every cycle -- lag climbs without bound until ServoJ
    # blocks, and the loop then runs at whatever rate the controller drains.
    #
    # The worker hands set_joints_state *twice* its loop period (a convention that suits
    # position-controlled arms, where it means "reach this within two periods"). Taken
    # literally as cmdT that is 2x the send interval, which is exactly the runaway case:
    # at 30 Hz it made every point take 66.7 ms, pacing teleoperation at 15 Hz with
    # visible lag. So 0 means half of what the worker passes -- the true send period.
    # The SDK recommends 0.001..0.0016 s and defaults to 0.008; a fixed value that small
    # only makes sense with a matching send rate (the standalone teleop script streams at
    # 250 Hz with cmdT=0.004), otherwise the arm lurches to each point and waits.
    servo_cmd_t: float = 0.0
    # How long connect() waits for the SDK's UDP state channel to deliver its first
    # packet. See _wait_for_state -- reading before it lands is a hard error, not a
    # stale value, so this is a correctness wait rather than a nicety.
    state_timeout_s: float = 5.0

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
    # Socket timeout on the gripper's XML-RPC channel. Bounds every gripper call so a
    # wedged controller degrades the gripper instead of stalling the arm.
    gripper_rpc_timeout_s: float = 3.0

    # -- collision -----------------------------------------------------------
    # Per-joint collision threshold, 1 (most sensitive) .. 10, applied at connect.
    # None leaves whatever the controller is configured with. Raise this if light
    # contact during teleoperation trips a fault; it does not make contact safe, it
    # only stops the controller reacting to normal teleoperation loads.
    collision_level: list[float] | None = None
    # Clear the fault and carry on rather than leaving the arm dead until the session
    # is restarted. Never resumes commanding on its own -- see resync_tolerance_deg.
    auto_recover: bool = True
    # After a fault the leader is usually still pointing into whatever was hit, so
    # commanding again would drive straight back in. Streaming stays suspended until
    # the operator brings the leader within this many degrees of where the arm actually
    # is, on every joint. That re-sync is the safety interlock, not the reset itself.
    resync_tolerance_deg: float = 5.0


class _TimeoutTransport(xmlrpc.client.Transport):
    """XML-RPC transport with a socket timeout (``ServerProxy`` exposes no such option)."""

    def __init__(self, timeout: float) -> None:
        super().__init__()
        self._timeout = timeout

    def make_connection(self, host: str | tuple[str, dict[str, str]]) -> http.client.HTTPConnection:
        conn = super().make_connection(host)
        conn.timeout = self._timeout
        return conn


def _as_int(value: object) -> int:
    """XML-RPC's declared return is a wide union; these calls always answer an int code."""
    if isinstance(value, int):
        return value
    raise TypeError(f"expected an integer code from the controller, got {value!r}")


class _GripperRPC:
    """The gripper's own XML-RPC channel to port 20003.

    Deliberately **not** a second ``Robot.RPC``. That constructor opens its own tcp/20004
    state connection, and the controller serves exactly one at a time: the second never
    completes, the SDK's state thread spins in ``reconnect()`` forever, and every gripper
    call then parks in ``while self.reconnect_flag: time.sleep(0.1)`` (``Robot.py:4994``).
    That hung ``connect()``, so ``setup()`` never finished, ``loaded_event`` never fired
    and the websocket sent no observations at all -- the arm looked frozen in the viewer.

    A bare ``ServerProxy`` still gives the worker thread the separate connection it needs
    (``ServerProxy`` is not thread-safe) without a second state channel. The SDK's gripper
    methods are thin wrappers over these same calls; only the return shapes differ, and
    those are normalised here so ``_GripperWorker`` sees what the SDK would have returned.

    The safety check ``MoveGripper`` would have run is not lost: it reads the cached state
    packet, which this channel has no access to anyway, and ``ServoJ`` already checks it on
    every arm command.
    """

    def __init__(self, ip: str, timeout: float) -> None:
        self._proxy = xmlrpc.client.ServerProxy(f"http://{ip}:20003", transport=_TimeoutTransport(timeout))

    def ActGripper(self, index: int, action: int) -> int:  # noqa: N802 -- mirrors the SDK's own name
        return _as_int(self._proxy.ActGripper(int(index), int(action)))

    def MoveGripper(self, *args: float) -> int:  # noqa: N802 -- mirrors the SDK's own name
        """Positional passthrough; the SDK's own signature is the contract.

        (index, pos%, vel%, force%, maxtime ms, block, type, rotNum, rotVel, rotTorque)
        """
        return _as_int(self._proxy.MoveGripper(*args))

    def close(self) -> None:
        """Drop the HTTP connection behind the proxy."""
        with suppress(Exception):
            self._proxy("close")()

    def GetGripperMotionDone(self):  # noqa: N802 -- mirrors the SDK's own name
        """Raw ``[err, fault, status]`` -> the SDK's ``(err, [fault, status])``."""
        raw = self._proxy.GetGripperMotionDone()
        if not isinstance(raw, list | tuple) or not raw:
            return -1, None
        err = _as_int(raw[0])
        if err != 0 or len(raw) < 3:
            return err, None
        return err, [raw[1], raw[2]]


class _GripperWorker:
    """Serialises MoveGripper on its own thread + RPC connection.

    The teleop loop only ever writes the latest target percent; this worker
    coalesces to it and never lets two open/close motions overlap (which hangs the
    controller's XML-RPC reply). Proven against hardware by the standalone
    uArm->FR5 gripper teleop script; the overlap guard is the load-bearing part.
    """

    def __init__(self, rpc, config: FR5FollowerConfig, initial_pct: float) -> None:
        self._rpc = rpc
        self._index = config.gripper_index
        self._vel = config.gripper_vel_pct
        self._force = config.gripper_force_pct
        self._thresh = config.gripper_thresh_pct
        self._timeout = config.gripper_motion_timeout_s
        self._target_pct: float | None = None
        # Seeded with where the follower believes the gripper already is, so a target that
        # never changes never commands. Without this the first cycle always fired one
        # MoveGripper -- and with no leader trigger configured that target comes from
        # FR5FollowerClient's *assumed* opening, not from any operator input, so the
        # gripper was being driven by a guess.
        self._last_cmd_pct: float | None = initial_pct
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="FR5GripperWorker", daemon=True)
        self._thread.start()

    def set_target_pct(self, pct: float) -> None:
        with self._lock:
            self._target_pct = _clamp(pct, 0.0, 100.0)

    def close_rpc(self) -> None:
        """Release the gripper's XML-RPC channel; safe on any rpc object."""
        closer = getattr(self._rpc, "close", None)
        if callable(closer):
            closer()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _motion_done(self) -> bool:
        try:
            md = self._rpc.GetGripperMotionDone()
            return isinstance(md, (list, tuple)) and md[0] == 0 and md[1][1] == 1
        except Exception:
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
                except Exception as e:
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
        # The vendored SDK ships no type information, so the handle is deliberately
        # untyped: annotating it None-until-connect makes every call site a type error.
        self._rpc: Any = None
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
        self._needs_resync = False
        self._reported_fault: tuple[int, int] | None = None

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

    def _wait_for_state(self) -> list[float]:
        """Block until the SDK's UDP state thread has delivered a state packet.

        ``RPC.__init__`` parks the *class* ``RobotStatePkg`` in ``robot_state_pkg`` and a
        background thread swaps in a real instance once the first packet arrives. Reading
        before that raises ``TypeError: '_ctypes.CField' object is not subscriptable``
        from deep inside the vendored SDK, so this polls instead of racing it. Everything
        that reads cached state hits the same window -- ``GetSafetyCode``, which ``ServoJ``
        calls on every command, reads ``robot_state_pkg`` too -- so waiting once here
        covers the whole driver.
        """
        deadline = time.perf_counter() + self._config.state_timeout_s
        last: Exception | None = None
        while time.perf_counter() < deadline:
            try:
                return self._read_joint_deg()
            except (TypeError, RuntimeError, AttributeError) as e:
                last = e
                time.sleep(0.05)
        raise RuntimeError(
            f"FR5 state channel (tcp {self._config.ip}:20004) silent for "
            f"{self._config.state_timeout_s}s. The controller serves one state connection "
            f"at a time, so the usual cause is another client already holding it -- an "
            f"orphaned worker from a previous backend run, or scripts/calibrate_uarm.py. "
            f"Check with: ss -tanp | grep 20004. Last error: {last}"
        )

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
        current = self._wait_for_state()  # validates the link and the state channel
        self._rpc.RobotEnable(1)
        self._rpc.Mode(0)  # auto mode
        self._resolve_ranges()
        self._last_target = current
        self._connected = True
        logger.info(
            "FR5 connected. usable deg ranges: "
            + ", ".join(f"J{j + 1}[{self._deg_lo[j]:.0f},{self._deg_hi[j]:.0f}]" for j in range(NUM_JOINTS))
        )
        self._apply_collision_level()
        if self._config.gripper_enabled:
            self._start_gripper()

    def _build_gripper_rpc(self):
        """Seam: overridden in tests so they never dial the real controller."""
        return _GripperRPC(self._config.ip, self._config.gripper_rpc_timeout_s)

    def _apply_collision_level(self) -> None:
        """Set the collision threshold for this session only (config=0 -- not persisted)."""
        level = self._config.collision_level
        if level is None:
            return
        if len(level) != NUM_JOINTS:
            raise ValueError(f"collision_level needs {NUM_JOINTS} values, got {len(level)}")
        err = self._rpc.SetAnticollision(0, list(level), 0)
        if err != 0:
            logger.warning(f"FR5 SetAnticollision({level}) err={err}")
        else:
            logger.info(f"FR5 collision level set to {level} for this session")

    def _start_gripper(self) -> None:
        # Its own XML-RPC channel, not the arm's: ServerProxy is not thread-safe. See
        # _GripperRPC for why this must not be a second Robot.RPC.
        try:
            gripper_rpc = self._build_gripper_rpc()
            if self._config.gripper_activate:
                err = gripper_rpc.ActGripper(self._config.gripper_index, 1)
                if err != 0:
                    logger.warning(f"FR5 ActGripper({self._config.gripper_index},1) err={err}")
            initial_pct = self._last_gripper_m / GRIPPER_STROKE_M * 100.0
            worker = _GripperWorker(gripper_rpc, self._config, initial_pct=initial_pct)
            worker.start()
            self._gripper = worker
            logger.info(f"FR5 gripper worker started (index={self._config.gripper_index})")
        except Exception as e:
            logger.warning(f"FR5 gripper unavailable, continuing without it: {e}")
            self._gripper = None

    def disconnect(self) -> None:
        logger.info("Disconnecting FR5 follower")
        if self._gripper is not None:
            self._gripper.stop()
            self._gripper.close_rpc()
            self._gripper = None
        try:
            if self._servo_started and self._rpc is not None:
                self._rpc.ServoMoveEnd()
        except Exception as e:
            logger.warning(f"FR5 ServoMoveEnd on disconnect failed: {e}")
        finally:
            self._close_rpc()
            self._servo_started = False
            self._connected = False
            # Torque is intentionally left enabled; use the pendant to disable.

    def _close_rpc(self) -> None:
        """Release the controller's single state connection.

        Without this every ``connect()`` leaks a tcp/20004 socket and its state thread.
        The controller serves exactly one such connection, so the *second* connect in the
        same backend process could never succeed: teleoperation worked, then recording
        failed for the rest of the process's life, and workers left the port held after
        their session ended.

        ``CloseRPC`` sets its stop event and closes the socket, then trips over
        ``self.thread`` -- a name its own ``__init__`` never assigns (it keeps the state
        thread in a local variable). The socket is already closed by that point, so the
        AttributeError is swallowed here rather than patched into the vendored copy,
        which has to stay byte-identical to upstream.
        """
        if self._rpc is None:
            return
        try:
            self._rpc.CloseRPC()
        except AttributeError:
            pass  # vendored CloseRPC's self.thread bug -- the socket is already closed
        except Exception as e:  # never let teardown mask the reason we are disconnecting
            logger.warning(f"FR5 CloseRPC failed: {e}")
        finally:
            self._rpc = None

    # -- state / mapping --------------------------------------------------

    def ping(self) -> dict:
        return self._create_event("pong")

    def _read_fault(self) -> tuple[int, int, bool] | None:
        """(main_code, sub_code, collided) if the controller is faulted, else None.

        Reads the cached state packet, so this costs nothing per cycle.
        """
        pkg: Any = getattr(self._rpc, "robot_state_pkg", None)
        try:
            main, sub = int(pkg.main_code), int(pkg.sub_code)
            collided = int(pkg.collisionState) == 1
        except (AttributeError, TypeError):
            return None  # state channel not up yet -- _wait_for_state covers that
        return (main, sub, collided) if (main != 0 or collided) else None

    def _handle_fault(self, main: int, sub: int, collided: bool) -> None:
        """Clear a controller fault once, then hold off until the leader re-syncs."""
        if self._reported_fault != (main, sub):
            self._reported_fault = (main, sub)
            what = "collision" if collided else "fault"
            logger.warning(f"FR5 {what} (main={main} sub={sub}); teleoperation suspended")
        if not self._config.auto_recover or self._needs_resync:
            return
        try:
            self._rpc.ResetAllError()
            self._rpc.RobotEnable(1)
            # Servo mode does not survive the fault; the next command re-opens it.
            self._servo_started = False
            # Resume from where the arm actually is, not the target it was pushing
            # towards when it hit -- that one points into the obstacle.
            self._last_target = self._read_joint_deg()
            self._needs_resync = True
            logger.info(
                f"FR5 fault cleared. Bring the leader back within "
                f"{self._config.resync_tolerance_deg:.0f} deg of the arm to resume."
            )
        except Exception as e:  # a failed reset must not kill the worker
            logger.warning(f"FR5 could not clear the fault: {e}")

    def _resynced(self, joints: dict) -> bool:
        """True once the leader points close enough to the arm's real pose to resume."""
        actual = self._read_joint_deg()
        worst = max(abs(float(joints[f"{n}.pos"]) - actual[j]) for j, n in enumerate(JOINT_NAMES))
        if worst > self._config.resync_tolerance_deg:
            return False
        logger.info(f"FR5 back in sync ({worst:.1f} deg); resuming teleoperation")
        self._needs_resync = False
        self._reported_fault = None
        self._last_target = actual
        return True

    def set_joints_state(self, joints: dict, goal_time: float) -> dict:
        fault = self._read_fault()
        if fault is not None:
            self._handle_fault(*fault)
            return self._create_event("joints_state_was_not_set", reason="fault", joints=joints)
        if self._needs_resync and not self._resynced(joints):
            return self._create_event("joints_state_was_not_set", reason="awaiting_resync", joints=joints)

        if not self._servo_started:
            self._rpc.ServoMoveStart()
            self._servo_started = True

        max_step = self._config.max_step_deg
        target: list[float] = []
        for j, name in enumerate(JOINT_NAMES):
            desired = _clamp(float(joints[f"{name}.pos"]), self._deg_lo[j], self._deg_hi[j])
            step = _clamp(desired - self._last_target[j], -max_step, max_step)
            target.append(self._last_target[j] + step)

        # goal_time is the worker's period doubled; halve it back. See servo_cmd_t.
        cmd_t = self._config.servo_cmd_t or goal_time / 2.0
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
