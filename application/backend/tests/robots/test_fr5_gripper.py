# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""FR5 follower gripper actuation, exercised without hardware via a fake RPC.

No FR5 controller is present in CI, so a stand-in ``_FakeRPC`` records the SDK calls
the driver makes. These tests pin the parts that are pure software: the gripper.pos
feature plumbing, the metres->percent conversion handed to ``MoveGripper``, the
overlap guard that waits on ``GetGripperMotionDone``, and clean worker shutdown. The
XML-RPC transport and physical motion themselves still require the real controller.
"""

import threading
import time

import pytest

from robots.drivers import GRIPPER_FEATURE, GRIPPER_STROKE_M
from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig, _GripperRPC


class _FakeRPC:
    """Minimal stand-in for the vendored Fairino ``Robot.RPC``."""

    def __init__(self, ip: str) -> None:
        self.ip = ip
        self.move_calls: list[tuple] = []
        self.act_calls: list[tuple] = []
        self._lock = threading.Lock()
        # a fresh command reports "in motion" once, then "done"
        self._done_after = 0

    # -- arm (enough for connect() to succeed) --
    def GetActualJointPosDegree(self):
        return [0, [0.0] * 6]

    def RobotEnable(self, _):
        return 0

    def Mode(self, _):
        return 0

    def GetJointSoftLimitDeg(self, _):
        return [0, [-175.0] * 6 + [175.0] * 6]

    def ServoMoveStart(self):
        return 0

    def ServoJ(self, joint_pos, axisPos, cmdT):  # noqa: N803
        return 0

    def ServoMoveEnd(self):
        return 0

    # -- gripper --
    def ActGripper(self, index, action):
        self.act_calls.append((index, action))
        return 0

    def MoveGripper(self, index, pos, vel, force, maxtime, block, gtype, rotNum, rotVel, rotTorque):  # noqa: N803
        with self._lock:
            self.move_calls.append((index, pos, vel, force, block, gtype))
            self._done_after = 2  # report not-done twice before done
        return 0

    def GetGripperMotionDone(self):
        with self._lock:
            if self._done_after > 0:
                self._done_after -= 1
                return [0, [0, 0]]  # fault=0, status=0 (moving)
            return [0, [0, 1]]  # fault=0, status=1 (done)


class _FakeRobotModule:
    RPC = _FakeRPC


def _connect_client(assumed_open_m: float | None = None, **cfg_kwargs) -> tuple[FR5FollowerClient, _FakeRPC]:
    """Connect against fakes.

    ``assumed_open_m`` overrides where the driver believes the gripper starts. The worker
    seeds its last-commanded value from it, so a command equal to it is deliberately a
    no-op; tests that exercise commanding must start somewhere else.
    """
    client = FR5FollowerClient(FR5FollowerConfig(**cfg_kwargs))
    if assumed_open_m is not None:
        client._last_gripper_m = assumed_open_m
    fake_module = _FakeRobotModule()
    client._import_sdk = lambda: fake_module  # type: ignore[method-assign]
    # The gripper opens its own XML-RPC channel to the controller; stub the seam so the
    # suite never dials real hardware (this machine can actually reach the FR5).
    gripper_rpc = _FakeRPC("fake")
    client._build_gripper_rpc = lambda: gripper_rpc  # type: ignore[method-assign]
    client.connect()
    return client, gripper_rpc


def _wait_until(pred, timeout=2.0) -> bool:
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_gripper_feature_is_appended_after_arm_joints():
    client = FR5FollowerClient(FR5FollowerConfig(gripper_enabled=True))
    feats = client.features()
    assert feats[:6] == [f"j{i}.pos" for i in range(1, 7)]
    assert feats[-1] == GRIPPER_FEATURE
    assert len(feats) == 7


def test_gripper_feature_absent_when_disabled():
    client = FR5FollowerClient(FR5FollowerConfig(gripper_enabled=False))
    assert GRIPPER_FEATURE not in client.features()


def test_connect_activates_gripper():
    client, rpc = _connect_client(gripper_enabled=True, gripper_index=1)
    try:
        assert rpc.act_calls == [(1, 1)]
    finally:
        client.disconnect()


def test_open_command_maps_metres_to_full_percent():
    client, rpc = _connect_client(assumed_open_m=0.0, gripper_enabled=True, gripper_thresh_pct=1.0)
    try:
        joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
        joints[GRIPPER_FEATURE] = GRIPPER_STROKE_M  # fully open
        client.set_joints_state(joints, goal_time=0.02)
        assert _wait_until(lambda: len(rpc.move_calls) >= 1)
        # (index, pos%, vel%, force%, block, type)
        index, pos, vel, force, block, gtype = rpc.move_calls[0]
        assert pos == 100  # GRIPPER_STROKE_M -> 100%
        assert gtype == 0  # parallel gripper
        assert block == 1  # non-blocking
    finally:
        client.disconnect()


def test_closed_command_maps_to_zero_percent():
    client, rpc = _connect_client(gripper_enabled=True, gripper_thresh_pct=1.0)
    try:
        joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
        joints[GRIPPER_FEATURE] = 0.0  # closed
        client.set_joints_state(joints, goal_time=0.02)
        assert _wait_until(lambda: len(rpc.move_calls) >= 1)
        assert rpc.move_calls[0][1] == 0  # 0%
    finally:
        client.disconnect()


def test_halfway_command_maps_to_fifty_percent():
    client, rpc = _connect_client(assumed_open_m=0.0, gripper_enabled=True, gripper_thresh_pct=1.0)
    try:
        joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
        joints[GRIPPER_FEATURE] = GRIPPER_STROKE_M / 2
        client.set_joints_state(joints, goal_time=0.02)
        assert _wait_until(lambda: len(rpc.move_calls) >= 1)
        assert rpc.move_calls[0][1] == 50
    finally:
        client.disconnect()


def test_threshold_suppresses_tiny_changes():
    client, rpc = _connect_client(assumed_open_m=0.0, gripper_enabled=True, gripper_thresh_pct=5.0)
    try:
        joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
        joints[GRIPPER_FEATURE] = GRIPPER_STROKE_M  # 100%
        client.set_joints_state(joints, goal_time=0.02)
        assert _wait_until(lambda: len(rpc.move_calls) == 1)
        # a sub-threshold nudge (< 5%) must not produce a second MoveGripper
        joints[GRIPPER_FEATURE] = GRIPPER_STROKE_M * 0.98  # 98%, delta 2% < 5%
        client.set_joints_state(joints, goal_time=0.02)
        time.sleep(0.3)
        assert len(rpc.move_calls) == 1
    finally:
        client.disconnect()


def test_read_state_echoes_last_gripper_command():
    client, _rpc = _connect_client(gripper_enabled=True)
    try:
        joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
        joints[GRIPPER_FEATURE] = GRIPPER_STROKE_M / 2
        client.set_joints_state(joints, goal_time=0.02)
        state = client.read_state()["state"]
        assert GRIPPER_FEATURE in state
        assert state[GRIPPER_FEATURE] == pytest.approx(GRIPPER_STROKE_M / 2)
    finally:
        client.disconnect()


def test_out_of_range_gripper_is_clamped():
    client, rpc = _connect_client(assumed_open_m=0.0, gripper_enabled=True, gripper_thresh_pct=1.0)
    try:
        joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
        joints[GRIPPER_FEATURE] = 1.0  # far beyond GRIPPER_STROKE_M
        client.set_joints_state(joints, goal_time=0.02)
        assert _wait_until(lambda: len(rpc.move_calls) >= 1)
        assert rpc.move_calls[0][1] == 100  # clamped to 100%
    finally:
        client.disconnect()


def test_disconnect_stops_worker_thread():
    client, _rpc = _connect_client(gripper_enabled=True)
    worker = client._gripper
    assert worker is not None
    client.disconnect()
    assert client._gripper is None
    assert worker._thread is None or not worker._thread.is_alive()


# --- the gripper must not open a second state channel ---------------------------------


def test_the_gripper_does_not_build_a_second_sdk_rpc():
    """Regression: a second Robot.RPC hung connect() for good.

    Its constructor opens another tcp/20004 state connection, which the controller
    refuses (it serves one). The SDK's state thread then spins in reconnect() and every
    gripper call parks in ``while self.reconnect_flag: time.sleep(0.1)`` -- so connect()
    never returned, setup() never finished, and the worker published no observations.
    """
    client = FR5FollowerClient(FR5FollowerConfig(gripper_enabled=True))
    built: list[str] = []

    class _CountingModule:
        @staticmethod
        def RPC(ip):  # noqa: N802 -- mirrors the SDK's own name
            built.append(ip)
            return _FakeRPC(ip)

    client._import_sdk = lambda: _CountingModule()  # type: ignore[method-assign]
    client._build_gripper_rpc = lambda: _FakeRPC("gripper")  # type: ignore[method-assign]
    client.connect()

    assert built == ["192.168.58.2"], f"expected exactly one SDK RPC, got {built}"
    client.disconnect()


def test_connect_survives_a_gripper_that_cannot_be_reached():
    """A dead gripper must degrade to no gripper, never stall the arm."""
    client = FR5FollowerClient(FR5FollowerConfig(gripper_enabled=True))
    client._import_sdk = lambda: _FakeRobotModule()  # type: ignore[method-assign]

    def _boom():
        raise OSError("connection refused")

    client._build_gripper_rpc = _boom  # type: ignore[method-assign]
    client.connect()

    assert client.is_connected
    assert client._gripper is None
    client.disconnect()


# --- raw XML-RPC shapes are normalised to what the SDK returned -----------------------


class _RawProxy:
    """Bare ServerProxy stand-in: returns the controller's raw list, not the SDK tuple."""

    def __init__(self, motion_done) -> None:
        self._motion_done = motion_done

    def GetGripperMotionDone(self):  # noqa: N802 -- mirrors the SDK's own name
        return self._motion_done

    def ActGripper(self, index, action):  # noqa: N802 -- mirrors the SDK's own name
        return 0

    def MoveGripper(self, *args):  # noqa: N802 -- mirrors the SDK's own name
        return 0


def _rpc_with(motion_done):
    rpc = _GripperRPC.__new__(_GripperRPC)
    rpc._proxy = _RawProxy(motion_done)
    return rpc


def test_motion_done_is_reshaped_like_the_sdk():
    # raw [err, fault, status] -> (err, [fault, status]), which _GripperWorker reads.
    assert _rpc_with([0, 0, 1]).GetGripperMotionDone() == (0, [0, 1])
    assert _rpc_with([0, 0, 0]).GetGripperMotionDone() == (0, [0, 0])


def test_a_motion_done_error_reports_no_payload():
    assert _rpc_with([7, 0, 1]).GetGripperMotionDone() == (7, None)


def test_a_malformed_motion_done_does_not_raise():
    assert _rpc_with([]).GetGripperMotionDone() == (-1, None)
    assert _rpc_with([0, 1]).GetGripperMotionDone() == (0, None)


# --- ServoJ pacing --------------------------------------------------------------------


def _servoj_cmd_t(goal_time: float, **cfg) -> float:
    """cmdT actually handed to ServoJ for one set_joints_state call."""
    client, _ = _connect_client(gripper_enabled=False, **cfg)
    rpc = client._rpc
    calls: list[float] = []
    rpc.ServoJ = lambda joint_pos, axisPos, cmdT: calls.append(cmdT) or 0  # type: ignore[assignment] # noqa: N803
    client.set_joints_state({f"j{i}.pos": 0.0 for i in range(1, 7)}, goal_time)
    client.disconnect()
    return calls[0]


def test_cmd_t_does_not_exceed_the_send_interval():
    """Regression: cmdT of 2x the period backed up the controller and halved the rate.

    The worker passes goal_time = 2 / frequency, so at 30 Hz points arrive every 33.3 ms.
    Consuming each over 66.7 ms grew the lookahead buffer every cycle until ServoJ blocked,
    pacing teleoperation at 15 Hz with lag the operator could feel.
    """
    period = 1 / 30
    cmd_t = _servoj_cmd_t(period * 2)
    assert cmd_t == pytest.approx(period, rel=1e-6)
    assert cmd_t <= period, "cmdT above the send interval makes the controller fall behind"


def test_cmd_t_tracks_the_configured_rate():
    for fps in (10.0, 30.0, 60.0):
        period = 1 / fps
        assert _servoj_cmd_t(period * 2) == pytest.approx(period, rel=1e-6)


def test_an_explicit_cmd_t_overrides_the_derived_one():
    assert _servoj_cmd_t(1 / 30 * 2, servo_cmd_t=0.004) == pytest.approx(0.004)


# --- the gripper is only driven by real input -----------------------------------------


def test_an_unchanged_target_never_commands_the_gripper():
    """Regression: one spurious MoveGripper fired on the first teleoperation cycle.

    With no leader trigger configured the leader publishes no gripper.pos, so the worker
    falls back to the follower's own value -- which is FR5FollowerClient's *assumed*
    opening, not operator input. Seeding _last_cmd_pct means that guess never reaches the
    hardware; only a target that actually moves does.
    """
    client, gripper_rpc = _connect_client()
    joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
    # Exactly what read_state() reports, echoed straight back -- no operator input.
    joints[GRIPPER_FEATURE] = client._last_gripper_m
    for _ in range(5):
        client.set_joints_state(joints, 0.03)
    assert _wait_until(lambda: False, timeout=0.3) is False  # let the worker run
    assert gripper_rpc.move_calls == []
    client.disconnect()


def test_a_real_change_still_commands_the_gripper():
    client, gripper_rpc = _connect_client()
    joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
    joints[GRIPPER_FEATURE] = 0.0  # operator closes the trigger
    client.set_joints_state(joints, 0.03)
    assert _wait_until(lambda: len(gripper_rpc.move_calls) > 0)
    assert gripper_rpc.move_calls[0][1] == 0  # 0 %
    client.disconnect()
