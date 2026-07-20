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
from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig


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


def _connect_client(**cfg_kwargs) -> tuple[FR5FollowerClient, _FakeRPC]:
    client = FR5FollowerClient(FR5FollowerConfig(**cfg_kwargs))
    fake_module = _FakeRobotModule()
    client._import_sdk = lambda: fake_module  # type: ignore[method-assign]
    client.connect()
    # the gripper worker holds its own RPC instance; grab it for assertions
    gripper_rpc = client._gripper._rpc  # type: ignore[union-attr]
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
    client, rpc = _connect_client(gripper_enabled=True, gripper_thresh_pct=1.0)
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
    client, rpc = _connect_client(gripper_enabled=True, gripper_thresh_pct=1.0)
    try:
        joints = {f"j{i}.pos": 0.0 for i in range(1, 7)}
        joints[GRIPPER_FEATURE] = GRIPPER_STROKE_M / 2
        client.set_joints_state(joints, goal_time=0.02)
        assert _wait_until(lambda: len(rpc.move_calls) >= 1)
        assert rpc.move_calls[0][1] == 50
    finally:
        client.disconnect()


def test_threshold_suppresses_tiny_changes():
    client, rpc = _connect_client(gripper_enabled=True, gripper_thresh_pct=5.0)
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
    client, rpc = _connect_client(gripper_enabled=True, gripper_thresh_pct=1.0)
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
