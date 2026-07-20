# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Collision recovery: clear the fault, then wait for the operator to re-sync.

Light contact during teleoperation trips the controller, which stops the arm and leaves it
stopped. Clearing the fault is easy; resuming safely is the hard part, because the leader
is still pointing at whatever was hit. So streaming stays suspended until the leader comes
back to where the arm actually is -- that interlock is what these tests pin.
"""

import pytest

from robots.drivers import JOINT_NAMES
from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig


class _FaultingRPC:
    """Controller fake whose fault state and pose the test drives directly."""

    def __init__(self, ip: str) -> None:
        self.ip = ip
        self.pose = [0.0] * 6
        self.servo_calls: list[list[float]] = []
        self.resets = 0
        self.enables = 0
        self.anticollision: list = []
        rpc = self

        class _Pkg:
            main_code = 0
            sub_code = 0
            collisionState = 0  # noqa: N815 -- mirrors the SDK's own field name

        self.robot_state_pkg = _Pkg()
        self._rpc = rpc

    def fault(self, main=14, sub=1, collided=True) -> None:
        self.robot_state_pkg.main_code = main
        self.robot_state_pkg.sub_code = sub
        self.robot_state_pkg.collisionState = 1 if collided else 0

    def clear(self) -> None:
        self.robot_state_pkg.main_code = 0
        self.robot_state_pkg.sub_code = 0
        self.robot_state_pkg.collisionState = 0

    def GetActualJointPosDegree(self, flag=1):  # noqa: N802 -- SDK's own name
        return [0, list(self.pose)]

    def GetJointSoftLimitDeg(self, flag):  # noqa: N802 -- SDK's own name
        return [0, [-175.0] * 6 + [175.0] * 6]

    def RobotEnable(self, _):  # noqa: N802 -- SDK's own name
        self.enables += 1
        return 0

    def Mode(self, _):  # noqa: N802 -- SDK's own name
        return 0

    def ResetAllError(self):  # noqa: N802 -- SDK's own name
        self.resets += 1
        self.clear()
        return 0

    def SetAnticollision(self, mode, level, config):  # noqa: N802 -- SDK's own name
        self.anticollision.append((mode, list(level), config))
        return 0

    def ServoMoveStart(self):  # noqa: N802 -- SDK's own name
        return 0

    def ServoMoveEnd(self):  # noqa: N802 -- SDK's own name
        return 0

    def ServoJ(self, joint_pos, axisPos, cmdT):  # noqa: N802, N803 -- SDK's own names
        self.servo_calls.append(list(joint_pos))
        return 0


def _client(**cfg) -> tuple[FR5FollowerClient, _FaultingRPC]:
    client = FR5FollowerClient(FR5FollowerConfig(gripper_enabled=False, **cfg))
    rpc = _FaultingRPC("1.2.3.4")
    client._import_sdk = lambda: type("M", (), {"RPC": lambda _ip: rpc})  # type: ignore[method-assign]
    client.connect()
    return client, rpc


def _cmd(deg: float) -> dict:
    return {f"{n}.pos": deg for n in JOINT_NAMES}


def test_a_fault_suspends_commanding():
    client, rpc = _client()
    rpc.fault()
    before = len(rpc.servo_calls)
    client.set_joints_state(_cmd(50.0), 0.066)
    assert len(rpc.servo_calls) == before, "commanded the arm while the controller was faulted"


def test_a_fault_is_cleared_once():
    client, rpc = _client()
    rpc.fault()
    client.set_joints_state(_cmd(50.0), 0.066)
    assert rpc.resets == 1
    assert rpc.enables >= 1
    # A second cycle must not reset again -- it is already waiting to re-sync.
    client.set_joints_state(_cmd(50.0), 0.066)
    assert rpc.resets == 1


def test_a_far_leader_does_not_resume():
    """The leader is still pointing into the obstacle: staying stopped is the point."""
    client, rpc = _client()
    rpc.pose = [10.0] * 6
    rpc.fault()
    client.set_joints_state(_cmd(80.0), 0.066)  # fault cycle clears and arms the interlock
    before = len(rpc.servo_calls)
    for _ in range(5):
        client.set_joints_state(_cmd(80.0), 0.066)
    assert len(rpc.servo_calls) == before
    assert client._needs_resync


def test_a_close_leader_resumes():
    client, rpc = _client()
    rpc.pose = [10.0] * 6
    rpc.fault()
    client.set_joints_state(_cmd(80.0), 0.066)
    before = len(rpc.servo_calls)
    client.set_joints_state(_cmd(12.0), 0.066)  # operator brings the leader back
    assert not client._needs_resync
    assert len(rpc.servo_calls) > before


def test_recovery_resumes_from_the_real_pose_not_the_old_target():
    """The pre-fault target points into the obstacle; resuming from it would push again."""
    client, rpc = _client()
    rpc.pose = [0.0] * 6
    for _ in range(20):  # slew towards 40 deg, building up _last_target
        client.set_joints_state(_cmd(40.0), 0.066)
    assert client._last_target[0] > 5.0
    rpc.pose = [3.0] * 6  # the arm actually stalled here, far short of the target
    rpc.fault()
    client.set_joints_state(_cmd(40.0), 0.066)
    assert client._last_target == pytest.approx([3.0] * 6)


def test_auto_recover_off_leaves_the_fault_alone():
    client, rpc = _client(auto_recover=False)
    rpc.fault()
    client.set_joints_state(_cmd(50.0), 0.066)
    assert rpc.resets == 0
    assert not client._needs_resync


def test_the_collision_level_is_applied_without_persisting_it():
    _, rpc = _client(collision_level=[7.0] * 6)
    # config=0: this session only, so a tuning experiment cannot brick the robot's setup.
    assert rpc.anticollision == [(0, [7.0] * 6, 0)]


def test_no_collision_level_leaves_the_controller_alone():
    _, rpc = _client()
    assert rpc.anticollision == []


def test_a_wrong_length_collision_level_is_rejected():
    with pytest.raises(ValueError, match="collision_level needs 6"):
        _client(collision_level=[7.0, 7.0])
