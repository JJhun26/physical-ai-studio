# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""uArm leader trigger -> gripper.pos publishing, exercised without hardware.

A fake Feetech reader replaces the serial bus so these tests pin the pure-software
behaviour: gripper.pos is only published when the trigger endpoints are configured,
and the trigger's raw value maps linearly onto [0, GRIPPER_STROKE_M].
"""

from robots.drivers import GRIPPER_FEATURE, GRIPPER_STROKE_M
from robots.drivers.uarm_leader import UArmLeaderClient, UArmLeaderConfig


class _FakeReader:
    """Stand-in for GroupSyncRead: returns a fixed raw per motor id."""

    def __init__(self, raws: dict[int, int]) -> None:
        self._raws = raws

    def txRxPacket(self):
        return 0

    def isAvailable(self, motor_id, addr, size):  # noqa: ARG002
        return [motor_id in self._raws]

    def getData(self, motor_id, addr, size):  # noqa: ARG002
        return self._raws[motor_id]


def _leader_with_reader(raws: dict[int, int], **cfg) -> UArmLeaderClient:
    client = UArmLeaderClient(UArmLeaderConfig(**cfg))
    client._reader = _FakeReader(raws)  # type: ignore[assignment]
    client._addr = 0
    return client


def test_gripper_feature_absent_without_calibration():
    client = UArmLeaderClient(UArmLeaderConfig())
    assert GRIPPER_FEATURE not in client.features()
    assert client._config.gripper_enabled is False


def test_gripper_feature_present_with_calibration():
    client = UArmLeaderClient(UArmLeaderConfig(gripper_raw_open=3000, gripper_raw_close=1000))
    feats = client.features()
    assert feats[-1] == GRIPPER_FEATURE
    assert len(feats) == 7


def test_trigger_maps_open_endpoint_to_full_stroke():
    joint_raws = {i: 2048 for i in range(1, 7)}
    client = _leader_with_reader({**joint_raws, 7: 3000}, gripper_raw_open=3000, gripper_raw_close=1000)
    state = client._read_norm()
    assert state[GRIPPER_FEATURE] == GRIPPER_STROKE_M


def test_trigger_maps_close_endpoint_to_zero():
    joint_raws = {i: 2048 for i in range(1, 7)}
    client = _leader_with_reader({**joint_raws, 7: 1000}, gripper_raw_open=3000, gripper_raw_close=1000)
    state = client._read_norm()
    assert state[GRIPPER_FEATURE] == 0.0


def test_trigger_maps_midpoint_to_half_stroke():
    joint_raws = {i: 2048 for i in range(1, 7)}
    client = _leader_with_reader({**joint_raws, 7: 2000}, gripper_raw_open=3000, gripper_raw_close=1000)
    state = client._read_norm()
    assert abs(state[GRIPPER_FEATURE] - GRIPPER_STROKE_M / 2) < 1e-9


def test_trigger_beyond_endpoints_is_clamped():
    joint_raws = {i: 2048 for i in range(1, 7)}
    client = _leader_with_reader({**joint_raws, 7: 3500}, gripper_raw_open=3000, gripper_raw_close=1000)
    state = client._read_norm()
    assert state[GRIPPER_FEATURE] == GRIPPER_STROKE_M  # clamped, not >stroke
