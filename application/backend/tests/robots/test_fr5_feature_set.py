# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""The gripper joins the feature set only when something can drive it.

Recording builds the dataset schema from the *follower's* features but fills actions from
the *leader's* state, and the two are not reconciled (``environment_integration`` returns
the leader's dict verbatim). So an FR5 that advertises ``gripper.pos`` while the uArm's
trigger is unconfigured produces a schema of 7 and actions of 6 -- ``KeyError:
'gripper.pos'`` on the first recorded frame. Beyond the crash, the column it wanted to
write was the follower's *assumed* opening: a constant, which is worse than absent.
"""

import pytest

from robots.drivers import GRIPPER_FEATURE
from robots.robot_client_factory import RobotClientFactory
from schemas.robot import FR5FollowerPayload, FR5Robot, RobotType

_UUID = "11111111-2222-3333-4444-555555555555"


def _fr5() -> FR5Robot:
    return FR5Robot(
        id=_UUID,
        name="fr5",
        type=RobotType.FR5_FOLLOWER,
        payload=FR5FollowerPayload(connection_string="192.168.58.2", serial_number=""),
    )


@pytest.fixture
def _no_trigger(monkeypatch):
    monkeypatch.delenv("UARM_TRIGGER_RAW_OPEN", raising=False)
    monkeypatch.delenv("UARM_TRIGGER_RAW_CLOSE", raising=False)


@pytest.fixture
def _with_trigger(monkeypatch):
    monkeypatch.setenv("UARM_TRIGGER_RAW_OPEN", "3100")
    monkeypatch.setenv("UARM_TRIGGER_RAW_CLOSE", "1800")


def test_without_a_trigger_the_gripper_is_not_a_feature(_no_trigger):
    client = RobotClientFactory._build_fr5(_fr5())
    feats = client.features()
    assert GRIPPER_FEATURE not in feats
    assert len(feats) == 6, "a 7th feature here means recording writes a constant guess"


def test_with_a_trigger_the_gripper_is_a_feature(_with_trigger):
    client = RobotClientFactory._build_fr5(_fr5())
    feats = client.features()
    assert feats[-1] == GRIPPER_FEATURE
    assert len(feats) == 7


def test_half_a_trigger_configuration_is_not_enough(monkeypatch):
    """One endpoint alone leaves the leader unable to publish gripper.pos."""
    monkeypatch.setenv("UARM_TRIGGER_RAW_OPEN", "3100")
    monkeypatch.delenv("UARM_TRIGGER_RAW_CLOSE", raising=False)
    assert GRIPPER_FEATURE not in RobotClientFactory._build_fr5(_fr5()).features()


def test_the_leader_and_follower_agree_on_the_gripper(_no_trigger):
    """Both sides key off the same configuration, so their feature sets cannot diverge."""
    from robots.drivers.uarm_leader import UArmLeaderClient, UArmLeaderConfig

    follower = RobotClientFactory._build_fr5(_fr5())
    leader = UArmLeaderClient(UArmLeaderConfig())  # no trigger endpoints
    assert (GRIPPER_FEATURE in follower.features()) == (GRIPPER_FEATURE in leader.features())
