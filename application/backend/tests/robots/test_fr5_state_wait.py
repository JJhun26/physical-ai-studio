# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""connect() must wait for the Fairino SDK's UDP state channel.

``RPC.__init__`` parks the *class* ``RobotStatePkg`` in ``robot_state_pkg``; a background
thread swaps in an instance when the first state packet lands. Reading in between raises
``TypeError: '_ctypes.CField' object is not subscriptable`` from inside the vendored SDK.
It is a race -- the headless calibration script happened to win it while the studio's
teleoperate worker lost -- so these tests pin the wait rather than the timing.
"""

import ctypes

import pytest

from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig


class _StatePkg(ctypes.Structure):
    """Shaped like the SDK's RobotStatePkg for the field that matters."""

    _fields_ = [("jt_cur_pos", ctypes.c_double * 6)]


class _LateStateRPC:
    """Reproduces the SDK's startup window: the class stands in for the instance.

    ``packets_until_ready`` reads fail exactly the way the real SDK fails, then succeed.
    """

    def __init__(self, ip: str, packets_until_ready: int = 3) -> None:
        self.ip = ip
        self._remaining = packets_until_ready
        self.read_attempts = 0
        # Exactly what Robot.py:255 does -- the class, not an instance.
        self.robot_state_pkg = _StatePkg

    def GetActualJointPosDegree(self):  # noqa: N802 -- mirrors the SDK's own name
        self.read_attempts += 1
        if self._remaining > 0:
            self._remaining -= 1
            if self._remaining == 0:  # the state thread's first packet lands
                self.robot_state_pkg = _StatePkg((ctypes.c_double * 6)(1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        # Indexing a CField raises TypeError, same as the real SDK.
        return [0, [self.robot_state_pkg.jt_cur_pos[i] for i in range(6)]]

    def RobotEnable(self, _):  # noqa: N802 -- mirrors the SDK's own name
        return 0

    def Mode(self, _):  # noqa: N802 -- mirrors the SDK's own name
        return 0

    def GetJointSoftLimitDeg(self, _):  # noqa: N802 -- mirrors the SDK's own name
        return [0, [-175.0] * 6 + [175.0] * 6]


class _NeverReadyRPC(_LateStateRPC):
    def __init__(self, ip: str) -> None:
        super().__init__(ip, packets_until_ready=10**9)


def _client(rpc_cls, **cfg) -> FR5FollowerClient:
    client = FR5FollowerClient(FR5FollowerConfig(gripper_enabled=False, **cfg))
    module = type("Module", (), {"RPC": rpc_cls})
    client._import_sdk = lambda: module  # type: ignore[method-assign]
    return client


def test_a_bare_read_still_fails_the_way_the_sdk_does():
    """Guard the guard: if this stops raising, the tests below prove nothing."""
    client = _client(_LateStateRPC)
    client._rpc = _LateStateRPC("1.2.3.4")
    with pytest.raises(TypeError, match="not subscriptable"):
        client._read_joint_deg()


def test_connect_waits_out_the_startup_window():
    client = _client(_LateStateRPC)
    client.connect()
    assert client.is_connected
    assert client._last_target == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert client._rpc.read_attempts > 1  # it really did retry


def test_wait_returns_the_first_good_reading():
    client = _client(_LateStateRPC)
    client._rpc = _LateStateRPC("1.2.3.4")
    assert client._wait_for_state() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_a_silent_state_channel_names_the_likely_cause():
    """The message has to carry the diagnosis: a held port looks identical to a dead arm."""
    client = _client(_NeverReadyRPC, state_timeout_s=0.2, ip="10.0.0.9")
    with pytest.raises(RuntimeError) as excinfo:
        client.connect()
    message = str(excinfo.value)
    assert "10.0.0.9:20004" in message
    assert "one state connection at a time" in message
    assert "ss -tanp" in message


def test_a_silent_state_channel_does_not_leave_the_client_connected():
    client = _client(_NeverReadyRPC, state_timeout_s=0.2)
    with pytest.raises(RuntimeError):
        client.connect()
    assert not client.is_connected
