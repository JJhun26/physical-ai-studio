# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Tests for IPDiscovery.ping timeout handling.

An unreachable controller is the normal case (arm powered off, cable out), so ping
must fail fast. It previously passed milliseconds to iputils' ``-W``, which counts
seconds — a 1s budget became a ~17 minute hang that stalled /robots/online for every
robot in the project, because the endpoint gathers them together.
"""

import asyncio
import sys
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from robots.discovery.ip import IPDiscovery


def test_linux_ping_timeout_is_seconds() -> None:
    """iputils' -W counts seconds; passing 1000 would mean ~17 minutes, not 1s."""
    with patch.object(sys, "platform", "linux"):
        command = IPDiscovery._ping_command("192.168.58.2", 1.0)

    assert command == ["ping", "-c", "1", "-W", "1", "192.168.58.2"]


def test_macos_ping_timeout_is_milliseconds() -> None:
    """BSD ping's -W counts milliseconds, unlike Linux."""
    with patch.object(sys, "platform", "darwin"):
        command = IPDiscovery._ping_command("192.168.58.2", 1.0)

    assert command == ["ping", "-c", "1", "-W", "1000", "192.168.58.2"]


def test_windows_ping_uses_lowercase_w_milliseconds() -> None:
    """Windows takes -n for count and -w (not -W) for a millisecond timeout."""
    with patch.object(sys, "platform", "win32"):
        command = IPDiscovery._ping_command("192.168.58.2", 1.0)

    assert command == ["ping", "-n", "1", "-w", "1000", "192.168.58.2"]


def test_sub_second_timeout_stays_pingable_on_linux() -> None:
    """Linux has integer-second granularity; a 0.2s budget must not floor to -W 0.

    ``-W 0`` is rejected by iputils, which would report every host as unreachable.
    """
    with patch.object(sys, "platform", "linux"):
        command = IPDiscovery._ping_command("192.168.58.2", 0.2)

    assert command[4] == "1"


@pytest.mark.parametrize(("exit_code", "expected"), [(0, True), (1, False), (2, False)])
@pytest.mark.anyio
async def test_ping_maps_exit_code_to_reachability(exit_code: int, expected: bool) -> None:
    """Only exit code 0 means reachable.

    ping exits 1 for "no reply" and 2 for other errors. Returning the raw code would
    make every *unreachable* host report as online, since 1 and 2 are truthy.
    """
    proc = MagicMock()
    proc.wait = AsyncMock(return_value=exit_code)

    with patch("asyncio.create_subprocess_exec", AsyncMock(return_value=proc)):
        result = await IPDiscovery.ping("192.168.58.2", ping_timeout=0.1)

    assert result is expected


@pytest.mark.anyio
async def test_ping_kills_a_hanging_process_and_reports_unreachable() -> None:
    """A ping that outlives its budget is killed rather than left holding the request."""
    killed = asyncio.Event()

    # Mirrors a real hung ping: wait() only returns once the process is killed.
    async def wait() -> int:
        await killed.wait()
        return -9

    proc = MagicMock()
    proc.wait = wait
    proc.kill = MagicMock(side_effect=killed.set)

    with patch("asyncio.create_subprocess_exec", AsyncMock(return_value=proc)):
        result = await asyncio.wait_for(IPDiscovery.ping("192.168.58.2", ping_timeout=0.1), timeout=5)

    assert result is False
    proc.kill.assert_called_once()
