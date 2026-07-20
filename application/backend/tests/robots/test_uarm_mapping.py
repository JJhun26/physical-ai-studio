# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Range-of-motion sweep -> uArm calibration, exercised without hardware.

The load-bearing test here is ``test_norm_at_matches_the_driver``: the mapping maths is
only correct insofar as it predicts what ``UArmLeaderClient`` actually publishes, so that
formula is pinned against the real driver rather than restated.
"""

import pytest

from robots.calibration.uarm_mapping import (
    DEFAULT_OUT_LIMIT_DEG,
    DEG_PER_TURN,
    TICKS_PER_TURN,
    UNIT_SCALE_SPAN,
    JointMapping,
    JointRom,
    check_mapping,
    homing_offset_for,
    mapping_from_fit,
    mapping_from_scale,
    norm_at,
    reachable_deg,
    unwrap,
)
from robots.drivers.uarm_leader import JointCalibration, UArmLeaderClient, UArmLeaderConfig


class _FakeReader:
    """Stand-in for GroupSyncRead: returns a fixed raw per motor id."""

    def __init__(self, raws: dict[int, int]) -> None:
        self._raws = raws

    def txRxPacket(self):  # noqa: N802 -- mirrors the SDK's own name
        return 0

    def isAvailable(self, motor_id, addr, size):  # noqa: N802 -- mirrors the SDK's own name
        return [motor_id in self._raws]

    def getData(self, motor_id, addr, size):  # noqa: N802 -- mirrors the SDK's own name
        return self._raws[motor_id]


def _driver_norm(raw: int, mapping: JointMapping) -> float:
    """What UArmLeaderClient actually publishes for j1 given this calibration."""
    joints = [
        JointCalibration(
            motor_id=mapping.motor_id,
            range_min=mapping.range_min,
            range_max=mapping.range_max,
            drive_mode=mapping.drive_mode,
            homing_offset=mapping.homing_offset,
        )
    ] + [JointCalibration(motor_id=i) for i in range(2, 7)]
    client = UArmLeaderClient(UArmLeaderConfig(joints=joints))
    client._reader = _FakeReader(dict.fromkeys(range(1, 7), raw))  # type: ignore[assignment]
    client._addr = 0
    return client._read_norm()["j1.pos"]


def _rom(raw_min=1000, raw_max=3000, raw_anchor=2000, motor_id=1) -> JointRom:
    return JointRom(motor_id=motor_id, raw_min=raw_min, raw_max=raw_max, raw_anchor=raw_anchor)


# --- the coupling to the driver -------------------------------------------------------


@pytest.mark.parametrize("drive_mode", [0, 1])
@pytest.mark.parametrize("raw", [0, 500, 1500, 2048, 3000, 4095])
def test_norm_at_matches_the_driver(raw, drive_mode):
    mapping = JointMapping(motor_id=1, homing_offset=137, range_min=900, range_max=3100, drive_mode=drive_mode)
    assert norm_at(raw, mapping) == pytest.approx(_driver_norm(raw, mapping), abs=1e-9)


def test_the_driver_publishes_past_a_hundred_degrees():
    """Regression for the +-100 clamp that made the FR5's real workspace unreachable.

    J2 habitually works near -150 deg and its soft limits reach -263; the leader must be
    able to say so rather than saturating at -100.
    """
    rom = _rom(raw_min=1000, raw_max=3000, raw_anchor=2000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-263.0, deg_hi=83.0)
    assert _driver_norm(1000, mapping) == pytest.approx(-263.0, abs=0.5)
    assert _driver_norm(2000, mapping) == pytest.approx(-90.0, abs=0.5)


def test_the_driver_still_applies_the_output_backstop():
    rom = _rom(raw_min=1000, raw_max=3000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-900.0, deg_hi=900.0)
    assert _driver_norm(1000, mapping) == pytest.approx(-DEFAULT_OUT_LIMIT_DEG, abs=0.5)


def test_scale_mapping_round_trips_through_the_driver():
    rom = _rom()
    mapping = mapping_from_scale(rom, scale=1.0, invert=False, follower_anchor_deg=30.0)
    assert _driver_norm(rom.raw_anchor, mapping) == pytest.approx(30.0, abs=0.1)


# --- scale mode -----------------------------------------------------------------------


def test_anchor_maps_to_the_follower_pose():
    rom = _rom(raw_anchor=2000)
    mapping = mapping_from_scale(rom, scale=1.0, invert=False, follower_anchor_deg=-42.0)
    assert norm_at(2000, mapping) == pytest.approx(-42.0, abs=0.1)


def test_unity_scale_gives_one_follower_degree_per_leader_degree():
    rom = _rom()
    mapping = mapping_from_scale(rom, scale=1.0, invert=False, follower_anchor_deg=0.0)
    ticks_per_deg = TICKS_PER_TURN / DEG_PER_TURN
    moved = norm_at(round(2000 + 10 * ticks_per_deg), mapping) - norm_at(2000, mapping)
    assert moved == pytest.approx(10.0, abs=0.1)


def test_scale_two_doubles_follower_motion():
    rom = _rom()
    mapping = mapping_from_scale(rom, scale=2.0, invert=False, follower_anchor_deg=0.0)
    ticks_per_deg = TICKS_PER_TURN / DEG_PER_TURN
    moved = norm_at(round(2000 + 10 * ticks_per_deg), mapping) - norm_at(2000, mapping)
    assert moved == pytest.approx(20.0, abs=0.1)


def test_scale_sets_the_window_width():
    mapping = mapping_from_scale(_rom(), scale=2.0, invert=False, follower_anchor_deg=0.0)
    assert mapping.span == pytest.approx(UNIT_SCALE_SPAN / 2.0, abs=1.0)


def test_invert_reverses_direction_but_keeps_the_anchor():
    rom = _rom()
    mapping = mapping_from_scale(rom, scale=1.0, invert=True, follower_anchor_deg=15.0)
    assert mapping.drive_mode == 1
    assert norm_at(2000, mapping) == pytest.approx(15.0, abs=0.1)
    assert norm_at(2200, mapping) < norm_at(2000, mapping)


def test_non_positive_scale_is_rejected():
    with pytest.raises(ValueError, match="scale must be positive"):
        mapping_from_scale(_rom(), scale=0.0, invert=False, follower_anchor_deg=0.0)


# --- fit mode -------------------------------------------------------------------------


def test_fit_stretches_rom_across_the_follower_window():
    rom = _rom(raw_min=1000, raw_max=3000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-80.0, deg_hi=40.0)
    assert norm_at(1000, mapping) == pytest.approx(-80.0, abs=0.2)
    assert norm_at(3000, mapping) == pytest.approx(40.0, abs=0.2)


def test_fit_with_invert_swaps_the_endpoints():
    rom = _rom(raw_min=1000, raw_max=3000)
    mapping = mapping_from_fit(rom, invert=True, deg_lo=-80.0, deg_hi=40.0)
    assert norm_at(1000, mapping) == pytest.approx(40.0, abs=0.2)
    assert norm_at(3000, mapping) == pytest.approx(-80.0, abs=0.2)


def test_fit_reaches_past_a_hundred_degrees():
    """The whole point of (b): J2/J4-style travel must be reachable."""
    rom = _rom(raw_min=1000, raw_max=3000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-263.0, deg_hi=83.0)
    assert norm_at(1000, mapping) == pytest.approx(-263.0, abs=0.5)
    assert norm_at(3000, mapping) == pytest.approx(83.0, abs=0.5)


def test_fit_clips_only_at_the_output_backstop():
    rom = _rom(raw_min=1000, raw_max=3000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-900.0, deg_hi=900.0)
    lo, hi = reachable_deg(rom, mapping)
    assert (lo, hi) == pytest.approx((-DEFAULT_OUT_LIMIT_DEG, DEFAULT_OUT_LIMIT_DEG), abs=0.5)


def test_an_asymmetric_window_keeps_its_centre():
    """J2's usable range is not centred on zero; the mapping must not re-centre it."""
    rom = _rom(raw_min=1000, raw_max=3000, raw_anchor=2000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-263.0, deg_hi=83.0)
    assert norm_at(2000, mapping) == pytest.approx(-90.0, abs=0.5)  # midpoint of the window


def test_fit_rejects_an_unswept_joint():
    with pytest.raises(ValueError, match="sweep the joint first"):
        mapping_from_fit(_rom(raw_min=2000, raw_max=2000), invert=False, deg_lo=-90.0, deg_hi=90.0)


def test_fit_rejects_an_empty_follower_window():
    with pytest.raises(ValueError, match="empty after clipping"):
        mapping_from_fit(_rom(), invert=False, deg_lo=500.0, deg_hi=600.0, out_limit_deg=360.0)


# --- seam handling --------------------------------------------------------------------


def test_unwrap_follows_a_forward_seam_crossing():
    assert unwrap([4080, 4090, 5, 20]) == [4080, 4090, 4101, 4116]


def test_unwrap_follows_a_backward_seam_crossing():
    assert unwrap([20, 5, 4090, 4080]) == [20, 5, -6, -16]


def test_unwrap_leaves_ordinary_motion_alone():
    assert unwrap([1000, 1100, 1200]) == [1000, 1100, 1200]


def test_unwrap_of_empty_is_empty():
    assert unwrap([]) == []


@pytest.mark.parametrize(("raw_min", "raw_max"), [(1000, 3000), (3000, 5000), (-500, 500), (4000, 4400)])
def test_homing_offset_puts_the_rom_midpoint_mid_turn(raw_min, raw_max):
    offset = homing_offset_for(raw_min, raw_max)
    midpoint = (raw_min + raw_max) // 2
    assert (midpoint - offset) == TICKS_PER_TURN // 2


def test_seam_straddling_rom_stays_linear():
    """A joint whose ROM crosses 0/4095 must still map monotonically."""
    rom = JointRom(motor_id=1, raw_min=3900, raw_max=4400, raw_anchor=4150)  # unwrapped past the seam
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-50.0, deg_hi=50.0)
    # 4400 unwrapped is raw 304 on the wire; the mapping must still see it as the top end.
    assert norm_at(3900, mapping) == pytest.approx(-50.0, abs=0.2)
    assert norm_at(304, mapping) == pytest.approx(50.0, abs=0.2)
    assert norm_at(4095, mapping) == pytest.approx(norm_at(4095 - TICKS_PER_TURN, mapping), abs=0.2)


# --- warnings -------------------------------------------------------------------------


def test_clean_mapping_warns_about_nothing():
    rom = _rom(raw_min=1000, raw_max=3000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-80.0, deg_hi=40.0)
    assert check_mapping(rom, mapping, deg_lo=-80.0, deg_hi=40.0) == []


def test_hitting_the_output_backstop_is_reported():
    rom = _rom(raw_min=1000, raw_max=3000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-900.0, deg_hi=900.0)
    assert any("output backstop" in w for w in check_mapping(rom, mapping, deg_lo=-900.0, deg_hi=900.0))


def test_a_realistic_fr5_joint_warns_about_nothing():
    """J2: soft limits -263..83, swept over a normal leader travel."""
    rom = _rom(raw_min=1000, raw_max=3000, raw_anchor=2000)
    mapping = mapping_from_fit(rom, invert=False, deg_lo=-263.0, deg_hi=83.0)
    assert check_mapping(rom, mapping, deg_lo=-263.0, deg_hi=83.0) == []


def test_mapping_past_the_soft_limits_is_reported():
    rom = _rom(raw_min=1000, raw_max=3000)
    # Unity scale over 175 deg of leader travel overruns a narrow follower.
    mapping = mapping_from_scale(rom, scale=1.0, invert=False, follower_anchor_deg=0.0)
    assert any("outside the follower soft limits" in w for w in check_mapping(rom, mapping, deg_lo=-20.0, deg_hi=20.0))


def test_barely_using_the_follower_range_is_reported():
    rom = _rom(raw_min=1900, raw_max=2100)
    mapping = mapping_from_scale(rom, scale=1.0, invert=False, follower_anchor_deg=0.0)
    assert any("uses only" in w for w in check_mapping(rom, mapping, deg_lo=-90.0, deg_hi=90.0))


def test_a_barely_swept_joint_is_reported():
    rom = _rom(raw_min=1990, raw_max=2010)
    mapping = mapping_from_scale(rom, scale=1.0, invert=False, follower_anchor_deg=0.0)
    assert any("sweep was probably incomplete" in w for w in check_mapping(rom, mapping, deg_lo=-90.0, deg_hi=90.0))


def test_an_anchor_outside_the_sweep_is_reported():
    rom = _rom(raw_min=1000, raw_max=3000, raw_anchor=3500)
    mapping = mapping_from_scale(rom, scale=1.0, invert=False, follower_anchor_deg=0.0)
    assert any("outside the swept range" in w for w in check_mapping(rom, mapping, deg_lo=-90.0, deg_hi=90.0))
