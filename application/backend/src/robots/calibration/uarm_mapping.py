# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Turn a uArm range-of-motion sweep into the five stored calibration fields.

The studio drives the FR5 with an *absolute* map: whatever the leader publishes as
``jN.pos`` lands on the follower as degrees (``fr5_follower.set_joints_state``). The
leader's whole contribution is the calibration read back in ``UArmLeaderClient._read_norm``:

    single = (raw - homing_offset) % 4096
    deg    = (single - range_min) / span * 200 - 100        # span = range_max - range_min
    if drive_mode == 1: deg = -deg
    deg    = clamp(deg, -out_limit_deg, +out_limit_deg)     # backstop, not a workspace limit

So ``homing_offset / range_min / range_max / drive_mode`` (plus ``motor_id`` order) must
encode everything the standalone ``teleop_uarm_fr5.py`` passes as ``--scale``, ``--invert``,
``--map`` and its anchor. ``norm_at`` below mirrors that formula and is pinned against the
real driver in ``tests/robots/test_uarm_mapping.py`` -- if the driver's maths changes, that
test fails rather than the calibration silently drifting.

Two ways to choose the window, both emitting the same five fields:

``mapping_from_scale`` (what the script proved)
    Reproduces a known ``--scale``/``--invert`` pair and pins the leader's pose at
    calibration time to the FR5's pose at calibration time. Use this to carry the
    hardware-validated feel over unchanged.

``mapping_from_fit``
    Stretches the measured ROM across the follower's usable window, so the full leader
    travel reaches the full FR5 travel. Needs no tuning constants, but changes the feel.

Note on scale: the window spans 200 output units, so mapping a leader joint that travels
less than 200 deg *is* an amplification. That is why the script needed ``--scale j6:2.0``
and friends -- fit mode recreates those factors on its own by sizing the window.

Two hard edges of the stored model, both handled here:

- ``% 4096`` means a joint whose ROM straddles the 0/4095 seam reads garbage. The sweep is
  unwrapped (``unwrap``) and ``homing_offset`` rotates the seam out of the way, so the
  linear part never sees a discontinuity.
- The window is only 200 units wide, so reaching a follower joint's full travel means
  sizing it *narrower* than the leader's own travel. Nothing caps that at 100 deg any
  more -- ``out_limit_deg`` is a mis-sizing backstop, and the follower's soft limits are
  the real bound. ``check_mapping`` reports what a mapping actually reaches.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

TICKS_PER_TURN = 4096
DEG_PER_TURN = 360.0
# norm spans -100..100
NORM_SPAN = 200.0
# Half the window's output span -- a formula constant, not a bound.
HALF_SPAN = NORM_SPAN / 2.0
# Backstop the leader applies to its published degrees; mirrors
# ``UArmLeaderConfig.out_limit_deg``. Not a workspace limit: the follower clamps to its
# own soft limits, which are what actually bound the arm.
DEFAULT_OUT_LIMIT_DEG = 360.0

# Ticks of window that give the follower one degree per degree of leader travel.
# 200 norm units over (2275.6 ticks = 200 deg) -> unity.
UNIT_SCALE_SPAN = NORM_SPAN * TICKS_PER_TURN / DEG_PER_TURN


@dataclass(frozen=True)
class JointRom:
    """One joint's sweep result, in *unwrapped* raw ticks.

    ``raw_anchor`` is the reading at the moment the follower pose was sampled; scale mode
    pins the two together so teleoperation starts without a jump.
    """

    motor_id: int
    raw_min: int
    raw_max: int
    raw_anchor: int

    @property
    def span_ticks(self) -> int:
        return self.raw_max - self.raw_min

    @property
    def span_deg(self) -> float:
        return self.span_ticks * DEG_PER_TURN / TICKS_PER_TURN


@dataclass(frozen=True)
class JointMapping:
    """The five fields persisted per joint, in the driver's own vocabulary."""

    motor_id: int
    homing_offset: int
    range_min: int
    range_max: int
    drive_mode: int

    @property
    def span(self) -> int:
        return self.range_max - self.range_min


def unwrap(samples: Sequence[int]) -> list[int]:
    """Undo 0/4095 seam crossings in a stream of single-turn reads.

    A jump of more than half a turn between consecutive samples is read as a wrap rather
    than real motion -- safe because the sweep is a human moving a joint by hand, which
    cannot cover 180 deg between two 30 Hz reads.
    """
    if not samples:
        return []
    out = [int(samples[0])]
    turns = 0
    for prev, cur in pairwise(samples):
        delta = int(cur) - int(prev)
        if delta > TICKS_PER_TURN // 2:
            turns -= 1
        elif delta < -(TICKS_PER_TURN // 2):
            turns += 1
        out.append(int(cur) + turns * TICKS_PER_TURN)
    return out


def homing_offset_for(raw_min: int, raw_max: int) -> int:
    """Rotate the ROM midpoint onto mid-turn so ``% 4096`` never splits the range."""
    mid = (raw_min + raw_max) / 2.0
    return round(mid) - TICKS_PER_TURN // 2


def norm_at(raw: int, mapping: JointMapping, *, out_limit_deg: float = DEFAULT_OUT_LIMIT_DEG) -> float:
    """Reproduce ``UArmLeaderClient._read_norm`` for one joint.

    Kept in lockstep with the driver by ``test_uarm_mapping.py``.
    """
    single = (int(raw) - mapping.homing_offset) % TICKS_PER_TURN
    span = mapping.span
    if span == 0:
        return 0.0
    deg = (single - mapping.range_min) / span * NORM_SPAN - HALF_SPAN
    if mapping.drive_mode == 1:
        deg = -deg
    return max(-out_limit_deg, min(out_limit_deg, deg))


def _finish(rom: JointRom, homing_offset: int, span: float, low_norm_pre: float, single_low: float) -> JointMapping:
    """Solve ``range_min`` from one known (position, pre-sign norm) pair plus the span."""
    range_min = single_low - span * (low_norm_pre + HALF_SPAN) / NORM_SPAN
    range_min_i = round(range_min)
    return JointMapping(
        motor_id=rom.motor_id,
        homing_offset=homing_offset,
        range_min=range_min_i,
        range_max=range_min_i + round(span),
        drive_mode=0,
    )


def mapping_from_scale(rom: JointRom, *, scale: float, invert: bool, follower_anchor_deg: float) -> JointMapping:
    """Rebuild the script's ``--scale``/``--invert`` behaviour as a stored calibration.

    ``scale`` is degrees of follower motion per degree of leader motion (the script's
    ``sign * scale * gain``, magnitude only -- direction comes from ``invert``).
    ``follower_anchor_deg`` is the FR5 joint angle sampled at the same instant as
    ``rom.raw_anchor``; the two are pinned together so the absolute map starts where the
    arm already is.
    """
    if scale <= 0:
        raise ValueError(f"scale must be positive, got {scale}")
    sign = -1.0 if invert else 1.0
    span = UNIT_SCALE_SPAN / scale
    homing_offset = homing_offset_for(rom.raw_min, rom.raw_max)
    single_anchor = rom.raw_anchor - homing_offset
    # norm(anchor) must equal follower_anchor_deg *after* the drive_mode flip.
    mapping = _finish(rom, homing_offset, span, sign * follower_anchor_deg, single_anchor)
    return JointMapping(
        motor_id=mapping.motor_id,
        homing_offset=mapping.homing_offset,
        range_min=mapping.range_min,
        range_max=mapping.range_max,
        drive_mode=1 if invert else 0,
    )


def mapping_from_fit(
    rom: JointRom,
    *,
    invert: bool,
    deg_lo: float,
    deg_hi: float,
    out_limit_deg: float = DEFAULT_OUT_LIMIT_DEG,
) -> JointMapping:
    """Stretch the measured ROM across the follower's usable window.

    ``deg_lo``/``deg_hi`` are the follower's soft limits, clipped to the leader's output
    backstop. With the backstop wide (the default) the full asymmetric window is used --
    J2 and J4 reach past -100 deg, which is where this arm actually works.
    """
    if rom.span_ticks <= 0:
        raise ValueError(f"motor {rom.motor_id}: ROM span is {rom.span_ticks} ticks; sweep the joint first")
    tgt_lo = max(deg_lo, -out_limit_deg)
    tgt_hi = min(deg_hi, out_limit_deg)
    if tgt_hi <= tgt_lo:
        raise ValueError(f"motor {rom.motor_id}: follower window [{deg_lo}, {deg_hi}] is empty after clipping")

    homing_offset = homing_offset_for(rom.raw_min, rom.raw_max)
    single_min = rom.raw_min - homing_offset
    single_max = rom.raw_max - homing_offset
    # Pre-sign norm at each ROM end: inverting swaps which end is which.
    low_norm_pre, high_norm_pre = (-tgt_hi, -tgt_lo) if invert else (tgt_lo, tgt_hi)
    span = NORM_SPAN * (single_max - single_min) / (high_norm_pre - low_norm_pre)
    mapping = _finish(rom, homing_offset, span, low_norm_pre, single_min)
    return JointMapping(
        motor_id=mapping.motor_id,
        homing_offset=mapping.homing_offset,
        range_min=mapping.range_min,
        range_max=mapping.range_max,
        drive_mode=1 if invert else 0,
    )


def reachable_deg(
    rom: JointRom, mapping: JointMapping, *, out_limit_deg: float = DEFAULT_OUT_LIMIT_DEG
) -> tuple[float, float]:
    """Follower degrees the ROM endpoints actually produce, after clamping."""
    a = norm_at(rom.raw_min, mapping, out_limit_deg=out_limit_deg)
    b = norm_at(rom.raw_max, mapping, out_limit_deg=out_limit_deg)
    return (min(a, b), max(a, b))


def check_mapping(
    rom: JointRom,
    mapping: JointMapping,
    *,
    deg_lo: float,
    deg_hi: float,
    out_limit_deg: float = DEFAULT_OUT_LIMIT_DEG,
) -> list[str]:
    """Warn about mappings that are valid arithmetic but poor on hardware.

    Returns human-readable warnings; empty means nothing to flag.
    """
    warnings: list[str] = []
    lo, hi = reachable_deg(rom, mapping, out_limit_deg=out_limit_deg)

    if abs(lo) >= out_limit_deg - 0.5 or abs(hi) >= out_limit_deg - 0.5:
        warnings.append(
            f"reaches the leader's +-{out_limit_deg:.0f} deg output backstop ([{lo:.1f}, {hi:.1f}]); "
            f"the calibration window is almost certainly mis-sized"
        )
    if lo < deg_lo - 0.5 or hi > deg_hi + 0.5:
        warnings.append(
            f"maps outside the follower soft limits: [{lo:.1f}, {hi:.1f}] vs [{deg_lo:.1f}, {deg_hi:.1f}]; "
            f"the follower will clamp at the limit"
        )
    usable = min(hi, deg_hi) - max(lo, deg_lo)
    available = deg_hi - deg_lo
    if available > 0 and usable < 0.25 * available:
        warnings.append(
            f"uses only {usable:.0f} deg of the follower's {available:.0f} deg range; "
            f"consider a larger scale or a wider sweep"
        )
    if rom.span_ticks < TICKS_PER_TURN // 40:  # < ~9 deg of leader travel
        warnings.append(f"leader travel was only {rom.span_deg:.1f} deg; the sweep was probably incomplete")
    if not (rom.raw_min <= rom.raw_anchor <= rom.raw_max):
        warnings.append(
            f"anchor {rom.raw_anchor} lies outside the swept range [{rom.raw_min}, {rom.raw_max}]; "
            f"the sweep and the anchor may have been taken in different poses"
        )
    return warnings
