# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Headless uArm leader calibration for the FR5 follower.

Sweeps the leader's range of motion, samples both arms in a matched pose, and turns the
result into the five per-joint fields the studio stores (see
``robots.calibration.uarm_mapping`` for the derivation). Without a calibration the leader
falls back to a full turn, which reads safely but is not aligned to the FR5.

Read-only against both arms: the leader is back-driven with torque off and nothing is ever
written to servo EEPROM, and the FR5 is only asked for its pose and soft limits -- no
``RobotEnable``, no ``Mode`` change, no motion. Printing the result is the default; ``--apply``
is required before anything is written to the studio.

Usage::

    # look at what it would store -- needs no backend and no ids
    uv run python scripts/calibrate_uarm.py

    # find the project/robot ids (needs the backend running)
    uv run python scripts/calibrate_uarm.py --list

    # store it and make it the robot's active calibration
    uv run python scripts/calibrate_uarm.py --project-id <uuid> --robot-id <uuid> --apply

Disconnect the robots in the studio UI first: the leader's serial port takes a single
opener, so a live teleoperation session and this script cannot both hold it.

The leader publishes follower degrees with no +-100 saturation, so the anchor may be taken
anywhere in the arm's real workspace -- including j2 near -150 deg, which earlier versions
could not represent. Both modes below can now reach a joint's full soft-limit range.

``--mode scale`` (default) reproduces the hardware-validated feel of the standalone
``teleop_uarm_fr5.py``: pass the same ``--scale``/``--invert``/``--map`` it uses and the
leader pose at calibration time is pinned to the FR5 pose at calibration time, so
teleoperation starts without a jump. ``--mode fit`` ignores those constants and stretches
the measured sweep across the follower's usable window instead.

Needs ``httpx`` for ``--apply``; it arrives with ``fastapi[standard]`` rather than being
declared directly, which is fine for a dev script but not something to rely on in ``src``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from robots.calibration.uarm_mapping import (
    JointMapping,
    JointRom,
    check_mapping,
    mapping_from_fit,
    mapping_from_scale,
    norm_at,
    reachable_deg,
    unwrap,
)
from robots.drivers import JOINT_NAMES, NUM_JOINTS
from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig

# (feature name, solved mapping, the sweep it came from) per FR5 joint.
Rows = list[tuple[str, JointMapping, JointRom]]


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


DEFAULT_BASE_URL = "http://localhost:7860"
SWEEP_HZ = 30.0


# --- hardware (read-only) -------------------------------------------------------------


class LeaderBus:
    """Raw single-turn reads from the leader's Feetech bus.

    Deliberately not ``UArmLeaderClient``: that applies a calibration, and this script
    exists to produce one. Same vendored SDK, no interpretation.
    """

    def __init__(self, port: str, baud: int, motor_ids: list[int]) -> None:
        from robots.drivers.vendor.scservo_sdk import SMS_STS_PRESENT_POSITION_L, GroupSyncRead, PortHandler, sms_sts

        self._addr = SMS_STS_PRESENT_POSITION_L
        self._ph = PortHandler(port)
        if not self._ph.openPort():
            raise RuntimeError(f"failed to open leader port {port}")
        self._ph.setBaudRate(baud)
        self._reader = GroupSyncRead(sms_sts(self._ph), SMS_STS_PRESENT_POSITION_L, 4)
        self._motor_ids = motor_ids
        for mid in motor_ids:
            self._reader.addParam(mid)

    def read(self) -> dict[int, int]:
        self._reader.txRxPacket()
        out: dict[int, int] = {}
        for mid in self._motor_ids:
            if self._reader.isAvailable(mid, self._addr, 2)[0]:
                out[mid] = self._reader.getData(mid, self._addr, 2) % 4096
        return out

    def close(self) -> None:
        self._ph.closePort()


def open_follower(ip: str, margin: float) -> tuple[list[float], list[float], list[float]]:
    """Return (joint degrees, soft-limit low, soft-limit high) without enabling the arm.

    ``FR5FollowerClient.connect()`` enables the robot and switches it to auto mode, which a
    calibration read has no business doing, so the RPC handle is attached directly and only
    the two read-only helpers are used.
    """
    client = FR5FollowerClient(FR5FollowerConfig(ip=ip, gripper_enabled=False))
    robot_cls = client._import_sdk()
    client._rpc = robot_cls.RPC(ip)
    # The SDK's state channel comes up asynchronously; reading before it does is a hard
    # error rather than a stale value, so wait rather than race it.
    degs = client._wait_for_state()
    limits = client._fetch_soft_limits(margin)
    if limits is None:
        raise RuntimeError("FR5 soft joint limits unavailable; cannot size the mapping")
    lo, hi = limits
    return degs, lo, hi


# --- sweep ----------------------------------------------------------------------------


@dataclass
class Sweep:
    """Per-motor unwrapped sample history."""

    samples: dict[int, list[int]]

    @classmethod
    def empty(cls, motor_ids: list[int]) -> Sweep:
        return cls(samples={mid: [] for mid in motor_ids})

    def add(self, reading: dict[int, int]) -> None:
        for mid, raw in reading.items():
            self.samples[mid].append(raw)

    def rom(self, motor_id: int, anchor_raw: int) -> JointRom:
        unwrapped = unwrap(self.samples[motor_id])
        if not unwrapped:
            raise RuntimeError(f"motor {motor_id}: no samples captured during the sweep")
        # The anchor was read on the wire (single-turn); lift it into the same unwrapped
        # frame by picking the turn that lands it nearest the swept band.
        mid_band = (min(unwrapped) + max(unwrapped)) / 2.0
        anchor = min(
            (anchor_raw + turn * 4096 for turn in (-1, 0, 1)),
            key=lambda cand: abs(cand - mid_band),
        )
        return JointRom(motor_id=motor_id, raw_min=min(unwrapped), raw_max=max(unwrapped), raw_anchor=anchor)


def run_sweep(bus: LeaderBus, motor_ids: list[int]) -> Sweep:
    """Sample the leader until the operator presses Enter, showing travel per joint."""
    sweep = Sweep.empty(motor_ids)
    period = 1.0 / SWEEP_HZ
    print("\nMove every leader joint slowly through its full range, then press Enter.")
    print("Sweeping... (per-joint travel in degrees)\n")

    import select

    while True:
        t0 = time.perf_counter()
        sweep.add(bus.read())
        travel = []
        for mid in motor_ids:
            vals = unwrap(sweep.samples[mid])
            deg = (max(vals) - min(vals)) * 360.0 / 4096.0 if vals else 0.0
            travel.append(f"{mid}:{deg:5.0f}")
        print("  " + "  ".join(travel), end="\r", flush=True)
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    print("\n")
    return sweep


# --- assembly -------------------------------------------------------------------------


def parse_scale(spec: str) -> dict[int, float]:
    """``'j6:2.0,j2:1.3'`` -> ``{6: 2.0, 2: 1.3}`` (FR5 joint numbers)."""
    out: dict[int, float] = {}
    for token in (t.strip() for t in spec.split(",") if t.strip()):
        name, _, value = token.partition(":")
        out[int(name.lstrip("jJ"))] = float(value)
    return out


def parse_invert(spec: str) -> set[int]:
    """``'1,2,4'`` -> the FR5 joint numbers whose direction is flipped."""
    return {int(t) for t in (t.strip() for t in spec.split(",")) if t}


def parse_map(spec: str) -> list[int]:
    """``'1,2,3,4,6,5'`` -> leader joint (1-based) driving each FR5 joint in turn."""
    src = [int(t) for t in (t.strip() for t in spec.split(",")) if t]
    if sorted(src) != list(range(1, NUM_JOINTS + 1)):
        raise ValueError(f"--map must be a permutation of 1..{NUM_JOINTS}, got {spec}")
    return src


def prompt_for_anchor(anchor: str) -> None:
    """Tell the operator what pose to hold before the anchor is sampled."""
    print("\nPose the leader to match the FR5's current pose above, then press Enter.")
    if anchor:
        print(f"(except {anchor}: hold the leader where you want that angle instead --")
        print(" that joint will NOT start without a jump; check with --check before switching on)")
    else:
        print("(this pins the two together so teleoperation starts without a jump)")


def build_mappings(
    args: argparse.Namespace,
    sweep: Sweep,
    anchor_raw: dict[int, int],
    follower_deg: list[float],
    deg_lo: list[float],
    deg_hi: list[float],
) -> Rows:
    """Solve one mapping per FR5 joint, honouring the leader->follower permutation."""
    scale = parse_scale(args.scale)
    invert = parse_invert(args.invert)
    src = parse_map(args.map)
    # Joints whose leader travel is mechanically short: stretch what little they have
    # across the follower's whole range rather than mapping it 1:1.
    fit_joints = parse_invert(args.fit_joints) if args.fit_joints else set()
    # Per-joint override of what the leader's anchor pose means on the follower. Without
    # it the anchor is the follower's *current* angle, which pins the two arms together;
    # with it you choose where the leader's present position should land instead, and the
    # joint's travel is redistributed around that point.
    anchor_deg = parse_scale(args.anchor) if args.anchor else {}

    rows: Rows = []
    for j in range(NUM_JOINTS):
        fr5_joint = j + 1
        motor_id = src[j]  # leader motor driving this FR5 joint
        rom = sweep.rom(motor_id, anchor_raw[motor_id])
        inv = fr5_joint in invert
        if args.mode == "scale" and fr5_joint not in fit_joints:
            mapping = mapping_from_scale(
                rom,
                scale=scale.get(fr5_joint, 1.0),
                invert=inv,
                follower_anchor_deg=anchor_deg.get(fr5_joint, follower_deg[j]),
            )
        else:
            mapping = mapping_from_fit(rom, invert=inv, deg_lo=deg_lo[j], deg_hi=deg_hi[j])
        rows.append((JOINT_NAMES[j], mapping, rom))
    return rows


def print_table(rows: Rows, deg_lo: list[float], deg_hi: list[float]) -> int:
    """Show what would be stored and what it reaches; returns the warning count."""
    print(f"{'joint':>6} {'motor':>5} {'homing':>7} {'min':>7} {'max':>7} {'drive':>5}   reachable / follower limits")
    print("-" * 88)
    warning_count = 0
    for j, (name, m, rom) in enumerate(rows):
        lo, hi = reachable_deg(rom, m)
        print(
            f"{name:>6} {m.motor_id:>5} {m.homing_offset:>7} {m.range_min:>7} {m.range_max:>7} {m.drive_mode:>5}   "
            f"[{lo:7.1f},{hi:7.1f}]  vs [{deg_lo[j]:7.1f},{deg_hi[j]:7.1f}]"
        )
        for warning in check_mapping(rom, m, deg_lo=deg_lo[j], deg_hi=deg_hi[j]):
            print(f"       ! {name}: {warning}")
            warning_count += 1
    return warning_count


def to_payload(rows: Rows, robot_id: str) -> dict:
    """Shape the mappings as the studio's Calibration schema."""
    return {
        "id": str(uuid.uuid4()),
        "robot_id": robot_id,
        # Overwritten by the service when it writes the file; required by the schema.
        "file_path": "",
        "values": {
            name: {
                "id": m.motor_id,
                "joint_name": name,
                "drive_mode": m.drive_mode,
                "homing_offset": m.homing_offset,
                "range_min": m.range_min,
                "range_max": m.range_max,
            }
            for name, m, _ in rows
        },
    }


def ensure_backend(base_url: str, project_id: str, robot_id: str) -> None:
    """Fail before the sweep, not after it.

    ``--apply`` needs the backend, but everything it needs the backend *for* happens after
    several minutes of hardware work. Checking up front turns "connection refused after
    the sweep" into "start the backend first".
    """
    import httpx

    try:
        with httpx.Client(timeout=5.0) as http:
            robot = http.get(f"{base_url}/api/projects/{project_id}/robots/{robot_id}")
            robot.raise_for_status()
    except httpx.ConnectError as e:
        raise SystemExit(
            f"--apply needs the studio backend at {base_url}, which is not accepting "
            f"connections ({e}).\n"
            f"Start it, or drop --apply and pass --json to save the result for later."
        ) from e
    except httpx.HTTPStatusError as e:
        raise SystemExit(f"--apply: backend rejected the robot lookup: {e}") from e


def apply_from_json(base_url: str, project_id: str, robot_id: str, path: Path) -> int:
    """Store a payload saved by an earlier run, without touching the hardware again."""
    payload = json.loads(path.read_text())
    payload["robot_id"] = robot_id
    apply(base_url, project_id, robot_id, payload)
    return 0


def check(base_url: str, project_id: str, robot_id: str, args: argparse.Namespace) -> int:
    """Pre-flight: where will the follower go the instant teleoperation is switched on?

    The map is absolute, so switching on commands the follower to wherever the leader
    currently maps -- regardless of where the follower happens to be. ``max_step_deg``
    turns a large difference into a bounded slew rather than a jump, but the arm still
    travels the whole way. This reads both arms through the stored calibration and prints
    that difference before anyone touches the switch.
    """
    import httpx

    with httpx.Client(timeout=30.0) as http:
        robot = http.get(f"{base_url}/api/projects/{project_id}/robots/{robot_id}")
        robot.raise_for_status()
        calibration_id = robot.json().get("active_calibration_id")
        if not calibration_id:
            print("This robot has no active calibration; the leader would fall back to a full turn.")
            return 1
        cal = http.get(f"{base_url}/api/projects/{project_id}/robots/{robot_id}/calibrations/{calibration_id}")
        cal.raise_for_status()
        values = cal.json()["values"]

    mappings = [
        JointMapping(
            motor_id=values[name]["id"],
            homing_offset=values[name]["homing_offset"],
            range_min=values[name]["range_min"],
            range_max=values[name]["range_max"],
            drive_mode=values[name]["drive_mode"],
        )
        for name in JOINT_NAMES
    ]

    follower_deg, deg_lo, deg_hi = open_follower(args.ip, args.margin)
    bus = LeaderBus(args.port, args.baud, [m.motor_id for m in mappings])
    try:
        raws = bus.read()
    finally:
        bus.close()

    print(f"\ncalibration {calibration_id}\n")
    print(f"{'joint':>6} {'follower now':>13} {'leader maps to':>15} {'will move':>10}")
    print("-" * 50)
    worst = 0.0
    for j, (name, m) in enumerate(zip(JOINT_NAMES, mappings, strict=True)):
        raw = raws.get(m.motor_id)
        if raw is None:
            print(f"{name:>6}   !! leader motor {m.motor_id} did not respond")
            return 1
        target = _clamp(norm_at(raw, m), deg_lo[j], deg_hi[j])
        delta = target - follower_deg[j]
        worst = max(worst, abs(delta))
        print(f"{name:>6} {follower_deg[j]:13.1f} {target:15.1f} {delta:+10.1f}")

    seconds = worst / max(args.max_step_deg * args.fps, 1e-9)
    print(
        f"\nlargest move: {worst:.1f} deg "
        f"(~{seconds:.1f}s of slew at {args.max_step_deg} deg per cycle, {args.fps} Hz)"
    )
    if worst > args.warn_deg:
        print(
            f"\n!! Over {args.warn_deg:.0f} deg. Move the leader closer to the follower's pose before\n"
            f"   switching on, or expect the arm to travel that far by itself."
        )
        return 1
    print("\nSafe to switch on: the follower is already close to where the leader points.")
    return 0


def apply(base_url: str, project_id: str, robot_id: str, payload: dict) -> None:
    """Save the calibration and point the robot at it."""
    import httpx

    robots_url = f"{base_url}/api/projects/{project_id}/robots"
    with httpx.Client(timeout=30.0) as http:
        saved = http.post(f"{robots_url}/{robot_id}/calibrations", json=payload)
        saved.raise_for_status()
        calibration_id = saved.json()["id"]
        print(f"saved calibration {calibration_id}")

        robot = http.get(f"{robots_url}/{robot_id}")
        robot.raise_for_status()
        body = robot.json()
        body["active_calibration_id"] = calibration_id
        # PUT takes the whole robot; server-managed fields would be rejected on the way back in.
        for field in ("created_at", "updated_at"):
            body.pop(field, None)
        updated = http.put(f"{robots_url}/{robot_id}", json=body)
        updated.raise_for_status()
        print(f"robot {robot_id} now uses calibration {calibration_id}")


def list_robots(base_url: str) -> int:
    """Print project and robot ids so --apply can be pointed at the right pair."""
    import httpx

    with httpx.Client(timeout=30.0) as http:
        projects = http.get(f"{base_url}/api/projects")
        projects.raise_for_status()
        for project in projects.json():
            print(f"\nproject {project['id']}  {project.get('name', '')}")
            robots = http.get(f"{base_url}/api/projects/{project['id']}/robots")
            robots.raise_for_status()
            for robot in robots.json():
                active = robot.get("active_calibration_id") or "-"
                print(f"  robot {robot['id']}  {robot['type']:<16} {robot.get('name', '')}  calibration={active}")
    print("\nPass the pair as --project-id / --robot-id (the UArm_Leader is the one to calibrate).")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """Command line for the calibration run."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    # Only needed to store the result: a preview neither reads nor writes the studio.
    ap.add_argument("--robot-id", help="studio robot id of the UArm_Leader; required with --apply")
    ap.add_argument("--project-id", help="studio project the robot belongs to; required with --apply")
    ap.add_argument("--list", action="store_true", help="list project/robot ids from a running backend and exit")
    ap.add_argument(
        "--check",
        action="store_true",
        help="pre-flight: show how far the follower will move when teleoperation is switched on",
    )
    ap.add_argument("--warn-deg", type=float, default=15.0, help="--check: flag a start-up move larger than this")
    ap.add_argument("--max-step-deg", type=float, default=2.0, help="--check: follower slew cap, for the time estimate")
    ap.add_argument("--fps", type=float, default=30.0, help="--check: teleoperation rate, for the time estimate")
    ap.add_argument("--base-url", default=DEFAULT_BASE_URL)
    ap.add_argument("--port", default="/dev/ttyACM0", help="leader serial port")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ip", default="192.168.58.2", help="FR5 controller address")
    ap.add_argument("--margin", type=float, default=2.0, help="degrees held back from each soft limit")
    ap.add_argument("--mode", choices=("scale", "fit"), default="scale")
    ap.add_argument("--scale", default="j6:2.0,j5:2.0,j4:1.5,j2:1.3", help="scale mode: follower deg per leader deg")
    ap.add_argument("--invert", default="1,2,4,5,6", help="FR5 joints whose direction is flipped")
    ap.add_argument("--map", default="1,2,3,4,5,6", help="leader joint driving each FR5 joint")
    ap.add_argument(
        "--anchor",
        default="",
        help="scale mode: map the leader's current pose to these follower angles instead "
        "of the follower's own, e.g. 'j3:150' -- redistributes that joint's travel",
    )
    ap.add_argument(
        "--fit-joints",
        default="",
        help="FR5 joints to fit across their full range even in scale mode, e.g. '3' "
        "-- for joints the leader cannot physically drive far enough",
    )
    ap.add_argument("--json", type=Path, help="also write the calibration payload here")
    ap.add_argument("--apply", action="store_true", help="save to the studio and make it active")
    ap.add_argument(
        "--from-json",
        type=Path,
        help="apply a payload saved by an earlier run instead of sweeping again",
    )
    return ap


def dispatch_early_modes(ap: argparse.ArgumentParser, args: argparse.Namespace) -> int | None:
    """Run whichever mode needs no sweep; None means "carry on and calibrate"."""
    if args.list:
        return list_robots(args.base_url)
    ids = args.project_id and args.robot_id
    if args.from_json:
        if not ids:
            ap.error("--from-json needs --project-id and --robot-id")
        return apply_from_json(args.base_url, args.project_id, args.robot_id, args.from_json)
    if args.check:
        if not ids:
            ap.error("--check needs --project-id and --robot-id (find them with --list)")
        return check(args.base_url, args.project_id, args.robot_id, args)
    if args.apply and not ids:
        ap.error("--apply needs --project-id and --robot-id (find them with --list)")
    return None


def main() -> int:
    """Sweep, solve, report, and optionally store."""
    ap = build_parser()
    args = ap.parse_args()

    early = dispatch_early_modes(ap, args)
    if early is not None:
        return early
    # The sweep watches stdin for Enter, which needs a real terminal rather than a pipe.
    if not sys.stdin.isatty():
        ap.error("run this from a terminal: the sweep waits on Enter")
    if args.apply:
        ensure_backend(args.base_url, args.project_id, args.robot_id)

    src = parse_map(args.map)

    print(f"FR5 {args.ip}: reading pose and soft limits (read-only)")
    follower_deg, deg_lo, deg_hi = open_follower(args.ip, args.margin)
    print("  usable: " + ", ".join(f"J{j + 1}[{deg_lo[j]:.0f},{deg_hi[j]:.0f}]" for j in range(NUM_JOINTS)))
    print("  pose:   " + ", ".join(f"J{j + 1}={follower_deg[j]:.1f}" for j in range(NUM_JOINTS)))

    bus = LeaderBus(args.port, args.baud, src)
    try:
        if args.mode == "scale":
            prompt_for_anchor(args.anchor)
            input()
        anchor_raw = bus.read()
        missing = [mid for mid in src if mid not in anchor_raw]
        if missing:
            raise RuntimeError(f"leader motors {missing} did not respond; check the bus")

        sweep = run_sweep(bus, src)
    finally:
        bus.close()

    rows = build_mappings(args, sweep, anchor_raw, follower_deg, deg_lo, deg_hi)
    warnings = print_table(rows, deg_lo, deg_hi)
    # Placeholder id in preview: the server overwrites robot_id on save anyway.
    payload = to_payload(rows, args.robot_id or str(uuid.UUID(int=0)))

    if args.json:
        args.json.write_text(json.dumps(payload, indent=2))
        print(f"\nwrote {args.json}")

    if not args.apply:
        print("\nNothing was saved. Re-run with --apply to store this and make it active.")
        return 0
    if warnings:
        print(f"\n{warnings} warning(s) above -- applying anyway because --apply was given.")
    try:
        apply(args.base_url, args.project_id, args.robot_id, payload)
    except Exception as e:  # the sweep is expensive; never lose it to an HTTP fault
        fallback = Path(f"calibration-{payload['id']}.json")
        fallback.write_text(json.dumps(payload, indent=2))
        print(f"\nSaving to the studio failed: {e}")
        print(f"The calibration itself is fine and is saved at {fallback}.")
        print("Re-apply it once the backend is up -- no need to sweep again:")
        print(
            f"  uv run python scripts/calibrate_uarm.py --from-json {fallback} "
            f"--project-id {args.project_id} --robot-id {args.robot_id}"
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
