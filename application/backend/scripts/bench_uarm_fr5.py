# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Time each step of the teleoperation loop, to find what makes it miss its frequency.

``TeleoperateWorker.run_loop`` does three hardware operations per cycle -- read the
follower, read the leader, command the follower -- inside a ``run_at_frequency`` budget
(33.3 ms at 30 Hz). When it overruns, the log says by how much but not which step is to
blame. This measures them separately.

Prime suspect is the leader read: ``syncReadRx`` busy-waits until *every* motor has
answered, and its timeout is ``LATENCY_TIMER = 50`` ms (``vendor/scservo_sdk/port_handler.py``).
One dropped response therefore costs ~50 ms, and ``UArmLeaderClient._read_norm`` hides the
drop by reusing the previous raw value -- so the loop goes slow instead of erroring. The
``leader`` section below reports the drop rate per motor, which is the number that matters.

Read-only by default. ``--write`` additionally times ``set_joints_state`` by commanding
the follower's *current* pose -- no motion is intended, but it does enable the robot and
enter servo mode, so keep the E-stop within reach and stay clear of the arm.

Free the hardware first: stop teleoperation in the UI (the leader's serial port and the
controller's tcp/20004 each take a single owner).

Usage::

    uv run python scripts/bench_uarm_fr5.py
    uv run python scripts/bench_uarm_fr5.py --write --cycles 300
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from robots.drivers import JOINT_NAMES, NUM_JOINTS
from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig
from robots.drivers.uarm_leader import UArmLeaderClient, UArmLeaderConfig


def summarise(label: str, samples: list[float], budget_ms: float) -> None:
    """One line per operation: mean / p95 / max in ms, against the per-cycle budget."""
    if not samples:
        print(f"{label:>22}  (no samples)")
        return
    ms = sorted(s * 1000 for s in samples)
    p95 = ms[min(len(ms) - 1, int(len(ms) * 0.95))]
    flag = "  <-- over budget on its own" if statistics.mean(ms) > budget_ms else ""
    print(f"{label:>22}  mean {statistics.mean(ms):7.2f}  p95 {p95:7.2f}  max {ms[-1]:7.2f}{flag}")


def main() -> int:
    """Measure the loop's hardware steps."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.58.2")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--fps", type=float, default=30.0, help="the rate the studio targets")
    ap.add_argument("--cycles", type=int, default=200)
    ap.add_argument("--write", action="store_true", help="also time set_joints_state (commands the current pose)")
    args = ap.parse_args()

    budget_ms = 1000.0 / args.fps
    print(f"budget at {args.fps:.0f} Hz: {budget_ms:.1f} ms per cycle\n")

    follower = FR5FollowerClient(FR5FollowerConfig(ip=args.ip))
    leader = UArmLeaderClient(UArmLeaderConfig(port=args.port))
    follower.connect()
    leader.connect()

    read_f: list[float] = []
    read_l: list[float] = []
    write_f: list[float] = []
    # A drop is a motor that failed to answer within the sync-read window.
    drops = dict.fromkeys(range(1, NUM_JOINTS + 1), 0)

    try:
        for _ in range(args.cycles):
            t = time.perf_counter()
            state = follower.read_state()["state"]
            read_f.append(time.perf_counter() - t)

            t = time.perf_counter()
            leader.read_state()
            read_l.append(time.perf_counter() - t)

            # Ask the bus directly which motors answered that round.
            reader = leader._reader
            if reader is not None:
                for jc in leader._joints:
                    if not reader.isAvailable(jc.motor_id, leader._addr, 2)[0]:
                        drops[jc.motor_id] += 1

            if args.write:
                t = time.perf_counter()
                # The worker passes twice its period; mirror that so cmdT matches teleop.
                follower.set_joints_state(
                    {f"{n}.pos": state[f"{n}.pos"] for n in JOINT_NAMES}, 2.0 / args.fps
                )
                write_f.append(time.perf_counter() - t)
    finally:
        leader.disconnect()
        follower.disconnect()

    report(args, budget_ms, read_f, read_l, write_f, drops)
    return 0


def report(args, budget_ms, read_f, read_l, write_f, drops) -> None:  # noqa: ANN001
    """Print the per-step timings and the leader's drop rate."""
    print(f"over {args.cycles} cycles:\n")
    summarise("follower read_state", read_f, budget_ms)
    summarise("leader read_state", read_l, budget_ms)
    if write_f:
        summarise("follower set_joints", write_f, budget_ms)

    total = statistics.mean(read_f) + statistics.mean(read_l) + (statistics.mean(write_f) if write_f else 0.0)
    print(f"\n{'cycle total':>22}  {total * 1000:7.2f} ms  ->  {1 / total:6.1f} Hz achievable")
    if total * 1000 > budget_ms:
        print(f"{'':>22}  over the {budget_ms:.1f} ms budget: this is why run_at_frequency reports misses")

    dropped = {mid: n for mid, n in drops.items() if n}
    print("\nleader response drops (each one costs a ~50 ms sync-read timeout):")
    if not dropped:
        print("  none -- every motor answered every round, so the leader read is not the bottleneck")
    else:
        for mid, n in sorted(dropped.items()):
            print(f"  motor {mid} ({JOINT_NAMES[mid - 1]}): {n}/{args.cycles} rounds  ({n / args.cycles:.0%})")


if __name__ == "__main__":
    raise SystemExit(main())
