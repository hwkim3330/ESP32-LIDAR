#!/usr/bin/env python3
"""Sweep gate schedules and record what each one does to this stream.

    tools/sweep.py                       # the default set
    tools/sweep.py --seconds 25          # longer windows, steadier numbers

Builds a table rather than an argument. Every row is a schedule applied to the port the receiver
hangs off, and the numbers beside it are what the receiver saw -- not what the switch said about
itself, which was wrong four different ways in one afternoon.

The interesting column is not the mean. Shaping does not change how much arrives; it changes when.
So the mean gap stays at the sensor's own spacing for every row that works, and what moves is the
count of gaps past the threshold, the worst gap, and the smallest one -- packets held during a
closed window come out back to back.

A row is only recorded if the traffic changed. A schedule the switch accepted and did not apply
is not a data point about shaping; it is a data point about the switch, and it is marked as such.
"""
import argparse
import re
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, "/home/kim/esp32-lidar/tools")
from shape import cli, switch_clock, apply_schedule   # noqa: E402

BOARD = "/dev/ttyACM0"
SWITCH = "/dev/ttyACM2"
OPEN_YAML = "/home/kim/esp32-lidar/tools/tas-b-open-timed.yaml"

DETAIL = re.compile(r"lidar \d+ pkt (\d+)/s \(\d+ B, gap (\d+) us mean / (\d+) us max")
GAPS = re.compile(r"gaps over \d+ us: (\d+) \| min gap (\d+)")

SCHEDULE = """- ? "/ietf-interfaces:interfaces/interface[name='1']/ieee802-dot1q-bridge:bridge-port/ieee802-dot1q-sched-bridge:gate-parameter-table"
  : gate-enabled: true
    admin-gate-states: 255
    admin-cycle-time:
      numerator: {cycle_ns}
      denominator: 1000000000
    admin-base-time:
      seconds: 0
      nanoseconds: 0
    admin-control-list:
      gate-control-entry:
        - index: 0
          operation-name: set-gate-states
          gate-states-value: 255
          time-interval-value: {open_ns}
        - index: 1
          operation-name: set-gate-states
          gate-states-value: 254
          time-interval-value: {closed_ns}
    config-change: true
"""


def observe(seconds):
    """Rate, mean gap, worst gap, gaps past threshold, smallest gap -- from the receiver."""
    import serial
    port = serial.Serial(BOARD, 115200, timeout=1)
    port.reset_input_buffer()
    rows, gaps = [], []
    deadline = time.time() + seconds
    while time.time() < deadline:
        line = port.readline().decode(errors="replace")
        detail = DETAIL.search(line)
        if detail:
            rows.append(tuple(int(x) for x in detail.groups()))
        gap = GAPS.search(line)
        if gap:
            gaps.append(tuple(int(x) for x in gap.groups()))
    port.close()
    # Drop the first samples: they straddle whatever was just done to the port.
    rows, gaps = rows[2:], gaps[2:]
    live = [r for r in rows if r[0] > 0]
    if not live or not gaps:
        return None
    return {
        "rate": sum(r[0] for r in live) // len(live),
        "mean": sum(r[1] for r in live) // len(live),
        "worst": max(r[2] for r in live),
        "outliers": sum(g[0] for g in gaps) // len(gaps),
        "smallest": min(g[1] for g in gaps if g[1] > 0) if any(g[1] > 0 for g in gaps) else 0,
    }


def run(cycle_ms, closed_percent, seconds):
    cycle_ns = int(cycle_ms * 1_000_000)
    closed_ns = int(cycle_ns * closed_percent / 100)
    open_ns = cycle_ns - closed_ns
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(SCHEDULE.format(cycle_ns=cycle_ns, open_ns=open_ns, closed_ns=closed_ns))
        path = f.name
    clock = switch_clock(SWITCH)
    if clock is None:
        return None, "no clock"
    if not apply_schedule(path, SWITCH, clock + 5):
        return None, "write refused"
    time.sleep(6)
    return observe(seconds), None


def restore():
    clock = switch_clock(SWITCH)
    if clock is not None:
        apply_schedule(OPEN_YAML, SWITCH, clock + 20)
    time.sleep(8)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=int, default=20)
    args = parser.parse_args()

    points = [(5, 40), (5, 60), (10, 20), (10, 50), (10, 80), (20, 50), (20, 75), (50, 50)]

    restore()
    base = observe(args.seconds)
    if not base:
        sys.exit("no stream at the receiver -- nothing to sweep")
    print(f"| cycle | TC0 closed | rate /s | mean gap | worst gap | gaps>6250 /s | smallest gap |")
    print(f"|---|---|---|---|---|---|---|")
    print(f"| — | none | {base['rate']} | {base['mean']} µs | {base['worst']} µs | "
          f"{base['outliers']} | {base['smallest']} µs |")

    results = []
    for cycle_ms, closed in points:
        seen, why = run(cycle_ms, closed, args.seconds)
        if seen is None:
            print(f"| {cycle_ms} ms | {closed}% | — | — | — | — | *{why or 'stream stopped'}* |")
            restore()
            continue
        results.append((cycle_ms, closed, seen))
        print(f"| {cycle_ms} ms | {closed}% | {seen['rate']} | {seen['mean']} µs | "
              f"{seen['worst']} µs | {seen['outliers']} | {seen['smallest']} µs |")
        restore()

    print()
    print(f"swept {len(results)} of {len(points)} points; port left open")


if __name__ == "__main__":
    main()
