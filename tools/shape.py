#!/usr/bin/env python3
"""Apply a gate schedule and decide from the traffic whether it took.

    tools/shape.py tools/tas-b-10ms.yaml --switch /dev/ttyACM2 --board /dev/ttyACM0
    tools/shape.py --off --switch /dev/ttyACM2

Why this exists rather than reading the switch. Every node this bench offers about a schedule was
misread today, each in the same direction -- as a statement about the change just made, when it
was nothing of the kind:

  out-discards         a counter since boot; its delta under a working schedule is zero
  config-change-error  also cumulative; reads 1 after a write that demonstrably worked
  oper-control-list    never populated on this device, empty while a schedule is actively shaping
  entries=N            the admin list, not the operational one

The receiver has never lied. Gaps over threshold go from none to about one per cycle when shaping
is really happening, and back when it is not. So: apply, watch the traffic, and say plainly
whether it took. If it did not, say that instead of reporting the write's return code as success.

The base time is the one thing that has to be computed rather than written down -- a schedule
cannot start in the past -- so it is read from the switch and set ahead each time.
"""
import argparse
import re
import subprocess
import sys
import tempfile
import time

CLI = "/home/kim/keti-tsn-cli-new"
OUTLIER_LINE = re.compile(r"gaps over \d+ us: (\d+)")
RATE_LINE = re.compile(r"lidar \d+ pkt (\d+)/s")


def cli(args, timeout=200):
    return subprocess.run([sys.executable and "node", f"{CLI}/bin/keti-tsn.js", *args],
                          cwd=CLI, capture_output=True, text=True, timeout=timeout)


def switch_clock(device):
    """Seconds from the switch's own clock. A base time in the past is refused."""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write("- \"/ietf-interfaces:interfaces/interface[name='1']/ieee802-dot1q-bridge:"
                "bridge-port/ieee802-dot1q-sched-bridge:gate-parameter-table\"\n")
        query = f.name
    out = cli(["fetch", query, "--transport", "serial", "-d", device]).stdout
    section = out.split("current-time", 1)
    if len(section) < 2:
        return None
    # Not "seconds" -- "nanoseconds" contains it, comes first in the output, and matching it
    # puts the base time hundreds of millions of seconds into the future, where the schedule
    # waits forever while every write reports success.
    found = re.search(r"(?<!nano)seconds:\s*(\d+)", section[1])
    return int(found.group(1)) if found else None


def watch(board, seconds):
    """What the receiver sees: packet rate, and how many gaps exceeded the threshold."""
    import serial
    port = serial.Serial(board, 115200, timeout=1)
    port.reset_input_buffer()
    rates, outliers = [], []
    deadline = time.time() + seconds
    while time.time() < deadline:
        line = port.readline().decode(errors="replace")
        rate = RATE_LINE.search(line)
        if rate:
            rates.append(int(rate.group(1)))
        outlier = OUTLIER_LINE.search(line)
        if outlier:
            outliers.append(int(outlier.group(1)))
    port.close()
    live = [r for r in rates if r > 0]
    # The first two samples of any window straddle whatever just happened to the port.
    return (min(live) if live else 0, max(live) if live else 0,
            outliers[2:] or outliers)


def describe(label, rate_low, rate_high, outliers):
    average = sum(outliers) / len(outliers) if outliers else 0
    print(f"  {label:22} {rate_low}-{rate_high}/s   gaps over threshold "
          f"{min(outliers) if outliers else 0}-{max(outliers) if outliers else 0}/s "
          f"(mean {average:.0f})")
    return average


def apply_schedule(yaml_path, switch, base_seconds):
    """Write the schedule with a base time the switch will accept."""
    text = open(yaml_path).read()
    text = re.sub(r"(admin-base-time:\s*\n\s*seconds:\s*)\d+", rf"\g<1>{base_seconds}", text)
    if "admin-base-time" not in text:
        print("  this file sets no admin-base-time; leaving it alone")
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write(text)
        staged = f.name
    result = cli(["patch", staged, "--transport", "serial", "-d", switch, "-v"])
    return "Failed: 0" in result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("schedule", nargs="?", help="a gate-parameter-table patch YAML")
    parser.add_argument("--off", action="store_true", help="restore an all-open schedule")
    parser.add_argument("--switch", default="/dev/ttyACM2")
    parser.add_argument("--board", default="/dev/ttyACM0")
    parser.add_argument("--open", default="/home/kim/esp32-lidar/tools/tas-b-open-timed.yaml")
    parser.add_argument("--seconds", type=int, default=20, help="seconds to watch per window")
    parser.add_argument("--retries", type=int, default=2)
    args = parser.parse_args()

    clock = switch_clock(args.switch)
    if clock is None:
        sys.exit("could not read the switch's clock -- is the serial device right?")
    print(f"switch clock {clock}")

    if args.off or not args.schedule:
        ok = apply_schedule(args.open, args.switch, clock + 20)
        print(f"all-open: {'written' if ok else 'FAILED'}")
        low, high, outliers = watch(args.board, args.seconds)
        describe("after opening", low, high, outliers)
        return

    print("baseline")
    low, high, outliers = watch(args.board, args.seconds)
    baseline = describe("gates open", low, high, outliers)

    for attempt in range(1, args.retries + 1):
        clock = switch_clock(args.switch) or clock
        ok = apply_schedule(args.schedule, args.switch, clock + 5)
        print(f"attempt {attempt}: write {'accepted' if ok else 'REFUSED'}, "
              f"base {clock + 5}")
        low, high, outliers = watch(args.board, args.seconds)
        shaped = describe("with the schedule", low, high, outliers)

        if low == 0:
            print("  the stream stopped -- restoring and giving up on this schedule")
            apply_schedule(args.open, args.switch, (switch_clock(args.switch) or clock) + 20)
            sys.exit(1)
        if shaped > baseline + 10:
            print(f"\n  SHAPING: gaps over threshold {baseline:.0f}/s -> {shaped:.0f}/s, "
                  f"rate unchanged at {low}-{high}/s")
            print("  leave it running, or: tools/shape.py --off")
            return
        print("  no change in the traffic -- the write returned success and did nothing")

    print("\n  gave up; restoring")
    apply_schedule(args.open, args.switch, (switch_clock(args.switch) or clock) + 20)
    sys.exit(1)


if __name__ == "__main__":
    main()
