# What a gate schedule does to this stream

Measured on the port the receiver hangs off, an OS1-16 at `512x10` sending 320 packets a second
3125 µs apart. Every number is what the receiver saw. Nothing here comes from the switch's account
of itself, which was wrong four different ways in one afternoon.

`tools/sweep.py` produced it; `tools/shape.py` is the single-shot version.

| cycle | TC0 closed | rate /s | mean gap | worst gap | gaps > 6250 µs /s | smallest gap |
|---|---|---|---|---|---|---|
| — | none | 320 | 3134 µs | 6850 µs | 0 | 260 µs |
| 5 ms | 40% | 319 | 3134 µs | 7662 µs | 19 | 186 µs |
| 5 ms | 60% | 320 | 3134 µs | 7316 µs | 18 | 186 µs |
| 10 ms | 20% | 320 | 3134 µs | 10110 µs | 51 | 185 µs |
| 10 ms | 50% | 320 | 3134 µs | 12057 µs | 99 | 186 µs |
| 10 ms | 80% | 320 | 3134 µs | 11647 µs | 100 | 185 µs |
| 20 ms | 50% | 320 | 3134 µs | 20009 µs | 50 | 185 µs |
| 20 ms | 75% | 300 | 3341 µs | 21473 µs | 50 | 186 µs |
| 50 ms | 50% | 180 | 5577 µs | 45206 µs | 20 | 186 µs |

## What the table says

**The mean does not move.** 3134 µs in every row that keeps its packets — the sensor's own spacing,
unchanged. Shaping does not alter how much arrives, only when, and a table where the mean moved
would be a table about loss rather than about shaping.

**The worst gap is the cycle.** 10 ms → 12.0, 20 ms → 20.0, 50 ms → 45.2. A packet that arrives
just as the gate shuts waits for it to open again, so the longest wait a schedule can impose is
its own period. That is the number to quote when someone asks what a schedule costs.

**The count of long gaps is one per cycle.** 100/s at 10 ms, 50/s at 20 ms, 20/s at 50 ms — 1000
divided by the cycle in milliseconds, exactly. Each cycle produces one held-and-released burst.

**The 5 ms rows are a threshold artefact, not an exception.** A 5 ms cycle with 2–3 ms closed
produces gaps of about 6 ms, which sits right on the 6250 µs threshold, so most of them are not
counted. The events are there; the ruler is too coarse for them. The same is visible in the 10 ms
20% row: a 2 ms closed window only catches the packets that arrive during it.

**The smallest gap collapses to 185 µs everywhere.** That is packets coming out back to back after
the gate opens — 185 µs is what it takes to pull one off the W5500, so it is the floor of the
instrument rather than of the wire. Its appearance is the signature that something was queued.

**Loss begins when the closed window exceeds about 10 ms.** 20 ms at 75% is 15 ms closed and drops
6% of packets; 50 ms at 50% is 25 ms closed and drops 44%. At 320 packets a second, 25 ms of
closure is eight packets that have to be held somewhere, so **this port's queue for the class is
somewhere between four and eight frames**. That is a property of the switch, obtained without
reading anything from the switch.

## How to use it

For shaping that is visible but harmless, **10 ms with half the cycle closed**: the mean is
untouched, nothing is lost, and a hundred clear events a second make it obvious on a graph.

For demonstrating the cost of a badly chosen schedule, **50 ms at 50%**: 44% of the stream gone,
which is the honest answer to "what happens if the cycle is much longer than the traffic's own
spacing".

Both need the base time computed from the switch's clock at the moment of writing. A constant in a
YAML file goes stale as the clock moves, and a schedule that starts in the past is accepted and
never runs — which looks exactly like a switch that ignores you.
