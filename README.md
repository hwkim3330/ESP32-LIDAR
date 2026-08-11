# esp32-lidar

An ESP32-S3 with a W5500 watching an Ouster OS1-16 across two LAN9662s, and putting the room on a
tablet over BLE.

```
LiDAR --1G--> [A .10] p2 => p1 --1G--> [B .11] p2 => p1 --100M--> ESP32-S3 --WiFi--> the page
```

Sibling to [keti-reconfig](https://github.com/hwkim3330/keti-reconfig) — same boards, same bench,
same W5500 pinout. That project drives the switch; this one measures what crosses it.

## What works (measured 2026-08-10)

The sensor streams into the board and the numbers agree with theory:

| | measured | expected |
|---|---|---|
| packet rate | **320/s**, steady, no loss | 512 columns ÷ 16 per packet × 10 Hz |
| gap between packets | **3134 µs** mean | 3125 µs |
| payload | **1280 B** | RNG15_RFL8_NIR8, 16 beams |
| IMU | **100/s** | 100 Hz |
| bandwidth | **3.3 Mbit/s** | inside what a W5500 can carry |

## The two walls, and why the switch is not optional

**The OS1 is gigabit-only.** Ouster's hardware manual is explicit: the sensor stops sending and
raises an error unless the link comes up at 1000 Mb/s full duplex. A W5500 is 10/100. So the
sensor cannot be wired to this board directly — not for bandwidth reasons, at the physical layer.

**Putting the LAN9662 in the path solves it.** The sensor gets its gigabit link on its own switch
port, the ESP gets 100M on its, and the switch bridges the two. This is confirmed, not reasoned:
with the switch in the path the ESP-side link reports `100M full` and the stream arrives anyway.

**The second wall is SPI.** The default profile on an OS1-16 is 3328 B every 781.25 µs — about
34 Mbit/s, which the SPI link to the W5500 cannot carry even though 100BASE-TX could. `512x10`
with the low-data-rate profile is a tenth of that. Both are ordinary sensor modes; this is
choosing an operating point the link can serve, not a workaround.

## Finding the sensor with no spare NIC

The PC on this bench has one NIC, committed to the office network, and no WiFi. So the board does
it alone:

1. **It listens for DHCP.** A powered Ouster broadcasts DISCOVER whether or not anyone answers,
   and DISCOVER carries its MAC and hostname. That is how this sensor identified itself:
   `BC:0F:A7:00:17:02 -> os1-122018000001` (`BC:0F:A7` is Ouster's OUI).
2. **It answers.** A one-lease DHCP server puts the sensor at a known `192.168.1.50` instead of a
   link-local address nobody knows.
3. **It asks the sensor about itself** over Ouster's HTTP API:
   `OS-1-16-A0`, firmware `v2.4.0`, serial `122018000001`.
4. **It points the sensor at itself** — `udp_dest`, mode and profile in one POST.

## The sensor's config does not persist, so the board remembers it

Power-cycle an OS1 and `udp_dest` is gone — the HTTP config is applied, not saved. This bench
gets power-cycled, so `c` and `C` write the request to NVS, and if a stream was asked for and
nothing has arrived for twenty seconds the board asks again, at most every thirty. It re-applies
only what was asked for; it never decides on its own that the sensor should be streaming.

## The morning it would not start (2026-08-11, resolved by a power cycle)

It ran on 2026-08-10 and would not the next morning. What it said about itself:

```
/api/v1/system/network  -> carrier true, duplex full, speed 1000
/api/v1/sensor/config   -> udp_dest 192.168.1.20, 512x10, RNG15_RFL8_NIR8   (as set)
/api/v1/sensor/metadata/sensor_info -> status INITIALIZING, initialization_id changing
                                       between consecutive requests
/api/v1/sensor/alerts   -> 0x0100002a STARTUP/WARNING, over and over:
                           "Unit has experienced an internal warning during startup
                            and is restarting."
```

Every entry in the log is that one alert, on a **12.6 second period** (16.2 s, 28.8, 41.4, 54.0,
66.7, 79.3, 92.0, 104.6, 117.2, 129.9, 142.5, 155.1, 167.8, 180.4). The unit reruns DHCP each
time, so the whole sensor is restarting, not just reinitializing. Its HTTP server drops out
mid-request as it goes.

So this was not a network fault: the link was gigabit, the configuration was right, and the board
got `HTTP 204` for every request. Ouster's first suspect for this alert is **power** — an OS1
draws 14–20 W and surges at motor spin-up, and a marginal supply produces exactly this. The
cabling had been disturbed between the working run and this one.

**A power cycle fixed it**, which is the same diagnosis arriving from the other direction:
nothing on the network was touched. The stream came back four seconds later without anyone
configuring anything — the remembered request did it — and then ran three minutes at
**320.5 packets/s with no loss and a 3134 µs mean gap**, identical to the day before. Worth
keeping in mind next time the sensor is silent: ask it, and check the alert log before the wiring.

## The room, on the tablet, over BLE (2026-08-11)

A page of timing statistics is not what a LiDAR is for. `app/` is an Android app that shows the
point cloud, and the link to it is BLE — so the tablet keeps whatever WiFi it is on. The office
network is where it belongs and this board has no credentials for it; BLE asks nothing of either.

**The packet layout was inferred from its size, and it lands exactly.** 32 byte packet header,
then 16 columns of (12 byte column header + 16 pixels x 4 bytes), then a 32 byte footer = 1280,
which is the payload this sensor sends in `RNG15_RFL8_NIR8`. Each pixel is a little-endian
uint32: range in bits 0..14 in 8 mm units, reflectivity 16..23, near-infrared 24..31. The proof
is that the numbers are a room:

```
column 256 (status 1):  5.50  6.38  7.68  0.00  7.56  3.38  3.38  3.41 ... 2.85  0.00 m
```

**The bandwidth decides the design.** 512 columns x 16 beams at 10 Hz is 8192 points, 164 kB/s as
raw ranges; BLE on this part does perhaps 20-40. So: one frame a second, every other column kept,
range as a uint16 in centimetres — 8 kB a frame, which arrives whole. Notifications are
unacknowledged and that is deliberate: a frame with a hole in it is still a picture of the room,
a stream stalled waiting for a retransmit is not.

**Geometry comes from the sensor, not from a datasheet.** Beam altitude angles (+14.18° to
−17.21° on this unit) and azimuth offsets are per-unit calibration; the board fetches
`beam_intrinsics` once and offers it as a readable characteristic, and the app places points with
it. Ranges are converted on the tablet rather than on the board — three floats where one short
would do is six times the bytes over the link that is actually scarce.

Measured on the tablet: `frame 194 · 4096/4096 points`. Full frames.

What the app shows, and why each piece is there:

- **The cloud**, coloured by distance, on a 1 m grid with **range rings at 2, 5 and 10 m**. A grid
  says there is a scale; a ring says what it is, and distance is what every point is measured in.
- **A legend with the ramp's actual ends** — 1.2 m to 7.5 m on a typical frame. Colour without a
  key is decoration.
- **Wire statistics from the board**: packets per second, mean gap, worst gap, link speed, each
  with two minutes of history. These come from the board over BLE, so the tablet sees what the
  Ethernet side sees without being on it.
- **The sensor's IMU**, acceleration and angular rate. It reads +1.00 g on z, which is the sensor
  telling you it is level.
- No title. The room is on the screen; naming it adds nothing.

Double-tap puts the camera back — orbiting a cloud is easy to get lost in.

**The flicker was the app clearing its buffer.** Every new sequence number blanked the frame and
refilled it, so a single lost notification took those columns off the screen for a second and
brought them back — once a second, forever. The room is still; the only thing moving was the
loss. The buffer is not cleared now: a column that did not arrive keeps its last value, which
says something true about a room that has not changed, where blanking it said something false.
The board also paces notifications at 12 ms rather than 6, since Android negotiates a connection
interval in the tens of milliseconds and drops what is queued faster than it can carry.

```
LiDAR --1G--> [A .10] --1G--> [B .11] --100M--> ESP32-S3 --BLE--> tablet (still on office WiFi)
```

## Three hops (2026-08-11)

A second LAN9662 went into the path, and the board reads both of them:

```
=== 192.168.1.10  A -- the sensor's switch ===
  port  link  speed        in-octets     out-octets  in-disc out-disc
  1     up    1000M            80142     4261072430        0        0   -> B
  2     up    1000M       4287978186          88481        0        0   <- LiDAR
=== 192.168.1.11  B -- this board's switch ===
  1     up    100M              3277      216461846        0        0   -> this board
  2     up    1000M        216476860           5261        0        0   <- A
```

**Both boards arrived on 192.168.1.10.** Their L3V1 MACs differ, so it is not a MAC clash, but
two devices answering for one address on one segment means whichever replies first wins the ARP
cache — the board would read counters from an arbitrary switch while believing it knew which.
That is a fault shaped like intermittence rather than like breakage. B moved to `.11` over
serial (`tools/switch-b-ip.yaml`); adding a second address is refused outright, so the old one
has to be deleted first, which CORECONF does with a null value.

The extra hop costs nothing measurable:

| | two hops | three hops |
|---|---|---|
| rate | 320/s | 319-321/s |
| mean gap | 3134 us | 3134 us |
| worst gap | 4.2-4.9 ms | 5.9 ms |
| gaps over 6250 us | 0 | 0 in 55 s |
| discards, every port | 0 | 0 |

Which is the expected answer at 3% utilisation with nothing competing — and now it is measured
rather than assumed. The interesting version of this question needs traffic worth queueing.

## The stream this board cannot survive (2026-08-11)

Loading the path to make queueing visible was the obvious next experiment, and it went badly in
a way worth keeping. `1024x10` with the full profile is 640 datagrams a second of 3392 bytes,
which fragment into roughly 2000 frames. What happened:

- **The switches delivered all of it.** B forwarded 3 GB with `out-discards: 0`, and the sensor's
  port counted 2044 packets a second going in. Nothing in the network dropped anything.
- **The board reassembled none of it.** `lidar 0 pkt 0/s` while the wire was saturated: fragments
  arrive faster than they can be pulled off the W5500, so no datagram ever completes.
- **It could not take the request back.** Every path to the sensor crosses the same flood, so the
  message that would slow it down could not get out. Priming the console with the command and
  raising the port at the same instant did not win the race either.

Two things had to be true for that to be unrecoverable, and both are now fixed:

**The drain loop was unbounded.** `while (parsePacket() > 0)` exits when the socket runs dry — so
when the sender outruns the reader it never exits, and nothing else in `loop()` runs: no
housekeeping, no console, no way to ask for help. It now reads at most eight packets per call and
says `OVERRUN` when it hits that, because hitting it means the arrival times being recorded have
already stopped meaning what they say.

**`C` is refused.** A board that can ask for a stream it cannot survive, and cannot then withdraw
the request, is the bug. Re-enable it alongside something that makes it recoverable — a rate the
link carries, or a control path that does not share it.

Getting out took the switch: `tools/port2-down.yaml` shuts the sensor's ingress over serial,
which is out of band from the flood. That stopped it long enough for the board to breathe, and in
the end the sensor was power-cycled — its configuration is not persistent, so a power cycle is
also the way to be certain a stream has stopped.

## The sensor gave itself an address nobody could answer (2026-08-11)

The stream stopped and the board could not get it back. A tap between the sensor and switch A
answered it in one capture:

```
169.254.195.68  ->  192.168.1.20 : 7502     639 datagrams/s, fragmenting into ~2000 frames
169.254.195.68  ->  192.168.1.20 : 7503     IMU
```

The sensor had **fallen back to link-local**. Its DHCP request went unanswered — this board is
the DHCP server, and it was drowning in the sensor's own full-rate stream at the time — so it
assigned itself 169.254.195.68 and stopped asking. It does not retry, not even across a link
flap. Meanwhile `udp_dest` still said 192.168.1.20, so its packets kept arriving here while
nothing sent from here could ever reach it: a reply to 169.254.195.68 has no route from a
192.168.1.20/24 interface. Not a dead sensor. Two addresses that cannot see each other.

The way out was to **move this board onto the sensor's network for as long as it took to talk**
(`L`, and `M` to come back). That fixes both halves at once, and the second half is the one that
matters: with the board no longer holding 192.168.1.20, the flood addressed to it is dropped
early instead of being reassembled, which is what made the board responsive enough to act at all.

Then the real fix, so none of it can recur:

```
PUT /api/v1/system/network/ipv4/override   <- "192.168.1.50/24"     200
POST /api/v1/sensor/config                 <- 512x10, low rate       204
```

**The sensor has a fixed address now.** DHCP is out of the path. `n` and `N` do those two.

One correction to an earlier claim here: bounding the drain does **not** protect the board from a
stream it cannot carry. Reception and reassembly happen in the driver and lwIP tasks, above
`loop()` in priority, so the board goes silent no matter how politely its own read loop yields.
The bound is still right — it is just not armour. Not being the destination is armour.

## The board talks to both ends

The switch is asked, not assumed. One CoAP request does it, and it is the only one that can come
first: the YANG catalog checksum is the single node whose SID is knowable without already having
that catalog's SID table (29304, fixed). The bench has carried both a LAN9662 and a LAN9692, and
the answer says which is here:

```
s  ->  5151bae07677b1501f9cf52637f2a38f  ->  LAN9662 (54 YANG / 54 SID)
```

Everything past that — port counters, gate schedules — needs a SID table generated for *that*
catalog, so nothing else honestly comes first. keti-reconfig's table is for the LAN9692 and does
not apply here; `tools/gen_sid_table` builds one for the 9662 from the catalog already cached in
keti-tsn-cli, without touching the device.

With that table, `S` reads the whole interface subtree in one request (1429 bytes over six
Block2 blocks) and the switch confirms, as a second witness, what the sensor claimed:

```
port     link   speed         in-octets     out-octets  in-disc out-disc
1        up     100M              33654      475373573        0        0   <- the ESP
2        up     1000M         479652843          31540        0        0   <- the LiDAR
L3V1     up     -                 20456           2210        0        0
TAS on both ports: gate-enabled 0, cycle 0/1, no entries
```

**The speed conversion is right there**: 1000M on the sensor's port, 100M on the board's, and
`in-discards` and `out-discards` zero on both. Nothing is being dropped by the switch, which is
what makes the ESP's steady 320/s meaningful — the losslessness is now attributable rather than
assumed. And no gate schedule is configured anywhere, so every number measured so far is the
unshaped baseline.

Per-port detail is parsed out of the subtree because this device answers a **keyed instance
query with 4.00 over Ethernet CoAP** (the same request works over serial). It costs nothing:
the gate parameters ride inside the subtree already.

## The UI rides WiFi

The measurement path is Ethernet; the page is served over the S3's own soft AP, so a phone,
tablet or laptop can see it without anything being on the 192.168.1.0/24 segment.

- SSID `KETI-LIDAR`, password `ketilidar` → **http://192.168.4.1/**
- Also at `http://192.168.1.20/` for anything already on the wired segment.

The page shows link speed and packet count with equal prominence, on purpose: a beautiful jitter
histogram of zero packets has answered nothing. Below that, packets per second over two minutes,
the gap between consecutive packets, and the distribution of those gaps.

## Serial console

Asking the sensor for a stream puts it into a different operating mode, so it is a keypress
rather than something the board does on its own.

| key | |
|---|---|
| `i` | ask the sensor what it is |
| `g<path>` | GET any path on the sensor, printed whole — `g/api/v1/sensor/alerts` |
| `s` | ask the switch for its catalog checksum |
| `S` | read every switch port: link, speed, counters, gate schedule |
| `c` | 512x10, low data rate, `udp_dest` → this board |
| `C` | 1024x10, full profile — 34 Mbit/s, expect loss |
| `r` | clear counters |

`g` exists because the useful questions were not the ones the firmware had been built to ask.
When the sensor stopped coming up, its own alert log said in one line what another day of
guessing from the outside would not have.

## Hardware

| | |
|---|---|
| board | ESP32-S3, 16MB flash, 8MB octal PSRAM, MAC `94:A9:90:CF:A5:38` |
| W5500 | SCK **48**, MOSI **21**, MISO **47**, CS **45** |
| sensor | Ouster OS-1-16-A0, FW v2.4.0, `192.168.1.50` |
| switch | LAN9662, between the two |

The pinout came from keti-reconfig's exhaustive bit-banged search, not a datasheet — seven
published pinouts for this board were all wrong. It is confirmed on this board too: VERSIONR
reads `0x04` at boot.

**GPIO19/20 are native USB** and **GPIO33–37 are the octal PSRAM bus** on this part. Neither is
available for wiring.

## Flashing

```sh
tools/flash.sh /dev/ttyACM0
```

Board options are in the script rather than the README because getting `PSRAM=opi` wrong produces
a board that flashes and then misbehaves instead of one that fails loudly.

## Watch out

**Nothing may touch the SPI bus once ETH is running.** `FSPI` and `SPI2_HOST` are the same
peripheral, so a raw register read taken while the driver is up blocks its `esp_timer` callback,
starves IDLE0 and reboots the board on the task watchdog about fifteen seconds in. The first
build of this firmware did exactly that. Raw access is confined to the boot-time pinout check,
and the bus is handed over afterwards.

## The 11 ms gap was Arduino's polling interval (2026-08-11, fixed)

The instrument was wrong, and for a while so were the theories about it. What it turned out to
be: **this board's W5500 has no interrupt line**, and with no interrupt the driver polls — at a
period Arduino hardcodes to ten milliseconds:

```
libraries/Ethernet/src/ETH.cpp:677    if (_pin_irq < 0) { mac_config.poll_period_ms = 10; }
```

So packets were handed over in batches. At 320 a second that is groups of three or four, each
171 us apart — the cost of pulling one frame off the chip over SPI — separated by ten
millisecond holes. **Exactly 100 holes a second**, which is what finally gave it away: one per
poll, not one per second. Every arrival time this rig had recorded was the moment a poll
collected the packet, not the moment it landed.

Three wrong theories died on the way, each cheap to kill once it was stated as a number:

- *The once-a-second housekeeping.* Timed it: 340 us. Not it.
- *The WiFi AP stealing the core.* Turned it off and measured: identical. Not it.
- *`server.handleClient()`, which really does cost 4-5 ms with no client connected.* Moving the
  reader into its own task made things **worse** — the smallest sleep a task can take is one
  tick, and while the tick here is 1 kHz, the packet reader then sat behind the same 10 ms
  batches with an extra layer on top. The web server is the thing that gets to sleep; the reader
  keeps `loop()`, which never does.

The fix is `firmware/lidar_probe/eth_w5500.h`: bring the W5500 up directly on esp_eth with
`poll_period_ms = 1` instead of going through Arduino's ETH class, and attach it to esp_netif
the same way ETH.begin would, so lwIP, NetworkUDP, WebServer and the soft AP never notice.
Two things it needs that ETH.begin was quietly doing: `esp_netif_init()` before any socket
exists — without it the first `udp.begin()` asserts on a null queue and the board boot-loops —
and the WiFi AP started first, since bringing the radio up initialises the same machinery.

| | before | after |
|---|---|---|
| gaps over 6250 us, per second | **100** | **0-1** |
| smallest gap | 171 us, identical every second | 1650-2550 us, and it varies |
| worst gap | 11.5 ms | 6.5 ms |
| packets per read | 4 | 1 |

A fixed minimum gap was the tell in hindsight: real traffic does not arrive the same distance
apart to the microsecond, second after second. That number was never the network. It was how
long this board takes to read one packet.

## TAS, seen (2026-08-11)

Gating **switch B port 1** — the last hop, the port the board actually receives through, because
gates are on egress — with a 10 ms cycle: 5 ms every class open, 5 ms with TC0 closed.

| | gates open | TC0 closed 5 ms in 10 |
|---|---|---|
| rate | 319-321/s | 319-323/s |
| mean gap | 3134 µs | 3134 µs |
| worst gap | 8.0 ms | **12.4 ms** |
| **gaps over 6250 µs, per second** | **0-1** | **96-101** |
| smallest gap | 293 µs | **187 µs** |

That is shaping, and the shape of the evidence is the point: **the rate and the mean do not
move** — nothing is lost, the sensor still sends 320 a second — while the distribution is
transformed. About a hundred long gaps a second is one per 10 ms cycle, and the packets held
during the closed window come out back to back at 187 µs. Average unchanged, arrival pattern
rewritten.

The earlier attempt saw nothing because its cycle was 1 ms with 200 µs closed, finer than the
3125 µs spacing it was trying to shape: nothing ever queued. **Shaping is only visible when the
window is coarser than the arrivals.**

### The tap says how much of the jitter is the network and how much is this board

Both measured on the same stream at the same time — the tap sits between the sensor and switch A,
the board sits two switches later:

| | tap, at the sensor | board, after two hops and a W5500 |
|---|---|---|
| rate | 320.0/s | 319-321/s |
| mean gap | **3125 µs** (the nominal figure exactly) | 3134 µs |
| standard deviation | **83 µs** | — |
| p99 | **3433 µs** | ~5000 µs |
| worst | **4053 µs** | 6913 µs |
| gaps over 6250 µs | **0 in 25 s** | 0-3 per second |

Nothing is lost anywhere: 320.0/s at both ends across two switches. But **the sensor is far more
regular than this board can measure** — the spread roughly doubles by the time it is timestamped
here, and that extra is the 1 ms poll, the SPI read and the scheduler, not the network.

So: the board's counts and rates are trustworthy, and for microsecond jitter the tap is the
reference. It matters for what can be measured of TAS — a 12 ms effect is plain in the board's
numbers, a 200 µs one is only visible on the tap.

### A table of what schedules actually do — `docs/tas-sweep.md`

Eight schedules, applied and measured at the receiver, and the physics comes out clean:

- **The mean never moves** — 3134 µs in every row that keeps its packets. Shaping changes when,
  not how much.
- **The worst gap is the cycle**: 10 ms → 12.0, 20 ms → 20.0, 50 ms → 45.2. The longest wait a
  schedule can impose is its own period.
- **Long gaps come one per cycle**: 100/s at 10 ms, 50/s at 20 ms, 20/s at 50 ms — 1000 over the
  cycle in milliseconds, exactly.
- **Loss begins when the closed window passes about 10 ms.** 15 ms closed drops 6%, 25 ms drops
  44%, which puts **this port's queue for the class at four to eight frames** — a property of the
  switch measured without reading anything from the switch.

`tools/sweep.py` runs it; `docs/tas-sweep.md` has the table and the reading.

### Decide from the traffic — `tools/shape.py`

The switch's account of itself was misread four times in one day, always the same way: as a
statement about the change just made, when it was nothing of the kind.

| read as | actually |
|---|---|
| `out-discards` | a **counter since boot**; its delta under a working schedule is **zero** |
| `config-change-error` | also **cumulative** — reads 1 after a write that demonstrably worked |
| `oper-control-list` | **never populated on this device**, empty while a schedule is actively shaping |
| `entries=N` from the board | the **admin** list, not the operational one |

The receiver never lied. So `tools/shape.py` applies a schedule and then decides from the traffic:

```
switch clock 11256
  gates open             320-320/s   gaps over threshold 0-1/s (mean 0)
attempt 1: write accepted, base 11280
  with the schedule      318-321/s   gaps over threshold 0-100/s (mean 82)

  SHAPING: gaps over threshold 0/s -> 82/s, rate unchanged at 318-321/s
```

It reads the switch's clock, sets the base a few seconds ahead, writes, watches, and says whether
anything happened — retrying with a fresh base, and restoring an all-open schedule if the stream
stops or nothing changes.

**Building it explained the inconsistency it was built to survive.** The same file had shaped at
96–101 outliers a second in the morning and done nothing an hour later; the difference was the
base time written into the YAML, which was a constant while the switch's clock kept moving. It
was never device flakiness. Computed fresh each time, shaping takes on the first attempt.

It also caught a bug in itself worth keeping: matching `seconds:` in the switch's output finds
**`nanoseconds:`** first, which put the base three hundred million seconds into the future, where
a schedule waits forever while every write reports success. The harness reported honestly that
nothing had happened, twice, instead of trusting the return code — which is the whole point of it.

### The nodes themselves

There is no working explanation for why a schedule written from the board stops the stream while
the identical bytes from the CLI shape it. What there is instead is a list of the signals that
looked like explanations and were not — and that list is the useful part, because every one of
them was read the same wrong way.

| read as | actually |
|---|---|
| `out-discards` | a **counter since boot**. 56103 looked like the mechanism; the delta while a working schedule ran is **zero over 25 s** |
| `config-change-error` | also **cumulative**. It reads **1 after a write that demonstrably worked** |
| `oper-control-list` | **never populated on this device** — empty while a schedule is actively shaping |
| `entries=2` from the board | the **admin** list, not the oper one |

So the claim in earlier commits — that the board's write lands in admin and is not adopted — rests
on a node that is empty even on success. **It is not supported.** The stream stopping after the
board writes is real and reproducible; the reason is not known.

Worse for tidiness, the CLI is not reliably reproducible either. The same file that produced
96–101 outliers a second earlier in the day produced **0–1** an hour later, with the switch
reporting `oper-cycle-time` of 10 ms in both cases. Whatever decides whether a schedule actually
takes hold is not visible in any node read so far.

**The one trustworthy instrument is the receiver.** Gaps over threshold at the board went from
0–1 to 96–101 per second when shaping was really happening, and that measurement has never lied.
Configuration read back from the switch has, four times.

Next time: apply, then decide from the traffic whether it took, and only then look at the
switch's own account of itself.

### `admin-base-time: 0` is why a schedule will not go away

Restoring all-open afterwards appeared to work — the write returned success and the admin list
read back as a single 255 window — and the stream stayed dead. The table said why:

```
config-change-error: 1        <- the change was refused
oper-gate-states:    254      <- TC0 still closed, old schedule still running
oper cycle numerator: 10000000
```

A base time of zero is in the past and a schedule cannot start in the past, so the switch takes
the new admin configuration and declines to adopt it. Read `current-time` from the same table,
put `admin-base-time` a few seconds ahead, and it is accepted: `config-change-error: 0`,
`oper-gate-states: 255`, stream back at 320/s. `tools/tas-b-open-timed.yaml`.

Two habits come out of this. **Check `oper-gate-states`, not the write's return code** — the
admin list is what was asked for, oper is what the port is doing. And **`gate-enabled: false` is
not a way out**: it was accepted, cleared the error, and left the port still closed to TC0.

## The earlier TAS attempt, which showed nothing (2026-08-11)

A 1 ms cycle with 800 us all-open and 200 us of TC0 closed, written to port 1, against the
sensor's stream:

| | mean gap | rate | worst gap |
|---|---|---|---|
| gates open | 3134 us | 318-321/s | 14.1 ms |
| TC0 closed 200 us in 1 ms | 3134 us | 319-321/s | 12.0 ms |
| open again | 3134 us | 320-323/s | 12.1 ms |

Not a failure of TAS — this rig could not see it. Packets arrive 3125 us apart on a link running
at 3% utilisation, so when the gate shuts there is nothing queued to delay. And at the time the
measurement itself was quantised to 10 ms (above), sixty times the window being tested. Worth
repeating now that the instrument resolves single packets, with a cycle long enough to matter:
the shaping has to be coarser than the arrival spacing, or the link busy enough to queue.

## Writing a schedule from the board is disabled, and why

The first attempt wrote the schedule as five patches — control list, numerator, denominator,
gate-enabled, config-change. The list was rejected and the rest were accepted, so the switch
gated against no valid schedule and shut port 1: the stream stopped, and so did the only path
this board had to withdraw its own mistake. `gate-enabled: false` over serial did not bring it
back; an all-open schedule did, and the ESP still needed a reboot afterwards because the W5500
stack stayed wedged.

Three things came out of it:

- **Write the whole container in one iPATCH.** `keti-tsn-cli` does, so it either lands or it does
  not. Five patches have a half-applied state and that state is a trap.
- **Turning gating off is an all-open schedule, not `gate-enabled: false`.** Only the first
  reliably restored traffic. `tools/tas-open.yaml`.
- **Out-of-band beats clever.** The serial console cannot be cut by anything written over
  Ethernet. `tools/tas-tc0-200us.yaml` and `tools/tas-open.yaml` run from the PC.

The firmware keeps the presets and refuses to send them, with the reason next to the code.
Re-enabling it means the atomic container write and two more SIDs (`admin-base-time` and its
leaves, `admin-gate-states`) — worth doing, since the point of this board is to need no PC.

## Contention needs a second board, not a bigger idea (2026-08-11)

The interesting TAS question is not "can a gate delay packets" — that is measured above — but
"does gating protect one stream from another". That needs two streams competing for one egress
port.

This README previously said the boards could not supply that competition. **That was wrong, and
wrong in a way worth writing down**: it is true that *one* board cannot, because gates are on
egress and a switch never returns a frame out the port it arrived on, so nothing a board sends
can compete with what that same board receives. From there it concluded the whole thing was
blocked, which does not follow. **A second board on a second port sends to the first, and that
contends exactly where it should.** The constraint was never the boards — it was the two data
ports on a LAN9662, which can only make a chain, and a chain has one source.

So the arrangement is a twelve-port LAN9692 and three boards:

```
LiDAR      --1G---> [ 9692 ]
generator  --100M-> [      ] --100M--> receiver        gate this egress port
second gen --100M-> [      ]
```

### The board generates too, from the spare core

`l5000:3` on the console sends tagged frames on PCP 3 from core 0 while core 1 keeps receiving.
Measured: **400 frames/s of 1200 bytes, 3.84 Mbit/s, steady**, and throughout it the sensor's
stream stays at 319–321/s with no extra outliers. Generating costs the measurement nothing, which
is the only reason it is allowed to share the board.

Asking for more does not get more, so the question became how much the chip can be made to give.

| SPI clock | frame | achieved | per frame |
|---|---|---|---|
| 20 MHz | 1200 B | 3.84 Mbit/s | 2400 µs |
| 40 MHz | 1200 B | 5.95 Mbit/s | 1613 µs |
| **40 MHz** | **1500 B** | **8.34 Mbit/s** | **1438 µs** |
| 60 MHz | 1500 B | 7.87 Mbit/s | — |
| 80 MHz | — | link dead | — |

Two and a bit times the starting figure. The last row is where this board's wiring gives up — the
part is rated to 80 MHz but these traces are not — and **the row above it is the interesting
one: 60 MHz performs the same as 40**, 656 frames a second against 653. Past 40 the clock stops
buying anything, which is the quantitative version of the paragraph below.

Stability was measured rather than assumed, with the switch's own FCS counter as the judge: three
minutes of load at 40 MHz, **117,562 frames, not one corrupt** (`in-error-fcs-frames +0`,
undersize +0), and the sensor never dropping below 315/s in 179 samples. So 40 MHz is the
setting: it reaches the ceiling while sitting furthest from where the wiring fails.

**The remaining wall is not the clock.** At 40 MHz a 1500 byte frame is 300 µs of SPI, and each
one costs 1438 — so 1138 µs per frame is the driver waiting for the transmit to complete. Raising
the clock cannot touch that, which is why doubling it bought 55% rather than 100%.

**So a W5500 will not give 100 Mbit/s and nothing will make it.** The link negotiates 100BASE-TX;
the host interface is SPI, and the chip cannot be fed fast enough to fill the wire it is attached
to. The reason there is a W5500 here at all is that **the ESP32-S3 has no Ethernet MAC** — the
original ESP32 and the P4 do, with RMII to a PHY, and those reach real 100 Mbit/s. On this part
the ceiling is about 8 Mbit/s out, 11.6 through the chip with the sensor arriving at the same
time, and the console reports what went out rather than what was asked for.

Four megabits will not congest a 100 Mbit/s port, and it does not need to. **Congestion and
discrimination are different demonstrations**: the first needs a full port and the PC, the second
needs two priorities and this is enough for it — gate PCP 3 and watch PCP 0 come through beside
it untouched.

**For congestion, the generator should be the PC, not a board.** A W5500 runs out of SPI long before it runs out
of PHY — perhaps 10 to 20 Mbit/s — so filling a 100 Mbit/s receiver port would take five or six
boards. The PC's adapter does about 95 on its own. `tools/load.sh` puts it on a VLAN with a chosen
PCP and sends at the receiver on a port the receiver will not mistake for sensor data. Boards can
be extra sources if more than one priority is wanted.

Sending the load on a **different priority from the sensor's** is the whole point. Gate that
priority and the sensor should come through untouched while the load is shaped — which is what
TAS is for, and what cannot be shown while the only traffic on the wire is the traffic you care
about.

**Forcing the receiver's link to 10 Mbit/s was tried and abandoned.** The idea was to make the
port contended without a fast generator: at 10M the sensor's own 3.3 Mbit/s is already a third of
it. Writing PHYCFGR before the driver starts does nothing — the driver resets the chip during
init — and the driver's own `ETH_CMD_S_SPEED` did not take either; the link came back up
negotiated at 100M. It would not have been worth much anyway: 100M is a real TSN edge speed and
10M is not, so shaping demonstrated on it would be a demonstration about a toy link. The firmware
keeps the toggle (`1` on the console) and the bench runs negotiated.

## What these switches can actually do (2026-08-11)

Asked the catalogs rather than guessed, and then asked the device rather than the catalog.

**RedBox is out.** `mchp-velocitysp-redbox` is modelled — modes `prp-san`, `hsr-san`, `hsr-prp`,
`hsr-hsr`, node tables, supervision frames, the lot — and it exists **only in the LAN9662's
catalog**; the LAN9692 build does not carry the module at all. And the 9662 that does carry it
says:

```
f39202  ->  BF 19 9922 9F FF FF      redbox-capable-list = []
```

**No port on it is RedBox-capable.** A model in the catalog is a description of the software, not
a promise about the silicon in front of you. Not worth a day.

**Frame replication is out too.** Neither catalog has `ieee802-dot1cb-frer`; both have
`ieee802-dot1cb-stream-identification`, which identifies streams without duplicating them. So a
switch here cannot be a traffic source under any configuration.

**Still unused and available on both:** `ieee802-dot1q-psfp` with `stream-filters-gates` — gates
and meters per *stream* rather than per port, which is a sharper instrument than TAS for
protecting one flow — and `ieee802-dot1q-preemption`, whose effect on a 100M port carrying 1280
byte frames is large enough to measure. The 9662 also has `mchp-velocitysp-acl`.

Otherwise the two catalogs are the same set: 54 modules against 53, and the only difference in
either direction is that one RedBox module.

`f<sid>` on the board's console fetches any SID from the switch and prints the CBOR. The CLI
cannot resolve every module's paths — the RedBox ones among them — and a SID is just a number, so
the board can ask for things the tool on the PC cannot.

## Tomorrow: the LAN9692

`docs/9692.md` is the runbook — arrangement, order of operations, and every trap this bench has
already paid for once.

The firmware is ready for it: it carries **both catalogs' SID tables** and picks between them from
the checksum the device reports about itself, because a table used against the wrong catalog
addresses the wrong nodes and returns plausible nonsense. `s` on the console says which it chose.

The 9662s do not become spare. **They are the fault injector** — a switch cannot generate traffic
(neither catalog carries frame replication; `ieee802-dot1cb-frer` is absent and only stream
identification is there), but it makes a remotely-cut cable, which is what RECON's link-failure
case needs. `tools/port2-down.yaml` and `port2-up.yaml` already do it.

Two capabilities turned up in the catalogs while checking that, both present on both parts and
neither used yet: **PSFP** (`ieee802-dot1q-psfp`, per-stream gates and meters rather than
per-port) and **frame preemption** (`ieee802-dot1q-preemption`), which on a 100M port with 1280
byte frames is a large enough effect to see.

## Not done yet

- The page has not been opened in a browser — the firmware serves it, and it has been rendered
  against mock data, but no device has joined the AP to look at the real thing.
- Three hops, and what TAS does to these numbers. The gap distribution above is the baseline to
  compare against.
- **Why a schedule written over Ethernet CoAP lands with an empty operational list**, when the
  identical bytes over serial do not. Everything else about TAS is understood; this is not.
- **The LAN9692**, which would end the contention problem: twelve ports instead of two.
- **The remaining 6.5 ms worst gap**, down from 11.5. Whether that is the sensor, the switch or
  still the 1 ms poll floor is not yet established.
- **Schedule writes from the board**, as one atomic container patch. The point of this rig is to
  need no PC, and sensor configuration already needs none; the switch side is what is left.
- **A second LAN9662**, for three hops.
