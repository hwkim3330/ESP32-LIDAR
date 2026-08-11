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

## TAS: measured, and it showed nothing (2026-08-11)

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

## Not done yet

- The page has not been opened in a browser — the firmware serves it, and it has been rendered
  against mock data, but no device has joined the AP to look at the real thing.
- Three hops, and what TAS does to these numbers. The gap distribution above is the baseline to
  compare against.
- **Repeat the TAS run** now that arrivals are resolved to a packet rather than to 10 ms.
- **The remaining 6.5 ms worst gap**, down from 11.5. Whether that is the sensor, the switch or
  still the 1 ms poll floor is not yet established.
- **Schedule writes from the board**, as one atomic container patch. The point of this rig is to
  need no PC, and sensor configuration already needs none; the switch side is what is left.
- **A second LAN9662**, for three hops.
