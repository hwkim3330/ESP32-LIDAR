# esp32-lidar

An ESP32-S3 with a W5500 watching an Ouster OS1-16 through a LAN9662, and serving what it sees.

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

## TAS: measured, and it showed nothing (2026-08-11)

A 1 ms cycle with 800 us all-open and 200 us of TC0 closed, written to port 1, against the
sensor's stream:

| | mean gap | rate | worst gap |
|---|---|---|---|
| gates open | 3134 us | 318-321/s | 14.1 ms |
| TC0 closed 200 us in 1 ms | 3134 us | 319-321/s | 12.0 ms |
| open again | 3134 us | 320-323/s | 12.1 ms |

Not a failure of TAS — this rig cannot see it. Packets arrive 3125 us apart on a link running at
3% utilisation, so when the gate shuts there is nothing queued to delay. And the baseline itself
has an **11-14 ms gap once a second**, which is sixty times the window being measured: the board
stalls during its own once-a-second housekeeping and misses packets. The instrument has to be
fixed before shaping is worth measuring at all, and that stall is the first real bug this project
has in its own code rather than in something it talks to.

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
- **The once-a-second stall.** 11-14 ms of missed packets, in this board's own loop. Everything
  timing-related is limited by it, so it comes before any more measurement.
- **Schedule writes from the board**, as one atomic container patch. The point of this rig is to
  need no PC, and sensor configuration already needs none; the switch side is what is left.
- **A second LAN9662**, for three hops.
