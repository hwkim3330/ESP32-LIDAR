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
| `c` | 512x10, low data rate, `udp_dest` → this board |
| `C` | 1024x10, full profile — 34 Mbit/s, expect loss |
| `r` | clear counters |

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

## Not done yet

- The page has not been opened in a browser — the firmware serves it, but no device has joined
  the AP yet to look at it.
- Three hops, and what TAS does to these numbers. The gap distribution above is the baseline to
  compare against.
- Whether the sensor's own switch port is actually negotiating 1000M is inferred from the stream
  existing, not read off the switch.
