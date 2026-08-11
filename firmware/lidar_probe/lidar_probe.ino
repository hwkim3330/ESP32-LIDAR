// ESP32-S3 + W5500: watch a LiDAR's UDP stream and serve what it looks like.
//
// The first question this board has to answer is not "how fast is the sensor" but "does the
// link come up at all". W5500 is 10/100 and several Ethernet LiDARs (every Ouster OS1 among
// them) refuse to transmit unless they negotiate 1000BASE-T full duplex. If that is the case
// here the symptom is specific and worth showing plainly rather than debugging blind: PHY
// says link UP at 100M, and not one packet ever arrives. So the page reports link speed and
// packet count with equal prominence, and the registers are read directly rather than trusted
// through the driver.
//
// Everything past that is timing. Arrival times come from esp_timer_get_time() (microseconds,
// monotonic), and what we keep is the interval between consecutive packets -- that is the
// quantity TAS has to accommodate later, and the one that changes when a switch is inserted
// into the path.
//
// Pinout is from keti-reconfig's exhaustive bit-banged search, not a datasheet. Do not change
// it casually: seven published pinouts for this board were all wrong.
#include <Arduino.h>
#include <HTTPClient.h>
#include <NetworkUdp.h>
#include <Preferences.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>

#include "eth_w5500.h"
#include "page.h"
#include "switch_link.h"

constexpr int kSck = 48, kMosi = 21, kCs = 45, kMiso = 47;

// Ouster's defaults. 7502 carries lidar columns, 7503 the IMU at ~100 Hz. Other sensors use
// other ports, so a third socket is opened on whatever the page asks for -- an unknown sensor
// is identified by finding the port it is actually talking on, and that is easier to do from
// a browser than by reflashing.
constexpr uint16_t kLidarPort = 7502;
constexpr uint16_t kImuPort = 7503;

// The bench is a closed net with no DHCP server (see keti-reconfig's address plan), but a
// sensor wired straight to this board may well be handing out or expecting something else, so
// DHCP is tried first and this is the fallback rather than the only option.
const IPAddress kStaticSelf(192, 168, 1, 20);
const IPAddress kStaticMask(255, 255, 255, 0);
const IPAddress kStaticGateway(192, 168, 1, 1);

SPIClass rawSpi(FSPI);
WebServer server(80);
NetworkUDP lidarUdp, imuUdp, scanUdp;
uint16_t scanPort = 0;

// Finding the sensor without a second NIC on the PC. An Ouster boots into DHCP and falls back
// to link-local, so a powered sensor on this segment broadcasts DISCOVER whether or not anyone
// answers -- and DISCOVER carries its MAC and its hostname (os1-<serial>), which identifies the
// unit exactly. Listening costs one socket and answers "is it even alive" without touching it.
//
// Having heard it, this board also answers. A one-lease DHCP server is about eighty lines and
// it removes the entire problem of reaching the sensor's HTTP API: without it the sensor falls
// back to a link-local address nobody knows, and configuring it needs a laptop on the segment.
// With it the sensor is always at kSensorAddress and the ESP can drive it alone.
NetworkUDP dhcpUdp;
char discovered[96] = "";

// Two switches now, and the sensor's stream crosses both:
//
//   LiDAR --1G--> [A] port2 => port1 --1G--> [B] port2 => port1 --100M--> this board
//
// Both arrived configured as 192.168.1.10, which is not a MAC clash -- their L3V1 addresses
// differ -- but two devices answering for one address on one segment means whichever replies
// first wins the ARP cache, and the board would read counters from an arbitrary switch while
// believing it knew which. B was moved to .11 over serial; tools/switch-b-ip.yaml.
//
// kSwitch is not const because coap_client.h reads it as the target and the console walks the
// list. It is that project's file unchanged.
IPAddress kSwitch(192, 168, 1, 10);

struct SwitchInPath {
  uint8_t lastOctet;
  const char *label;
};
static const SwitchInPath kSwitches[] = {
    {10, "A -- the sensor's switch"},
    {11, "B -- this board's switch"},
};
NetworkUDP udp;
uint16_t messageId = 1;
const uint16_t kCoapPort = 5683;
String switchCatalog = "";
PortTable portTable;
const IPAddress kSensorAddress(192, 168, 1, 50);
uint8_t sensorMac[6] = {0};
bool sensorLeased = false;

// Filled from the sensor's own HTTP API once it has an address. Which firmware it runs decides
// what can be asked of it, so it is read rather than assumed.
String sensorInfo = "";
String configResult = "";

// Declared by hand: a default argument defeats the prototype the Arduino build generates, so
// setup()'s handlers cannot see this function without it.
bool configureSensor(const char *mode, const char *profile, bool remember = true);

// What the operator asked for, remembered across reboots of either end. The sensor's config is
// not persistent -- power-cycle it and udp_dest is gone -- and this bench gets power-cycled. So
// the board re-applies the mode that was asked for, and only that: it never decides on its own
// that the sensor should be streaming.
Preferences prefs;
bool streamRequested = false;
String requestedMode = "512x10", requestedProfile = "RNG15_RFL8_NIR8";
int64_t lastReapply = 0;
uint32_t packetsAtLastCheck = 0;
int64_t quietSince = 0;

// The page has to serve to something. The PC on this bench has one NIC and it is committed to
// the office network, so the UI rides the S3's WiFi as a soft AP: any phone, tablet or laptop
// joins it directly and the Ethernet side stays purely the measurement path.
bool apRunning = true;
const char *kApSsid = "KETI-LIDAR";
const char *kApPassword = "ketilidar";

// A packet buffer big enough for a jumbo-ish lidar packet. Ouster's RNG19 profile on a 16
// beam sensor is 3328 bytes of payload; anything larger than this gets truncated, which is
// visible in the size histogram rather than silently wrong.
constexpr size_t kPacketBuffer = 4096;
uint8_t packet[kPacketBuffer];

// ---------------------------------------------------------------------------- timing state

// Intervals are what we actually measure. 1024 of them is about 0.8 s of an OS1-16 at 781 us,
// which is long enough to see the shape of the distribution and short enough that the page
// stays responsive when it is serialised.
constexpr int kIntervalRing = 1024;
struct Flow {
  uint32_t packets = 0;
  uint64_t bytes = 0;
  int64_t lastArrival = 0;
  uint32_t intervals[kIntervalRing] = {0};
  int intervalCount = 0;
  int intervalHead = 0;
  uint32_t minInterval = UINT32_MAX;
  uint32_t maxInterval = 0;
  uint16_t lastSize = 0;
  uint8_t firstBytes[32] = {0};
  bool sawFirst = false;
  IPAddress source;
  uint16_t sourcePort = 0;
};
Flow lidar, imu, scan;

// One second of history per bucket, two minutes deep. Rate and jitter are computed here rather
// than in the browser so that a page reload does not lose the record.
constexpr int kHistory = 120;
struct HistoryBucket {
  uint16_t packets = 0;
  uint32_t maxInterval = 0;
  uint32_t meanInterval = 0;
};

// One outlier a second says something happens once a second. Thirty say the stream is ragged
// throughout. The single worst gap cannot tell those apart, and they have different causes.
constexpr uint32_t kOutlierThresholdUs = 6250;  // twice the nominal spacing
uint32_t bucketOutliers = 0, bucketMinInterval = UINT32_MAX;
uint32_t lastOutliers = 0, lastMinInterval = 0;
HistoryBucket history[kHistory];
int historyHead = 0;
int64_t bucketStart = 0;
int64_t housekeepingUs = 0;

// Where a loop pass spends its time, worst case per second. If one section owns the stall it is
// this code's fault; if the worst pass lands in whichever section happened to be running, the
// loop was preempted and the fault is another task.
int64_t worstDrain = 0, worstServe = 0, worstPass = 0;
uint32_t bucketPackets = 0;
uint64_t bucketIntervalSum = 0;
uint32_t bucketIntervalMax = 0;

void recordArrival(Flow &s, int size, const IPAddress &from, uint16_t fromPort, bool timed) {
  const int64_t now = esp_timer_get_time();
  s.packets++;
  s.bytes += size;
  s.lastSize = size;
  s.source = from;
  s.sourcePort = fromPort;

  if (s.lastArrival != 0) {
    const int64_t delta = now - s.lastArrival;
    // A gap longer than a second is a restart, not an interval. Folding it into the statistics
    // would swamp every real measurement with one meaningless outlier.
    if (delta > 0 && delta < 1000000) {
      const uint32_t interval = uint32_t(delta);
      s.intervals[s.intervalHead] = interval;
      s.intervalHead = (s.intervalHead + 1) % kIntervalRing;
      if (s.intervalCount < kIntervalRing) s.intervalCount++;
      if (interval < s.minInterval) s.minInterval = interval;
      if (interval > s.maxInterval) s.maxInterval = interval;
      if (timed) {
        bucketIntervalSum += interval;
        if (interval > bucketIntervalMax) bucketIntervalMax = interval;
        if (interval > kOutlierThresholdUs) bucketOutliers++;
        if (interval < bucketMinInterval) bucketMinInterval = interval;
      }
    }
  }
  s.lastArrival = now;
  if (timed) bucketPackets++;
}

void closeBucket() {
  history[historyHead].packets = uint16_t(min<uint32_t>(bucketPackets, 65535));
  history[historyHead].maxInterval = bucketIntervalMax;
  history[historyHead].meanInterval = bucketPackets > 1 ? uint32_t(bucketIntervalSum / (bucketPackets - 1)) : 0;
  historyHead = (historyHead + 1) % kHistory;
  lastOutliers = bucketOutliers;
  lastMinInterval = bucketMinInterval == UINT32_MAX ? 0 : bucketMinInterval;
  bucketPackets = 0;
  bucketIntervalSum = 0;
  bucketIntervalMax = 0;
  bucketOutliers = 0;
  bucketMinInterval = UINT32_MAX;
}

// ------------------------------------------------------------------------- raw W5500 access

// Reading registers directly is only safe BEFORE the driver takes the bus. FSPI and SPI2_HOST
// are the same peripheral, so a raw transaction taken while ETH is running blocks the driver's
// esp_timer callback, starves IDLE0 and reboots the board on the task watchdog about fifteen
// seconds in -- which is exactly what the first build of this firmware did. So the raw path
// exists for one job, the boot-time pinout check, and the bus is handed over afterwards.
uint8_t readRegisterRaw(uint16_t addr) {
  rawSpi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(kCs, LOW);
  rawSpi.transfer(addr >> 8);
  rawSpi.transfer(addr & 0xFF);
  rawSpi.transfer(0x01);
  const uint8_t v = rawSpi.transfer(0x00);
  digitalWrite(kCs, HIGH);
  rawSpi.endTransaction();
  return v;
}

uint8_t versionr = 0;

// ------------------------------------------------------------------------------- HTTP layer

void appendStream(String &out, const char *name, const Flow &s) {
  // Percentiles say more about a real stream than mean and standard deviation do -- a shaper
  // that misses one packet in a thousand shows up in p99 and nowhere else. Sorting a copy of
  // the ring costs a few milliseconds and happens only when the page asks.
  static uint32_t sorted[kIntervalRing];
  const int n = s.intervalCount;
  for (int i = 0; i < n; i++) sorted[i] = s.intervals[i];
  for (int i = 1; i < n; i++) {  // insertion sort: n is small and the array is nearly ordered
    const uint32_t key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  uint64_t sum = 0;
  for (int i = 0; i < n; i++) sum += sorted[i];
  const uint32_t mean = n ? uint32_t(sum / n) : 0;
  uint64_t variance = 0;
  for (int i = 0; i < n; i++) {
    const int64_t d = int64_t(sorted[i]) - int64_t(mean);
    variance += uint64_t(d * d);
  }
  const uint32_t stddev = n ? uint32_t(sqrt(double(variance) / n)) : 0;

  out += "\"";
  out += name;
  out += "\":{\"packets\":" + String(s.packets);
  out += ",\"bytes\":" + String((unsigned long long)s.bytes);
  out += ",\"lastSize\":" + String(s.lastSize);
  out += ",\"source\":\"" + s.source.toString() + ":" + String(s.sourcePort) + "\"";
  out += ",\"min\":" + String(s.minInterval == UINT32_MAX ? 0 : s.minInterval);
  out += ",\"max\":" + String(s.maxInterval);
  out += ",\"mean\":" + String(mean);
  out += ",\"stddev\":" + String(stddev);
  out += ",\"p50\":" + String(n ? sorted[n / 2] : 0);
  out += ",\"p99\":" + String(n ? sorted[(n * 99) / 100] : 0);
  out += ",\"intervals\":[";
  // Newest first, so the browser can draw without knowing where the ring head is.
  for (int i = 0; i < n; i++) {
    const int idx = (s.intervalHead - 1 - i + kIntervalRing * 2) % kIntervalRing;
    if (i) out += ",";
    out += String(s.intervals[idx]);
  }
  out += "]";
  out += ",\"head\":\"";
  for (int i = 0; i < 32 && s.sawFirst; i++) {
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", s.firstBytes[i]);
    out += hex;
  }
  out += "\"}";
}

void handleStats() {
  String out;
  out.reserve(16384);
  out += "{";
  out += "\"uptime\":" + String((unsigned long)(esp_timer_get_time() / 1000000));
  out += ",\"link\":{\"up\":" + String(ethLinkUp() ? "true" : "false");
  out += ",\"speed\":" + String(ethLinkSpeed());
  out += ",\"duplex\":\"" + String(ethFullDuplex() ? "full" : "half") + "\"";
  out += ",\"versionr\":" + String(versionr) + "}";
  out += ",\"ip\":\"" + ethLocalIP().toString() + "\"";
  out += ",\"mac\":\"" + ethMacAddress() + "\"";
  out += ",\"scanPort\":" + String(scanPort);
  out += ",\"discovered\":\"" + String(discovered) + "\"";
  out += ",\"leased\":" + String(sensorLeased ? "true" : "false");
  out += ",\"sensorIp\":\"" + kSensorAddress.toString() + "\"";
  out += ",\"configResult\":\"" + configResult + "\"";
  out += ",\"history\":[";
  for (int i = 0; i < kHistory; i++) {
    const int idx = (historyHead + i) % kHistory;
    if (i) out += ",";
    out += "[" + String(history[idx].packets) + "," + String(history[idx].meanInterval) + "," +
           String(history[idx].maxInterval) + "]";
  }
  out += "],";
  appendStream(out, "lidar", lidar);
  out += ",";
  appendStream(out, "imu", imu);
  out += ",";
  appendStream(out, "scan", scan);
  out += "}";
  server.send(200, "application/json", out);
}

// Reopening one socket on demand is how an unidentified sensor gets found: point the scan
// socket at a candidate port from the browser and watch whether the counter moves.
void handleListen() {
  const uint16_t port = server.arg("port").toInt();
  if (scanPort) scanUdp.stop();
  scanPort = port;
  scan = Flow();
  if (scanPort) scanUdp.begin(scanPort);
  server.send(200, "text/plain", scanPort ? String("listening on ") + scanPort : "stopped");
}

void handleReset() {
  lidar = Flow();
  imu = Flow();
  scan = Flow();
  for (int i = 0; i < kHistory; i++) history[i] = HistoryBucket();
  server.send(200, "text/plain", "cleared");
}

// ------------------------------------------------------------------------------------ setup

// The CORECONF decoder walks a nested CBOR tree by recursion, and the interface subtree is deep
// enough -- interfaces, interface, bridge-port, gate-parameter-table, control list, entry -- to
// blow the 8 KB Arduino gives loopTask by default. It does not fail gracefully when it does: the
// stack canary fires and the board reboots mid-parse.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// Placed here rather than at the top of the file: the macro expands to a definition, and the
// Arduino build inserts its generated prototypes after the last thing that looks like one --
// putting it above the sketch's own types makes every function referring to them fail to parse.
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nesp32-lidar probe");

  pinMode(kCs, OUTPUT);
  digitalWrite(kCs, HIGH);
  rawSpi.begin(kSck, kMiso, kMosi, -1);
  delay(200);

  // Sanity-check the chip before the driver touches it: VERSIONR has a known reset value, so a
  // wrong pinout says so here rather than looking like a dead sensor later. Then give the bus
  // back -- see readRegisterRaw() for why nothing may touch it again.
  // Started before Ethernet on purpose. The UI rides WiFi so that a machine with no spare NIC --
  // or no WiFi at all, like the PC on this bench -- is not what decides whether the measurement
  // can be seen; and bringing the radio up first is also what initialises the netif and lwIP
  // machinery that the hand-rolled Ethernet bring-up below then joins.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);

  versionr = readRegisterRaw(0x0039);
  Serial.printf("W5500 VERSIONR 0x%02X (expect 0x04)\n", versionr);
  rawSpi.end();

  // One millisecond rather than Arduino's ten. See eth_w5500.h: with no interrupt line the
  // driver polls, and the poll period is the resolution of every arrival time this rig records.
  if (!ethStart(kSck, kMiso, kMosi, kCs, 1, kStaticSelf, kStaticMask, kStaticGateway)) {
    Serial.println("ethStart failed -- check the pinout above all else");
  }
  delay(500);
  Serial.printf("ip %s  mac %s\n", ethLocalIP().toString().c_str(), ethMacAddress().c_str());

  prefs.begin("lidar", false);
  streamRequested = prefs.getBool("stream", false);
  requestedMode = prefs.getString("mode", requestedMode);
  requestedProfile = prefs.getString("profile", requestedProfile);
  if (streamRequested)
    Serial.printf("remembered: %s %s -- will re-apply if the sensor goes quiet\n",
                  requestedMode.c_str(), requestedProfile.c_str());

  lidarUdp.begin(kLidarPort);
  imuUdp.begin(kImuPort);
  dhcpUdp.begin(67);
  udp.begin(5684);  // any local port; the switch answers to wherever it came from

  server.on("/", []() { server.send_P(200, "text/html", kPage); });
  server.on("/api/stats", handleStats);
  server.on("/api/listen", handleListen);
  server.on("/api/reset", handleReset);
  server.on("/api/sensor/info", []() {
    fetchSensorInfo();
    server.send(200, "application/json", sensorInfo);
  });
  server.on("/api/sensor/configure", []() {
    const String mode = server.hasArg("mode") ? server.arg("mode") : "512x10";
    const String profile = server.hasArg("profile") ? server.arg("profile") : "RNG15_RFL8_NIR8";
    const bool ok = configureSensor(mode.c_str(), profile.c_str());
    server.send(ok ? 200 : 502, "text/plain", configResult);
  });
  server.begin();
  Serial.printf("http://%s/  (ethernet)\n", ethLocalIP().toString().c_str());
  Serial.printf("http://%s/  (wifi ap \"%s\", password \"%s\")\n",
                WiFi.softAPIP().toString().c_str(), kApSsid, kApPassword);

  bucketStart = esp_timer_get_time();
  quietSince = bucketStart;

  xTaskCreatePinnedToCore(serverTask, "http", 8192, nullptr, 1, nullptr, 0);
  Serial.printf("FreeRTOS tick %u Hz -- one tick is %u ms, which is why HTTP is the task\n",
                configTICK_RATE_HZ, 1000 / configTICK_RATE_HZ);
}

// How many packets one call pulls out. If the board were keeping up this would be one; anything
// more means they were sitting in the W5500 while the loop was busy elsewhere, and the arrival
// times recorded for them are when this code got round to them rather than when they landed.
int maxBurst = 0, lastBurst = 0;

void drain(NetworkUDP &udp, Flow &s, bool timed) {
  int size;
  int burst = 0;
  while ((size = udp.parsePacket()) > 0) {
    if (timed && ++burst > maxBurst) maxBurst = burst;
    const IPAddress from = udp.remoteIP();
    const uint16_t fromPort = udp.remotePort();
    const int read = udp.read(packet, min<size_t>(size, kPacketBuffer));
    if (!s.sawFirst && read > 0) {
      memcpy(s.firstBytes, packet, min(read, 32));
      s.sawFirst = true;
    }
    recordArrival(s, size, from, fromPort, timed);
  }
  if (timed && burst) lastBurst = burst;
}

// BOOTP/DHCP is fixed-layout up to the options, so what is needed here takes no real parser:
// chaddr sits at offset 28, and the options after the magic cookie at 240 are tag/length/value.
// Option 53 is the message type, option 12 the hostname the sensor calls itself.
//
// Only one lease is ever handed out, always the same address, and only to a client whose
// hostname we have seen. A general DHCP server on a bench with real infrastructure on it would
// be a menace; this one can only ever say "you are 192.168.1.50".
void writeIp(uint8_t *dst, const IPAddress &ip) {
  for (int i = 0; i < 4; i++) dst[i] = ip[i];
}

void sendDhcpReply(const uint8_t *request, int length, uint8_t messageType) {
  static uint8_t reply[300];
  memset(reply, 0, sizeof(reply));
  reply[0] = 2;                        // BOOTREPLY
  reply[1] = 1;                        // Ethernet
  reply[2] = 6;                        // hardware address length
  memcpy(reply + 4, request + 4, 4);   // xid, echoed
  memcpy(reply + 10, request + 10, 2); // flags
  writeIp(reply + 16, kSensorAddress);  // yiaddr -- the address being offered
  writeIp(reply + 20, ethLocalIP());   // siaddr -- us
  memcpy(reply + 28, request + 28, 16);
  reply[236] = 0x63; reply[237] = 0x82; reply[238] = 0x53; reply[239] = 0x63;  // magic cookie

  int i = 240;
  reply[i++] = 53; reply[i++] = 1; reply[i++] = messageType;
  reply[i++] = 54; reply[i++] = 4; writeIp(reply + i, ethLocalIP()); i += 4;
  reply[i++] = 51; reply[i++] = 4;
  reply[i++] = 0; reply[i++] = 0x01; reply[i++] = 0x51; reply[i++] = 0x80;  // 86400 s
  reply[i++] = 1;  reply[i++] = 4; writeIp(reply + i, kStaticMask); i += 4;
  reply[i++] = 3;  reply[i++] = 4; writeIp(reply + i, ethLocalIP()); i += 4;
  reply[i++] = 255;

  // Broadcast rather than unicast: the client has no address yet, so a unicast reply would need
  // an ARP entry it cannot answer.
  dhcpUdp.beginPacket(IPAddress(255, 255, 255, 255), 68);
  dhcpUdp.write(reply, i);
  dhcpUdp.endPacket();
}

void drainDhcp() {
  int size;
  while ((size = dhcpUdp.parsePacket()) > 0) {
    const int read = dhcpUdp.read(packet, min<size_t>(size, kPacketBuffer));
    if (read < 240 || packet[0] != 1) continue;   // only requests
    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X", packet[28], packet[29], packet[30],
             packet[31], packet[32], packet[33]);
    char host[48] = "";
    uint8_t messageType = 0;
    for (int i = 240; i + 1 < read;) {
      const uint8_t tag = packet[i], len = packet[i + 1];
      if (tag == 255) break;
      if (tag == 0) { i++; continue; }
      if (tag == 53 && len == 1) messageType = packet[i + 2];
      if (tag == 12 && i + 2 + len <= read) {
        const int n = min<int>(len, sizeof(host) - 1);
        memcpy(host, packet + i + 2, n);
        host[n] = 0;
      }
      i += 2 + len;
    }
    snprintf(discovered, sizeof(discovered), "%s %s", mac, host[0] ? host : "(no hostname)");
    memcpy(sensorMac, packet + 28, 6);

    if (messageType == 1) {           // DISCOVER
      Serial.printf("DHCP DISCOVER from %s (%s) -> offering %s\n", mac, host,
                    kSensorAddress.toString().c_str());
      sendDhcpReply(packet, read, 2); // OFFER
    } else if (messageType == 3) {    // REQUEST
      Serial.printf("DHCP REQUEST from %s -> ack %s\n", mac, kSensorAddress.toString().c_str());
      sendDhcpReply(packet, read, 5); // ACK
      sensorLeased = true;
    }
  }
}

// ---------------------------------------------------------------------------- sensor control

// Ouster's HTTP API, firmware 2.x. Reading sensor_info first is not politeness -- it says which
// firmware is running, and that decides whether the low-data-rate profile below even exists.
bool fetchSensorInfo() {
  HTTPClient http;
  http.setTimeout(4000);
  if (!http.begin("http://" + kSensorAddress.toString() + "/api/v1/sensor/metadata/sensor_info")) return false;
  const int code = http.GET();
  sensorInfo = code == 200 ? http.getString() : String("HTTP ") + code;
  http.end();
  Serial.printf("sensor_info: %s\n", sensorInfo.substring(0, 300).c_str());
  return code == 200;
}

// Point the sensor at this board and slow it down enough that a W5500 can keep up. An OS1-16 in
// its default profile is about 34 Mbit/s, which the SPI link cannot carry; 512x10 with the
// low-data-rate profile is roughly a tenth of that. Both are legitimate sensor modes -- nothing
// here is a workaround, it is choosing an operating point the link can actually serve.
bool configureSensor(const char *mode, const char *profile, bool remember) {
  if (remember) {
    streamRequested = true;
    requestedMode = mode;
    requestedProfile = profile;
    prefs.putBool("stream", true);
    prefs.putString("mode", requestedMode);
    prefs.putString("profile", requestedProfile);
  }
  HTTPClient http;
  http.setTimeout(remember ? 8000 : 3000);
  if (!http.begin("http://" + kSensorAddress.toString() + "/api/v1/sensor/config")) return false;
  http.addHeader("Content-Type", "application/json");
  String body = "{\"udp_dest\":\"" + ethLocalIP().toString() + "\"";
  body += ",\"udp_port_lidar\":" + String(kLidarPort);
  body += ",\"udp_port_imu\":" + String(kImuPort);
  body += ",\"lidar_mode\":\"" + String(mode) + "\"";
  body += ",\"udp_profile_lidar\":\"" + String(profile) + "\"}";
  const int code = http.POST(body);
  configResult = String("HTTP ") + code + " " + http.getString().substring(0, 200);
  http.end();
  Serial.printf("configure -> %s\n", configResult.c_str());
  return code >= 200 && code < 300;
}

// The whole interface subtree in one request, for whichever switch kSwitch points at. Keyed
// instance queries are refused with 4.00 on this device's Ethernet endpoint (they work over
// serial), so per-port detail is parsed out of the subtree -- which costs nothing extra, since
// the gate parameters are already in there.
void readPorts() {
  static uint8_t payload[4096];
  uint8_t code = 0;
  int blocks = 0;
  const int n = fetchSid(ketiSidFor("ietf-interfaces:interfaces"), payload, sizeof(payload),
                         &code, &blocks);
  Serial.printf("interfaces: %d bytes in %d block(s), code %d.%02d\n", n, blocks, code >> 5,
                code & 0x1F);
  if (n <= 0) return;
  portTable.count = 0;
  if (!parseInterfaces(payload, n, &portTable)) { Serial.println("  parse failed"); return; }
  Serial.printf("  %-8s %-6s %-8s %14s %14s %8s %8s\n", "port", "link", "speed", "in-octets",
                "out-octets", "in-disc", "out-disc");
  for (int i = 0; i < portTable.count; i++) {
    const PortState &p = portTable.ports[i];
    char speed[10];
    if (p.speedMbps) snprintf(speed, sizeof(speed), "%uM", p.speedMbps);
    else snprintf(speed, sizeof(speed), "-");
    Serial.printf("  %-8s %-6s %-8s %14llu %14llu %8llu %8llu\n", p.name,
                  p.operStatus == 1 ? "up" : "down", speed, (unsigned long long)p.inOctets,
                  (unsigned long long)p.outOctets, (unsigned long long)p.inDiscards,
                  (unsigned long long)p.outDiscards);
    if (p.tasSeen)
      Serial.printf("           TAS gate-enabled=%llu cycle=%llu/%llu entries=%d\n",
                    (unsigned long long)p.gateEnabled, (unsigned long long)p.cycleNumerator,
                    (unsigned long long)p.cycleDenominator, p.gateCount);
  }
}

// The serial console is the only way in until something is on the WiFi AP, and it is also the
// right place for the one action that changes the sensor rather than observing it. Asking for
// the stream is deliberately a keypress and not something that happens on its own: it puts the
// sensor into a different operating mode, and a tool that does that unbidden is a bad tool.
void handleConsole() {
  if (!Serial.available()) return;
  switch (Serial.read()) {
    case 'i':
      Serial.println("asking the sensor about itself...");
      fetchSensorInfo();
      break;
    case 'c':
      Serial.println("configuring: 512x10, low data rate, udp_dest -> this board");
      configureSensor("512x10", "RNG15_RFL8_NIR8");
      break;
    case 'C':
      Serial.println("configuring: 1024x10, full profile (34 Mbit/s -- expect loss on W5500)");
      configureSensor("1024x10", "RNG19_RFL8_SIG16_NIR16");
      break;
    case 'r':
      lidar = Flow(); imu = Flow(); scan = Flow();
      for (int i = 0; i < kHistory; i++) history[i] = HistoryBucket();
      Serial.println("counters cleared");
      break;
    case 'g': {
      // Anything the sensor exposes, without a reflash per question. When the sensor will not
      // come up, what it says about itself -- alerts especially -- is worth more than another
      // round of guessing from the outside.
      const String path = Serial.readStringUntil('\n');
      HTTPClient http;
      http.setTimeout(5000);
      const String url = "http://" + kSensorAddress.toString() + path;
      Serial.printf("GET %s\n", url.c_str());
      if (http.begin(url)) {
        const int code = http.GET();
        // Printed whole, in chunks. Truncating showed the head of the alert log, which is the
        // oldest entries -- exactly the part that does not say what is wrong now.
        const String body = http.getString();
        Serial.printf("  %d (%u bytes)\n", code, body.length());
        for (unsigned i = 0; i < body.length(); i += 512) Serial.println(body.substring(i, i + 512));
        http.end();
      }
      break;
    }
    case 's': {
      for (const SwitchInPath &sw : kSwitches) {
        kSwitch = IPAddress(192, 168, 1, sw.lastOctet);
        Serial.printf("%s (%s): ", kSwitch.toString().c_str(), sw.label);
        static uint8_t payload[256];
        uint8_t code = 0;
        int blocks = 0;
        const int n = fetchSid(KETI_SID_YANG_CHECKSUM, payload, sizeof(payload), &code, &blocks);
        switchCatalog = n > 0 ? checksumFromPayload(payload, n) : String("");
        if (!switchCatalog.length()) { Serial.println("no answer"); continue; }
        Serial.printf("%s -> %s\n", switchCatalog.c_str(), nameForCatalog(switchCatalog));
        // A table built against another catalog addresses the wrong nodes and returns plausible
        // nonsense, which is worse than returning nothing.
        if (switchCatalog != KETI_SID_CATALOG_CHECKSUM)
          Serial.printf("  WARNING: this firmware's SID table is for %s\n",
                        KETI_SID_CATALOG_CHECKSUM);
      }
      break;
    }
    case 'S': {
      for (const SwitchInPath &sw : kSwitches) {
        kSwitch = IPAddress(192, 168, 1, sw.lastOctet);
        Serial.printf("\n=== %s  %s ===\n", kSwitch.toString().c_str(), sw.label);
        readPorts();
      }
      break;
    }
    case 'T': {
      // The port the sensor's stream comes out of, which is also this board's way back to the
      // switch. Deliberately not a default and not automatic.
      const String id = Serial.readStringUntil('\n');
      const String want = id.length() ? id : String("half");
      const SchedulePreset *preset = presetFor(want.c_str());
      if (!preset) {
        Serial.printf("unknown preset '%s'; have:", want.c_str());
        for (const SchedulePreset &p : kPresets) Serial.printf(" %s", p.id);
        Serial.println();
        break;
      }
      uint8_t code = 0;
      if (!writeSchedule("1", *preset, &code)) {
        Serial.println("refused: writing a schedule from here is disabled -- see switch_link.h.");
        Serial.println("use tools/tas-tc0-200us.yaml over serial from the PC instead.");
      }
      break;
    }
    case 't': {
      uint8_t code = 0;
      const bool ok = clearSchedule("1", &code);
      Serial.printf("gating off on port 1: %s, code %d.%02d\n", ok ? "accepted" : "rejected",
                    code >> 5, code & 0x1F);
      break;
    }
    case 'w':
      // A/B the radio against the stall. The AP shares a core with this loop, and a periodic
      // radio task is the kind of thing that holds it for ten milliseconds without ever showing
      // up inside any section timed above.
      apRunning = !apRunning;
      if (apRunning) WiFi.softAP(kApSsid, kApPassword); else WiFi.softAPdisconnect(true);
      Serial.printf("wifi ap %s\n", apRunning ? "on" : "off");
      break;
    case 'p': {
      // Find the W5500's interrupt line, if it is wired at all. Same method that found the SPI
      // pins on this board: do not trust a datasheet, watch every pin at once and let the one
      // that behaves like the answer identify itself. With the sensor streaming, INT asserts on
      // every arriving frame and clears when the driver services it, so it is the pin that
      // toggles in step with traffic while everything idle stays still.
      //
      // Excluded and why: 19/20 are native USB, 33-37 are the octal PSRAM bus, and 21/45/47/48
      // are the SPI bus itself, which obviously toggles.
      static const int kSkip[] = {19, 20, 21, 33, 34, 35, 36, 37, 45, 47, 48};
      uint64_t first = 0, changed = 0;
      auto sample = []() -> uint64_t {
        return uint64_t(REG_READ(GPIO_IN_REG)) | (uint64_t(REG_READ(GPIO_IN1_REG)) << 32);
      };
      first = sample();
      const int64_t until = esp_timer_get_time() + 1000000;
      uint32_t samples = 0;
      while (esp_timer_get_time() < until) { changed |= sample() ^ first; samples++; }
      Serial.printf("watched %u samples over 1 s; pins that moved:\n", samples);
      bool any = false;
      for (int pin = 0; pin <= 48; pin++) {
        if (!((changed >> pin) & 1)) continue;
        bool skip = false;
        for (int k : kSkip) if (k == pin) skip = true;
        Serial.printf("  GPIO%-2d %s\n", pin, skip ? "(bus or USB -- expected)" : "<-- candidate");
        if (!skip) any = true;
      }
      if (!any) Serial.println("  nothing outside the known buses moved: INT is not wired here");
      break;
    }
    case '?':
      Serial.println("i=info  g<path>=GET  s=catalog  S=ports  T<preset>=gate on  t=gate off  "
                     "c=512x10  C=1024x10  r=reset");
      break;
    default:
      break;
  }
}

// Reading packets is the one thing on this board that cannot be late, so it does not share a
// thread with anything else. It used to: draining ran in loop() next to server.handleClient(),
// and handleClient costs four to five milliseconds once a second even with no client connected
// -- long enough for four packets to pile up in the W5500 and then be read in one burst, all
// four stamped with the time the loop got round to them rather than when they arrived. That
// showed up as an 11 ms gap every second and it was entirely this code's doing.
//
// So the web server moves out instead, and the reader keeps loop() -- which is the only thread
// here that spins without sleeping. Putting the reader in a task of its own looked right and was
// worse: the smallest sleep a task can take is one tick, Arduino's tick is 100 Hz, and so
// vTaskDelay(1) parked the reader for ten milliseconds at a time. Packets then arrived in groups
// of three or four about 190 us apart -- the time it takes to pull one off the W5500 over SPI --
// separated by ten millisecond holes. Exactly 100 of those holes a second, which is what gave it
// away: one per tick, not one per second.
//
// HTTP does not care about ten milliseconds, so it is the thing that gets to sleep.
//
// The statistics are written by loop() and read here without a lock. A torn read costs one wrong
// number in a display that refreshes every second; a lock would cost what this split is for.
void serverTask(void *) {
  for (;;) {
    const int64_t start = esp_timer_get_time();
    server.handleClient();
    const int64_t spent = esp_timer_get_time() - start;
    if (spent > worstServe) worstServe = spent;
    vTaskDelay(1);
  }
}

void loop() {
  const int64_t passStart = esp_timer_get_time();
  handleConsole();

  drain(lidarUdp, lidar, true);
  drain(imuUdp, imu, false);
  drainDhcp();
  if (scanPort) drain(scanUdp, scan, false);

  const int64_t afterDrain = esp_timer_get_time();
  if (afterDrain - passStart > worstDrain) worstDrain = afterDrain - passStart;
  if (afterDrain - passStart > worstPass) worstPass = afterDrain - passStart;

  const int64_t now = esp_timer_get_time();
  if (now - bucketStart >= 1000000) {
    const int64_t houseStart = now;
    bucketStart = now;
    closeBucket();
    // If a stream was asked for and none is arriving, ask again -- but not oftener than every
    // thirty seconds, and never before giving it twenty to start on its own. A sensor that is
    // restarting will simply refuse, which costs one failed connection.
    if (lidar.packets != packetsAtLastCheck) { packetsAtLastCheck = lidar.packets; quietSince = now; }
    if (streamRequested && quietSince && now - quietSince > 20000000 &&
        now - lastReapply > 30000000) {
      lastReapply = now;
      Serial.println("no stream: re-applying the remembered configuration");
      configureSensor(requestedMode.c_str(), requestedProfile.c_str(), false);
    }

    // Mean alone hides what a shaper does: gating changes the spread, not the average, because
    // the sensor keeps sending at the same rate. So the second's worst gap goes out beside it.
    const HistoryBucket &last = history[(historyHead - 1 + kHistory) % kHistory];
    Serial.printf("link %s %uM %s | lidar %lu pkt %u/s (%u B, gap %lu us mean / %lu us max) | imu %lu\n",
                  ethLinkUp() ? "UP" : "DOWN", ethLinkSpeed(),
                  ethFullDuplex() ? "full" : "half", (unsigned long)lidar.packets, last.packets,
                  lidar.lastSize, (unsigned long)last.meanInterval,
                  (unsigned long)last.maxInterval, (unsigned long)imu.packets);
    // Printed a cycle late on purpose: the cost of the housekeeping cannot be measured by the
    // housekeeping that reports it, so this is the previous second's figure.
    Serial.printf("   gaps over %u us: %lu | min gap %lu us | burst %d | serve %ld us | house %ld us\n",
                  kOutlierThresholdUs, (unsigned long)lastOutliers,
                  (unsigned long)lastMinInterval, maxBurst, (long)worstServe,
                  (long)housekeepingUs);
    worstDrain = worstServe = worstPass = 0;
    housekeepingUs = esp_timer_get_time() - houseStart;
    maxBurst = 0;
  }
}
