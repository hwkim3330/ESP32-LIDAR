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
#include <ETH.h>
#include <HTTPClient.h>
#include <NetworkUdp.h>
#include <Preferences.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>

#include "page.h"

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
HistoryBucket history[kHistory];
int historyHead = 0;
int64_t bucketStart = 0;
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
  bucketPackets = 0;
  bucketIntervalSum = 0;
  bucketIntervalMax = 0;
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
  out += ",\"link\":{\"up\":" + String(ETH.linkUp() ? "true" : "false");
  out += ",\"speed\":" + String(ETH.linkSpeed());
  out += ",\"duplex\":\"" + String(ETH.fullDuplex() ? "full" : "half") + "\"";
  out += ",\"versionr\":" + String(versionr) + "}";
  out += ",\"ip\":\"" + ETH.localIP().toString() + "\"";
  out += ",\"mac\":\"" + ETH.macAddress() + "\"";
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
  versionr = readRegisterRaw(0x0039);
  Serial.printf("W5500 VERSIONR 0x%02X (expect 0x04)\n", versionr);
  rawSpi.end();

  if (!ETH.begin(ETH_PHY_W5500, 1, kCs, -1, -1, SPI2_HOST, kSck, kMiso, kMosi)) {
    Serial.println("ETH.begin failed -- check the pinout above all else");
  }

  // Give DHCP a bounded chance. A sensor plugged straight in is unlikely to serve it, so this
  // is a short wait rather than a blocking one.
  const int64_t deadline = esp_timer_get_time() + 6000000;
  while (ETH.localIP() == IPAddress(0, 0, 0, 0) && esp_timer_get_time() < deadline) delay(200);
  if (ETH.localIP() == IPAddress(0, 0, 0, 0)) {
    Serial.println("no DHCP lease, falling back to static 192.168.1.20");
    ETH.config(kStaticSelf, kStaticGateway, kStaticMask);
  }
  Serial.printf("ip %s  mac %s\n", ETH.localIP().toString().c_str(), ETH.macAddress().c_str());

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
  // The UI rides WiFi so that a machine with no spare NIC -- or no WiFi at all, like the PC on
  // this bench -- is not what decides whether the measurement can be seen.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  server.begin();
  Serial.printf("http://%s/  (ethernet)\n", ETH.localIP().toString().c_str());
  Serial.printf("http://%s/  (wifi ap \"%s\", password \"%s\")\n",
                WiFi.softAPIP().toString().c_str(), kApSsid, kApPassword);

  bucketStart = esp_timer_get_time();
  quietSince = bucketStart;
}

void drain(NetworkUDP &udp, Flow &s, bool timed) {
  int size;
  while ((size = udp.parsePacket()) > 0) {
    const IPAddress from = udp.remoteIP();
    const uint16_t fromPort = udp.remotePort();
    const int read = udp.read(packet, min<size_t>(size, kPacketBuffer));
    if (!s.sawFirst && read > 0) {
      memcpy(s.firstBytes, packet, min(read, 32));
      s.sawFirst = true;
    }
    recordArrival(s, size, from, fromPort, timed);
  }
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
  writeIp(reply + 20, ETH.localIP());   // siaddr -- us
  memcpy(reply + 28, request + 28, 16);
  reply[236] = 0x63; reply[237] = 0x82; reply[238] = 0x53; reply[239] = 0x63;  // magic cookie

  int i = 240;
  reply[i++] = 53; reply[i++] = 1; reply[i++] = messageType;
  reply[i++] = 54; reply[i++] = 4; writeIp(reply + i, ETH.localIP()); i += 4;
  reply[i++] = 51; reply[i++] = 4;
  reply[i++] = 0; reply[i++] = 0x01; reply[i++] = 0x51; reply[i++] = 0x80;  // 86400 s
  reply[i++] = 1;  reply[i++] = 4; writeIp(reply + i, kStaticMask); i += 4;
  reply[i++] = 3;  reply[i++] = 4; writeIp(reply + i, ETH.localIP()); i += 4;
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
  String body = "{\"udp_dest\":\"" + ETH.localIP().toString() + "\"";
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
    case '?':
      Serial.println("i=info  g<path>=GET  c=512x10 low-rate  C=1024x10 full  r=reset");
      break;
    default:
      break;
  }
}

void loop() {
  handleConsole();

  // Drain before serving: at 1280 packets a second the W5500's socket buffer is the scarce
  // resource, and a page render that blocks for tens of milliseconds would show up as a
  // measurement artefact rather than as slow HTTP.
  drain(lidarUdp, lidar, true);
  drain(imuUdp, imu, false);
  drainDhcp();
  if (scanPort) drain(scanUdp, scan, false);

  server.handleClient();

  const int64_t now = esp_timer_get_time();
  if (now - bucketStart >= 1000000) {
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

    Serial.printf("link %s %uM %s | lidar %lu pkt (%u B, %lu us mean) | imu %lu\n",
                  ETH.linkUp() ? "UP" : "DOWN", ETH.linkSpeed(),
                  ETH.fullDuplex() ? "full" : "half", (unsigned long)lidar.packets, lidar.lastSize,
                  (unsigned long)history[(historyHead - 1 + kHistory) % kHistory].meanInterval,
                  (unsigned long)imu.packets);
  }
}
