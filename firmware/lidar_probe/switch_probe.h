// Ask the switch in the path what it is, over CoAP/CORECONF.
//
// One request, and it is the right first one: the YANG catalog checksum is the only node whose
// SID can be known without already having that catalog's SID table (29304, fixed). So this
// works against a LAN9662 or a LAN9692 without knowing in advance which is on the bench, and
// the answer says which. Everything else -- port counters, gate schedules -- needs a SID table
// generated for the catalog this returns, so nothing else can honestly come first.
//
// Request shape is keti-reconfig's, which took a while to get right: FETCH 0.05, Uri-Path "c",
// Content-Format 141, payload a CBOR array of SIDs. The reply is an indefinite-length map
// (0xBF...0xFF) whose value is a byte string, not text -- both details cost a debugging session
// there and are not rediscovered here.
#pragma once
#include <Arduino.h>
#include <NetworkUdp.h>

constexpr uint16_t kCoapPort = 5683;
constexpr uint32_t kSidYangChecksum = 29304u;

// Known catalogs, so the answer means something without a lookup elsewhere.
struct KnownCatalog {
  const char *checksum;
  const char *device;
};
static const KnownCatalog kKnownCatalogs[] = {
    {"5151bae07677b1501f9cf52637f2a38f", "LAN9662 (54 YANG / 54 SID)"},
    {"440057a11e66eed82bc8e838347f694c", "LAN9692 (12 ports + L3V1)"},
};

inline size_t cborUint(uint8_t *out, uint32_t value, uint8_t majorType) {
  const uint8_t mt = majorType << 5;
  if (value < 24) { out[0] = mt | value; return 1; }
  if (value < 0x100) { out[0] = mt | 24; out[1] = value; return 2; }
  if (value < 0x10000) { out[0] = mt | 25; out[1] = value >> 8; out[2] = value; return 3; }
  out[0] = mt | 26;
  out[1] = value >> 24; out[2] = value >> 16; out[3] = value >> 8; out[4] = value;
  return 5;
}

// Returns the checksum as hex text, or an empty string if the switch did not answer.
inline String fetchCatalogChecksum(NetworkUDP &udp, const IPAddress &target, uint16_t &messageId) {
  uint8_t request[32];
  int n = 0;
  request[n++] = 0x40;             // version 1, CON, no token
  request[n++] = 0x05;             // FETCH
  request[n++] = messageId >> 8;
  request[n++] = messageId & 0xFF;
  messageId++;
  request[n++] = 0xB1; request[n++] = 'c';        // Uri-Path "c"
  request[n++] = 0x11; request[n++] = 141;        // Content-Format 141
  request[n++] = 0xFF;                            // payload marker
  request[n++] = 0x81;                            // CBOR array of one
  n += cborUint(request + n, kSidYangChecksum, 0);

  udp.beginPacket(target, kCoapPort);
  udp.write(request, n);
  udp.endPacket();

  uint8_t reply[512];
  const int64_t deadline = esp_timer_get_time() + 2000000;
  int length = 0;
  while (esp_timer_get_time() < deadline) {
    length = udp.parsePacket();
    if (length > 0) break;
    delay(10);
  }
  if (length <= 0) return "";
  const int read = udp.read(reply, min<size_t>(length, sizeof(reply)));

  // Step over the CoAP header, token and options to the payload marker.
  int i = 4 + (reply[0] & 0x0F);
  while (i < read && reply[i] != 0xFF) i++;
  if (i >= read) return "";
  i++;

  if (i < read && (reply[i] >> 5) == 5) {          // map, counted or indefinite
    i++;
    if (i >= read) return "";
    const uint8_t keyInfo = reply[i] & 0x1F;       // step over the integer key
    i++;
    if (keyInfo == 24) i += 1; else if (keyInfo == 25) i += 2; else if (keyInfo == 26) i += 4;
  }
  if (i >= read) return "";
  const uint8_t major = reply[i] >> 5;
  if (major != 2 && major != 3) return "";
  uint32_t stringLength = reply[i] & 0x1F;
  i++;
  if (stringLength == 24) { stringLength = reply[i]; i += 1; }
  else if (stringLength == 25) { stringLength = (reply[i] << 8) | reply[i + 1]; i += 2; }
  if (i + int(stringLength) > read) return "";

  String out;
  if (major == 3) {                                 // already text
    for (uint32_t k = 0; k < stringLength; k++) out += char(reply[i + k]);
  } else {                                          // bytes -- render as hex
    char hex[3];
    for (uint32_t k = 0; k < stringLength; k++) {
      snprintf(hex, sizeof(hex), "%02x", reply[i + k]);
      out += hex;
    }
  }
  return out;
}

inline const char *nameForCatalog(const String &checksum) {
  for (const KnownCatalog &c : kKnownCatalogs)
    if (checksum == c.checksum) return c.device;
  return "unknown catalog -- a SID table would have to be generated for it";
}
