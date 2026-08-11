// What this rig asks the LAN9662 in its path.
//
// The CoAP/CORECONF machinery underneath (coap_client.h, coreconf.h) is keti-reconfig's,
// unchanged. Only the SID table differs, and it has to: SIDs belong to a catalog, that project's
// table is for the LAN9692, and this bench has a LAN9662. The generator is the same one; see
// tools/gen_sid_table.
//
// Two questions only. What speed did each port negotiate -- the sensor claims 1000M and the
// switch is the independent witness for that -- and what is the gate schedule, once there is
// one. Writing schedules is deliberately not here: a schedule on the port this board reaches
// the switch through would cut the only path back, and the 9662's serial console is not
// connected to anything that could undo it.
#pragma once
#include <Arduino.h>

#include "coap_client.h"
#include "coreconf.h"
#include "sid_table.h"

// The catalogs this bench has produced, so a checksum means something without a lookup.
struct KnownCatalog {
  const char *checksum;
  const char *device;
};
static const KnownCatalog kKnownCatalogs[] = {
    {"5151bae07677b1501f9cf52637f2a38f", "LAN9662 (54 YANG / 54 SID)"},
    {"440057a11e66eed82bc8e838347f694c", "LAN9692 (12 ports + L3V1)"},
};

inline const char *nameForCatalog(const String &checksum) {
  for (const KnownCatalog &c : kKnownCatalogs)
    if (checksum == c.checksum) return c.device;
  return "unknown catalog -- generate a SID table for it before trusting anything else";
}

// The checksum comes back as CBOR {29304: h'...'}. Two details worth keeping: the switch sends
// an indefinite-length map (0xBF...0xFF) rather than a counted one, and the value is a byte
// string, so it has to be rendered as hex rather than copied out as characters.
inline String checksumFromPayload(const uint8_t *payload, int length) {
  int i = 0;
  if (i >= length) return "";
  if ((payload[i] >> 5) == 5) {
    ++i;
    if (i >= length) return "";
    const uint8_t keyInfo = payload[i] & 0x1F;
    ++i;
    if (keyInfo == 24) i += 1; else if (keyInfo == 25) i += 2; else if (keyInfo == 26) i += 4;
  }
  if (i >= length) return "";
  const uint8_t major = payload[i] >> 5;
  if (major != 2 && major != 3) return "";
  uint32_t stringLength = payload[i] & 0x1F;
  ++i;
  if (stringLength == 24) { stringLength = payload[i]; i += 1; }
  else if (stringLength == 25) { stringLength = (payload[i] << 8) | payload[i + 1]; i += 2; }
  if (i + int(stringLength) > length) return "";

  String out;
  if (major == 3) {
    for (uint32_t k = 0; k < stringLength; k++) out += char(payload[i + k]);
  } else {
    char hex[3];
    for (uint32_t k = 0; k < stringLength; k++) {
      snprintf(hex, sizeof(hex), "%02x", payload[i + k]);
      out += hex;
    }
  }
  return out;
}
