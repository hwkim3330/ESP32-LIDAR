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

// ------------------------------------------------------------------------- writing a schedule

// A gate schedule is the one thing here that changes the switch, and the port it has to go on --
// the one this board receives the sensor's stream through -- is also the only path this board
// has back to the switch. So three rules hold this down:
//
//   1. No window may close every traffic class for the whole cycle. A schedule that never opens
//      would strand the control path with no way to withdraw it from this end.
//   2. save-config is never called. Whatever is written dies with the next switch reboot, which
//      makes power the last resort escape hatch.
//   3. The 9662's serial console has to be reachable before any of this is worth running. It is
//      the only thing that can undo a schedule this board can no longer talk past.
//
// The write itself is keti-reconfig's sequence, which took that project a while to get right:
// the control list, then the cycle, then gate-enabled, then config-change -- and config-change
// last, because it is what makes the switch adopt the admin list rather than merely store it.
struct GateWindow {
  uint32_t intervalNs;
  uint8_t mask;  // one bit per traffic class; 0xFF is all open
};

struct SchedulePreset {
  const char *id;
  const char *description;
  uint64_t cycleNumerator, cycleDenominator;
  int windowCount;
  GateWindow windows[4];
};

// Cycle is a fraction of a second: 1/1000 is one millisecond. A millisecond against the sensor's
// 3125 us spacing means roughly three cycles per packet, so every packet meets a closed gate and
// the effect lands in the gap distribution rather than hiding between packets.
static const SchedulePreset kPresets[] = {
    {"half", "1 ms cycle, 500 us open / 500 us closed", 1, 1000, 2,
     {{500000, 0xFF}, {500000, 0x00}}},
    {"quarter", "1 ms cycle, 250 us open / 750 us closed", 1, 1000, 2,
     {{250000, 0xFF}, {750000, 0x00}}},
};

inline const SchedulePreset *presetFor(const char *id) {
  for (const SchedulePreset &p : kPresets)
    if (strcmp(p.id, id) == 0) return &p;
  return nullptr;
}

// Disabled, and the reason is worth keeping rather than deleting. Written as five separate
// patches -- control list, numerator, denominator, gate-enabled, config-change -- this took the
// switch out on 2026-08-11: the list was rejected while gate-enabled and config-change were
// accepted, so the switch gated against no valid schedule, shut port 1, and with it the only
// path this board had to withdraw the mistake. Recovery needed the serial console on the PC,
// and then an ESP reboot, because the W5500 stack stayed wedged after the gates reopened.
//
// The fix is not a better retry. keti-tsn-cli writes the whole gate-parameter-table container in
// ONE iPATCH, so it either lands or it does not -- there is no half-applied state to be trapped
// by. Doing that from here needs SIDs this table does not carry yet (admin-base-time and its
// two leaves, admin-gate-states) and CBOR for a nested container rather than four flat leaves.
// Until then tools/tas-*.yaml does it from the PC over serial, which is out of band and cannot
// cut its own path back.
inline bool writeSchedule(const char *port, const SchedulePreset &preset, uint8_t *code) {
  return false;
}

inline bool clearSchedule(const char *port, uint8_t *code) {
  static uint8_t buffer[128];
  char path[192];
  const char *base =
      "ietf-interfaces:interfaces/interface/ieee802-dot1q-bridge:bridge-port/"
      "ieee802-dot1q-sched-bridge:gate-parameter-table";
  snprintf(path, sizeof(path), "%s/gate-enabled", base);
  bool ok = patchRaw(buffer, buildPatchBool(buffer, ketiSidFor(path), port, false), code);
  snprintf(path, sizeof(path), "%s/config-change", base);
  ok = patchRaw(buffer, buildPatchBool(buffer, ketiSidFor(path), port, true), code) && ok;
  return ok;
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
