// Turn a CORECONF response into per-port state.
//
// RFC 9254 encodes map keys as deltas against the SID of the enclosing node, so the same
// number means different things at different depths and nothing can be matched positionally.
// This walks the structure carrying the parent SID down, resolves each key to an absolute SID,
// and keeps only the leaves the dashboard needs -- anything else is skipped structurally, so
// an unfamiliar branch costs nothing and never gets mistaken for a value.
//
// Ports are counted as they arrive rather than assumed. The bench LAN9662 has two; the target
// LAN9692 has many more, and nothing here changes when it does.
#pragma once

#include <Arduino.h>

#include "sid_table.h"

constexpr int kMaxPorts = 32;

struct PortState {
  char name[16];
  char physAddress[24];
  uint8_t operStatus;  // ietf-interfaces enum: 1 = up, 2 = down
  uint64_t inOctets, outOctets;
  uint64_t inErrors, outErrors;
  uint64_t inDiscards, outDiscards;
  uint64_t inUnicast, outUnicast;

  // Gate parameters ride along inside the interface subtree, so they cost no extra request.
  // This matters: the device's Ethernet CoAP endpoint rejects keyed instance queries with
  // 4.00, so per-port fetches are not available on this transport at all -- see the README.
  bool tasSeen;
  uint64_t gateEnabled, gateStates, cycleNumerator, cycleDenominator;

  // One cycle of the gate control list. Eight windows is more than the demo schedules use and
  // keeps the whole port table in static memory.
  static constexpr int kMaxGates = 8;
  uint64_t fcsErrors, oversizeFrames, undersizeFrames;
  uint32_t speedMbps;   // 0 when the device did not report one

  int gateCount;
  int gateSlot;                       // entry currently being filled, -1 if out of range
  uint64_t gateInterval[kMaxGates];   // nanoseconds
  uint8_t gateMask[kMaxGates];        // one bit per traffic class
};

struct PortTable {
  PortState ports[kMaxPorts];
  int count;
};

struct CborCursor {
  const uint8_t *data;
  int length;
  int offset;
};

inline bool cborHead(CborCursor &c, uint8_t &major, uint64_t &value, bool &indefinite,
                     bool &isBreak) {
  isBreak = false;
  indefinite = false;
  value = 0;
  if (c.offset >= c.length) return false;
  const uint8_t initial = c.data[c.offset++];
  if (initial == 0xFF) { isBreak = true; return true; }
  major = initial >> 5;
  const uint8_t info = initial & 0x1F;
  if (info < 24) { value = info; return true; }
  if (info == 31) { indefinite = true; return true; }
  int width = 0;
  if (info == 24) width = 1;
  else if (info == 25) width = 2;
  else if (info == 26) width = 4;
  else if (info == 27) width = 8;
  else return false;
  if (c.offset + width > c.length) return false;
  for (int i = 0; i < width; ++i) value = (value << 8) | c.data[c.offset + i];
  c.offset += width;
  return true;
}

void cborSkip(CborCursor &c);

inline void cborSkipItems(CborCursor &c, uint64_t count, bool indefinite, bool pairs) {
  if (indefinite) {
    while (c.offset < c.length) {
      if (c.data[c.offset] == 0xFF) { ++c.offset; return; }
      cborSkip(c);
      if (pairs) cborSkip(c);
    }
    return;
  }
  for (uint64_t i = 0; i < count; ++i) {
    cborSkip(c);
    if (pairs) cborSkip(c);
  }
}

inline void cborSkip(CborCursor &c) {
  uint8_t major = 0;
  uint64_t value = 0;
  bool indefinite = false, isBreak = false;
  if (!cborHead(c, major, value, indefinite, isBreak) || isBreak) return;
  switch (major) {
    case 2:  // byte string
    case 3:  // text string
      c.offset += int(value);
      break;
    case 4:  // array
      cborSkipItems(c, value, indefinite, false);
      break;
    case 5:  // map
      cborSkipItems(c, value, indefinite, true);
      break;
    case 6:  // tag: the tagged item follows
      cborSkip(c);
      break;
    default:
      break;  // integers and simple values carry no payload
  }
}

inline void cborText(CborCursor &c, uint64_t length, char *out, size_t capacity) {
  const uint64_t copied = min(length, uint64_t(capacity - 1));
  memcpy(out, c.data + c.offset, size_t(copied));
  out[copied] = 0;
  c.offset += int(length);
}

// Reads the value for `sid`, storing it if it is a leaf this dashboard shows, and otherwise
// descending so that nested containers (statistics, and whatever else the device sends) are
// traversed rather than skipped.
void coreconfValue(CborCursor &c, uint32_t sid, PortTable *table, int portIndex);

inline void coreconfMap(CborCursor &c, uint64_t count, bool indefinite, uint32_t parentSid,
                        PortTable *table, int portIndex) {
  uint64_t seen = 0;
  while (true) {
    if (indefinite) {
      if (c.offset >= c.length) return;
      if (c.data[c.offset] == 0xFF) { ++c.offset; return; }
    } else if (seen++ >= count) {
      return;
    }
    uint8_t major = 0;
    uint64_t delta = 0;
    bool inner = false, isBreak = false;
    if (!cborHead(c, major, delta, inner, isBreak) || isBreak) return;
    if (major != 0) { cborSkip(c); continue; }  // only unsigned keys are SID deltas
    coreconfValue(c, parentSid + uint32_t(delta), table, portIndex);
  }
}

inline void coreconfValue(CborCursor &c, uint32_t sid, PortTable *table, int portIndex) {
  const int start = c.offset;
  uint8_t major = 0;
  uint64_t value = 0;
  bool indefinite = false, isBreak = false;
  if (!cborHead(c, major, value, indefinite, isBreak) || isBreak) return;

  PortState *port = (portIndex >= 0 && portIndex < table->count) ? &table->ports[portIndex]
                                                                : nullptr;

  if (major == 3) {  // text
    if (port != nullptr && sid == ketiSidFor("ietf-interfaces:interfaces/interface/name")) {
      cborText(c, value, port->name, sizeof(port->name));
      return;
    }
    if (port != nullptr &&
        sid == ketiSidFor("ietf-interfaces:interfaces/interface/phys-address")) {
      cborText(c, value, port->physAddress, sizeof(port->physAddress));
      return;
    }
    c.offset += int(value);
    return;
  }

  if (major == 0 && port != nullptr) {
    struct Leaf { const char *path; uint64_t PortState::*field; };
    static const Leaf kLeaves[] = {
        {"ietf-interfaces:interfaces/interface/statistics/in-octets", &PortState::inOctets},
        {"ietf-interfaces:interfaces/interface/statistics/out-octets", &PortState::outOctets},
        {"ietf-interfaces:interfaces/interface/statistics/in-errors", &PortState::inErrors},
        {"ietf-interfaces:interfaces/interface/statistics/out-errors", &PortState::outErrors},
        {"ietf-interfaces:interfaces/interface/statistics/in-discards", &PortState::inDiscards},
        {"ietf-interfaces:interfaces/interface/statistics/out-discards", &PortState::outDiscards},
        {"ietf-interfaces:interfaces/interface/statistics/in-unicast-pkts",
         &PortState::inUnicast},
        {"ietf-interfaces:interfaces/interface/statistics/out-unicast-pkts",
         &PortState::outUnicast},
    };
    for (const auto &leaf : kLeaves) {
      if (sid == ketiSidFor(leaf.path)) {
        port->*(leaf.field) = value;
        return;
      }
    }
    if (sid == ketiSidFor("ietf-interfaces:interfaces/interface/oper-status")) {
      port->operStatus = uint8_t(value);
      return;
    }
    static const char *kEthBase =
        "ietf-interfaces:interfaces/interface/ieee802-ethernet-interface:ethernet";
    char ethPath[200];
    snprintf(ethPath, sizeof(ethPath), "%s/statistics/frame/in-error-fcs-frames", kEthBase);
    if (sid == ketiSidFor(ethPath)) { port->fcsErrors = value; return; }
    snprintf(ethPath, sizeof(ethPath), "%s/statistics/frame/in-error-oversize-frames", kEthBase);
    if (sid == ketiSidFor(ethPath)) { port->oversizeFrames = value; return; }
    snprintf(ethPath, sizeof(ethPath), "%s/statistics/frame/in-error-undersize-frames", kEthBase);
    if (sid == ketiSidFor(ethPath)) { port->undersizeFrames = value; return; }

    static const char *kGateBase =
        "ietf-interfaces:interfaces/interface/ieee802-dot1q-bridge:bridge-port/"
        "ieee802-dot1q-sched-bridge:gate-parameter-table";
    char path[220];
    snprintf(path, sizeof(path), "%s/oper-gate-states", kGateBase);
    if (sid == ketiSidFor(path)) { port->gateStates = value; port->tasSeen = true; return; }
    snprintf(path, sizeof(path), "%s/admin-cycle-time/numerator", kGateBase);
    if (sid == ketiSidFor(path)) { port->cycleNumerator = value; port->tasSeen = true; return; }
    snprintf(path, sizeof(path), "%s/admin-cycle-time/denominator", kGateBase);
    if (sid == ketiSidFor(path)) { port->cycleDenominator = value; port->tasSeen = true; return; }

    // Gate control entries arrive as a list of maps. Nothing in the encoding announces "new
    // entry", so the index leaf is what separates them -- the device sends it first in each.
    snprintf(path, sizeof(path), "%s/admin-control-list/gate-control-entry/index", kGateBase);
    if (sid == ketiSidFor(path)) {
      port->gateSlot = int(value) < PortState::kMaxGates ? int(value) : -1;
      if (port->gateSlot >= 0) port->gateCount = max(port->gateCount, port->gateSlot + 1);
      port->tasSeen = true;
      return;
    }
    snprintf(path, sizeof(path),
             "%s/admin-control-list/gate-control-entry/time-interval-value", kGateBase);
    if (sid == ketiSidFor(path)) {
      if (port->gateSlot >= 0) port->gateInterval[port->gateSlot] = value;
      return;
    }
    snprintf(path, sizeof(path),
             "%s/admin-control-list/gate-control-entry/gate-states-value", kGateBase);
    if (sid == ketiSidFor(path)) {
      if (port->gateSlot >= 0) port->gateMask[port->gateSlot] = uint8_t(value);
      return;
    }
    return;
  }

  // gate-enabled is a boolean, so it arrives as a CBOR simple value rather than an integer.
  if (major == 7 && port != nullptr && (value == 20 || value == 21)) {
    char path[220];
    snprintf(path, sizeof(path),
             "ietf-interfaces:interfaces/interface/ieee802-dot1q-bridge:bridge-port/"
             "ieee802-dot1q-sched-bridge:gate-parameter-table/gate-enabled");
    if (sid == ketiSidFor(path)) {
      port->gateEnabled = value == 21 ? 1 : 0;
      port->tasSeen = true;
    }
    return;
  }

  // gate-enabled is a boolean, so it arrives as a CBOR simple value rather than an integer.
  if (major == 7 && port != nullptr && (value == 20 || value == 21)) {
    char path[220];
    snprintf(path, sizeof(path),
             "ietf-interfaces:interfaces/interface/ieee802-dot1q-bridge:bridge-port/"
             "ieee802-dot1q-sched-bridge:gate-parameter-table/gate-enabled");
    if (sid == ketiSidFor(path)) {
      port->gateEnabled = value == 21 ? 1 : 0;
      port->tasSeen = true;
    }
    return;
  }

  // Speed is a decimal64, which arrives as CBOR tag 4: [exponent, mantissa]. Skipping tags
  // wholesale would silently drop it, and a port list without link speeds hides a 100M port
  // sitting in a gigabit ring.
  if (major == 6 && value == 4 && port != nullptr &&
      sid == ketiSidFor("ietf-interfaces:interfaces/interface/"
                        "ieee802-ethernet-interface:ethernet/speed")) {
    uint8_t innerMajor = 0;
    uint64_t count = 0;
    bool innerIndefinite = false, innerBreak = false;
    if (cborHead(c, innerMajor, count, innerIndefinite, innerBreak) && innerMajor == 4 &&
        count == 2) {
      uint8_t expMajor = 0, mantMajor = 0;
      uint64_t expValue = 0, mantValue = 0;
      bool ignoredA = false, ignoredB = false, breakA = false, breakB = false;
      if (cborHead(c, expMajor, expValue, ignoredA, breakA) &&
          cborHead(c, mantMajor, mantValue, ignoredB, breakB)) {
        // Reported in Gbps, so 0.1 is a 100 Mbps link. Negative exponents arrive as CBOR
        // negative integers, where the encoded value is -(n+1).
        const int64_t exponent = expMajor == 1 ? -int64_t(expValue) - 1 : int64_t(expValue);
        double gbps = double(mantValue);
        for (int64_t i = 0; i < -exponent; ++i) gbps /= 10.0;
        for (int64_t i = 0; i < exponent; ++i) gbps *= 10.0;
        port->speedMbps = uint32_t(gbps * 1000.0 + 0.5);
      }
    }
    return;
  }

  if (major == 4) {  // array
    const bool isInterfaceList = sid == ketiSidFor("ietf-interfaces:interfaces/interface");
    uint64_t seen = 0;
    while (true) {
      if (indefinite) {
        if (c.offset >= c.length) return;
        if (c.data[c.offset] == 0xFF) { ++c.offset; return; }
      } else if (seen++ >= value) {
        return;
      }
      int childIndex = portIndex;
      if (isInterfaceList && table->count < kMaxPorts) {
        childIndex = table->count;
        table->ports[childIndex] = PortState{};
        ++table->count;
      }
      // Each element of a list carries the list's own SID as its parent.
      const int before = c.offset;
      uint8_t elementMajor = 0;
      uint64_t elementValue = 0;
      bool elementIndefinite = false, elementBreak = false;
      if (!cborHead(c, elementMajor, elementValue, elementIndefinite, elementBreak) ||
          elementBreak) {
        return;
      }
      if (elementMajor == 5) {
        coreconfMap(c, elementValue, elementIndefinite, sid, table, childIndex);
      } else {
        c.offset = before;
        cborSkip(c);
      }
    }
  }

  if (major == 5) {  // map: a container, so its children hang off this SID
    coreconfMap(c, value, indefinite, sid, table, portIndex);
    return;
  }

  c.offset = start;
  cborSkip(c);
}

// Finds the text value stored under one absolute SID, wherever it sits in the tree. Used for
// the device's own name: hardcoding "LAN9662" in the console would silently be wrong the day a
// LAN9692 is plugged in, and nothing would flag it.
inline bool coreconfFindText(CborCursor &c, uint32_t parentSid, uint32_t wantedSid, char *out,
                             size_t capacity) {
  uint8_t major = 0;
  uint64_t value = 0;
  bool indefinite = false, isBreak = false;
  const int start = c.offset;
  if (!cborHead(c, major, value, indefinite, isBreak) || isBreak) return false;

  if (major == 3) {
    if (parentSid == wantedSid) {
      cborText(c, value, out, capacity);
      return true;
    }
    c.offset += int(value);
    return false;
  }
  if (major == 5) {  // map: keys are deltas from this node
    uint64_t seen = 0;
    while (true) {
      if (indefinite) {
        if (c.offset >= c.length) return false;
        if (c.data[c.offset] == 0xFF) { ++c.offset; return false; }
      } else if (seen++ >= value) {
        return false;
      }
      uint8_t keyMajor = 0;
      uint64_t delta = 0;
      bool keyIndefinite = false, keyBreak = false;
      if (!cborHead(c, keyMajor, delta, keyIndefinite, keyBreak) || keyBreak) return false;
      if (keyMajor != 0) { cborSkip(c); continue; }
      if (coreconfFindText(c, parentSid + uint32_t(delta), wantedSid, out, capacity)) {
        return true;
      }
    }
  }

  if (major == 4) {  // array: elements inherit the list SID
    uint64_t seen = 0;
    while (true) {
      if (indefinite) {
        if (c.offset >= c.length) return false;
        if (c.data[c.offset] == 0xFF) { ++c.offset; return false; }
      } else if (seen++ >= value) {
        return false;
      }
      if (coreconfFindText(c, parentSid, wantedSid, out, capacity)) return true;
    }
  }
  c.offset = start;
  cborSkip(c);
  return false;
}

/// Finds an unsigned or boolean value stored under one absolute SID. Booleans come back as 1
/// or 0 so a single call covers both, which is what the gate table needs: gate-enabled is a
/// boolean sitting beside integer cycle times.
inline bool coreconfFindUint(CborCursor &c, uint32_t parentSid, uint32_t wantedSid,
                             uint64_t *out) {
  uint8_t major = 0;
  uint64_t value = 0;
  bool indefinite = false, isBreak = false;
  const int start = c.offset;
  if (!cborHead(c, major, value, indefinite, isBreak) || isBreak) return false;

  if (major == 0 && parentSid == wantedSid) { *out = value; return true; }
  if (major == 7 && parentSid == wantedSid && (value == 20 || value == 21)) {
    *out = value == 21 ? 1 : 0;   // CBOR false is 20, true is 21
    return true;
  }
  if (major == 5) {
    uint64_t seen = 0;
    while (true) {
      if (indefinite) {
        if (c.offset >= c.length) return false;
        if (c.data[c.offset] == 0xFF) { ++c.offset; return false; }
      } else if (seen++ >= value) {
        return false;
      }
      uint8_t keyMajor = 0;
      uint64_t delta = 0;
      bool keyIndefinite = false, keyBreak = false;
      if (!cborHead(c, keyMajor, delta, keyIndefinite, keyBreak) || keyBreak) return false;
      if (keyMajor != 0) { cborSkip(c); continue; }
      if (coreconfFindUint(c, parentSid + uint32_t(delta), wantedSid, out)) return true;
    }
  }
  if (major == 4) {
    uint64_t seen = 0;
    while (true) {
      if (indefinite) {
        if (c.offset >= c.length) return false;
        if (c.data[c.offset] == 0xFF) { ++c.offset; return false; }
      } else if (seen++ >= value) {
        return false;
      }
      if (coreconfFindUint(c, parentSid, wantedSid, out)) return true;
    }
  }
  c.offset = start;
  cborSkip(c);
  return false;
}

inline bool coreconfUint(const uint8_t *payload, int length, uint32_t wantedSid,
                         uint64_t *out) {
  CborCursor c{payload, length, 0};
  return coreconfFindUint(c, 0, wantedSid, out);
}

inline bool coreconfMachine(const uint8_t *payload, int length, uint32_t wantedSid, char *out,
                            size_t capacity) {
  CborCursor c{payload, length, 0};
  return coreconfFindText(c, 0, wantedSid, out, capacity);
}

// Entry point: the outermost map is keyed by absolute SIDs rather than deltas.
inline bool parseInterfaces(const uint8_t *payload, int length, PortTable *table) {
  table->count = 0;
  CborCursor c{payload, length, 0};
  uint8_t major = 0;
  uint64_t value = 0;
  bool indefinite = false, isBreak = false;
  if (!cborHead(c, major, value, indefinite, isBreak) || isBreak || major != 5) return false;
  coreconfMap(c, value, indefinite, 0, table, -1);
  return true;
}
