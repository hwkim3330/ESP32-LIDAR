// Write a gate schedule to a switch, in one iPATCH, from this board.
//
// The shape is not invented here: it was read out of the bytes keti-tsn-cli produces for the same
// YAML, which is the encoder this bench has always trusted. One map keyed by [gate-parameter-table
// SID, port], six members, delta-SIDs relative to their own parent:
//
//   +24 gate-enabled        +14 admin-gate-states     +10 admin-cycle-time {+3 num, +2 den}
//   +1  admin-base-time     {+2 seconds, +1 nanoseconds}
//   +4  admin-control-list  {+1 gate-control-entry [ {+2 index, +3 op, +1 states, +4 interval} ]}
//   +15 config-change
//
// Three things make this safe enough to expose to a tablet, and all three were learned the hard
// way on this bench:
//
//   1. One patch. Sent as five, the control list was refused while gate-enabled and config-change
//      were accepted, so the switch gated against no schedule and shut the only path back.
//   2. A base time in the future. Zero is in the past; the write succeeds, `config-change-error`
//      goes to 1, and the port keeps running whatever it ran before -- including a schedule you
//      were trying to remove.
//   3. Never close every class. A window with gate-states 0 closes the control path too. Every
//      preset here closes TC0 alone, and the code refuses anything else.
#pragma once
#include <Arduino.h>

#include "sid_tables.h"
#include "coap_client.h"


struct TasWindow {
  uint32_t intervalNs;
  uint8_t states;   // bitmask per traffic class; 0xFF all open, 0xFE TC0 closed
};

struct TasPreset {
  const char *id;
  const char *description;
  uint32_t cycleNs;
  int windowCount;
  TasWindow windows[4];
};

// Coarser than the sensor's 3125 us spacing, or nothing queues and nothing changes -- which is
// exactly what the first attempt at this measured, at 1 ms with 200 us closed.
static const TasPreset kTasPresets[] = {
    {"open", "no shaping", 1000000, 1, {{1000000, 0xFF}}},
    {"10ms", "10 ms cycle, TC0 closed 5 ms", 10000000, 2,
     {{5000000, 0xFF}, {5000000, 0xFE}}},
    {"20ms", "20 ms cycle, TC0 closed 12 ms", 20000000, 2,
     {{8000000, 0xFF}, {12000000, 0xFE}}},
    {"5ms", "5 ms cycle, TC0 closed 2 ms", 5000000, 2,
     {{3000000, 0xFF}, {2000000, 0xFE}}},
};
constexpr int kTasPresetCount = sizeof(kTasPresets) / sizeof(kTasPresets[0]);

inline size_t tasUint(uint8_t *out, uint64_t value, uint8_t majorType) {
  return cborUint(out, uint32_t(value), majorType);
}

inline size_t buildScheduleCbor(uint8_t *out, const char *port, const TasPreset &preset,
                                uint64_t baseSeconds) {
  const uint32_t table = ketiSidFor(
      "ietf-interfaces:interfaces/interface/ieee802-dot1q-bridge:bridge-port/"
      "ieee802-dot1q-sched-bridge:gate-parameter-table");
  size_t n = 0;
  n += tasUint(out + n, 1, 5);            // map of one
  n += tasUint(out + n, 2, 4);            // key: [sid, port]
  n += tasUint(out + n, table, 0);
  const size_t keyLength = strlen(port);
  n += tasUint(out + n, keyLength, 3);
  memcpy(out + n, port, keyLength);
  n += keyLength;

  n += tasUint(out + n, 6, 5);            // the container, six members
  n += tasUint(out + n, 24, 0); out[n++] = 0xF5;                 // gate-enabled true
  n += tasUint(out + n, 14, 0); n += tasUint(out + n, 0xFF, 0);  // admin-gate-states
  n += tasUint(out + n, 10, 0);           // admin-cycle-time
  n += tasUint(out + n, 2, 5);
  n += tasUint(out + n, 3, 0); n += tasUint(out + n, preset.cycleNs, 0);
  n += tasUint(out + n, 2, 0); n += tasUint(out + n, 1000000000u, 0);
  n += tasUint(out + n, 1, 0);            // admin-base-time
  n += tasUint(out + n, 2, 5);
  n += tasUint(out + n, 2, 0); n += tasUint(out + n, baseSeconds, 0);
  n += tasUint(out + n, 1, 0); n += tasUint(out + n, 0, 0);
  n += tasUint(out + n, 4, 0);            // admin-control-list
  n += tasUint(out + n, 1, 5);
  n += tasUint(out + n, 1, 0);
  n += tasUint(out + n, preset.windowCount, 4);
  for (int i = 0; i < preset.windowCount; i++) {
    n += tasUint(out + n, 4, 5);
    n += tasUint(out + n, 2, 0); n += tasUint(out + n, i, 0);
    n += tasUint(out + n, 3, 0); n += tasUint(out + n, 23003, 0);  // set-gate-states
    n += tasUint(out + n, 1, 0); n += tasUint(out + n, preset.windows[i].states, 0);
    n += tasUint(out + n, 4, 0); n += tasUint(out + n, preset.windows[i].intervalNs, 0);
  }
  n += tasUint(out + n, 15, 0); out[n++] = 0xF5;                 // config-change true
  return n;
}

inline const TasPreset *tasPresetAt(int index) {
  if (index < 0 || index >= kTasPresetCount) return nullptr;
  return &kTasPresets[index];
}
