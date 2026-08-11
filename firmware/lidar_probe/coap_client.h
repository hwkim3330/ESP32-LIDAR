// CoAP/CORECONF client bits for the switch controller.
//
// These live in a header rather than the .ino because the Arduino preprocessor hoists function
// prototypes above everything else in a sketch, which breaks any prototype that mentions a
// type the sketch itself declares.
#pragma once

#include <Arduino.h>
#include <NetworkUdp.h>

constexpr uint8_t kCoapFetch = 0x05;
constexpr uint8_t kCoapPost = 0x02;
constexpr uint8_t kCoapIpatch = 0x07;
constexpr uint16_t kContentFormatYangIdentifiersCbor = 141;
constexpr uint16_t kContentFormatYangInstancesCbor = 142;
constexpr uint16_t kContentFormatYangDataCborSid = 140;

// Owned here because the fetch helpers below are the only users.
extern NetworkUDP udp;
extern uint16_t messageId;
// Not const: the switch address comes from the controller table at startup, keyed on the
// board's MAC, so one firmware serves every controller.
extern IPAddress kSwitch;
extern const uint16_t kCoapPort;

size_t cborUint(uint8_t *out, uint32_t value, uint8_t majorType) {
  const uint8_t mt = majorType << 5;
  if (value < 24)          { out[0] = mt | value; return 1; }
  if (value < 0x100)       { out[0] = mt | 24; out[1] = value; return 2; }
  if (value < 0x10000)     { out[0] = mt | 25; out[1] = value >> 8; out[2] = value; return 3; }
  out[0] = mt | 26;
  out[1] = value >> 24; out[2] = value >> 16; out[3] = value >> 8; out[4] = value;
  return 5;
}

// A CoAP response, split into the parts this firmware cares about.
struct CoapResponse {
  uint8_t code = 0;
  const uint8_t *payload = nullptr;
  int payloadLength = 0;
  bool hasBlock2 = false;
  uint32_t blockNumber = 0;
  bool moreBlocks = false;
  uint8_t sizeExponent = 0;
};

// Walks the option list to the payload, picking up Block2 (option 23) on the way. Options are
// delta-encoded against the previous option number, so they can only be read in order.
bool parseCoapResponse(const uint8_t *buffer, int length, CoapResponse *out) {
  if (length < 4) return false;
  out->code = buffer[1];
  int i = 4 + (buffer[0] & 0x0F);
  uint32_t optionNumber = 0;
  while (i < length && buffer[i] != 0xFF) {
    uint32_t delta = buffer[i] >> 4;
    uint32_t optionLength = buffer[i] & 0x0F;
    ++i;
    if (delta == 13) { delta = 13 + buffer[i]; i += 1; }
    else if (delta == 14) { delta = 269 + ((buffer[i] << 8) | buffer[i + 1]); i += 2; }
    if (optionLength == 13) { optionLength = 13 + buffer[i]; i += 1; }
    else if (optionLength == 14) { optionLength = 269 + ((buffer[i] << 8) | buffer[i + 1]); i += 2; }
    optionNumber += delta;
    if (optionNumber == 23) {  // Block2
      uint32_t value = 0;
      for (uint32_t b = 0; b < optionLength && b < 4; ++b) value = (value << 8) | buffer[i + b];
      out->hasBlock2 = true;
      out->blockNumber = value >> 4;
      out->moreBlocks = (value >> 3) & 1;
      out->sizeExponent = value & 0x07;
    }
    i += optionLength;
  }
  if (i < length && buffer[i] == 0xFF) {
    ++i;
    out->payload = buffer + i;
    out->payloadLength = length - i;
  }
  return true;
}

size_t encodeOption(uint8_t *out, uint32_t delta, uint32_t value) {
  size_t n = 0;
  const uint8_t valueLength = value < 0x100 ? 1 : (value < 0x10000 ? 2 : 3);
  out[n++] = (delta << 4) | valueLength;
  for (int b = valueLength - 1; b >= 0; --b) out[n++] = (value >> (8 * b)) & 0xFF;
  return n;
}

// Fetches one SID, assembling a block-wise response. The switch splits anything larger than
// one block, and the interface subtree is far larger -- the checksum reply fitted in a single
// datagram only because it is 22 bytes.
int fetchSid(uint32_t sid, uint8_t *out, size_t capacity, uint8_t *codeOut, int *blockCount) {
  constexpr uint8_t kSizeExponent = 4;  // 256-byte blocks, matching keti-tsn-cli
  int total = 0;
  uint32_t blockNumber = 0;
  if (blockCount != nullptr) *blockCount = 0;

  for (int guard = 0; guard < 128; ++guard) {
    uint8_t packet[64];
    size_t n = 0;
    packet[n++] = 0x40;
    packet[n++] = kCoapFetch;
    const uint16_t id = messageId++;
    packet[n++] = id >> 8;
    packet[n++] = id & 0xFF;
    packet[n++] = 0xB1;  // Uri-Path (11)
    packet[n++] = 'c';
    packet[n++] = 0x11;  // Content-Format (12), delta 1
    packet[n++] = uint8_t(kContentFormatYangIdentifiersCbor);
    n += encodeOption(packet + n, 11, (blockNumber << 4) | kSizeExponent);  // Block2 (23)
    packet[n++] = 0xFF;
    n += cborUint(packet + n, 1, 4);
    n += cborUint(packet + n, sid, 0);

    udp.beginPacket(kSwitch, kCoapPort);
    udp.write(packet, n);
    udp.endPacket();

    bool answered = false;
    const uint32_t deadline = millis() + 3000;
    while (millis() < deadline && !answered) {
      const int size = udp.parsePacket();
      if (size <= 0) { delay(2); continue; }
      static uint8_t buffer[1500];
      const int got = udp.read(buffer, sizeof(buffer));
      if (got < 4) continue;
      if (uint16_t((buffer[2] << 8) | buffer[3]) != id) continue;
      CoapResponse response;
      if (!parseCoapResponse(buffer, got, &response)) continue;
      *codeOut = response.code;
      if ((response.code >> 5) != 2) return total;  // an error class ends the exchange
      if (response.payloadLength > 0) {
        const int room = int(capacity) - total;
        const int copied = min(response.payloadLength, room);
        memcpy(out + total, response.payload, copied);
        total += copied;
        if (copied < response.payloadLength) return total;  // out of room; say what we have
      }
      if (blockCount != nullptr) ++*blockCount;
      answered = true;
      if (!response.hasBlock2 || !response.moreBlocks) return total;
      blockNumber = response.blockNumber + 1;
    }
    if (!answered) return total > 0 ? total : -1;
  }
  return total;
}


// Fetches one keyed list entry: the payload is [[sid, key]] rather than the bare [sid] used
// for a whole subtree. Used for per-port detail that is too big to carry in every snapshot --
// the interface subtree is already 6.5 KB across thirteen ports, and a gate table each would
// swamp it. The inspector asks for one port at a time instead.
int fetchSidKeyed(uint32_t sid, const char *key, uint8_t *out, size_t capacity,
                  uint8_t *codeOut, int *blockCount, bool sendBlock2 = true) {
  constexpr uint8_t kSizeExponent = 4;
  int total = 0;
  uint32_t blockNumber = 0;
  if (blockCount != nullptr) *blockCount = 0;

  for (int guard = 0; guard < 128; ++guard) {
    uint8_t packet[96];
    size_t n = 0;
    packet[n++] = 0x40;
    packet[n++] = kCoapFetch;
    const uint16_t id = messageId++;
    packet[n++] = id >> 8;
    packet[n++] = id & 0xFF;
    packet[n++] = 0xB1;
    packet[n++] = 'c';
    packet[n++] = 0x11;
    packet[n++] = uint8_t(kContentFormatYangIdentifiersCbor);
    if (sendBlock2) n += encodeOption(packet + n, 11, (blockNumber << 4) | kSizeExponent);
    packet[n++] = 0xFF;
    n += cborUint(packet + n, 1, 4);          // array of one query
    n += cborUint(packet + n, 2, 4);          // [sid, key]
    n += cborUint(packet + n, sid, 0);
    const size_t keyLength = strlen(key);
    n += cborUint(packet + n, keyLength, 3);
    memcpy(packet + n, key, keyLength);
    n += keyLength;

    if (blockNumber == 0) {
      Serial.print("  keyed fetch request: ");
      for (size_t i = 0; i < n; ++i) Serial.printf("%02X ", packet[i]);
      Serial.println();
    }
    udp.beginPacket(kSwitch, kCoapPort);
    udp.write(packet, n);
    udp.endPacket();

    bool answered = false;
    const uint32_t deadline = millis() + 3000;
    while (millis() < deadline && !answered) {
      const int size = udp.parsePacket();
      if (size <= 0) { delay(2); continue; }
      static uint8_t buffer[1500];
      const int got = udp.read(buffer, sizeof(buffer));
      if (got < 4) continue;
      if (uint16_t((buffer[2] << 8) | buffer[3]) != id) continue;
      CoapResponse response;
      if (!parseCoapResponse(buffer, got, &response)) continue;
      *codeOut = response.code;
      if ((response.code >> 5) != 2) {
        Serial.print("  reply: ");
        for (int i = 0; i < got && i < 24; ++i) Serial.printf("%02X ", buffer[i]);
        Serial.println();
        return total;
      }
      if (response.payloadLength > 0) {
        const int room = int(capacity) - total;
        const int copied = min(response.payloadLength, room);
        memcpy(out + total, response.payload, copied);
        total += copied;
        if (copied < response.payloadLength) return total;
      }
      if (blockCount != nullptr) ++*blockCount;
      answered = true;
      if (!response.hasBlock2 || !response.moreBlocks) return total;
      blockNumber = response.blockNumber + 1;
    }
    if (!answered) return total > 0 ? total : -1;
  }
  return total;
}

// Invokes an RPC: POST with {sid: null}. Same endpoint and framing as a write; only the
// method and the shape of the payload differ.
bool postRpc(uint32_t sid, uint8_t *codeOut) {
  uint8_t packet[64];
  size_t n = 0;
  packet[n++] = 0x40;
  packet[n++] = kCoapPost;
  const uint16_t id = messageId++;
  packet[n++] = id >> 8;
  packet[n++] = id & 0xFF;
  packet[n++] = 0xB1;
  packet[n++] = 'c';
  packet[n++] = 0x11;
  packet[n++] = uint8_t(kContentFormatYangInstancesCbor);
  packet[n++] = 0x51;
  packet[n++] = uint8_t(kContentFormatYangInstancesCbor);
  packet[n++] = 0xFF;
  n += cborUint(packet + n, 1, 5);   // map(1)
  n += cborUint(packet + n, sid, 0);
  packet[n++] = 0xF6;                // null: the RPC takes no input

  udp.beginPacket(kSwitch, kCoapPort);
  udp.write(packet, n);
  udp.endPacket();

  const uint32_t deadline = millis() + 5000;   // saving to flash is slower than a read
  while (millis() < deadline) {
    const int size = udp.parsePacket();
    if (size <= 0) { delay(2); continue; }
    uint8_t buffer[256];
    const int got = udp.read(buffer, sizeof(buffer));
    if (got < 4) continue;
    if (uint16_t((buffer[2] << 8) | buffer[3]) != id) continue;
    *codeOut = buffer[1];
    return (buffer[1] >> 5) == 2;
  }
  return false;
}

// Sends one iPATCH whose payload is already encoded. Everything below builds a payload and
// hands it here, because the switch takes each item as its own request -- that is what the CLI
// emits for a schedule, and matching what works beats inventing a batched form.
bool patchRaw(const uint8_t *payload, size_t payloadLength, uint8_t *codeOut) {
  uint8_t packet[256];
  size_t n = 0;
  packet[n++] = 0x40;
  packet[n++] = kCoapIpatch;
  const uint16_t id = messageId++;
  packet[n++] = id >> 8;
  packet[n++] = id & 0xFF;
  packet[n++] = 0xB1;
  packet[n++] = 'c';
  packet[n++] = 0x11;
  packet[n++] = uint8_t(kContentFormatYangInstancesCbor);
  packet[n++] = 0x51;
  packet[n++] = uint8_t(kContentFormatYangDataCborSid);
  packet[n++] = 0xFF;
  if (n + payloadLength > sizeof(packet)) return false;
  memcpy(packet + n, payload, payloadLength);
  n += payloadLength;

  udp.beginPacket(kSwitch, kCoapPort);
  udp.write(packet, n);
  udp.endPacket();

  const uint32_t deadline = millis() + 3000;
  while (millis() < deadline) {
    const int size = udp.parsePacket();
    if (size <= 0) { delay(2); continue; }
    uint8_t buffer[512];
    const int got = udp.read(buffer, sizeof(buffer));
    if (got < 4) continue;
    if (uint16_t((buffer[2] << 8) | buffer[3]) != id) continue;
    *codeOut = buffer[1];
    return (buffer[1] >> 5) == 2;
  }
  return false;
}

// {[leafSid, key]: <unsigned>}
size_t buildPatchUint(uint8_t *out, uint32_t leafSid, const char *key, uint64_t value) {
  size_t n = 0;
  n += cborUint(out + n, 1, 5);
  n += cborUint(out + n, 2, 4);
  n += cborUint(out + n, leafSid, 0);
  const size_t keyLength = strlen(key);
  n += cborUint(out + n, keyLength, 3);
  memcpy(out + n, key, keyLength);
  n += keyLength;
  n += cborUint(out + n, value, 0);
  return n;
}

// {[leafSid, key]: <bool>}
size_t buildPatchBool(uint8_t *out, uint32_t leafSid, const char *key, bool value) {
  size_t n = 0;
  n += cborUint(out + n, 1, 5);
  n += cborUint(out + n, 2, 4);
  n += cborUint(out + n, leafSid, 0);
  const size_t keyLength = strlen(key);
  n += cborUint(out + n, keyLength, 3);
  memcpy(out + n, key, keyLength);
  n += keyLength;
  out[n++] = value ? 0xF5 : 0xF4;
  return n;
}

// Writes one leaf of one list entry: iPATCH with a payload of {[leafSid, key]: value}.
//
// The shape came from the device, not from a guess. The first attempt addressed the list entry
// and nested the leaf inside it -- {[2033,"2"]: {3: false}} -- which the switch answered with
// "List keys not allowed", error-data-node 2033. Over Ethernet that same request simply got no
// reply at all, which is why it had to be reproduced over the serial link where the CLI could
// print the error. The accepted form names the leaf directly and carries the value inline:
//   A1 82 19 07F4 61 32 F5   =  {[2036 "enabled", "2"]: true}
bool patchListLeafBool(const char *key, uint32_t leafSid, bool value, uint8_t *codeOut,
                       bool sendAccept) {
  uint8_t packet[96];
  size_t n = 0;
  packet[n++] = 0x40;
  packet[n++] = kCoapIpatch;
  const uint16_t id = messageId++;
  packet[n++] = id >> 8;
  packet[n++] = id & 0xFF;

  packet[n++] = 0xB1;                                        // Uri-Path (11)
  packet[n++] = 'c';
  packet[n++] = 0x11;                                        // Content-Format (12), delta 1
  packet[n++] = uint8_t(kContentFormatYangInstancesCbor);
  if (sendAccept) {
    packet[n++] = 0x51;                                      // Accept (17), delta 5
    packet[n++] = uint8_t(kContentFormatYangDataCborSid);
  }

  packet[n++] = 0xFF;
  n += cborUint(packet + n, 1, 5);                           // map(1)
  n += cborUint(packet + n, 2, 4);                           // key: array(2) = [leaf SID, key]
  n += cborUint(packet + n, leafSid, 0);
  const size_t keyLength = strlen(key);
  n += cborUint(packet + n, keyLength, 3);                   // text(keyLength)
  memcpy(packet + n, key, keyLength);
  n += keyLength;
  packet[n++] = value ? 0xF5 : 0xF4;                         // the value, not a nested map

  udp.beginPacket(kSwitch, kCoapPort);
  udp.write(packet, n);
  udp.endPacket();

  const uint32_t deadline = millis() + 3000;
  while (millis() < deadline) {
    const int size = udp.parsePacket();
    if (size <= 0) { delay(2); continue; }
    uint8_t buffer[512];
    const int got = udp.read(buffer, sizeof(buffer));
    if (got < 4) continue;
    if (uint16_t((buffer[2] << 8) | buffer[3]) != id) continue;
    *codeOut = buffer[1];
    // Type lives in bits 5-4 of the first byte: 0 CON, 1 NON, 2 ACK, 3 RST. A reset with code
    // 0.00 means the message was rejected at the protocol level rather than by the datastore,
    // which points at the options rather than at the payload.
    Serial.printf("  iPATCH reply: hdr 0x%02X (type %d), code %d.%02d, %d bytes\n", buffer[0],
                  (buffer[0] >> 4) & 0x03, buffer[1] >> 5, buffer[1] & 0x1F, got);
    for (int i = 0; i < got && i < 24; ++i) Serial.printf("%02X ", buffer[i]);
    Serial.println();
    return (buffer[1] >> 5) == 2;                            // 2.xx is success
  }
  return false;
}
