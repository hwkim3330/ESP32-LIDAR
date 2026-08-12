// Put IPv4 fragments back together, because this build will not.
//
// `CONFIG_LWIP_IP4_REASSEMBLY is not set` in the Arduino ESP32 libraries, and `IP_REASSEMBLY 0`
// in lwipopts. Fragmented datagrams are not delayed or dropped under load -- they are discarded
// on arrival, always. That is invisible from above: the wire is saturated and the socket reads
// zero, which looks exactly like a board too slow to keep up. It is not. It is a board that was
// never going to receive them.
//
// It matters now because a 64 beam sensor cannot avoid fragmenting. A column is 12 bytes plus
// four per pixel, so 64 beams make 268, and the sensor refuses any columns-per-packet that is
// not a multiple of 16 -- "columns_per_packet must be a positive multiple of 16". The smallest
// datagram it will send is 32 + 16*268 + 32 = 4352 bytes, which is three fragments on any
// ordinary link. Narrowing the azimuth window changes how many arrive, not whether they survive.
//
// So the frames are taken before lwIP sees them. Everything that is not a fragment of the
// sensor's stream is handed straight on, so CoAP, HTTP, DHCP and the rest carry on unaware.
#pragma once
#include <Arduino.h>
#include <esp_eth.h>
#include <esp_netif.h>

extern esp_netif_t *gEthNetif;
extern esp_eth_handle_t gEthHandle;

// One datagram at a time. The sensor sends its fragments back to back and in order, so a single
// slot is enough; a fragment for a different identifier while one is in progress means the one in
// progress was never going to complete, and dropping it is better than holding it.
constexpr int kMaxDatagram = 8192;
uint8_t reassembled[kMaxDatagram];
int reassembledLength = 0;
uint16_t reassemblyId = 0;
int reassemblyHave = 0;
bool reassemblyOpen = false;

uint32_t fragmentsSeen = 0, datagramsCompleted = 0, datagramsAbandoned = 0;

// Set by the sketch: what to do with a datagram once it is whole.
void (*onReassembled)(const uint8_t *payload, int length, uint32_t sourceIp) = nullptr;
uint16_t reassemblyPort = 7502;

esp_err_t ethernetInput(esp_eth_handle_t handle, uint8_t *frame, uint32_t length, void *context) {
  // 14 byte Ethernet header, IPv4 only, and only the sensor's port. Everything else is lwIP's.
  const bool isIpv4 = length > 34 && frame[12] == 0x08 && frame[13] == 0x00;
  if (isIpv4) {
    const uint8_t *ip = frame + 14;
    const int headerBytes = (ip[0] & 0x0F) * 4;
    const uint16_t identifier = (ip[4] << 8) | ip[5];
    const uint16_t flagsOffset = (ip[6] << 8) | ip[7];
    const bool moreFragments = flagsOffset & 0x2000;
    const int offset = (flagsOffset & 0x1FFF) * 8;
    const int totalLength = (ip[2] << 8) | ip[3];
    const int payloadBytes = totalLength - headerBytes;
    const uint8_t *payload = ip + headerBytes;

    if ((moreFragments || offset > 0) && ip[9] == 17 /* UDP */) {
      // A first fragment carries the UDP header; later ones do not, so the port is only knowable
      // from the first. Everything belonging to an identifier we started is kept.
      bool mine = offset > 0 && reassemblyOpen && identifier == reassemblyId;
      if (offset == 0) {
        const uint16_t destinationPort = (payload[2] << 8) | payload[3];
        mine = destinationPort == reassemblyPort;
        if (mine) {
          if (reassemblyOpen) datagramsAbandoned++;
          reassemblyOpen = true;
          reassemblyId = identifier;
          reassemblyHave = 0;
          reassembledLength = 0;
        }
      }
      if (mine) {
        fragmentsSeen++;
        // Offsets are into the datagram including its UDP header; the payload we want starts
        // after those eight bytes, but they arrive inside fragment zero, so copy verbatim and
        // step over them at the end.
        if (offset + payloadBytes <= kMaxDatagram) {
          memcpy(reassembled + offset, payload, payloadBytes);
          reassemblyHave += payloadBytes;
          if (!moreFragments) reassembledLength = offset + payloadBytes;
        }
        if (reassembledLength && reassemblyHave >= reassembledLength) {
          reassemblyOpen = false;
          datagramsCompleted++;
          if (onReassembled)
            onReassembled(reassembled + 8, reassembledLength - 8,
                          (ip[12] << 24) | (ip[13] << 16) | (ip[14] << 8) | ip[15]);
        }
        free(frame);
        return ESP_OK;
      }
    }
  }
  return esp_netif_receive(gEthNetif, frame, length, nullptr);
}

inline void reassemblyBegin() {
  esp_eth_update_input_path(gEthHandle, ethernetInput, nullptr);
}
