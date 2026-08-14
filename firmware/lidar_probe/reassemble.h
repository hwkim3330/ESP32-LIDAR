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

// Two slots, and nothing but copying happens in the driver's thread.
//
// The first version decoded the datagram inside the input callback -- 4352 bytes copied and a
// thousand pixels unpacked, at 320 a second, in the thread whose job is to empty the W5500 before
// it overflows. It does overflow: the driver starts reporting `spi transmit failed` and
// `read payload failed, len=1434, offset=48102`, which is a read pointer that has run off the
// end, and from then on it is wedged and lwIP receives nothing either. The SPI errors and the
// deaf network were one fault, not two.
//
// So the callback fills a buffer and swaps it. A task on the other core does the work. The board
// has two cores and this is what the second one is for.
// A ring, not a pair. Two slots means the decoder has to finish one datagram before the next
// completes, and at ninety a second it sometimes does not -- 263 reassembled, 157 dropped, with
// nothing wrong but the handoff.
//
// Two, and it has to be two. These are static and therefore internal DRAM, which the radios also
// want. Six slots is 48 kB: the board came up asserting inside the BLE controller and died on the
// interrupt watchdog. Three is 24 kB: BLE init hangs, silently, right after the web server starts.
// The megabytes this part advertises are PSRAM, and neither DMA nor a controller's queues can use
// them. So the handoff gets one slot being filled and one being decoded, and the way to lose
// fewer datagrams is to stop interrupting the decoder rather than to give it more room.
constexpr int kMaxDatagram = 8192;
constexpr int kSlots = 2;
uint8_t reassembleSlots[kSlots][kMaxDatagram];
int reassembleSlot = 0;
volatile int readyLength[kSlots] = {0};
volatile uint32_t readySourceOf[kSlots] = {0};
volatile bool slotReady[kSlots] = {false};
volatile int nextToDecode = 0;
volatile uint32_t datagramsDropped = 0;
#define reassembled (reassembleSlots[reassembleSlot])
int reassembledLength = 0;
uint16_t reassemblyId = 0;
int reassemblyHave = 0;
bool reassemblyOpen = false;

uint32_t fragmentsSeen = 0, datagramsCompleted = 0, datagramsAbandoned = 0;

// Set while something above needs lwIP to see every frame. Declared here rather than beside its
// helper because the input path is below and reads it on every frame.
volatile bool reassemblyPaused = false;

// Set by the sketch: what to do with a datagram once it is whole.
void (*onReassembled)(const uint8_t *payload, int length, uint32_t sourceIp) = nullptr;
uint16_t reassemblyPort = 7502;

esp_err_t ethernetInput(esp_eth_handle_t handle, uint8_t *frame, uint32_t length, void *context) {
  // 14 byte Ethernet header, IPv4 only, and only the sensor's port. Everything else is lwIP's.
  const bool isIpv4 = !reassemblyPaused && length > 34 && frame[12] == 0x08 && frame[13] == 0x00;
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
          // Hand it over and take the other slot. If the decoder has not finished with the last
          // one, this one is dropped -- late is worse than missing for a sensor that will send
          // another in three milliseconds, and stalling here is what wedged the driver before.
          if (!slotReady[reassembleSlot]) {
            readyLength[reassembleSlot] = reassembledLength - 8;
            readySourceOf[reassembleSlot] =
                (ip[12] << 24) | (ip[13] << 16) | (ip[14] << 8) | ip[15];
            slotReady[reassembleSlot] = true;
            reassembleSlot = (reassembleSlot + 1) % kSlots;
          } else {
            datagramsDropped++;
          }
        }
        free(frame);
        return ESP_OK;
      }
    }
  }
  return esp_netif_receive(gEthNetif, frame, length, nullptr);
}

// The other core. Priority above the Arduino loop so a datagram is decoded promptly, and a tick
// of sleep when there is nothing, so the idle task still runs and the watchdog stays quiet.
void reassemblyTask(void *) {
  for (;;) {
    const int slot = nextToDecode;
    if (!slotReady[slot]) { vTaskDelay(1); continue; }
    if (onReassembled)
      onReassembled(reassembleSlots[slot] + 8, readyLength[slot], readySourceOf[slot]);
    slotReady[slot] = false;
    nextToDecode = (slot + 1) % kSlots;
  }
}

bool reassemblyActive = false;

// Hand everything back to lwIP for a moment.
//
// Reassembly takes the driver's input path, and while the sensor is streaming that path is busy
// enough that a TCP request to the sensor times out reading its reply -- `beam_intrinsics -11`,
// with the connection made and the body never arriving. The beam angles are fetched exactly once
// per boot and every point drawn depends on them, so it is worth standing aside for a second.
// Fragments that arrive while paused are lost, which costs a frame of a room that is not moving.
struct ReassemblyPause {
  ReassemblyPause() { reassemblyPaused = true; vTaskDelay(pdMS_TO_TICKS(20)); }
  ~ReassemblyPause() { reassemblyPaused = false; }
};

// Taking the driver's input path has to be redone after the driver is reinstalled, and it is
// reinstalled whenever the receive path wedges. Without this the first recovery silently returns
// every frame to lwIP, reassembly stops, and the stream looks like it never came back.
inline void reassemblyHook() {
  if (reassemblyActive) esp_eth_update_input_path(gEthHandle, ethernetInput, nullptr);
}

// Making the task and taking the input path are separate on purpose, and have to be.
//
// The task wants 8 kB of stack out of internal DRAM. Called after BLE is up, there is not enough
// left and xTaskCreatePinnedToCore fails -- and nothing above it fails, so reassembly keeps
// completing datagrams into a slot nobody ever empties. It reads as 63 reassembled and 62
// dropped, with a healthy stream and a decoder that has never once run.
//
// So the task is made early, while the memory is there, and the path is taken later, after the
// beam angles have been fetched over a stack that is still able to answer.
inline bool reassemblyStartTask() {
  if (reassemblyActive) return true;
  const BaseType_t ok =
      xTaskCreatePinnedToCore(reassemblyTask, "reassemble", 8192, nullptr, 3, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("reassembly task could not be created -- every datagram would be dropped");
    return false;
  }
  reassemblyActive = true;
  return true;
}

inline void reassemblyBegin() {
  if (reassemblyStartTask()) reassemblyHook();
}
