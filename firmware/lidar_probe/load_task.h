// Generate load from the spare core.
//
// The receiving loop owns core 1 and must never wait; core 0 already carries HTTP and BLE, which
// sleep. A generator belongs there: it is the only thing on this board that can be interrupted
// without costing a measurement.
//
// Raw frames, not sockets. The priority a switch sorts on lives in the 802.1Q tag, and lwIP will
// not put one there -- so these are built by hand and handed to esp_eth_transmit, which is also
// the shortest path to the wire. There is no IP or UDP inside: nothing is meant to receive this
// traffic, only to compete with the sensor's for a port, so a local experimental EtherType and a
// pattern of bytes is the honest shape for it.
//
// It cannot saturate a 100 Mbit/s port. Measured, not estimated: asked for 800, 2000, 5000 and
// 10000 frames a second it sends 284, 363, 404 and 424 -- **about 4 Mbit/s whatever you ask for**,
// because esp_eth_transmit takes roughly 2.4 ms per 1200 byte frame and the SPI bus is what runs
// out. With 3.3 Mbit/s arriving at the same time that is around 7.4 Mbit/s through the chip,
// which looks like its practical ceiling.
//
// So the PC remains the generator when congestion is the point. What this is for is the other
// demonstration, which needs two priorities rather than a full port: gate the class this sends
// on, and watch the sensor's class come through untouched beside it. Throughout all of the
// above the sensor's stream stayed at 319-321/s with no extra outliers -- generating from the
// spare core costs the measurement nothing, which is the only reason it is allowed here.
#pragma once
#include <Arduino.h>
#include <esp_eth.h>

extern esp_eth_handle_t gEthHandle;

// Broadcast, because nothing is listening and a switch floods it to every port but the one it
// came from -- which is exactly where the competing traffic has to go.
static const uint8_t kLoadDestination[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint16_t kLoadEtherType = 0x88B5;   // IEEE local experimental 1
constexpr int kLoadFrameBytes = 1500;

volatile bool loadRunning = false;
volatile uint8_t loadPcp = 3;
volatile uint16_t loadVlan = 1;
volatile uint32_t loadFramesPerSecond = 800;   // ~7.7 Mbit/s at 1200 bytes
volatile uint32_t loadSent = 0;
volatile uint32_t loadRateAchieved = 0;

void loadTask(void *) {
  static uint8_t frame[kLoadFrameBytes];
  uint8_t self[6] = {0};
  for (;;) {
    if (!loadRunning) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (self[0] == 0 && gEthHandle) esp_eth_ioctl(gEthHandle, ETH_CMD_G_MAC_ADDR, self);

    memcpy(frame, kLoadDestination, 6);
    memcpy(frame + 6, self, 6);
    frame[12] = 0x81; frame[13] = 0x00;                       // 802.1Q
    const uint16_t tag = (uint16_t(loadPcp & 7) << 13) | (loadVlan & 0x0FFF);
    frame[14] = tag >> 8; frame[15] = tag & 0xFF;
    frame[16] = kLoadEtherType >> 8; frame[17] = kLoadEtherType & 0xFF;
    for (int i = 18; i < kLoadFrameBytes; i++) frame[i] = uint8_t(i);

    // Paced rather than blasted: a tight loop would starve the SPI bus the receiving core needs,
    // and the point is to compete on the wire, not inside this board.
    const uint32_t perSecond = loadFramesPerSecond ? loadFramesPerSecond : 1;
    const uint32_t burst = perSecond >= 100 ? perSecond / 100 : 1;
    for (uint32_t i = 0; i < burst && loadRunning; i++) {
      if (esp_eth_transmit(gEthHandle, frame, kLoadFrameBytes) == ESP_OK) loadSent++;
    }
    vTaskDelay(pdMS_TO_TICKS(perSecond >= 100 ? 10 : 1000 / perSecond));

    // What went out, not what was asked for. The two differ by a factor of twenty at the top of
    // the range and reporting the request would be reporting a wish.
    static int64_t window = 0;
    static uint32_t atWindow = 0;
    const int64_t now = esp_timer_get_time();
    if (now - window > 1000000) {
      loadRateAchieved = loadSent - atWindow;
      atWindow = loadSent;
      window = now;
    }
  }
}

inline bool loadBegin() {
  const BaseType_t ok = xTaskCreatePinnedToCore(loadTask, "load", 4096, nullptr, 1, nullptr, 0);
  if (ok != pdPASS) Serial.println("load task could not be created -- 'l' will do nothing");
  return ok == pdPASS;
}
