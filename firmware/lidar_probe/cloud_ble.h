// Assemble frames from the sensor's packets and publish them over BLE.
//
// Why BLE and not the WiFi page: the tablet belongs on the office network, and this board has no
// credentials for it. BLE reaches the tablet without asking it to leave anything.
//
// Why a decimated frame: an OS1-16 at 512x10 is 512 columns x 16 beams, 8192 points, ten times a
// second. As raw ranges that is 164 kB/s, and BLE on this part does perhaps 20-40. So one frame
// a second, every other column kept, range as a uint16 in centimetres -- 8 kB a frame. The room
// still looks like the room; what is lost is motion, and a room does not move.
//
// The geometry lives with the app, not here. Ranges alone are meaningless without the beam
// altitude angles and azimuth offsets, so those are fetched once from the sensor and offered as
// a readable characteristic: the app asks once and can then place every point it receives.
#pragma once
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>

// Made up for this project, and stable so the app can look for them.
#define KETI_CLOUD_SERVICE "6b1e0001-4b2a-4f6d-9c3a-0f1e2d3c4b5a"
#define KETI_CLOUD_FRAME   "6b1e0002-4b2a-4f6d-9c3a-0f1e2d3c4b5a"  // notify: frame chunks
#define KETI_CLOUD_GEOM    "6b1e0003-4b2a-4f6d-9c3a-0f1e2d3c4b5a"  // read: beam geometry JSON
#define KETI_CLOUD_STATUS  "6b1e0004-4b2a-4f6d-9c3a-0f1e2d3c4b5a"  // read/notify: one status line
#define KETI_CLOUD_IMU     "6b1e0005-4b2a-4f6d-9c3a-0f1e2d3c4b5a"  // notify: accel + gyro

constexpr int kCloudBeams = 16;
constexpr int kCloudColumns = 512;   // one full revolution at 512x10
constexpr int kSendColumns = 256;    // every other column
constexpr int kChunkPoints = 240;    // 480 bytes of ranges + 6 byte header, inside a 517 MTU

// Ranges in centimetres, indexed [column][beam]. Written by the packet path, read by the sender.
uint16_t frameRanges[kCloudColumns][kCloudBeams];
volatile bool frameReady = false;
uint32_t frameSequence = 0;

BLEServer *cloudServer = nullptr;
BLECharacteristic *frameCharacteristic = nullptr;
BLECharacteristic *geometryCharacteristic = nullptr;
BLECharacteristic *statusCharacteristic = nullptr;
BLECharacteristic *imuCharacteristic = nullptr;

// The sensor's IMU, latest reading. Six floats is 24 bytes, so unlike the cloud it costs nothing
// to send -- the reason it goes at 10 Hz rather than the sensor's 100 is that a number changing
// a hundred times a second is unreadable, not that the link could not carry it.
volatile float imuAccel[3] = {0, 0, 0};
volatile float imuGyro[3] = {0, 0, 0};
volatile bool imuFresh = false;
volatile bool cloudConnected = false;

class CloudServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { cloudConnected = true; }
  void onDisconnect(BLEServer *server) override {
    cloudConnected = false;
    // Without this the board stops being findable after the first disconnect, which looks like
    // the app's fault and is not.
    server->startAdvertising();
  }
};

inline void cloudBleBegin(const char *name) {
  BLEDevice::init(name);
  BLEDevice::setMTU(517);
  cloudServer = BLEDevice::createServer();
  cloudServer->setCallbacks(new CloudServerCallbacks());

  BLEService *service = cloudServer->createService(BLEUUID(KETI_CLOUD_SERVICE), 20, 0);
  frameCharacteristic = service->createCharacteristic(
      KETI_CLOUD_FRAME, BLECharacteristic::PROPERTY_NOTIFY);
  frameCharacteristic->addDescriptor(new BLE2902());
  geometryCharacteristic = service->createCharacteristic(
      KETI_CLOUD_GEOM, BLECharacteristic::PROPERTY_READ);
  statusCharacteristic = service->createCharacteristic(
      KETI_CLOUD_STATUS, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  statusCharacteristic->addDescriptor(new BLE2902());
  imuCharacteristic = service->createCharacteristic(
      KETI_CLOUD_IMU, BLECharacteristic::PROPERTY_NOTIFY);
  imuCharacteristic->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(KETI_CLOUD_SERVICE);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// Ouster's IMU packet is 48 bytes: three 8-byte timestamps, then three float32 accelerations in
// g at offset 24 and three float32 angular rates in degrees per second at 36.
inline void decodeImu(const uint8_t *p, int length) {
  if (length < 48) return;
  memcpy((void *)imuAccel, p + 24, 12);
  memcpy((void *)imuGyro, p + 36, 12);
  imuFresh = true;
}

// What the board sees on the wire, once a second, so the tablet can graph the link rather than
// only the picture: rate, mean gap, worst gap, outliers, link speed. Sixteen bytes.
inline void cloudSendStatus(uint16_t rate, uint32_t meanGapUs, uint32_t maxGapUs,
                            uint16_t outliers, uint16_t linkMbit) {
  if (!cloudConnected) return;
  uint8_t payload[16];
  int n = 0;
  payload[n++] = rate; payload[n++] = rate >> 8;
  payload[n++] = meanGapUs; payload[n++] = meanGapUs >> 8;
  payload[n++] = meanGapUs >> 16; payload[n++] = meanGapUs >> 24;
  payload[n++] = maxGapUs; payload[n++] = maxGapUs >> 8;
  payload[n++] = maxGapUs >> 16; payload[n++] = maxGapUs >> 24;
  payload[n++] = outliers; payload[n++] = outliers >> 8;
  payload[n++] = linkMbit; payload[n++] = linkMbit >> 8;
  payload[n++] = 0; payload[n++] = 0;
  statusCharacteristic->setValue(payload, n);
  statusCharacteristic->notify();
}

inline void cloudSendImu() {
  if (!cloudConnected || !imuFresh) return;
  imuFresh = false;
  uint8_t payload[24];
  memcpy(payload, (const void *)imuAccel, 12);
  memcpy(payload + 12, (const void *)imuGyro, 12);
  imuCharacteristic->setValue(payload, sizeof(payload));
  imuCharacteristic->notify();
}

// A frame goes out as chunks of a fixed shape, so the app can drop a lost one without losing the
// rest: [u32 sequence][u16 chunkIndex][u16 chunkCount][u16 firstColumn][ranges...]. Notifications
// are unacknowledged, and at 8 kB a frame some will be lost -- a frame with a hole in it is still
// a picture of the room, whereas a stream that stalls waiting for a retransmit is not.
inline void cloudSendFrame() {
  if (!cloudConnected || !frameReady) return;
  frameReady = false;

  static uint8_t chunk[10 + kChunkPoints * 2];
  const int pointsPerFrame = kSendColumns * kCloudBeams;
  const int chunks = (pointsPerFrame + kChunkPoints - 1) / kChunkPoints;
  int point = 0;
  for (int c = 0; c < chunks; c++) {
    int n = 0;
    chunk[n++] = frameSequence;        chunk[n++] = frameSequence >> 8;
    chunk[n++] = frameSequence >> 16;  chunk[n++] = frameSequence >> 24;
    chunk[n++] = c;                    chunk[n++] = c >> 8;
    chunk[n++] = chunks;               chunk[n++] = chunks >> 8;
    chunk[n++] = point / kCloudBeams;  chunk[n++] = (point / kCloudBeams) >> 8;
    for (int i = 0; i < kChunkPoints && point < pointsPerFrame; i++, point++) {
      const uint16_t range = frameRanges[(point / kCloudBeams) * 2][point % kCloudBeams];
      chunk[n++] = range;
      chunk[n++] = range >> 8;
    }
    frameCharacteristic->setValue(chunk, n);
    frameCharacteristic->notify();
    // Twelve, not six. Android negotiates a connection interval in the tens of
    // milliseconds and notifications queued faster than it can carry them are dropped
    // outright -- which the app used to show as columns blinking out once a second.
    // Thirty-five chunks at 12 ms is still comfortably inside the one second budget.
    delay(12);
  }
  frameSequence++;
}
