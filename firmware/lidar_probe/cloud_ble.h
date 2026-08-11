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
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(KETI_CLOUD_SERVICE);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
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
    delay(6);  // the stack drops notifications queued faster than the connection interval
  }
  frameSequence++;
}
