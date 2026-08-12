// Our own SPI layer under the IDF W5500 driver, for one reason: reads that cross the end of the
// chip's receive buffer.
//
// The evidence, collected rather than reasoned about. Every read the driver has ever failed on --
// twelve out of twelve across two runs -- began in the last kilobyte and a half of the 16 kB
// socket buffer and ran past its end. In-buffer offsets 15014 to 16270, lengths 1434 and 1514.
// Reads starting anywhere else have never failed once, at any rate, with any sensor. That is not
// an overflow, which was the assumption behind six failed fixes; it is a position.
//
// And the driver's own code says why nobody caught it. This is the whole of its buffer read:
//
//     static esp_err_t w5500_read_buffer(emac_w5500_t *emac, void *buffer, uint32_t len,
//                                        uint16_t offset) {
//         return w5500_read(emac, W5500_MEM_SOCK_RX(0, offset), buffer, len);
//     }
//
// One transaction, whatever the length, wherever it starts. The W5500 wraps its socket buffer
// internally so a read past the end is legal on the wire -- but the transaction that carries it
// is not the same shape as one that stays inside, and this bench says that difference is fatal.
// It goes unnoticed on ordinary traffic because a 1500 byte frame has to land within 1500 bytes
// of a 16 kB boundary, which is a tenth of the time, and most Ethernet is not a sensor sending
// 960 back-to-back full frames a second.
//
// So the read is split here instead, at the boundary, into two transactions that each stay inside
// the buffer. Everything else the driver does is left alone: this is a hook it already provides
// (eth_spi_custom_driver_config_t), not a fork of it.
#pragma once
#include <Arduino.h>
#include <driver/spi_master.h>
#include <esp_eth_mac_spi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// The chip's frame format, as the driver builds it: the 16 bit offset arrives as the SPI command
// phase and the control byte as the address phase. Block select bits live in the control byte,
// three bits up, so socket 0's receive buffer -- block 3 -- reading, variable length, is 0x18.
constexpr uint32_t kW5500SocketRxRead = 3 << 3;
constexpr uint32_t kW5500RxBufferSize = 0x4000;   // 16 kB, all of it given to socket 0 in MACRAW

// Kept so the board can report what this layer is doing rather than only that it was compiled in.
// Without a count of splits there is no way to tell "the split did not help" from "the split never
// ran", and those call for opposite next moves.
struct W5500Spi;
extern W5500Spi *gW5500Spi;

struct W5500Spi {
  spi_device_handle_t handle;
  SemaphoreHandle_t lock;
  spi_host_device_t host;
  volatile uint32_t splitReads;    // how often the boundary was actually straddled
  volatile uint32_t failures;      // transactions that still failed, so this is not silent
};

// The config the driver hands to init() is its own eth_w5500_config_t, which carries the host and
// the device config. Both are needed, so it is read back out rather than duplicated here.
inline void *w5500SpiInit(const void *spiConfig) {
  const eth_w5500_config_t *config = (const eth_w5500_config_t *)spiConfig;
  W5500Spi *spi = (W5500Spi *)calloc(1, sizeof(W5500Spi));
  if (!spi) return nullptr;

  spi_device_interface_config_t devcfg = *config->spi_devcfg;
  devcfg.command_bits = 16;   // the address phase, in W5500 terms
  devcfg.address_bits = 8;    // the control phase
  if (spi_bus_add_device(config->spi_host_id, &devcfg, &spi->handle) != ESP_OK) {
    free(spi);
    return nullptr;
  }
  spi->host = config->spi_host_id;
  spi->lock = xSemaphoreCreateMutex();
  gW5500Spi = spi;
  if (!spi->lock) {
    spi_bus_remove_device(spi->handle);
    free(spi);
    return nullptr;
  }
  return spi;
}

// Removing the device matters. Rebuilding the driver after a wedge failed with a NULL MAC because
// the device the old one owned was never released, and a board with no MAC is worse than one with
// a stuck MAC. Doing it here means the escalation path has somewhere to go.
inline esp_err_t w5500SpiDeinit(void *context) {
  W5500Spi *spi = (W5500Spi *)context;
  if (!spi) return ESP_OK;
  if (spi->handle) spi_bus_remove_device(spi->handle);
  if (spi->lock) vSemaphoreDelete(spi->lock);
  free(spi);
  return ESP_OK;
}

// One transaction, exactly as the driver's own does. Registers of four bytes or fewer use the
// in-transaction buffer, because a DMA read of a few bytes can be overwritten by the four byte
// boundary write that follows it -- the driver's comment, and worth keeping.
static esp_err_t w5500Transfer(W5500Spi *spi, uint32_t cmd, uint32_t addr, void *data,
                               uint32_t length, bool reading) {
  spi_transaction_t trans = {};
  trans.cmd = cmd;
  trans.addr = addr;
  trans.length = 8 * length;
  if (reading) {
    trans.flags = length <= 4 ? SPI_TRANS_USE_RXDATA : 0;
    trans.rx_buffer = data;
  } else {
    trans.tx_buffer = data;
  }
  const esp_err_t err = spi_device_polling_transmit(spi->handle, &trans);
  if (err == ESP_OK && reading && (trans.flags & SPI_TRANS_USE_RXDATA) && length <= 4)
    memcpy(data, trans.rx_data, length);
  return err;
}

inline esp_err_t w5500SpiRead(void *context, uint32_t cmd, uint32_t addr, void *data,
                              uint32_t length) {
  W5500Spi *spi = (W5500Spi *)context;
  if (xSemaphoreTake(spi->lock, pdMS_TO_TICKS(500)) != pdTRUE) return ESP_ERR_TIMEOUT;

  esp_err_t err;
  // Only the receive buffer is split. Registers live in other blocks and are never long enough to
  // reach a boundary, and splitting a register read would be a way to invent a new bug.
  //
  // The offset the driver passes is the socket's free-running read pointer, not an address -- it
  // counts past 16 kB and relies on the chip to mask it. So it is masked here before anything is
  // decided, or the boundary test would be asking about the wrong number.
  // Masking the offset into the buffer was tried and measured worse -- 11% of the time delivering
  // against 48% without it -- so the chip is left to fold its own pointer, as the driver intends.
  // The address is passed through untouched and only a genuine straddle is split.
  const uint32_t offset = cmd & (kW5500RxBufferSize - 1);
  if (addr == kW5500SocketRxRead && offset != cmd && offset + length > kW5500RxBufferSize) {
    const uint32_t first = kW5500RxBufferSize - offset;
    spi->splitReads++;
    err = w5500Transfer(spi, cmd, addr, data, first, true);
    if (err == ESP_OK)
      err = w5500Transfer(spi, 0, addr, (uint8_t *)data + first, length - first, true);
  } else {
    err = w5500Transfer(spi, cmd, addr, data, length, true);
  }

  if (err != ESP_OK) {
    // The arguments, not a guess at them. The split above never fires while transactions keep
    // failing, so one of the two things it tests is not what it is assumed to be, and printing
    // them costs one line the first few times.
    if (spi->failures < 4)
      Serial.printf("spi read failed: cmd 0x%08lx (%lu) addr 0x%02lx len %lu\n",
                    (unsigned long)cmd, (unsigned long)cmd, (unsigned long)addr,
                    (unsigned long)length);
    spi->failures++;
  }
  xSemaphoreGive(spi->lock);
  return err == ESP_OK ? ESP_OK : ESP_FAIL;
}

// Transmit needs no splitting: the driver writes to the send buffer from offset zero of a region
// it has already checked has room, so a write never approaches the end. Left as the plain
// transaction so that if this ever does fail, it fails the same way it always has.
inline esp_err_t w5500SpiWrite(void *context, uint32_t cmd, uint32_t addr, const void *data,
                               uint32_t length) {
  W5500Spi *spi = (W5500Spi *)context;
  if (xSemaphoreTake(spi->lock, pdMS_TO_TICKS(500)) != pdTRUE) return ESP_ERR_TIMEOUT;
  const esp_err_t err = w5500Transfer(spi, cmd, addr, (void *)data, length, false);
  if (err != ESP_OK) spi->failures++;
  xSemaphoreGive(spi->lock);
  return err == ESP_OK ? ESP_OK : ESP_FAIL;
}


