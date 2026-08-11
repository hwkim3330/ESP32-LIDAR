// Bring up the W5500 without Arduino's ETH class, for one reason: the poll period.
//
// This board's W5500 has no interrupt line -- probing every pin while the sensor streamed found
// nothing outside the SPI, flash and PSRAM buses, so INT is unwired here exactly as RST is. With
// no interrupt the driver polls, and Arduino hardcodes that poll at ten milliseconds
// (libraries/Ethernet/src/ETH.cpp: `if (_pin_irq < 0) mac_config.poll_period_ms = 10;`). There is
// no ioctl to change it afterwards; the value is baked in when the MAC is created.
//
// Ten milliseconds is not a detail here. It means packets are handed over in batches: at 320 a
// second the board saw groups of three or four, 171 us apart -- the cost of pulling one frame
// over SPI -- separated by ten millisecond holes, exactly 100 holes a second. Every arrival time
// this rig recorded was the poll that collected the packet, not the moment it landed. That is
// the whole of the "11 ms gap once a second" this project spent an afternoon chasing through its
// own code, the sensor and the switch, none of which had anything to do with it.
//
// So the MAC is created here with the period we want. Everything above it is untouched: this
// attaches to esp_netif the same way ETH.begin does, so lwIP, NetworkUDP, WebServer and the WiFi
// soft AP all carry on unaware.
#pragma once
#include <Arduino.h>
#include <esp_eth.h>
#include <esp_eth_mac_spi.h>
#include <esp_event.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <driver/spi_master.h>

// Raise carefully and check VERSIONR still reads 0x04 afterwards.
constexpr int kEthSpiMhz = 40;

static esp_eth_handle_t gEthHandle = nullptr;
static esp_netif_t *gEthNetif = nullptr;

// There is no ioctl for link state -- the driver reports it as an event -- so it is tracked
// here. The W5500's PHYCFGR would answer directly but the bus belongs to the driver now, and
// reading it behind the driver's back is what rebooted this board on the task watchdog once.
static volatile bool gEthLinkUp = false;

inline void ethEventHandler(void *, esp_event_base_t, int32_t id, void *) {
  if (id == ETHERNET_EVENT_CONNECTED) gEthLinkUp = true;
  else if (id == ETHERNET_EVENT_DISCONNECTED) gEthLinkUp = false;
}

// Poll period in milliseconds. One is the floor the driver accepts, and at the sensor's 3.1 ms
// spacing it means a poll collects one packet rather than four.
inline bool ethStart(int sck, int miso, int mosi, int cs, int pollPeriodMs, const IPAddress &ip,
                     const IPAddress &mask, const IPAddress &gateway, bool tenMegabit = false) {
  // esp_netif and the lwIP task have to exist before any of this. Arduino normally brings them
  // up inside ETH.begin or WiFi.mode; skipping ETH means asking for them explicitly, and
  // skipping THAT means a null-queue assert the moment the first socket is opened.
  esp_netif_init();
  esp_event_loop_create_default();  // both are safe to call twice

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = mosi;
  buscfg.miso_io_num = miso;
  buscfg.sclk_io_num = sck;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  if (spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) return false;

  spi_device_interface_config_t devcfg = {};
  devcfg.mode = 0;
  // The link is 100BASE-TX; the pipe is this. Every byte in or out of a W5500 crosses SPI, so
  // the clock here -- not the PHY -- is what decides throughput. Arduino's ETH used 20 MHz and
  // the board managed about 7.4 Mbit/s in and out together, which is what 20 MHz looks like once
  // each frame has paid for its address phase, its length read and its pointer update.
  //
  // The part is rated to 80. Higher is not free: SPI at 40 MHz and above is a signal integrity
  // question about this particular board's traces, and the way it fails is corrupt registers
  // rather than a clean error. VERSIONR is checked at boot for exactly this reason -- if it
  // stops reading 0x04, the clock is too high for the wiring.
  devcfg.clock_speed_hz = kEthSpiMhz * 1000 * 1000;
  devcfg.input_delay_ns = 20;
  devcfg.spics_io_num = cs;
  devcfg.queue_size = 20;

  eth_w5500_config_t macConfig = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &devcfg);
  macConfig.int_gpio_num = -1;                 // not wired on this board; established by probe
  macConfig.poll_period_ms = pollPeriodMs;     // the whole point of not using ETH.begin

  eth_mac_config_t macCommon = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phyConfig = ETH_PHY_DEFAULT_CONFIG();
  phyConfig.phy_addr = 1;
  phyConfig.reset_gpio_num = -1;

  esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&macConfig, &macCommon);
  esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phyConfig);
  if (!mac || !phy) return false;

  esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, ethEventHandler, nullptr);

  esp_eth_config_t ethConfig = ETH_DEFAULT_CONFIG(mac, phy);
  if (esp_eth_driver_install(&ethConfig, &gEthHandle) != ESP_OK) return false;

  // The W5500 has no MAC of its own to read, so one has to be given. Derived from the chip's
  // own efuse address so two boards on the same segment cannot collide.
  uint8_t macAddress[6];
  esp_read_mac(macAddress, ESP_MAC_ETH);
  esp_eth_ioctl(gEthHandle, ETH_CMD_S_MAC_ADDR, macAddress);

  esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
  esp_netif_config_t netifConfig = ESP_NETIF_DEFAULT_ETH();
  netifConfig.base = &base;
  gEthNetif = esp_netif_new(&netifConfig);
  if (!gEthNetif) return false;

  // Static, and no DHCP client at all. This is a closed bench with no server on it, so waiting
  // for a lease only delays the first measurement -- and this board is itself the DHCP server
  // the sensor uses.
  esp_netif_dhcpc_stop(gEthNetif);
  esp_netif_ip_info_t ipInfo = {};
  ipInfo.ip.addr = uint32_t(ip);
  ipInfo.netmask.addr = uint32_t(mask);
  ipInfo.gw.addr = uint32_t(gateway);
  esp_netif_set_ip_info(gEthNetif, &ipInfo);

  if (esp_netif_attach(gEthNetif, esp_eth_new_netif_glue(gEthHandle)) != ESP_OK) return false;
  if (esp_eth_start(gEthHandle) != ESP_OK) return false;

  // Ten megabit on purpose, and through the driver rather than behind its back: writing PHYCFGR
  // before esp_eth_start does nothing, because the driver resets the chip during init and the
  // mode goes with it. Negotiation has to be off before a speed can be forced.
  //
  // Why a receiver asks for a slower link: to make its own port contended. Filling 100 Mbit/s
  // takes about 95 and a W5500 cannot produce a fifth of that -- SPI runs out long before the
  // PHY -- so generators leave the port idle, and an idle port has no queue for a gate to act
  // on. At 10 Mbit/s the sensor's 3.3 is already a third of it.
  if (tenMegabit) {
    bool negotiate = false;
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_FULL;
    esp_eth_ioctl(gEthHandle, ETH_CMD_S_AUTONEGO, &negotiate);
    esp_eth_ioctl(gEthHandle, ETH_CMD_S_SPEED, &speed);
    esp_eth_ioctl(gEthHandle, ETH_CMD_S_DUPLEX_MODE, &duplex);
  }
  return true;
}

inline bool ethLinkUp() {
  return gEthLinkUp;
}

inline uint16_t ethLinkSpeed() {
  eth_speed_t speed = ETH_SPEED_10M;
  if (gEthHandle) esp_eth_ioctl(gEthHandle, ETH_CMD_G_SPEED, &speed);
  return speed == ETH_SPEED_100M ? 100 : 10;
}

inline bool ethFullDuplex() {
  eth_duplex_t duplex = ETH_DUPLEX_HALF;
  if (gEthHandle) esp_eth_ioctl(gEthHandle, ETH_CMD_G_DUPLEX_MODE, &duplex);
  return duplex == ETH_DUPLEX_FULL;
}

inline IPAddress ethLocalIP() {
  esp_netif_ip_info_t info = {};
  if (gEthNetif) esp_netif_get_ip_info(gEthNetif, &info);
  return IPAddress(info.ip.addr);
}

inline String ethMacAddress() {
  uint8_t mac[6] = {0};
  if (gEthHandle) esp_eth_ioctl(gEthHandle, ETH_CMD_G_MAC_ADDR, mac);
  char out[18];
  snprintf(out, sizeof(out), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3],
           mac[4], mac[5]);
  return String(out);
}
