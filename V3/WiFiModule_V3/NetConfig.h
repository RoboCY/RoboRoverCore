// Network settings stored in the ESP's flash (EEPROM emulation).
// Changed at runtime by "NET ..." commands from the dashboard or the Kypruino.
#pragma once
#include <Arduino.h>
#include <EEPROM.h>

enum : uint8_t { NET_DASHBOARD = 0, NET_INTERNET = 1 };

struct __attribute__((packed)) NetConfig {
  uint32_t magic;
  uint8_t  mode;      // NET_DASHBOARD | NET_INTERNET
  uint8_t  remote;    // 1 = accept driving commands over MQTT
  uint8_t  rcMax;     // remote speed cap, percent
  uint8_t  tls;       // 1 = MQTT over TLS (server certificate is not verified)
  uint16_t port;
  char ssid[33];
  char pass[64];
  char host[64];      // MQTT broker, empty = no data link
  char user[64];
  char mpass[96];
  char base[64];      // topic prefix, e.g. roborover/3f2a1c
  uint32_t sum;
};

static const uint32_t NETCFG_MAGIC = 0x524F5631;  // "ROV1"
static const size_t   NETCFG_EEPROM_SIZE = 512;

inline uint32_t netCfgSum(const NetConfig& c) {
  const uint8_t* p = (const uint8_t*)&c;
  uint32_t h = 2166136261UL;  // FNV-1a
  for (size_t i = 0; i < offsetof(NetConfig, sum); i++) h = (h ^ p[i]) * 16777619UL;
  return h;
}

inline void netCfgDefaults(NetConfig& c, uint32_t chipId) {
  memset(&c, 0, sizeof(c));
  c.magic = NETCFG_MAGIC;
  c.mode = NET_DASHBOARD;
  c.rcMax = 60;
  c.port = 1883;
  strcpy(c.host, "broker.hivemq.com");
  snprintf(c.base, sizeof(c.base), "roborover/%06x", (unsigned)chipId);
}

inline void netCfgLoad(NetConfig& c, uint32_t chipId) {
  EEPROM.begin(NETCFG_EEPROM_SIZE);
  EEPROM.get(0, c);
  if (c.magic != NETCFG_MAGIC || c.sum != netCfgSum(c) || !c.base[0]) {
    netCfgDefaults(c, chipId);
    return;
  }
  c.ssid[sizeof(c.ssid) - 1] = c.pass[sizeof(c.pass) - 1] = c.host[sizeof(c.host) - 1] = 0;
  c.user[sizeof(c.user) - 1] = c.mpass[sizeof(c.mpass) - 1] = c.base[sizeof(c.base) - 1] = 0;
}

inline void netCfgSave(NetConfig& c) {
  c.sum = netCfgSum(c);
  EEPROM.put(0, c);
  EEPROM.commit();
}
