/**
 * storage.cpp — EEPROM persistence implementation
 */
#include "storage.h"
#include <EEPROM.h>

void Storage_init() {
  EEPROM.begin(EEPROM_SIZE);
}

void Storage_load(ProxyConfig* cfg) {
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  if (magic != EEPROM_MAGIC) {
    Serial.println(F("[Storage] No valid config found, using defaults."));
    cfg->setDefaults();
    Storage_save(cfg); // Write defaults
    return;
  }

  // Read struct from EEPROM
  uint8_t* ptr = (uint8_t*)cfg;
  for (size_t i = 0; i < sizeof(ProxyConfig); i++) {
    ptr[i] = EEPROM.read(EEPROM_ADDR_CONFIG + i);
  }

  // Sanitize after load
  cfg->channelOffset      = constrain(cfg->channelOffset, 0, 511);
  cfg->inputMode          = constrain(cfg->inputMode, 0, 2);
  cfg->mergeMode          = constrain(cfg->mergeMode, 0, 2);
  cfg->artnetUniverse     = constrain(cfg->artnetUniverse, 0, 15);
  cfg->rdmDmxStartAddress = constrain(cfg->rdmDmxStartAddress, 1, 512);

  // Ensure null termination
  cfg->rdmDeviceLabel[32]       = '\0';
  cfg->rdmManufacturerLabel[32] = '\0';

  Serial.printf("[Storage] Config loaded OK. Offset=%d Mode=%d\n",
                cfg->channelOffset, cfg->inputMode);
}

void Storage_save(const ProxyConfig* cfg) {
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);

  const uint8_t* ptr = (const uint8_t*)cfg;
  for (size_t i = 0; i < sizeof(ProxyConfig); i++) {
    EEPROM.write(EEPROM_ADDR_CONFIG + i, ptr[i]);
  }

  EEPROM.commit();
  Serial.println(F("[Storage] Config saved."));
}

void Storage_reset() {
  EEPROM.write(EEPROM_ADDR_MAGIC, 0x00); // Invalidate magic
  EEPROM.commit();
  Serial.println(F("[Storage] Config reset."));
}
