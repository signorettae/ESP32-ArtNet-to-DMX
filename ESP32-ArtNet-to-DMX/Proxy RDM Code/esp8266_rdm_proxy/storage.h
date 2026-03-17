/**
 * storage.h / storage.cpp — EEPROM persistence for ProxyConfig
 *
 * Layout:
 *   Byte 0       : Magic byte (0xA5) — indicates valid data
 *   Bytes 1..N   : ProxyConfig struct (raw binary)
 */
#pragma once
#include "config.h"

void Storage_init();
void Storage_load(ProxyConfig* cfg);
void Storage_save(const ProxyConfig* cfg);
void Storage_reset();
