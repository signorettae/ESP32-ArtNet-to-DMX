/**
 * rdm_handler.h — RDM (Remote Device Management) proxy logic
 *
 * RDM Proxy Operation:
 * ─────────────────────────────────────────────────────────────
 * The ESP8266 presents itself on the RDM network as a virtual device
 * with its own unique UID. It responds to discovery and parameter
 * requests as if it were the connected luminaire.
 *
 * Supported PIDs:
 *   DISC_UNIQUE_BRANCH (0x0001) — Discovery range query
 *   DISC_MUTE          (0x0002) — Mute this device in discovery
 *   DISC_UN_MUTE       (0x0003) — Unmute this device
 *   DEVICE_INFO        (0x0060) — Device model/footprint info
 *   DMX_START_ADDRESS  (0x00F0) — GET/SET the virtual start address
 *   DEVICE_LABEL       (0x0082) — GET/SET device label
 *   IDENTIFY_DEVICE    (0x1000) — Flash/identify the device
 *   SOFTWARE_VERSION   (0x00C0) — Firmware version string
 *   SUPPORTED_PARAMS   (0x0050) — List of supported PIDs
 *   MANUFACTURER_LABEL (0x0081) — Manufacturer name string
 */
#pragma once
#include "config.h"

// Initialize RDM state, generate UID from ESP8266 chip ID
void RDM_init(RDMState* state, const ProxyConfig* cfg);

// Process incoming RDM packet (if any) and send response
void RDM_process(RDMState* state, DMXState* dmxState, const ProxyConfig* cfg);

// Add an entry to the RDM log (circular buffer)
void RDM_log(RDMState* state, const uint8_t* uid, uint16_t pid,
             uint8_t cc, const char* desc);

// Calculate RDM checksum (sum of all bytes mod 65536)
uint16_t RDM_checksum(const uint8_t* buf, uint8_t len);
