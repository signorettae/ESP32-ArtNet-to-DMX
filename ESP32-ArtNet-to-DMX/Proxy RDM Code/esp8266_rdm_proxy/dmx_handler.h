/**
 * dmx_handler.h — DMX512 send/receive via UART + MAX485
 *
 * How DMX transmission works on ESP8266:
 *  1. Pull DE/RE pin HIGH → enable RS485 transmit
 *  2. Send BREAK: set UART to low baud, transmit 0x00, restore baud
 *  3. Send Start Code (0x00) + 512 channel bytes at 250,000 baud
 *  4. Pull DE/RE pin LOW → enable RS485 receive
 *
 * How RDM reception works:
 *  - RDM packets arrive on the same line with Start Code = 0xCC
 *  - We detect this in the receive buffer and route to RDM handler
 */
#pragma once
#include "config.h"

// Initialize DMX buffers
void DMX_init(DMXState* state);

// Start UART for DMX, configure direction pin
void DMX_begin(DMXState* state, const ProxyConfig* cfg);

// Stop DMX (called before OTA, etc.)
void DMX_stop();

// Receive incoming DMX/RDM frame (call from loop)
// Returns true if a complete frame was received
bool DMX_receive(DMXState* state);

// Apply channel offset from inputBuffer → outputBuffer, then transmit
void DMX_applyOffsetAndTransmit(DMXState* state, const ProxyConfig* cfg);

// Get raw RDM packet if one was detected (returns length, 0 if none)
uint8_t DMX_getRDMPacket(uint8_t* buf, uint16_t maxLen);

// Send RDM response packet on the DMX line
void DMX_sendRDMResponse(const uint8_t* buf, uint8_t len);

// Merge Art-Net + physical DMX into inputBuffer according to merge mode
void DMX_merge(DMXState* state, const ProxyConfig* cfg);
