/**
 * artnet_handler.cpp — Art-Net UDP reception
 *
 * Art-Net ArtDmx packet structure:
 *   Bytes 0-7   : "Art-Net\0" (ID string)
 *   Bytes 8-9   : OpCode = 0x0050 (little-endian) → ArtDmx
 *   Bytes 10-11 : Protocol version (14, big-endian)
 *   Byte  12    : Sequence number (0 = disabled)
 *   Byte  13    : Physical port
 *   Bytes 14-15 : Universe (little-endian, 15-bit)
 *   Bytes 16-17 : Length of DMX data (big-endian, 2-byte aligned)
 *   Bytes 18+   : DMX channel data (up to 512 bytes)
 */
#include "artnet_handler.h"
#include <WiFiUdp.h>
#include <Arduino.h>

static WiFiUDP udp;
static DMXState* dmxStatePtr = nullptr;
static const ProxyConfig* cfgPtr = nullptr;

// Art-Net constants
static const char ARTNET_ID[] = "Art-Net";
static const uint16_t ARTNET_OPDMX = 0x5000;

void ArtNet_init(const ProxyConfig* cfg, DMXState* state) {
  cfgPtr = cfg;
  dmxStatePtr = state;
  udp.begin(ARTNET_PORT);
  Serial.printf("[ArtNet] Listening on UDP port %d, universe %d\n",
                ARTNET_PORT, cfg->artnetUniverse);
}

void ArtNet_loop() {
  if (!dmxStatePtr || !cfgPtr) return;

  int pktSize = udp.parsePacket();
  if (pktSize < 18) return; // Too short for Art-Net header

  uint8_t buf[530]; // 18 header + 512 DMX
  int len = udp.read(buf, sizeof(buf));
  if (len < 18) return;

  // Validate ID string
  if (memcmp(buf, ARTNET_ID, 7) != 0) return;

  // Validate OpCode (little-endian 0x0050)
  uint16_t opcode = buf[8] | ((uint16_t)buf[9] << 8);
  if (opcode != ARTNET_OPDMX) return;

  // Universe (15-bit, little-endian)
  uint16_t universe = buf[14] | ((uint16_t)(buf[15] & 0x7F) << 8);
  if (universe != cfgPtr->artnetUniverse) return;

  // Data length (big-endian)
  uint16_t dataLen = ((uint16_t)buf[16] << 8) | buf[17];
  dataLen = min(dataLen, (uint16_t)512);

  if (len < 18 + dataLen) return;

  // Copy to Art-Net buffer
  memcpy(dmxStatePtr->artnetBuffer, &buf[18], dataLen);
  // Zero out any channels not sent
  if (dataLen < 512) {
    memset(&dmxStatePtr->artnetBuffer[dataLen], 0, 512 - dataLen);
  }

  dmxStatePtr->lastArtnetReceive = millis();
  dmxStatePtr->artnetActive = true;
}
