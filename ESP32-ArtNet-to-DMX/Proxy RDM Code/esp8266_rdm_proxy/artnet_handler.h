/**
 * artnet_handler.h — Art-Net UDP receiver
 *
 * Art-Net is a protocol for sending DMX512 over UDP/IP.
 * Default port: 6454
 * Each packet (ArtDmx) carries one universe of 512 channels.
 *
 * Implementation note: We use a raw UDP socket instead of the
 * full ArtnetWifi library to reduce memory usage on the ESP8266.
 * Only the ArtDmx opcode (0x5000) is handled.
 */
#pragma once
#include "config.h"

void ArtNet_init(const ProxyConfig* cfg, DMXState* state);
void ArtNet_loop();
