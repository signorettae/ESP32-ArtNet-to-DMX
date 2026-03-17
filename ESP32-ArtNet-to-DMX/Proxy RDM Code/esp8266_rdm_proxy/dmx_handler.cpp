/**
 * dmx_handler.cpp — DMX512 / RDM physical layer implementation
 *
 * Uses ESP8266 hardware UART (Serial) for 250kbps DMX.
 * BREAK generation uses a brief baud-rate switch trick.
 *
 * RDM Detection Logic:
 *  - Normal DMX frames begin with start code 0x00
 *  - RDM frames begin with start code 0xCC
 *  - We peek at the first byte after BREAK+MAB to decide routing
 */
#include "dmx_handler.h"
#include <Arduino.h>

// ─── Internal State ──────────────────────────────────────────
static uint8_t  rxBuf[600];      // Raw receive buffer (>512 for RDM overhead)
static uint16_t rxLen = 0;
static bool     rxComplete = false;
static bool     rxIsRDM = false;

static uint8_t  rdmPacket[256];
static uint8_t  rdmPacketLen = 0;
static bool     rdmPacketReady = false;

static uint32_t lastTransmit = 0;
static const uint32_t TX_INTERVAL_MS = 25; // ~40fps DMX output

// ─── Helpers ─────────────────────────────────────────────────
static void setTxMode() {
  digitalWrite(PIN_RS485_DIR, HIGH);
  delayMicroseconds(2);
}

static void setRxMode() {
  delayMicroseconds(2);
  digitalWrite(PIN_RS485_DIR, LOW);
}

// Generates DMX BREAK (88µs low) + MAB (12µs high) using baud trick
static void sendBreak() {
  // Switch to very low baud to produce a long low pulse (BREAK)
  Serial.flush();
  Serial.end();
  pinMode(1, OUTPUT);       // TX pin
  digitalWrite(1, LOW);     // Force line LOW = BREAK
  delayMicroseconds(DMX_BREAK_US);
  digitalWrite(1, HIGH);    // MAB (Mark After Break)
  delayMicroseconds(DMX_MAB_US);
  Serial.begin(DMX_BAUD_RATE, SERIAL_8N2);
}

// ─── Public API ──────────────────────────────────────────────
void DMX_init(DMXState* state) {
  memset(state->inputBuffer,  0, DMX_UNIVERSE_SIZE);
  memset(state->outputBuffer, 0, DMX_UNIVERSE_SIZE);
  memset(state->dmxPhysical,  0, DMX_UNIVERSE_SIZE);
  memset(state->artnetBuffer, 0, DMX_UNIVERSE_SIZE);
  state->lastDMXReceive    = 0;
  state->lastArtnetReceive = 0;
  state->dmxActive         = false;
  state->artnetActive      = false;
  state->transmitting      = false;
}

void DMX_begin(DMXState* state, const ProxyConfig* cfg) {
  Serial.begin(DMX_BAUD_RATE, SERIAL_8N2);
  setRxMode();
  Serial.println(F("[DMX] Hardware UART initialized at 250kbps"));
}

void DMX_stop() {
  Serial.flush();
  setRxMode();
}

/**
 * DMX_receive — Non-blocking frame accumulator.
 *
 * DMX frames start with a BREAK condition (>88µs low).
 * We can't detect BREAK directly in software UART easily, so we use
 * a timing-based heuristic: if >1ms passes with no bytes, treat the
 * next byte as a new frame start code.
 *
 * Returns true when a complete frame has been received.
 */
bool DMX_receive(DMXState* state) {
  static uint32_t lastByteTime = 0;
  static bool inFrame = false;
  static uint16_t pos = 0;

  bool gotFrame = false;

  while (Serial.available() > 0 && !state->transmitting) {
    uint8_t b = Serial.read();
    uint32_t now = millis();

    // Gap > 1ms = new frame / BREAK detected
    if (now - lastByteTime > 1 || !inFrame) {
      // Process previous frame if complete enough
      if (inFrame && pos > 1) {
        uint8_t startCode = rxBuf[0];

        if (startCode == 0x00) {
          // Normal DMX frame
          uint16_t channels = min((uint16_t)(pos - 1), (uint16_t)DMX_UNIVERSE_SIZE);
          memcpy(state->dmxPhysical, &rxBuf[1], channels);
          state->lastDMXReceive = now;
          state->dmxActive = true;
          gotFrame = true;
          digitalWrite(PIN_LED_DMX, LOW); // Flash DMX LED
        }
        else if (startCode == RDM_START_CODE) {
          // RDM frame — copy to rdmPacket for processing
          uint8_t len = min((uint8_t)(pos), (uint8_t)255);
          memcpy(rdmPacket, rxBuf, len);
          rdmPacketLen = len;
          rdmPacketReady = true;
        }
      }

      // Reset for new frame
      pos = 0;
      inFrame = true;
    }

    if (inFrame && pos < sizeof(rxBuf)) {
      rxBuf[pos++] = b;
    }

    lastByteTime = now;
  }

  // Turn off DMX activity LED after a delay
  if (state->dmxActive && millis() - state->lastDMXReceive > 100) {
    digitalWrite(PIN_LED_DMX, HIGH);
  }

  return gotFrame;
}

/**
 * DMX_applyOffsetAndTransmit
 *
 * Maps inputBuffer channels to outputBuffer with offset, then sends DMX.
 *
 * Example: offset=130
 *   inputBuffer[130] → outputBuffer[0]   (channel 131 → output ch 1)
 *   inputBuffer[131] → outputBuffer[1]
 *   ...up to end of universe
 *
 * Channels before the offset wrap or are set to 0 (configurable).
 */
void DMX_applyOffsetAndTransmit(DMXState* state, const ProxyConfig* cfg) {
  // Merge sources first
  DMX_merge(state, cfg);

  // Apply offset mapping
  memset(state->outputBuffer, 0, DMX_UNIVERSE_SIZE);
  if (cfg->channelOffset == 0) {
    // No offset — pass through directly
    memcpy(state->outputBuffer, state->inputBuffer, DMX_UNIVERSE_SIZE);
  } else {
    uint16_t offset = cfg->channelOffset;  // e.g. 130
    uint16_t count  = DMX_UNIVERSE_SIZE - offset;
    if (count > 0 && count <= DMX_UNIVERSE_SIZE) {
      memcpy(state->outputBuffer, &state->inputBuffer[offset], count);
    }
    // outputBuffer[count..511] remain 0
  }

  // Transmit at controlled rate
  uint32_t now = millis();
  if (now - lastTransmit < TX_INTERVAL_MS) return;
  lastTransmit = now;

  state->transmitting = true;
  setTxMode();

  sendBreak();

  // Start code 0x00
  Serial.write((uint8_t)0x00);

  // Channel data
  Serial.write(state->outputBuffer, DMX_UNIVERSE_SIZE);
  Serial.flush();

  setRxMode();
  state->transmitting = false;
}

/**
 * DMX_merge — Combines physical DMX and Art-Net based on merge mode.
 * Result goes into state->inputBuffer.
 */
void DMX_merge(DMXState* state, const ProxyConfig* cfg) {
  uint32_t now = millis();
  bool dmxFresh    = (now - state->lastDMXReceive    < 3000);
  bool artnetFresh = (now - state->lastArtnetReceive < 3000);

  switch (cfg->inputMode) {
    case INPUT_DMX:
      memcpy(state->inputBuffer, state->dmxPhysical, DMX_UNIVERSE_SIZE);
      break;

    case INPUT_ARTNET:
      memcpy(state->inputBuffer, state->artnetBuffer, DMX_UNIVERSE_SIZE);
      break;

    case INPUT_MERGE:
    default:
      switch (cfg->mergeMode) {
        case MERGE_HTP:
          // Highest Takes Precedence
          for (int i = 0; i < DMX_UNIVERSE_SIZE; i++) {
            state->inputBuffer[i] = max(
              dmxFresh    ? state->dmxPhysical[i]  : 0,
              artnetFresh ? state->artnetBuffer[i] : 0
            );
          }
          break;

        case MERGE_DMX_PRIO:
          // Physical DMX wins if active, else fallback to Art-Net
          if (dmxFresh)
            memcpy(state->inputBuffer, state->dmxPhysical, DMX_UNIVERSE_SIZE);
          else
            memcpy(state->inputBuffer, state->artnetBuffer, DMX_UNIVERSE_SIZE);
          break;

        case MERGE_ARTNET_PRIO:
          // Art-Net wins if active, else fallback to physical DMX
          if (artnetFresh)
            memcpy(state->inputBuffer, state->artnetBuffer, DMX_UNIVERSE_SIZE);
          else
            memcpy(state->inputBuffer, state->dmxPhysical, DMX_UNIVERSE_SIZE);
          break;
      }
      break;
  }
}

uint8_t DMX_getRDMPacket(uint8_t* buf, uint16_t maxLen) {
  if (!rdmPacketReady) return 0;
  rdmPacketReady = false;
  uint8_t len = min(rdmPacketLen, (uint8_t)maxLen);
  memcpy(buf, rdmPacket, len);
  return len;
}

/**
 * DMX_sendRDMResponse — Sends an RDM response packet on the RS485 line.
 * RDM responses do NOT have a BREAK; they follow immediately after the
 * request's inter-slot gap.
 */
void DMX_sendRDMResponse(const uint8_t* buf, uint8_t len) {
  setTxMode();
  delayMicroseconds(180); // RDM inter-packet delay
  Serial.write(buf, len);
  Serial.flush();
  setRxMode();
}
