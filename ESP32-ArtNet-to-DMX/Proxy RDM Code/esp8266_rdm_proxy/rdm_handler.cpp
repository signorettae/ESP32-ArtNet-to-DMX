/**
 * rdm_handler.cpp — Full RDM proxy implementation
 *
 * RDM Frame Structure (ANSI E1.20):
 * ─────────────────────────────────────────────────────────────
 *  Byte  0    : Start Code      (0xCC)
 *  Byte  1    : Sub-Start Code  (0x01)
 *  Byte  2    : Message Length  (includes bytes 0..N-2, excl. checksum)
 *  Bytes 3-8  : Destination UID (6 bytes, MSB first)
 *  Bytes 9-14 : Source UID      (6 bytes, MSB first)
 *  Byte  15   : Transaction Number
 *  Byte  16   : Port / Response Type
 *  Byte  17   : Message Count
 *  Bytes 18-19: Sub Device (0x0000 = root)
 *  Byte  20   : Command Class
 *  Bytes 21-22: Parameter ID (PID)
 *  Byte  23   : PDL (Parameter Data Length)
 *  Bytes 24..: Parameter Data
 *  Last 2     : Checksum (sum of all bytes, 16-bit)
 * ─────────────────────────────────────────────────────────────
 *
 * How the proxy "fakes" being a device:
 *  When an RDM controller sends DISC_UNIQUE_BRANCH, it searches a UID
 *  range for responding devices. Our proxy responds with its own UID.
 *  For GET_CMD / SET_CMD, the proxy checks if the destination UID
 *  matches ours (or is broadcast 0xFFFFFFFFFFFF) and responds directly,
 *  applying any SET commands to its local state (e.g. DMX start address).
 */
#include "rdm_handler.h"
#include "dmx_handler.h"
#include <Arduino.h>

// ─── Internal Helpers ─────────────────────────────────────────

// Build and send an RDM response packet
static void sendResponse(RDMState* state, const ProxyConfig* cfg,
                         const uint8_t* destUID,
                         uint8_t cc, uint16_t pid,
                         const uint8_t* data, uint8_t dataLen,
                         uint8_t transNum, uint8_t responseType = 0x00) {
  uint8_t pkt[256];
  uint8_t msgLen = 24 + dataLen; // Fixed header + PDL

  pkt[0]  = RDM_START_CODE;   // 0xCC
  pkt[1]  = RDM_SUB_START;    // 0x01
  pkt[2]  = msgLen;
  // Destination UID (who sent the request becomes destination of response)
  memcpy(&pkt[3], destUID, 6);
  // Source UID = our UID
  memcpy(&pkt[9], state->uid, 6);
  pkt[15] = transNum;
  pkt[16] = responseType;     // 0x00 = ACK
  pkt[17] = 0x00;             // Message count
  pkt[18] = 0x00;             // Sub device high
  pkt[19] = 0x00;             // Sub device low
  pkt[20] = cc;               // Command class (response)
  pkt[21] = (pid >> 8) & 0xFF;
  pkt[22] = pid & 0xFF;
  pkt[23] = dataLen;          // PDL
  if (dataLen > 0 && data != nullptr) {
    memcpy(&pkt[24], data, dataLen);
  }

  // Checksum over bytes 0..msgLen-1
  uint16_t csum = RDM_checksum(pkt, msgLen);
  pkt[msgLen]     = (csum >> 8) & 0xFF;
  pkt[msgLen + 1] = csum & 0xFF;

  DMX_sendRDMResponse(pkt, msgLen + 2);
}

// ─── UID Utilities ────────────────────────────────────────────

static bool uidEquals(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

static bool uidIsBroadcast(const uint8_t* uid) {
  static const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  return uidEquals(uid, bcast);
}

static bool uidInRange(const uint8_t* uid,
                       const uint8_t* lower, const uint8_t* upper) {
  // Lexicographic comparison of 6-byte UIDs
  int cmpL = memcmp(uid, lower, 6);
  int cmpU = memcmp(uid, upper, 6);
  return (cmpL >= 0 && cmpU <= 0);
}

// ─── PID Handlers ────────────────────────────────────────────

static void handleDiscovery(RDMState* state, const ProxyConfig* cfg,
                            const uint8_t* srcUID, uint8_t transNum,
                            const uint8_t* data, uint8_t dataLen) {
  // DISC_UNIQUE_BRANCH: data = [lowerUID(6)] [upperUID(6)]
  if (state->muted) return; // Do not respond if muted
  if (dataLen < 12) return;

  const uint8_t* lower = data;
  const uint8_t* upper = data + 6;

  if (uidInRange(state->uid, lower, upper)) {
    // Respond with DISC_UNIQUE_BRANCH_RESPONSE (no checksum, preamble only)
    // Per E1.20 §7.5.1: response is 7 preamble bytes + 7 separator + encoded UID
    uint8_t resp[24];
    // 7 x 0xFE preamble
    for (int i = 0; i < 7; i++) resp[i] = 0xFE;
    resp[7] = 0xAA; // Separator
    // Encode UID with OR masks
    resp[8]  = state->uid[0] | 0xAA;
    resp[9]  = state->uid[0] | 0x55;
    resp[10] = state->uid[1] | 0xAA;
    resp[11] = state->uid[1] | 0x55;
    resp[12] = state->uid[2] | 0xAA;
    resp[13] = state->uid[2] | 0x55;
    resp[14] = state->uid[3] | 0xAA;
    resp[15] = state->uid[3] | 0x55;
    resp[16] = state->uid[4] | 0xAA;
    resp[17] = state->uid[4] | 0x55;
    resp[18] = state->uid[5] | 0xAA;
    resp[19] = state->uid[5] | 0x55;
    // Checksum (sum of UID bytes)
    uint16_t cs = 0;
    for (int i = 0; i < 6; i++) cs += state->uid[i];
    resp[20] = (cs >> 8) | 0xAA;
    resp[21] = (cs >> 8) | 0x55;
    resp[22] = (cs & 0xFF) | 0xAA;
    resp[23] = (cs & 0xFF) | 0x55;

    DMX_sendRDMResponse(resp, 24);
    RDM_log(state, srcUID, RDM_PID_DISC_UNIQUE_BRANCH,
            RDM_CC_DISC_CMD, "DISC_UNIQUE_BRANCH - responded");
  }
}

static void handleGetDeviceInfo(RDMState* state, const ProxyConfig* cfg,
                                const uint8_t* srcUID, uint8_t transNum) {
  uint8_t data[19];
  // RDM Protocol version
  data[0]  = 0x01; data[1] = 0x00;
  // Device Model ID
  data[2]  = (cfg->rdmDeviceModelId >> 8) & 0xFF;
  data[3]  = cfg->rdmDeviceModelId & 0xFF;
  // Product Category
  data[4]  = (cfg->rdmProductCategory >> 8) & 0xFF;
  data[5]  = cfg->rdmProductCategory & 0xFF;
  // Software version (big-endian 32-bit)
  data[6]  = 0x00; data[7] = 0x00; data[8] = 0x01; data[9] = 0x00;
  // DMX512 footprint (channels used)
  uint16_t footprint = DMX_UNIVERSE_SIZE - cfg->channelOffset;
  data[10] = (footprint >> 8) & 0xFF;
  data[11] = footprint & 0xFF;
  // Current personality + total personalities
  data[12] = 0x01; data[13] = 0x01;
  // DMX start address
  data[14] = (cfg->rdmDmxStartAddress >> 8) & 0xFF;
  data[15] = cfg->rdmDmxStartAddress & 0xFF;
  // Sub-device count
  data[16] = 0x00; data[17] = 0x00;
  // Sensor count
  data[18] = 0x00;

  sendResponse(state, cfg, srcUID,
               RDM_CC_GET_CMD_RSP, RDM_PID_DEVICE_INFO,
               data, 19, transNum);

  RDM_log(state, srcUID, RDM_PID_DEVICE_INFO,
          RDM_CC_GET_CMD, "GET DEVICE_INFO - responded");
}

static void handleGetDMXStartAddr(RDMState* state, const ProxyConfig* cfg,
                                  const uint8_t* srcUID, uint8_t transNum,
                                  ProxyConfig* mutableCfg) {
  uint8_t data[2];
  data[0] = (mutableCfg->rdmDmxStartAddress >> 8) & 0xFF;
  data[1] = mutableCfg->rdmDmxStartAddress & 0xFF;
  sendResponse(state, cfg, srcUID,
               RDM_CC_GET_CMD_RSP, RDM_PID_DMX_START_ADDRESS,
               data, 2, transNum);
  RDM_log(state, srcUID, RDM_PID_DMX_START_ADDRESS,
          RDM_CC_GET_CMD, "GET DMX_START_ADDRESS");
}

static void handleSetDMXStartAddr(RDMState* state, ProxyConfig* cfg,
                                  const uint8_t* srcUID, uint8_t transNum,
                                  const uint8_t* data, uint8_t dataLen) {
  if (dataLen < 2) return;
  uint16_t newAddr = ((uint16_t)data[0] << 8) | data[1];
  newAddr = constrain(newAddr, 1, 512);
  cfg->rdmDmxStartAddress = newAddr;
  // Also update the channel offset to match the new start address
  cfg->channelOffset = newAddr - 1;

  sendResponse(state, cfg, srcUID,
               RDM_CC_SET_CMD_RSP, RDM_PID_DMX_START_ADDRESS,
               nullptr, 0, transNum);

  char buf[48];
  snprintf(buf, sizeof(buf), "SET DMX_START_ADDRESS → %d", newAddr);
  RDM_log(state, srcUID, RDM_PID_DMX_START_ADDRESS, RDM_CC_SET_CMD, buf);
}

static void handleIdentify(RDMState* state, ProxyConfig* cfg,
                           const uint8_t* srcUID, uint8_t transNum,
                           const uint8_t* data, uint8_t dataLen, bool isSet) {
  if (isSet) {
    if (dataLen < 1) return;
    cfg->rdmIdentifyActive = (data[0] != 0);
    // Flash error LED to identify
    digitalWrite(PIN_LED_ERROR, cfg->rdmIdentifyActive ? LOW : HIGH);
    sendResponse(state, cfg, srcUID,
                 RDM_CC_SET_CMD_RSP, RDM_PID_IDENTIFY_DEVICE,
                 nullptr, 0, transNum);
    char buf[40];
    snprintf(buf, sizeof(buf), "SET IDENTIFY → %s",
             cfg->rdmIdentifyActive ? "ON" : "OFF");
    RDM_log(state, srcUID, RDM_PID_IDENTIFY_DEVICE, RDM_CC_SET_CMD, buf);
  } else {
    uint8_t resp = cfg->rdmIdentifyActive ? 1 : 0;
    sendResponse(state, cfg, srcUID,
                 RDM_CC_GET_CMD_RSP, RDM_PID_IDENTIFY_DEVICE,
                 &resp, 1, transNum);
    RDM_log(state, srcUID, RDM_PID_IDENTIFY_DEVICE,
            RDM_CC_GET_CMD, "GET IDENTIFY");
  }
}

static void handleGetLabel(RDMState* state, const ProxyConfig* cfg,
                           const uint8_t* srcUID, uint8_t transNum,
                           uint16_t pid, const char* label) {
  uint8_t len = strnlen(label, 32);
  sendResponse(state, cfg, srcUID,
               RDM_CC_GET_CMD_RSP, pid,
               (const uint8_t*)label, len, transNum);
}

static void handleGetSupportedParams(RDMState* state, const ProxyConfig* cfg,
                                     const uint8_t* srcUID, uint8_t transNum) {
  const uint16_t pids[] = {
    RDM_PID_DEVICE_INFO,
    RDM_PID_DMX_START_ADDRESS,
    RDM_PID_IDENTIFY_DEVICE,
    RDM_PID_SOFTWARE_VERSION,
    RDM_PID_DEVICE_LABEL,
    RDM_PID_MANUFACTURER_LABEL
  };
  uint8_t count = sizeof(pids) / sizeof(pids[0]);
  uint8_t data[count * 2];
  for (uint8_t i = 0; i < count; i++) {
    data[i*2]     = (pids[i] >> 8) & 0xFF;
    data[i*2 + 1] = pids[i] & 0xFF;
  }
  sendResponse(state, cfg, srcUID,
               RDM_CC_GET_CMD_RSP, RDM_PID_SUPPORTED_PARAMS,
               data, count * 2, transNum);
  RDM_log(state, srcUID, RDM_PID_SUPPORTED_PARAMS,
          RDM_CC_GET_CMD, "GET SUPPORTED_PARAMS");
}

static void handleGetSoftwareVersion(RDMState* state, const ProxyConfig* cfg,
                                     const uint8_t* srcUID, uint8_t transNum) {
  const char* ver = "ESP8266 RDM Proxy v1.0";
  sendResponse(state, cfg, srcUID,
               RDM_CC_GET_CMD_RSP, RDM_PID_SOFTWARE_VERSION,
               (const uint8_t*)ver, strlen(ver), transNum);
}

// ─── Public API ──────────────────────────────────────────────

void RDM_init(RDMState* state, const ProxyConfig* cfg) {
  // Generate UID from ESP chip ID:
  //   Manufacturer ID: 0x4553 ("ES" for ESP)
  //   Device ID: lower 32 bits of chip ID
  uint32_t chipId = ESP.getChipId();
  state->uid[0] = 0x45;                   // 'E'
  state->uid[1] = 0x53;                   // 'S'
  state->uid[2] = (chipId >> 24) & 0xFF;
  state->uid[3] = (chipId >> 16) & 0xFF;
  state->uid[4] = (chipId >> 8)  & 0xFF;
  state->uid[5] = chipId & 0xFF;

  state->muted          = false;
  state->logCount       = 0;
  state->logHead        = 0;
  state->transactionNum = 0;
}

/**
 * RDM_process — Called from main loop.
 * Checks if a new RDM packet arrived, validates it,
 * checks if it's addressed to us, and dispatches to the right handler.
 */
void RDM_process(RDMState* state, DMXState* dmxState, const ProxyConfig* cfg) {
  uint8_t pkt[256];
  uint8_t len = DMX_getRDMPacket(pkt, 256);
  if (len == 0) return;

  // Minimum valid RDM frame: 26 bytes (header + checksum)
  if (len < 26) return;
  if (pkt[0] != RDM_START_CODE || pkt[1] != RDM_SUB_START) return;

  uint8_t msgLen = pkt[2];
  if (msgLen < 24 || msgLen + 2 > len) return;

  // Validate checksum
  uint16_t calcCS = RDM_checksum(pkt, msgLen);
  uint16_t rxCS   = ((uint16_t)pkt[msgLen] << 8) | pkt[msgLen + 1];
  if (calcCS != rxCS) {
    Serial.printf("[RDM] Checksum mismatch: calc=%04X rx=%04X\n", calcCS, rxCS);
    return;
  }

  const uint8_t* destUID = &pkt[3];
  const uint8_t* srcUID  = &pkt[9];
  uint8_t  transNum  = pkt[15];
  uint8_t  cc        = pkt[20];
  uint16_t pid       = ((uint16_t)pkt[21] << 8) | pkt[22];
  uint8_t  pdl       = pkt[23];
  const uint8_t* data = &pkt[24];

  // Check if packet is addressed to us (our UID or broadcast)
  bool addressed = uidEquals(destUID, state->uid) || uidIsBroadcast(destUID);

  // ─── Discovery Commands (always respond to, even if muted — except BRANCH)
  if (cc == RDM_CC_DISC_CMD) {
    if (pid == RDM_PID_DISC_UNIQUE_BRANCH) {
      handleDiscovery(state, cfg, srcUID, transNum, data, pdl);
      return;
    }
    if (pid == RDM_PID_DISC_MUTE && addressed) {
      state->muted = true;
      // Respond with MUTE_RESPONSE
      uint8_t resp[2] = {0x00, 0x00}; // Control field, binding UID count
      sendResponse(state, cfg, srcUID,
                   RDM_CC_DISC_CMD_RSP, RDM_PID_DISC_MUTE,
                   resp, 2, transNum);
      RDM_log(state, srcUID, RDM_PID_DISC_MUTE, cc, "DISC_MUTE");
      return;
    }
    if (pid == RDM_PID_DISC_UN_MUTE && addressed) {
      state->muted = false;
      uint8_t resp[2] = {0x00, 0x00};
      sendResponse(state, cfg, srcUID,
                   RDM_CC_DISC_CMD_RSP, RDM_PID_DISC_UN_MUTE,
                   resp, 2, transNum);
      RDM_log(state, srcUID, RDM_PID_DISC_UN_MUTE, cc, "DISC_UN_MUTE");
      return;
    }
  }

  if (!addressed) return; // Not for us

  // Cast away const for handlers that may save config
  ProxyConfig* mutableCfg = (ProxyConfig*)cfg;

  // ─── GET / SET Commands ──────────────────────────────────────
  switch (pid) {
    case RDM_PID_DEVICE_INFO:
      if (cc == RDM_CC_GET_CMD)
        handleGetDeviceInfo(state, cfg, srcUID, transNum);
      break;

    case RDM_PID_DMX_START_ADDRESS:
      if (cc == RDM_CC_GET_CMD)
        handleGetDMXStartAddr(state, cfg, srcUID, transNum, mutableCfg);
      else if (cc == RDM_CC_SET_CMD)
        handleSetDMXStartAddr(state, mutableCfg, srcUID, transNum, data, pdl);
      break;

    case RDM_PID_IDENTIFY_DEVICE:
      handleIdentify(state, mutableCfg, srcUID, transNum, data, pdl,
                     cc == RDM_CC_SET_CMD);
      break;

    case RDM_PID_DEVICE_LABEL:
      if (cc == RDM_CC_GET_CMD)
        handleGetLabel(state, cfg, srcUID, transNum, pid,
                       cfg->rdmDeviceLabel);
      else if (cc == RDM_CC_SET_CMD && pdl <= 32) {
        memcpy(mutableCfg->rdmDeviceLabel, data, pdl);
        mutableCfg->rdmDeviceLabel[pdl] = '\0';
        sendResponse(state, cfg, srcUID,
                     RDM_CC_SET_CMD_RSP, pid, nullptr, 0, transNum);
        RDM_log(state, srcUID, pid, cc, "SET DEVICE_LABEL");
      }
      break;

    case RDM_PID_MANUFACTURER_LABEL:
      if (cc == RDM_CC_GET_CMD)
        handleGetLabel(state, cfg, srcUID, transNum, pid,
                       cfg->rdmManufacturerLabel);
      break;

    case RDM_PID_SUPPORTED_PARAMS:
      if (cc == RDM_CC_GET_CMD)
        handleGetSupportedParams(state, cfg, srcUID, transNum);
      break;

    case RDM_PID_SOFTWARE_VERSION:
      if (cc == RDM_CC_GET_CMD)
        handleGetSoftwareVersion(state, cfg, srcUID, transNum);
      break;

    default:
      // Unknown PID — send NACK with reason "Unknown PID"
      uint8_t nack[2] = {0x00, 0x03}; // NACK reason: Unknown PID
      sendResponse(state, cfg, srcUID,
                   (cc == RDM_CC_GET_CMD) ? RDM_CC_GET_CMD_RSP : RDM_CC_SET_CMD_RSP,
                   pid, nack, 2, transNum, 0x02); // 0x02 = NACK
      break;
  }
}

void RDM_log(RDMState* state, const uint8_t* uid, uint16_t pid,
             uint8_t cc, const char* desc) {
  uint8_t idx = state->logHead % RDM_LOG_MAX;
  RDMLogEntry& e = state->log[idx];

  e.timestamp = millis();
  memcpy(e.uid, uid, 6);
  e.pid = pid;
  e.cc  = cc;
  strncpy(e.description, desc, 47);
  e.description[47] = '\0';

  state->logHead = (state->logHead + 1) % RDM_LOG_MAX;
  if (state->logCount < RDM_LOG_MAX) state->logCount++;
}

uint16_t RDM_checksum(const uint8_t* buf, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += buf[i];
  return sum;
}
