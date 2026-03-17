/**
 * config.h — Global configuration, pin definitions, and shared structs
 */
#pragma once

#include <Arduino.h>

// ─── Pin Definitions ─────────────────────────────────────────
#define PIN_RS485_DIR   5    // GPIO5 (D1) — MAX485 DE/RE direction control
#define PIN_RESET_BTN   0    // GPIO0 (D3) — Reset button (active LOW)
#define PIN_LED_WIFI    2    // GPIO2 (D4) — Built-in LED, WiFi status
#define PIN_LED_DMX     4    // GPIO4 (D2) — DMX activity LED
#define PIN_LED_ERROR   14   // GPIO14 (D5) — Error LED

// ─── DMX Constants ───────────────────────────────────────────
#define DMX_UNIVERSE_SIZE   512
#define DMX_BAUD_RATE       250000
#define DMX_BREAK_US        92      // Break duration in microseconds
#define DMX_MAB_US          12      // Mark After Break duration

// ─── RDM Constants ───────────────────────────────────────────
#define RDM_START_CODE      0xCC
#define RDM_SUB_START       0x01
#define RDM_MAX_PARAMS      23
#define RDM_LOG_MAX         20      // Max RDM log entries to keep

// RDM Command Classes
#define RDM_CC_DISC_CMD         0x10
#define RDM_CC_DISC_CMD_RSP     0x11
#define RDM_CC_GET_CMD          0x20
#define RDM_CC_GET_CMD_RSP      0x21
#define RDM_CC_SET_CMD          0x30
#define RDM_CC_SET_CMD_RSP      0x31

// RDM Parameter IDs
#define RDM_PID_DISC_UNIQUE_BRANCH  0x0001
#define RDM_PID_DISC_MUTE           0x0002
#define RDM_PID_DISC_UN_MUTE        0x0003
#define RDM_PID_DEVICE_INFO         0x0060
#define RDM_PID_DMX_START_ADDRESS   0x00F0
#define RDM_PID_IDENTIFY_DEVICE     0x1000
#define RDM_PID_SOFTWARE_VERSION    0x00C0
#define RDM_PID_DEVICE_LABEL        0x0082
#define RDM_PID_MANUFACTURER_LABEL  0x0081
#define RDM_PID_SUPPORTED_PARAMS    0x0050

// ─── Art-Net Constants ───────────────────────────────────────
#define ARTNET_PORT         6454
#define ARTNET_MAX_UNIVERSE 15

// ─── Input Mode Options ──────────────────────────────────────
enum InputMode {
  INPUT_DMX   = 0,   // Physical DMX only
  INPUT_ARTNET= 1,   // Art-Net only
  INPUT_MERGE = 2    // Merge (HTP or priority)
};

enum MergeMode {
  MERGE_HTP       = 0,  // Highest Takes Precedence
  MERGE_DMX_PRIO  = 1,  // Physical DMX always wins
  MERGE_ARTNET_PRIO = 2 // Art-Net always wins
};

// ─── EEPROM Layout ───────────────────────────────────────────
#define EEPROM_SIZE         256
#define EEPROM_MAGIC        0xA5  // Validity check byte
#define EEPROM_ADDR_MAGIC   0
#define EEPROM_ADDR_CONFIG  1

// ─── Configuration Struct (saved to EEPROM) ──────────────────
struct ProxyConfig {
  uint16_t  channelOffset;        // e.g. 130 → channels 131-256 map to 1-126
  uint8_t   inputMode;            // InputMode enum
  uint8_t   mergeMode;            // MergeMode enum
  uint8_t   artnetUniverse;       // Art-Net universe to listen to (0-15)
  uint16_t  rdmDmxStartAddress;   // Simulated DMX start address
  char      rdmDeviceLabel[33];   // Null-terminated device label (max 32 chars)
  char      rdmManufacturerLabel[33];
  uint16_t  rdmDeviceModelId;
  uint16_t  rdmProductCategory;
  bool      rdmIdentifyActive;    // Is identify mode active?

  // Defaults
  void setDefaults() {
    channelOffset         = 0;
    inputMode             = INPUT_DMX;
    mergeMode             = MERGE_HTP;
    artnetUniverse        = 0;
    rdmDmxStartAddress    = 1;
    rdmDeviceModelId      = 0x0001;
    rdmProductCategory    = 0x0100; // Dimmer
    rdmIdentifyActive     = false;
    strncpy(rdmDeviceLabel,       "RDM Proxy",    32);
    strncpy(rdmManufacturerLabel, "ESP8266 Proxy", 32);
  }
};

// ─── DMX State (live buffers, NOT saved) ─────────────────────
struct DMXState {
  uint8_t  inputBuffer[DMX_UNIVERSE_SIZE];   // Raw received DMX (or Art-Net)
  uint8_t  outputBuffer[DMX_UNIVERSE_SIZE];  // After offset mapping
  uint8_t  dmxPhysical[DMX_UNIVERSE_SIZE];   // Direct physical DMX input
  uint8_t  artnetBuffer[DMX_UNIVERSE_SIZE];  // Art-Net received
  uint32_t lastDMXReceive;    // millis() of last DMX frame
  uint32_t lastArtnetReceive; // millis() of last Art-Net frame
  bool     dmxActive;
  bool     artnetActive;
  volatile bool transmitting; // True while sending DMX output
};

// ─── RDM Log Entry ───────────────────────────────────────────
struct RDMLogEntry {
  uint32_t timestamp;
  uint8_t  uid[6];
  uint16_t pid;
  uint8_t  cc;
  char     description[48];
};

// ─── RDM State ───────────────────────────────────────────────
struct RDMState {
  uint8_t     uid[6];             // Our simulated UID
  bool        muted;              // Discovery mute state
  RDMLogEntry log[RDM_LOG_MAX];
  uint8_t     logCount;
  uint8_t     logHead;            // Circular buffer head
  uint8_t     transactionNum;     // Rolling transaction counter
};
