/*
  ArtNet2DMX_RP2040_Eth0_4Univ(Nexus-Pro).ino
  RP2040 Pico W â€” WiFi CYW43 + Ethernet W5500 â€” 4 Universos DMX512
  Bidirecionais
  =============================================================================

  Cada universo pode ser TX (ArtNetâ†’DMX) ou RX (DMXâ†’ArtNet)
  independentemente.

  Pinagem RP2040 Pico W:
  â”€â”€ Universo 1 â€” UART0 (Serial1, hardware):
      GP0  TX â†’ DI  MAX485 U1       GP1  RX â† RO  MAX485 U1
      GP2  DE â†’ DE+/RE MAX485 U1   (HIGH=TX, LOW=RX)
  â”€â”€ Universo 2 â€” UART1 (Serial2, hardware):
      GP4  TX â†’ DI  MAX485 U2       GP5  RX â† RO  MAX485 U2
      GP6  DE â†’ DE+/RE MAX485 U2
  â”€â”€ Universo 3 â€” SerialPIO (PIO state machine):
      GP8  TX â†’ DI  MAX485 U3       GP9  RX â† RO  MAX485 U3
      GP10 DE â†’ DE+/RE MAX485 U3
  â”€â”€ Universo 4 â€” SerialPIO (PIO state machine):
      GP12 TX â†’ DI  MAX485 U4       GP13 RX â† RO  MAX485 U4
      GP14 DE â†’ DE+/RE MAX485 U4
  â”€â”€ Ethernet W5500 â€” SPI0 (hardware):
      GP16 MISO  GP17 CS  GP18 SCK  GP19 MOSI  GP20 RST

  MAX485 â€” ligaÃ§Ã£o por universo (idÃªntica Ã—4):
      Pino 1 (RO)  â†’ RX Pico    Pino 2 (/RE) â†’ DE_PIN (LOW=RX)
      Pino 3 (DE)  â†’ DE_PIN (HIGH=TX)  Pino 4 (DI) â†’ TX Pico
      Pino 5 GND   Pino 6 (A/Y) RS-485+  Pino 7 (B/Z) RS-485âˆ’  Pino 8 VCC 3V3

  Bibliotecas:
      - arduino-pico core (Earle Philhower) â‰¥ 3.x  [Board Manager]
      - WiFiManager  (tzapu/WiFiManager)
      - Ethernet     (arduino-libraries/Ethernet â‰¥ 2.0)
      - Ethernet     (arduino-libraries/Ethernet ≥ 2.0)
*/

#include <EEPROM.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <MDNS_Generic.h>
#include <SPI.h>
#include <SerialPIO.h>
// Remoção da biblioteca UPnP_Generic que não possui o responder necessário.
// O SSDP será implementado manualmente para garantir descoberta no Windows.

// ===========================================================================
//  EEPROM Layout — 48 bytes, magic "OBNP"
//  [0..3]  magic   [4..5] port  [6] net  [7] subnet
//  [8..11] artnetUniverse[0..3]
//  [12]    uRxMode bitmask (bit0=U1..bit3=U4)
//  [13..14] routeInStart  [15..16] routeInCount  [17..18] routeOutStart
//  [19]    routeEnable   remainder: reserved
// ===========================================================================
#define CFG_SIZE 48
#define CFG_MAGIC0 'O'
#define CFG_MAGIC1 'B'
#define CFG_MAGIC2 'N'
#define CFG_MAGIC3 'P'

// ===========================================================================
//  PINAGEM
// ===========================================================================
// UARTs
static const int DMX_TX[4] = {0, 4, 8, 12};  // TX pinos (U1..U4)
static const int DMX_RX[4] = {1, 5, 9, 13};  // RX pinos
static const int DMX_DE[4] = {2, 6, 10, 14}; // DE/RE pinos (HIGH=TX, LOW=RX)

// W5500 SPI0
static const int ETH_MISO = 16;
static const int ETH_CS = 17;
static const int ETH_SCK = 18;
static const int ETH_MOSI = 19;
static const int ETH_RST = 20;

// SerialPIO para U3 e U4 (U1=Serial1, U2=Serial2 — hardware UART)
SerialPIO Serial3(8, 9, 513);   // U3: TX=GP8,  RX=GP9,  buf=513
SerialPIO Serial4(12, 13, 513); // U4: TX=GP12, RX=GP13, buf=513

// ===========================================================================
//  CONFIGURAÇÕES BÁSICAS
// ===========================================================================
const char *NODE_SHORT_NAME = "OneBitNexusDMXPro";
const char *NODE_LONG_NAME = "OneBit Nexus DMX Pro — 4 Universe ArtNet Node";
const char *NODE_MDNS_NAME =
    "onebitdmxpro"; // mDNS: http://onebitdmxpro.local:8080

EthernetUDP mdns_udp;
MDNS mdns(mdns_udp);

uint16_t artnetPort = 6454;
uint8_t artnetNet = 0;
uint8_t artnetSubnet = 0;
uint8_t artnetUniverse[4] = {0, 1, 2, 3};       // universo ArtNet de cada porta
bool uRxMode[4] = {false, false, false, false}; // false=TX, true=RX

bool routeEnable = false;
uint16_t routeInStart = 1;
uint16_t routeInCount = 512;
uint16_t routeOutStart = 1;

void saveConfig() {
  EEPROM.begin(CFG_SIZE);
  EEPROM.write(0, CFG_MAGIC0);
  EEPROM.write(1, CFG_MAGIC1);
  EEPROM.write(2, CFG_MAGIC2);
  EEPROM.write(3, CFG_MAGIC3);
  EEPROM.write(4, artnetPort & 0xFF);
  EEPROM.write(5, artnetPort >> 8);
  EEPROM.write(6, artnetNet);
  EEPROM.write(7, artnetSubnet);
  for (int i = 0; i < 4; i++)
    EEPROM.write(8 + i, artnetUniverse[i]);
  uint8_t rxm = 0;
  for (int i = 0; i < 4; i++)
    if (uRxMode[i])
      rxm |= (1 << i);
  EEPROM.write(12, rxm);
  EEPROM.write(13, (routeInStart - 1) & 0xFF);
  EEPROM.write(14, (routeInStart - 1) >> 8);
  uint16_t cnt = routeInCount > 0 ? routeInCount - 1 : 0;
  EEPROM.write(15, cnt & 0xFF);
  EEPROM.write(16, cnt >> 8);
  EEPROM.write(17, (routeOutStart - 1) & 0xFF);
  EEPROM.write(18, (routeOutStart - 1) >> 8);
  EEPROM.write(19, routeEnable ? 1 : 0);
  Serial.printf("Saving Config... Magic: %c%c%c%c\n", CFG_MAGIC0, CFG_MAGIC1,
                CFG_MAGIC2, CFG_MAGIC3);
  Serial.printf(
      "  Writing: Port=%d Net=%d Sub=%d Uni=%d,%d,%d,%d ModeMask=0x%02X\n",
      artnetPort, artnetNet, artnetSubnet, artnetUniverse[0], artnetUniverse[1],
      artnetUniverse[2], artnetUniverse[3], rxm);
  if (EEPROM.commit()) {
    Serial.println("EEPROM Commit Success.");
  } else {
    Serial.println("EEPROM Commit FAILED!");
  }
  EEPROM.end();
}

void loadConfig() {
  EEPROM.begin(CFG_SIZE);
  uint8_t m0 = EEPROM.read(0), m1 = EEPROM.read(1), m2 = EEPROM.read(2),
          m3 = EEPROM.read(3);
  Serial.printf("Loading Config... Magic Read: %02X %02X %02X %02X\n", m0, m1,
                m2, m3);
  bool valid = (m0 == CFG_MAGIC0 && m1 == CFG_MAGIC1 && m2 == CFG_MAGIC2 &&
                m3 == CFG_MAGIC3);
  if (valid) {
    Serial.println("EEPROM Magic Valid. Loading values...");
    artnetPort = EEPROM.read(4) | ((uint16_t)EEPROM.read(5) << 8);
    artnetNet = EEPROM.read(6);
    artnetSubnet = EEPROM.read(7);
    for (int i = 0; i < 4; i++)
      artnetUniverse[i] = EEPROM.read(8 + i);
    uint8_t rxm = EEPROM.read(12);
    for (int i = 0; i < 4; i++)
      uRxMode[i] = !!(rxm & (1 << i));
    routeInStart = constrain((uint16_t)(((uint16_t)EEPROM.read(13) |
                                         ((uint16_t)EEPROM.read(14) << 8)) +
                                        1),
                             1, 512);
    uint16_t cnt = (uint16_t)EEPROM.read(15) | ((uint16_t)EEPROM.read(16) << 8);
    routeInCount = constrain((uint16_t)(cnt + 1), 1, 512);
    routeOutStart = constrain((uint16_t)(((uint16_t)EEPROM.read(17) |
                                          ((uint16_t)EEPROM.read(18) << 8)) +
                                         1),
                              1, 512);
    routeEnable = EEPROM.read(19) == 1;
  }
  EEPROM.end();
}

// ===========================================================================
//  ESTADO DMX por universo
// ===========================================================================
const int DMX_CHANNELS = 512;
const int DMX_BREAK_US = 100; // Valor idêntico ao projeto ESP8266 (Nexus)
const int DMX_MAB_US = 12;    // Valor idêntico ao projeto ESP8266 (Nexus)

static uint8_t dmxData[4][DMX_CHANNELS + 1]; // [u][0]=SC, [u][1..512]=canais TX
static uint8_t dmxIn[4][DMX_CHANNELS + 1];   // buffer entrada RX
static uint8_t dmxInPrev[4][DMX_CHANNELS + 1]; // para detecção de mudança

struct UniState {
  uint16_t outLen = DMX_CHANNELS;
  uint32_t lastPktMs = 0;
  uint32_t lastSendMs = 0;
  bool running = false;
  uint32_t pktOk = 0;
  uint32_t pktFilt = 0;
  // RX
  uint8_t rxBuf[DMX_CHANNELS + 1];
  uint16_t rxPos = 0;
  bool rxInFrm = false;
  bool rxReady = false;
  uint16_t rxLen = 0;
  uint32_t rxPktOk = 0;
  bool rxRunning = false;
  uint32_t rxLastMs = 0;
  uint32_t rxLastByteUs = 0;
  uint32_t rxArtNetSendMs = 0;
};
UniState uSt[4];

bool testContinuous = false;
uint8_t testValue = 255;
bool lastPktEth = false;
volatile bool liveMode = false; // true = faders ativos, sem timeout de 3s

// ===========================================================================
//  NETWORK CONFIGURATION (DHCP vs STATIC)
// ===========================================================================
bool useDHCP = false; // Set to false to use the static IP below
IPAddress staticIP(192, 168, 137, 150);
IPAddress staticGateway(192, 168, 137, 1);
IPAddress staticSubnet(255, 255, 255, 0);

// ===========================================================================
//  ETHERNET
// ===========================================================================
uint8_t ethMac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xAB, 0x02};
bool ethLinked = false;
unsigned long lastEthRetryMs = 0;
const unsigned long ETH_RETRY_MS = 30000;

EthernetUDP udp_eth;
EthernetServer ethServer(8080);

static uint8_t pollReplyBuf[239];
static uint8_t udpBuf[640];
uint8_t artnetSeq = 0;

// SSDP Manual
EthernetUDP ssdpUDP;
IPAddress ssdpIP(239, 255, 255, 250);
const uint16_t ssdpPort = 1900;

void initSSDP() {
  if (ssdpUDP.beginMulticast(ssdpIP, ssdpPort)) {
    Serial.println("SSDP Manual started on port 1900");
  } else {
    Serial.println("Error: SSDP UDP failed to start.");
  }
}

void handleSSDP() {
  int packetSize = ssdpUDP.parsePacket();
  if (packetSize) {
    char packetBuffer[512];
    int len = ssdpUDP.read(packetBuffer, 511);
    if (len > 0)
      packetBuffer[len] = 0;

    if (strstr(packetBuffer, "M-SEARCH")) {
      // Responde apenas se procurar por tudo ou rootdevice
      if (strstr(packetBuffer, "ssdp:all") ||
          strstr(packetBuffer, "upnp:rootdevice")) {
        char response[512];
        IPAddress ip = Ethernet.localIP();
        snprintf(response, sizeof(response),
                 "HTTP/1.1 200 OK\r\n"
                 "CACHE-CONTROL: max-age=1800\r\n"
                 "EXT:\r\n"
                 "LOCATION: http://%d.%d.%d.%d:8080/description.xml\r\n"
                 "SERVER: Arduino/1.0 UPnP/1.1 OneBitNexusPro/1.0\r\n"
                 "ST: upnp:rootdevice\r\n"
                 "USN: "
                 "uuid:8a32d1e0-da50-4d3e-9005-%02x%02x%02x%02x%02x%02x::upnp:"
                 "rootdevice\r\n"
                 "\r\n",
                 ip[0], ip[1], ip[2], ip[3], ethMac[0], ethMac[1], ethMac[2],
                 ethMac[3], ethMac[4], ethMac[5]);

        ssdpUDP.beginPacket(ssdpUDP.remoteIP(), ssdpUDP.remotePort());
        ssdpUDP.write(response);
        ssdpUDP.endPacket();
      }
    }
  }
}

// ===========================================================================
//  HELPERS — acesso aos seriais por índice
// ===========================================================================
Stream &getSerial(int u) {
  if (u == 0)
    return Serial1;
  if (u == 1)
    return Serial2;
  if (u == 2)
    return Serial3;
  return Serial4;
}
bool isHWUart(int u) { return u < 2; }

// ===========================================================================
//  DMX TX — geração BREAK via GPIO + write()
// ===========================================================================
// IMPORTANTE: dmxSendU() é chamado do Core 1 (loop1()).
// Usamos uart_tx_wait_blocking() do pico-sdk diretamente pois o
// HardwareSerial::flush() do arduino-pico pode não funcionar corretamente
// quando chamado de um core diferente do que inicializou a UART.
#include "hardware/gpio.h"
#include "hardware/uart.h"

// Heartbeat do Core 1 — incrementado a cada frame DMX enviado.
// Visível no Serial para diagnóstico: se parar de crescer, Core 1 travou.
volatile uint32_t core1HeartBeat = 0;

static void dmxWaitTxDone(int u) {
  if (u == 0) {
    uart_tx_wait_blocking(uart0); // pico-sdk: seguro de qualquer core
  } else if (u == 1) {
    uart_tx_wait_blocking(uart1);
  } else if (u == 2) {
    Serial3.flush();
    delayMicroseconds(100);
  } else {
    Serial4.flush();
    delayMicroseconds(100);
  }
}

static void doBreak(int u) {
  int txPin = DMX_TX[u];
  int dePin = DMX_DE[u];

  gpio_function_t func = gpio_get_function(txPin);
  gpio_set_function(txPin, GPIO_FUNC_SIO);
  gpio_set_dir(txPin, GPIO_OUT);

  noInterrupts();
  digitalWrite(dePin, HIGH); // habilita driver RS-485 (TX)
  digitalWrite(txPin, LOW);  // BREAK
  delayMicroseconds(DMX_BREAK_US);
  digitalWrite(txPin, HIGH); // MAB
  delayMicroseconds(DMX_MAB_US);
  interrupts();

  if (u < 2)
    gpio_set_function(txPin, GPIO_FUNC_UART);
  else
    gpio_set_function(txPin, func);
}

void dmxSendU(int u, int nCh = DMX_CHANNELS) {
  if (uRxMode[u])
    return;
  nCh = constrain(nCh, 1, DMX_CHANNELS);

  // Aguarda fim da transmissão anterior (seguro multi-core)
  dmxWaitTxDone(u);

  // BREAK + MAB
  doBreak(u);

  // Envia SC + dados
  if (isHWUart(u)) {
    // Escrita direta no FIFO da UART via pico-sdk (seguro do Core 1)
    uart_inst_t *hw = (u == 0) ? uart0 : uart1;
    const uint8_t *buf = dmxData[u];
    for (int i = 0; i <= nCh; i++) {
      uart_putc_raw(hw, buf[i]);
    }
  } else {
    SerialPIO *pio = (u == 2) ? &Serial3 : &Serial4;
    pio->write(dmxData[u], nCh + 1);
  }

  core1HeartBeat++;

  // Log ocasional
  static uint32_t lastPrint[4] = {0, 0, 0, 0};
  if (millis() - lastPrint[u] > 5000) {
    Serial.printf("DMX TX U%d len=%d hb=%lu\n", u + 1, nCh,
                  (unsigned long)core1HeartBeat);
    lastPrint[u] = millis();
  }
}

// ===========================================================================
//  DMX RX — polling
// ===========================================================================
static void applyDePin(int u) {
  digitalWrite(DMX_DE[u], uRxMode[u] ? LOW : HIGH);
}

static void saveRxFrame(int u) {
  auto &st = uSt[u];
  if (st.rxPos < 2)
    return; // mínimo: SC + 1 canal
  int nCh = constrain(st.rxPos - 1, 1, DMX_CHANNELS);
  memcpy(dmxIn[u], st.rxBuf, 1 + nCh);
  memset(dmxIn[u] + 1 + nCh, 0, DMX_CHANNELS - nCh);
  st.rxLen = nCh;
  st.rxReady = true;
  st.rxPktOk++;
  st.rxRunning = true;
  st.rxLastMs = millis();
}

void dmxPollRx(int u) {
  auto &st = uSt[u];
  bool hw = isHWUart(u);
  Stream &port = getSerial(u);

  if (!uRxMode[u]) {
    while (port.available())
      port.read(); // descarta bytes espurios
    return;
  }

  // DEBUG: imprime estado do RX a cada 2s para diagnostico
  static uint32_t _dbgMs[4] = {0, 0, 0, 0};
  if (millis() - _dbgMs[u] > 2000) {
    _dbgMs[u] = millis();
    Serial.printf("[RX U%d] avail=%d inFrm=%d pos=%d pkts=%lu DE=%d\n", u + 1,
                  port.available(), (int)st.rxInFrm, st.rxPos,
                  (unsigned long)st.rxPktOk, digitalRead(DMX_DE[u]));
  }

  // ── DETECCAO DE BREAK / COLETA DE BYTES ────────────────────────────────────
  if (hw) {
    // HW UART: Leitura DIRETA do FIFO hardware com verificacao de flags BE/FE.
    // Isto bypassa o ring buffer do arduino-pico e permite distinguir bytes
    // de framing error (do BREAK) de bytes validos (do frame DMX).
    // Elimina o "channel offset" e as piscadas causadas por bytes 0x00 do
    // BREAK sendo confundidos com o Start Code.
    uart_inst_t *uart_hw = (u == 0) ? uart0 : uart1;
    while (uart_is_readable(uart_hw)) {
      uint32_t dr = uart_get_hw(uart_hw)->dr;      // dado + flags de erro
      bool isFE = (dr & UART_UARTDR_FE_BITS) != 0; // Framing Error
      bool isBE = (dr & UART_UARTDR_BE_BITS) != 0; // Break Error
      uint8_t b = (uint8_t)(dr & 0xFF);

      if (isFE || isBE) {
        // Byte de erro = pertence ao BREAK. Finaliza frame anterior.
        if (st.rxInFrm && st.rxPos > 0)
          saveRxFrame(u);
        st.rxInFrm = false;
        st.rxPos = 0;
        // Continua lendo para esgotar os outros bytes de erro do mesmo BREAK
      } else {
        // Byte valido: processa normalmente
        if (!st.rxInFrm) {
          if (b == 0x00) { // Start Code — inicia novo frame
            st.rxBuf[0] = 0x00;
            st.rxPos = 1;
            st.rxInFrm = true;
          }
          // Bytes nao-zero sem SC sao ignorados (lixo residual)
        } else {
          if (st.rxPos < 513)
            st.rxBuf[st.rxPos++] = b;
          if (st.rxPos >= DMX_CHANNELS + 1) {
            saveRxFrame(u);
            st.rxInFrm = false;
            st.rxPos = 0;
          }
        }
      }
    }
    return; // HW UART processado acima
  }

  // ── PIO UART: auto-save em 512 canais (sem deteccao de BREAK por hardware) ─
  while (port.available()) {
    uint8_t b = port.read();
    if (!st.rxInFrm) {
      if (b == 0x00) {
        st.rxBuf[0] = 0x00;
        st.rxPos = 1;
        st.rxInFrm = true;
      }
    } else {
      if (st.rxPos < 513)
        st.rxBuf[st.rxPos++] = b;
      if (st.rxPos >= DMX_CHANNELS + 1) {
        saveRxFrame(u);
        st.rxInFrm = false;
        st.rxPos = 0;
      }
    }
  }
}

// ===========================================================================
//  ArtNet TX (RX mode → broadcast)
// ===========================================================================
static void sendArtDmxInput(const uint8_t *data, uint8_t uni, int nCh) {
  nCh = max(2, nCh % 2 == 0 ? nCh : nCh + 1);
  nCh = min(nCh, DMX_CHANNELS);
  static uint8_t artBuf[18 + DMX_CHANNELS];
  memcpy(artBuf, "Art-Net", 7);
  artBuf[7] = 0x00;
  artBuf[8] = 0x00;
  artBuf[9] = 0x50;
  artBuf[10] = 0x00;
  artBuf[11] = 0x0E;
  artBuf[12] = artnetSeq++;
  artBuf[13] = 0x00;
  artBuf[14] = ((artnetSubnet & 0x0F) << 4) | (uni & 0x0F);
  artBuf[15] = artnetNet & 0x7F;
  artBuf[16] = nCh >> 8;
  artBuf[17] = nCh & 0xFF;
  memcpy(artBuf + 18, data + 1, nCh);
  int pktLen = 18 + nCh;
  if (ethLinked) {
    IPAddress ip = Ethernet.localIP(), mask = Ethernet.subnetMask();
    IPAddress bcast(ip[0] | ~mask[0], ip[1] | ~mask[1], ip[2] | ~mask[2],
                    ip[3] | ~mask[3]);
    udp_eth.beginPacket(bcast, artnetPort);
    udp_eth.write(artBuf, pktLen);
    udp_eth.endPacket();
  }
}

// Processa frame RX pronto → atualiza status + envia ArtNet + roteia se
// configurado
void processRxFrame(int u) {
  auto &st = uSt[u];
  if (!st.rxReady)
    return;
  st.rxReady = false;

  // Atualiza contadores de status para a tela Web
  uSt[u].running = true;
  uSt[u].lastPktMs = millis();
  uSt[u].pktOk++;

  // Detecção de mudança + Keepalive
  bool isChanged = memcmp(dmxIn[u] + 1, dmxInPrev[u] + 1, st.rxLen) != 0;
  bool isKeepalive = (millis() - st.rxArtNetSendMs) >= 1000UL;

  if (isChanged || isKeepalive) {
    memcpy(dmxInPrev[u], dmxIn[u], 1 + st.rxLen);
    st.rxArtNetSendMs = millis();

    // Broadcast ArtNet
    sendArtDmxInput(dmxIn[u], artnetUniverse[u], st.rxLen);

    // Roteamento interno (Mirror)
    if (routeEnable && !testContinuous && !liveMode) {
      uint16_t inStart = constrain(routeInStart, 1, 512);
      uint16_t outStart = constrain(routeOutStart, 1, 512);
      uint16_t maxSafe = 512 - max(inStart, outStart) + 1;
      uint16_t nCh = constrain(routeInCount, 1, maxSafe);

      for (int target = 0; target < 4; target++) {
        if (target == u)
          continue;
        if (!uRxMode[target]) {
          memcpy(dmxData[target] + outStart, dmxIn[u] + inStart, nCh);
          dmxData[target][0] = 0x00;
          uSt[target].outLen = (outStart - 1) + nCh;
          uSt[target].lastPktMs = millis();
          uSt[target].running = true;
          // Nao chama dmxSendU() aqui pois bloqueia o Core 0.
          // O loop1() (Core 1) envia na proxima iteracao (<25ms).
        }
      }
    }
  }
}

// ===========================================================================
//  ArtPollReply
// ===========================================================================
IPAddress getNodeIP() {
  return ethLinked ? Ethernet.localIP() : IPAddress(0, 0, 0, 0);
}

void buildPollReply(uint8_t *buf, IPAddress ip, const uint8_t *mac) {
  memset(buf, 0, 239);
  memcpy(buf, "Art-Net", 7);
  buf[8] = 0x00;
  buf[9] = 0x21;
  buf[10] = ip[0];
  buf[11] = ip[1];
  buf[12] = ip[2];
  buf[13] = ip[3];
  buf[14] = artnetPort & 0xFF;
  buf[15] = (artnetPort >> 8) & 0xFF;
  buf[16] = 0x00;
  buf[17] = 0x01;
  buf[18] = artnetNet & 0x7F;
  buf[19] = artnetSubnet & 0x0F;
  buf[20] = 0xFF;
  buf[21] = 0xFF;
  buf[23] = 0xD2;
  strncpy((char *)&buf[26], NODE_SHORT_NAME, 17);
  strncpy((char *)&buf[44], NODE_LONG_NAME, 63);
  strncpy((char *)&buf[108], "#0001 [0000] OK", 63);
  buf[173] = 0x04; // NumPorts = 4
  for (int i = 0; i < 4; i++) {
    // Bit 7: Can Input DMX, Bit 6: Can Output DMX
    if (uRxMode[i]) {
      buf[174 + i] = 0x80;                     // Somente Entrada (Input)
      buf[178 + i] = 0x80;                     // GoodInput
      buf[182 + i] = 0x00;                     // No Output
      buf[186 + i] = 0x00;                     // SwOut nulo para entrada
      buf[190 + i] = artnetUniverse[i] & 0x0F; // SwIn
    } else {
      buf[174 + i] = 0x40; // Somente Saída (Output)
      buf[178 + i] = 0x00;
      buf[182 + i] = 0x80;                     // GoodOutput
      buf[186 + i] = artnetUniverse[i] & 0x0F; // SwOut
      buf[190 + i] = 0x00;                     // SwIn nulo para saída
    }
  }
  memcpy(&buf[201], mac, 6);
  buf[207] = ip[0];
  buf[208] = ip[1];
  buf[209] = ip[2];
  buf[210] = ip[3];
  buf[211] = 0x01;
  buf[212] = 0x08;
}

void sendPollReply() {
  if (!ethLinked)
    return;
  IPAddress ip = Ethernet.localIP(), mask = Ethernet.subnetMask();
  IPAddress bcast(ip[0] | ~mask[0], ip[1] | ~mask[1], ip[2] | ~mask[2],
                  ip[3] | ~mask[3]);
  buildPollReply(pollReplyBuf, ip, ethMac);
  udp_eth.beginPacket(bcast, artnetPort);
  udp_eth.write(pollReplyBuf, 239);
  udp_eth.endPacket();
}

void sendPollReplyTo(IPAddress dest) {
  if (!ethLinked)
    return;
  buildPollReply(pollReplyBuf, Ethernet.localIP(), ethMac);
  udp_eth.beginPacket(dest, artnetPort);
  udp_eth.write(pollReplyBuf, 239);
  udp_eth.endPacket();
}

// ===========================================================================
//  Recepção ArtNet
// ===========================================================================
void processArtNetPacket(uint8_t *buf, int len, IPAddress sender) {
  // Ignora DMX externo se o painel de teste web estiver ativo
  if (testContinuous || liveMode)
    return;

  if (memcmp(buf, "Art-Net", 7) != 0 || buf[7] != 0x00)
    return;
  uint16_t opcode = buf[8] | ((uint16_t)buf[9] << 8);
  if (opcode == 0x2000) { // ArtPoll
    sendPollReplyTo(sender);
    sendPollReply();
    return;
  }
  if (opcode != 0x5000 || len < 18)
    return; // não é ArtDmx
  uint8_t pktNet = buf[15] & 0x7F;
  uint8_t pktSubUni = buf[14];
  uint8_t pktSub = (pktSubUni >> 4) & 0x0F;
  uint8_t pktUni = pktSubUni & 0x0F;
  uint16_t dataLen = ((uint16_t)buf[16] << 8) | buf[17];
  if (dataLen > DMX_CHANNELS)
    dataLen = DMX_CHANNELS;

  bool anyMatch = false;
  for (int u = 0; u < 4; u++) {
    if (pktNet != artnetNet || pktSub != artnetSubnet ||
        pktUni != artnetUniverse[u])
      continue;
    anyMatch = true;
    if (uRxMode[u])
      continue; // Ignora pacotes de rede se a porta estiver em modo ENTRADA
                // física

    // Se o roteamento estiver ligado, as saídas são escravas da mesa física,
    // ignorando o Art-Net externo para evitar conflito/flicker.
    if (routeEnable)
      continue;
    dmxData[u][0] = 0x00;
    memcpy(dmxData[u] + 1, buf + 18, dataLen);
    // Highest channel: envia até o último canal com valor > 0 para garantir
    // que aparelhos em endereços altos recebam o sinal
    int highCh = 0;
    for (int i = (int)dataLen; i >= 1; i--) {
      if (dmxData[u][i] > 0) {
        highCh = i;
        break;
      }
    }
    // Mínimo de 24 canais (compatibilidade com dimmers antigos)
    uSt[u].outLen =
        constrain(highCh < 24 ? (int)dataLen : highCh, 24, DMX_CHANNELS);
    uSt[u].lastSendMs = millis() - 25; // forçar envio na próxima iteração
    uSt[u].lastPktMs = millis();
    uSt[u].running = true;
  }
}

void handleArtNetEth() {
  if (!ethLinked)
    return;
  while (true) {
    int sz = udp_eth.parsePacket();
    if (sz < 10)
      break;
    IPAddress sndr = udp_eth.remoteIP();
    // Ignora pacotes enviados pelo proprio dispositivo (loopback de broadcast).
    // Sem isso, o ArtNet gerado pelo modo RX retorna para os universos TX,
    // causando o efeito de piscadas (flickering) na saida fisica.
    if (sndr == Ethernet.localIP()) {
      udp_eth.read(udpBuf, sizeof(udpBuf)); // descarta
      continue;
    }
    int len = udp_eth.read(udpBuf, sizeof(udpBuf));
    if (len < 10)
      break;
    processArtNetPacket(udpBuf, len, sndr);
  }
}

// ===========================================================================
//  SETUP
// ===========================================================================
// ===========================================================================
//  PAINEL WEB — HTML ESTÁTICO E SCRIPT
// ===========================================================================
static const char HTML_HEAD[] = R"HTML(<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OneBit Nexus DMX Pro</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;display:flex;flex-direction:column;min-height:100vh}
.topbar{display:flex;align-items:center;gap:12px;padding:10px 16px;background:#10192e;border-bottom:1px solid #1e2a4a;flex-shrink:0}
.title-wrap{flex:1;min-width:0}
h1{color:#e94560;font-size:1.25em;white-space:nowrap}
.sub{color:#aaa;font-size:.75em;line-height:1.5}
.sub b{color:#eee}
#dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#555;margin-left:6px;vertical-align:middle}
.main-wrap{display:flex;flex:1;min-height:0}
.sidebar{width:200px;min-width:200px;background:#10192e;border-right:1px solid #1e2a4a;display:flex;flex-direction:column;flex-shrink:0}
.sb-nav{list-style:none;flex:1}
.sb-nav li a{display:flex;align-items:center;gap:10px;padding:11px 14px;color:#bbb;text-decoration:none;font-size:.88em;border-left:3px solid transparent;transition:background .15s,color .15s}
.sb-nav li a:hover{background:#162040;color:#eee}
.sb-nav li a.active{background:#0f3460;color:#7ec8e3;border-left-color:#7ec8e3}
.content{flex:1;padding:20px;overflow-y:auto;min-width:0}
.section{display:none}.section.active{display:block}
.card{background:#16213e;border-radius:8px;padding:16px;margin-bottom:16px}
.card h2{font-size:1em;color:#7ec8e3;margin-bottom:12px;padding-bottom:8px;border-bottom:1px solid #1e2a4a}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:14px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}
@media(max-width:800px){.g2,.g3,.g4{grid-template-columns:1fr 1fr}}
@media(max-width:500px){.g2,.g3,.g4{grid-template-columns:1fr}}
.row{display:flex;justify-content:space-between;align-items:center;font-size:.88em;padding:5px 0;border-bottom:1px solid #1e2a4a}
.row:last-child{border:none}
.val{color:#e94560;font-weight:bold}
.lbl{font-size:.72em;color:#aaa;margin-bottom:4px}
.sec-title{font-size:.72em;color:#7ec8e3;padding:8px 0 4px;border-bottom:1px solid #1e2a4a;margin:8px 0 4px;font-weight:bold;letter-spacing:.05em;text-transform:uppercase}
input[type=number],input[list],select{width:100%;padding:6px 8px;border-radius:4px;border:1px solid #444;background:#0f3460;color:#eee;font-size:.92em}
.btn{display:block;width:100%;padding:9px;border:none;border-radius:4px;cursor:pointer;font-size:.95em;margin-top:8px;color:#fff;background:#e94560}
.btn:hover{background:#c73652}
.toggle-sw{position:relative;display:inline-block;width:46px;height:26px;flex-shrink:0}
.toggle-sw input{opacity:0;width:0;height:0}
.toggle-sl{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#444;border-radius:26px;transition:.3s}
.toggle-sl:before{position:absolute;content:"";height:20px;width:20px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.3s}
.toggle-sw input:checked+.toggle-sl{background:#2196f3}
.toggle-sw input:checked+.toggle-sl:before{transform:translateX(20px)}
.ok{color:#4caf50}.warn{color:#ff9800}.err{color:#f44336}
.banner-info{background:#1a3a5c;border-left:3px solid #7ec8e3;padding:7px 10px;border-radius:4px;font-size:.82em;margin-bottom:10px}
.banner-ok{background:#0d2c1e;border-left:3px solid #4caf50;padding:7px 10px;border-radius:4px;font-size:.82em;margin-bottom:10px}
.banner-warning{background:#1a3a5c;border-left:3px solid #eb4141;padding:7px 10px;border-radius:4px;font-size:.82em;margin-bottom:10px}
.u-card{background:#0f2040;border-radius:6px;padding:12px;border:1px solid #1e2a4a}
.u-card h3{font-size:.88em;color:#7ec8e3;margin-bottom:8px}
.ch-cell{background:#0f2040;border-radius:4px;padding:5px 7px;text-align:center}
.ch-cell .lbl{font-size:.65em;margin-bottom:2px}
.ch-cell .val{font-size:1.1em}
#ch-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(56px,1fr));gap:6px;margin-top:8px}
.fader-wrap{display:flex;gap:0;overflow-x:auto;padding:12px 10px 10px;border:1px solid #1e2a4a;border-radius:6px;background:#080e1c;margin-bottom:16px;align-items:flex-end;min-height:210px}
.fader-col{display:flex;flex-direction:column;align-items:center;min-width:44px;flex-shrink:0}
.fader-val{font-size:.75em;font-weight:bold;color:#7ec8e3;margin-bottom:6px;min-width:32px;text-align:center;background:#0f2040;border-radius:3px;padding:1px 2px}
.fader-sl{-webkit-appearance:slider-vertical;appearance:slider-vertical;writing-mode:vertical-lr;direction:rtl;width:26px;height:150px;cursor:ns-resize;outline:none;background:transparent;padding:0;border:none}
.fader-sl::-webkit-slider-runnable-track{background:#1a3a5a;border-radius:4px}
.fader-sl::-webkit-slider-thumb{-webkit-appearance:none;width:30px;height:14px;background:#4aaae8;border-radius:4px;cursor:grab}
.fader-ch{font-size:.70em;color:#aaa;margin-top:6px;text-align:center;white-space:nowrap}
.fader-div{width:1px;background:#1e2a4a;align-self:stretch;margin:0 6px;flex-shrink:0}
</style>)HTML";

static const char HTML_SCRIPT[] = R"HTML(<script>
var T=null,U=1;
function showSection(id){
  document.querySelectorAll('.section').forEach(function(s){s.classList.remove('active')});
  document.querySelectorAll('.sb-nav a').forEach(function(a){a.classList.remove('active')});
  var s=document.getElementById('sec-'+id);if(s)s.classList.add('active');
  var a=document.querySelector('.sb-nav a[data-sec="'+id+'"]');if(a)a.classList.add('active');
  localStorage.setItem('activeSec',id);
}
function setUni(u){U=u;sched();}
function sched(){if(T)clearTimeout(T);T=setTimeout(upd,0);}
var _fetching=false,_chStart=0,_chCount=0;
var _dragging=0,_liveTimer=null,_liveVals=[],_liveStart=1,_liveCount=16;
var _pushTimer=null,_lastPush=0;

function upd(){
  if(T){clearTimeout(T);T=null;}
  if(_fetching)return;
  _fetching=true;
  var st=parseInt(document.getElementById('ch-start').value)||1;
  var cnt=parseInt(document.getElementById('ch-count').value)||16;
  if(st<1)st=1;if(st>512)st=512;if(cnt<1)cnt=1;if(cnt>128)cnt=128;
  fetch('/data?start='+st+'&count='+cnt+'&uni='+U+'&v='+Date.now()).then(r=>r.json()).then(function(d){
    // Sincroniza campos de rede/ArtNet (agora usando IDs únicos e proteção _dirty)
    var p=document.getElementById('cfg-port'),n=document.getElementById('cfg-net'),s=document.getElementById('cfg-subnet');
    if(p&&d.artnetPort!==undefined){
      if(p._dirty && p.value == d.artnetPort) p._dirty=false;
      if(!p._dirty && p!==document.activeElement) p.value=d.artnetPort;
    }
    if(n&&d.artnetNet!==undefined){
      if(n._dirty && n.value == d.artnetNet) n._dirty=false;
      if(!n._dirty && n!==document.activeElement) n.value=d.artnetNet;
    }
    if(s&&d.artnetSubnet!==undefined){
      if(s._dirty && s.value == d.artnetSubnet) s._dirty=false;
      if(!s._dirty && s!==document.activeElement) s.value=d.artnetSubnet;
    }
    
    // Status por universo
    for(var i=0;i<4;i++){
      var u=document.getElementById('cfg-uni-'+(i+1)),rx=document.getElementById('cfg-rx-'+(i+1));
      if(u&&d.universes){
        if(u._dirty && u.value == d.universes[i]) u._dirty=false;
        if(!u._dirty && u!==document.activeElement) u.value=d.universes[i];
      }
      if(rx&&d.rxModes){
        var rv = d.rxModes[i]?'1':'0';
        if(rx._dirty && rx.value == rv) rx._dirty=false;
        if(!rx._dirty && rx!==document.activeElement) rx.value=rv;
      }

      var rs=document.getElementById('rx-status-'+i);
      if(rs){
        var run=d.running[i],ago=d.ago[i];
        if(!run){rs.className='val err';rs.textContent='Aguardando...';}
        else if(ago<3){rs.className='val ok';rs.textContent='Ativo';}
        else{rs.className='val warn';rs.textContent='Sem pacotes';}
      }
      _t('rx-ago-'+i,d.running[i]?(d.ago[i]+'s atrás'):'—');
      _t('rx-ok-'+i,d.pktOk[i]);
      var rl=document.getElementById('rx-mode-lbl-'+i);
      if(rl)rl.textContent=d.rxModes[i]?'◄ Entrada':'► Saída';
    }
    _t('rx-filt',d.pktFilt);
    
    // Status Rede
    var es=document.getElementById('eth-status');
    if(es){
      if(d.ethLinked){es.className='val ok';es.textContent=d.ethIp;}
      else{es.className='val err';es.textContent='Sem link Ethernet';}
    }
    
    // Grade canais
    var g=document.getElementById('ch-grid');
    if(g){
      var ns=d.chStart,nl=d.ch.length;
      if(ns!==_chStart||nl!==_chCount){
        _chStart=ns;_chCount=nl;
        var h='';
        for(var i=0;i<nl;i++)h+='<div class="ch-cell"><div class="lbl">Ch'+(ns+i)+'</div><div class="val" id="chv-'+(ns+i)+'">'+d.ch[i]+'</div></div>';
        g.innerHTML=h;
      } else {
        for(var i=0;i<nl;i++){var cv=document.getElementById('chv-'+(ns+i));if(cv){var nv=d.ch[i]+'';if(cv.textContent!==nv)cv.textContent=nv;}}
      }
    }
    var isRx = d.rxModes[U-1];
    _t('card-ch-title','Canais DMX U'+U+(isRx?' (RX)':' (TX)')+' ('+d.chStart+'–'+(d.chStart+d.ch.length-1)+')');
    var rxb = document.getElementById('ch-rx-banner');
    if(rxb) rxb.style.display = isRx ? 'block' : 'none';
    
    // Sincroniza faders (fora do modo live)
    if(!_dragging&&!_liveTimer){
      for(var i=0;i<d.ch.length;i++){
        var ch=d.chStart+i,fi=ch-_liveStart+1;
        if(fi>=1&&fi<=_liveCount){
          _liveVals[fi-1]=d.ch[i];
          var sld=document.getElementById('fsl-'+fi);
          var fvl=document.getElementById('fval-'+fi);
          if(sld)sld.value=d.ch[i];
          if(fvl)fvl.textContent=d.ch[i];
        }
      }
    }
    
    var sb=document.getElementById('cont-status'),ss=document.getElementById('cont-start');
    if(sb&&ss){
      if(d.cont){sb.style.display='block';sb.querySelector('span').textContent=d.val;ss.style.display='none';}
      else{sb.style.display='none';ss.style.display='flex';}
    }
    var dot=document.getElementById('dot');
    if(dot)dot.style.background=dot.style.background==='#4caf50'?'#555':'#4caf50';
    var ren=document.getElementById('route-en');
    if(ren&&!ren._dirty)ren.checked=!!d.routeEnable;
    var rs2=document.getElementById('route-status');
    if(rs2){rs2.textContent=d.routeEnable?'Ativo ✔':'Desativado';rs2.className=d.routeEnable?'val ok':'val err';}
    _fetching=false;
    T=setTimeout(upd,50); // Latência reduzida para 50ms (20 FPS)
  }).catch(function(){_fetching=false;T=setTimeout(upd,100);});
}
function _t(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}
function buildFaders(n){
  _liveCount=Math.max(1,Math.min(32,n));
  _liveVals=new Array(_liveCount).fill(0);
  var strip=document.getElementById('fader-strip');if(!strip)return;
  var h='<div class="fader-col"><div class="fader-val" style="color:#eee;background:#1a3a5a">M</div>'+
    '<input type="range" orient="vertical" min="0" max="255" value="0" id="master-sl" class="fader-sl" oninput="masterSlide(this.value)">'+
    '<div class="fader-ch" style="color:#7ec8e3;font-weight:bold">ALL</div></div><div class="fader-div"></div>';
  for(var i=1;i<=_liveCount;i++)
    h+='<div class="fader-col"><div class="fader-val" id="fval-'+i+'">0</div>'+
      '<input type="range" orient="vertical" min="0" max="255" value="0" class="fader-sl" id="fsl-'+i+'" oninput="chSlide('+i+',this.value)" onpointerdown="++_dragging" onpointerup="setTimeout(function(){if(_dragging>0)--_dragging;},100)" ontouchstart="++_dragging" ontouchend="setTimeout(function(){if(_dragging>0)--_dragging;},100)">'+
      '<div class="fader-ch" id="flbl-'+i+'">CH'+(_liveStart+i-1)+'</div></div>';
  strip.innerHTML=h;
  _t('lbl-last',_liveStart+_liveCount-1);
}
function updateChLabels(){
  var s=Math.max(1,Math.min(Math.max(1,513-_liveCount),parseInt((document.getElementById('fader-start')||{value:'1'}).value)||1));
  _liveStart=s;_t('lbl-first',s);_t('lbl-last',s+_liveCount-1);
  for(var i=1;i<=_liveCount;i++){var el=document.getElementById('flbl-'+i);if(el)el.textContent='CH'+(s+i-1);}
}
function updateChCount(){
  var el=document.getElementById('fader-count');
  var n=Math.max(1,Math.min(32,parseInt((el||{value:'16'}).value)||16));
  if(el)el.value=n;buildFaders(n);updateChLabels();
}
function masterSlide(v){
  _t('master-val',v);v=parseInt(v);
  for(var i=1;i<=_liveCount;i++){_liveVals[i-1]=v;var s=document.getElementById('fsl-'+i),d=document.getElementById('fval-'+i);if(s)s.value=v;if(d)d.textContent=v;}
  if(!_liveTimer)startLive();schedulePush();
}
function schedulePush(){
  if(!_liveTimer||_pushTimer)return;
  var delay=_lastPush?Math.max(0,30-(Date.now()-_lastPush)):0;
  _pushTimer=setTimeout(function(){_pushTimer=null;_lastPush=Date.now();pushLive(false);},delay);
}
function chSlide(i,v){_liveVals[i-1]=parseInt(v);_t('fval-'+i,v);if(!_liveTimer)startLive();schedulePush();}
function pushLive(clr){
  var u=document.querySelector('input[name="test-uni"]:checked');
  var url='/test-dmx-live?u='+(u?u.value:'both')+'&start='+_liveStart+'&vals='+_liveVals.slice(0,_liveCount).join(',');
  if(clr)url+='&clear=1';
  fetch(url).catch(function(){});
}
function startLive(){
  if(_liveTimer)return;
  document.getElementById('live-start-btn').style.display='none';
  document.getElementById('live-stop-btn').style.display='';
  document.getElementById('live-banner').style.display='block';
  _liveTimer=true;
  fetch('/test-dmx?val=0&cont=0').then(function(){pushLive(true);}).catch(function(){pushLive(true);});
}
function stopLive(){
  _liveTimer=null;if(_pushTimer){clearTimeout(_pushTimer);_pushTimer=null;}
  document.getElementById('live-start-btn').style.display='';
  document.getElementById('live-stop-btn').style.display='none';
  document.getElementById('live-banner').style.display='none';
  _liveVals.fill(0);
  for(var i=1;i<=_liveCount;i++){var s=document.getElementById('fsl-'+i),d=document.getElementById('fval-'+i);if(s)s.value=0;if(d)d.textContent=0;}
  var ms=document.getElementById('master-sl');if(ms)ms.value=0;_t('master-val','0');
  fetch('/test-dmx-live?u=both&clear=1').catch(function(){});
}
function setRxMode(uni,rx){fetch('/set-rx-mode?uni='+uni+'&rx='+rx).then(function(){sched();}).catch(function(){});}
var _routeTimer=null;
function setRoute(){
  var en=document.getElementById('route-en');if(!en)return;
  if(en)en._dirty=true;if(_routeTimer)clearTimeout(_routeTimer);
  _routeTimer=setTimeout(function(){
    var is=document.getElementById('route-in-start'),ic=document.getElementById('route-in-count'),os=document.getElementById('route-out-start');
    var enV=en.checked?1:0,isV=Math.max(1,Math.min(512,parseInt((is||{value:'1'}).value)||1));
    var icV=Math.max(1,Math.min(512,parseInt((ic||{value:'512'}).value)||512));
    var osV=Math.max(1,Math.min(512,parseInt((os||{value:'1'}).value)||1));
    fetch('/set-route?en='+enV+'&inStart='+isV+'&inCount='+icV+'&outStart='+osV)
      .then(function(r){return r.json();}).then(function(d){if(en)en._dirty=false;sched();}).catch(function(){});
  },300);
}
window.onload=function(){
  var sec=localStorage.getItem('activeSec')||'status';showSection(sec);
  buildFaders(16);upd();
};
</script>)HTML";

static const char HTML_SEC_STATUS[] = R"HTML(
<div id='sec-status' class='section'>
<div class='card'><h2>▶ Status por Universo</h2>
<div class='g4'>
<div class='u-card'><h3>U1 — <span id='rx-mode-lbl-0'>► Saída</span></h3>
<div class='row'><span>Status</span><span id='rx-status-0' class='val err'>Aguardando...</span></div>
<div class='row'><span>Último pkt</span><span id='rx-ago-0' class='val'>—</span></div>
<div class='row'><span>Aceitos</span><span id='rx-ok-0' class='val'>0</span></div></div>
<div class='u-card'><h3>U2 — <span id='rx-mode-lbl-1'>► Saída</span></h3>
<div class='row'><span>Status</span><span id='rx-status-1' class='val err'>Aguardando...</span></div>
<div class='row'><span>Último pkt</span><span id='rx-ago-1' class='val'>—</span></div>
<div class='row'><span>Aceitos</span><span id='rx-ok-1' class='val'>0</span></div></div>
<div class='u-card'><h3>U3 — <span id='rx-mode-lbl-2'>► Saída</span></h3>
<div class='row'><span>Status</span><span id='rx-status-2' class='val err'>Aguardando...</span></div>
<div class='row'><span>Último pkt</span><span id='rx-ago-2' class='val'>—</span></div>
<div class='row'><span>Aceitos</span><span id='rx-ok-2' class='val'>0</span></div></div>
<div class='u-card'><h3>U4 — <span id='rx-mode-lbl-3'>► Saída</span></h3>
<div class='row'><span>Status</span><span id='rx-status-3' class='val err'>Aguardando...</span></div>
<div class='row'><span>Último pkt</span><span id='rx-ago-3' class='val'>—</span></div>
<div class='row'><span>Aceitos</span><span id='rx-ok-3' class='val'>0</span></div></div>
</div>
<div class='row' style='margin-top:10px'><span>Pacotes filtrados</span><span id='rx-filt' class='val'>0</span></div>
</div>
<div class='g2'>
<div class='card'><h2>🌐 Rede</h2>
<div class='row'><span>Ethernet (W5500)</span><span id='eth-status' class='val'>—</span></div>
</div>
</div>
</div>)HTML";

static const char HTML_SEC_CHANNELS[] = R"HTML(
<div id='sec-channels' class='section'><div class='card'>
<h2 id='card-ch-title'>Canais DMX</h2>
<div id='ch-rx-banner' class='banner-ok' style='display:none;margin-bottom:12px'>&#9664; Exibindo canais <b>recebidos</b> (DMX&rarr;ArtNet)</div>
<div style='display:flex;gap:18px;flex-wrap:wrap;margin-bottom:12px'>
<label style='font-size:.88em;cursor:pointer'><input type='radio' name='uni-sel' value='1' checked onchange='setUni(1)'> U1</label>
<label style='font-size:.88em;cursor:pointer'><input type='radio' name='uni-sel' value='2' onchange='setUni(2)'> U2</label>
<label style='font-size:.88em;cursor:pointer'><input type='radio' name='uni-sel' value='3' onchange='setUni(3)'> U3</label>
<label style='font-size:.88em;cursor:pointer'><input type='radio' name='uni-sel' value='4' onchange='setUni(4)'> U4</label>
</div>
<div class='g2' style='margin-bottom:12px'>
<div><div class='lbl'>Canal inicial (1–512)</div><input type='number' id='ch-start' min='1' max='512' value='1' oninput='sched()'></div>
<div><div class='lbl'>Quantidade (1–128)</div><input type='number' id='ch-count' min='1' max='128' value='32' oninput='sched()'></div>
</div>
<div id='ch-grid'></div></div></div>)HTML";

static const char HTML_SEC_TEST[] = R"HTML(
<div id='sec-test' class='section'><div class='card'>
<h2>Teste DMX (sem ArtNet)</h2>
<div class='g3' style='margin-bottom:8px;align-items:end'>
<div><div class='lbl'>Canal inicial (1–512)</div><input type='number' id='fader-start' min='1' max='512' value='1' oninput='updateChLabels()' style='width:90px'></div>
<div><div class='lbl'>Nº de canais (1–32)</div><input type='number' id='fader-count' min='1' max='32' value='16' oninput='updateChCount()' style='width:80px'></div>
<div style='display:flex;gap:12px;align-items:center;flex-wrap:wrap'>
<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='1'> U1</label>
<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='2'> U2</label>
<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='3'> U3</label>
<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='4'> U4</label>
<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='both' checked> Todos</label>
</div>
</div>
<div style='display:flex;gap:8px;margin-bottom:12px'>
<button id='live-start-btn' class='btn' style='background:#2196f3' onclick='startLive()'>▶ Iniciar envio contínuo</button>
<button id='live-stop-btn' class='btn' style='background:#f44336;display:none' onclick='stopLive()'>■■ Parar e zerar</button>
</div>
<div id='live-banner' class='banner-info' style='display:none;margin-bottom:10px'>● Enviando — CH <span id='lbl-first'>1</span>–<span id='lbl-last'>16</span> &nbsp;|&nbsp; Master: <span id='master-val'>0</span></div>
<div class='banner-warning' style='margin-bottom:10px'>⚠ Desative roteamento externo durante o teste.</div>
<div class='fader-wrap' id='fader-strip'></div>
<div class='sec-title'>Quadros pré-definidos</div>
<div class='g2' style='margin-bottom:10px'>
<a href='/test-dmx?val=255'><button class='btn' style='background:#4caf50'>■■ 1 Frame FULL (255)</button></a>
<a href='/test-dmx?val=0'><button class='btn' style='background:#555'>■■ 1 Frame ZERO (0)</button></a>
</div>
<div id='cont-start' class='g2'>
<a href='/test-dmx?val=255&cont=1'><button class='btn' style='background:#2196f3'>▶ Contínuo FULL (255)</button></a>
<a href='/test-dmx?val=128&cont=1'><button class='btn' style='background:#ff9800'>▶ Contínuo HALF (128)</button></a>
</div>
<div id='cont-status' style='display:none;margin-top:10px'>
<div class='banner-ok'>▶ Enviando contínuo val=<span>0</span> (~10fps)</div>
<a href='/test-dmx?val=0&cont=0'><button class='btn' style='background:#f44336'>■■ PARAR contínuo</button></a>
</div>
</div></div>)HTML";

// ===========================================================================
//  Monta e envia a página completa (streaming)
// ===========================================================================
static void sendChunked(Print &c, const char *str) {
  int len = strlen(str);
  int pos = 0;
  while (pos < len) {
    int toWrite = len - pos;
    if (toWrite > 512)
      toWrite = 512;
    c.write((const uint8_t *)(str + pos), toWrite);
    pos += toWrite;
    c.flush(); // Força o envio antes do buffer (2KB) encher
    delay(2);
  }
}

void sendPageStreaming(Print &c) {
  char buf[256];
  sendChunked(c, HTML_HEAD);
  sendChunked(c, HTML_SCRIPT);

  // Topbar
  c.print("</head><body>");
  c.print("<div class='topbar'>");
  c.print(
      "<div class='title-wrap'><h1>Nexus DMX Pro <span id='dot'></span></h1>");
  String ethIp =
      Ethernet.linkStatus() == LinkON ? Ethernet.localIP().toString() : "-";
  snprintf(buf, sizeof(buf),
           "<div class='sub'>ETH: <b>%s</b> &nbsp;·&nbsp; "
           "Net:<b>%d</b> Sub:<b>%d</b> U1:<b>%d</b> U2:<b>%d</b> U3:<b>%d</b> "
           "U4:<b>%d</b> Porta:<b>%d</b></div>",
           ethIp.c_str(), artnetNet, artnetSubnet, artnetUniverse[0],
           artnetUniverse[1], artnetUniverse[2], artnetUniverse[3], artnetPort);
  c.print(buf);
  c.print("</div></div>"); // fecha title-wrap e topbar

  // Layout
  c.print("<div class='main-wrap'>");

  // Sidebar
  c.print("<nav class='sidebar' id='sidebar'><ul class='sb-nav'>");
  c.print("<li><a href='#' data-sec='status' "
          "onclick=\"showSection('status');return false;\">▶ "
          "Status</a></li>");
  c.print("<li><a href='#' data-sec='channels' "
          "onclick=\"showSection('channels');return false;\">■ Canais "
          "DMX</a></li>");
  c.print("<li><a href='#' data-sec='artnet' "
          "onclick=\"showSection('artnet');return false;\">◆ "
          "ArtNet</a></li>");
  c.print(
      "<li><a href='#' data-sec='route' onclick=\"showSection('route');return "
      "false;\">⇄ Roteamento</a></li>");
  c.print(
      "<li><a href='#' data-sec='test' onclick=\"showSection('test');return "
      "false;\">⤤ Teste DMX</a></li>");
  c.print("</ul></nav>");

  c.print("<div class='content'>");

  // Seção Status
  sendChunked(c, HTML_SEC_STATUS);

  // Seção Canais
  sendChunked(c, HTML_SEC_CHANNELS);

  // Seção ArtNet
  c.print("<div id='sec-artnet' class='section'>"
          "<div class='card'>"
          "<h2>Endereço ArtNet</h2>");
  c.print("<form action='/artnet' method='POST' autocomplete='off' "
          "onsubmit=\"this.querySelectorAll('input,select').forEach(e=>e._"
          "dirty=false)\">");
  c.print("<div class='g3' style='margin-bottom:14px'>");
  snprintf(buf, sizeof(buf),
           "<div>"
           "<div class='lbl'>Porta UDP</div>"
           "<input type='number' name='port' id='cfg-port' min='1' max='65535' "
           "value='%d' oninput='this._dirty=true'>"
           "</div>",
           artnetPort);
  c.print(buf);
  snprintf(buf, sizeof(buf),
           "<div>"
           "<div class='lbl'>Net (0-127)</div>"
           "<input type='number' name='net' id='cfg-net' min='0' max='127' "
           "value='%d' oninput='this._dirty=true'>"
           "</div>",
           artnetNet);
  c.print(buf);
  snprintf(buf, sizeof(buf),
           "<div>"
           "<div class='lbl'>Sub-Net (0-15)</div>"
           "<input type='number' name='subnet' id='cfg-subnet' min='0' "
           "max='15' value='%d' oninput='this._dirty=true'>"
           "</div>",
           artnetSubnet);
  c.print(buf);
  c.print("</div>");
  // Definimos a lista de sugestões uma única vez
  c.print("<datalist id='unilist'>");
  for (int j = 0; j <= 15; j++) {
    snprintf(buf, sizeof(buf), "<option value='%d'>Universo %d</option>", j, j);
    c.print(buf);
  }
  c.print("</datalist>");

  c.print("<div class='g4' style='margin-bottom:14px'>");
  for (int i = 0; i < 4; i++) {
    snprintf(buf, sizeof(buf),
             "<div>"
             "<div class='lbl'>Universo %d (0-15)</div>"
             "<input list='unilist' name='uni%d' id='cfg-uni-%d' value='%d' "
             "style='margin-bottom:6px' oninput='this._dirty=true'>"
             "</div>",
             i + 1, i + 1, i + 1, artnetUniverse[i]);
    c.print(buf);
  }
  c.print("</div>"); // Fechamento da div de mapeamento de Universos

  c.print("<div class='g4' style='margin-bottom:14px'>");
  for (int i = 0; i < 4; i++) {
    snprintf(buf, sizeof(buf),
             "<div>"
             "<div class='lbl'>U%d Modo</div>"
             "<select name='rx%d' id='cfg-rx-%d' onchange='this._dirty=true'>"
             "<option value='0'%s>&#9654; Sa&iacute;da (TX)</option>"
             "<option value='1'%s>&#9664; Entrada (RX)</option>"
             "</select>"
             "</div>",
             i + 1, i + 1, i + 1, uRxMode[i] ? "" : " selected",
             uRxMode[i] ? " selected" : "");
    c.print(buf);
  }
  c.print("</div>");
  c.print("<button class='btn' type='submit'>Aplicar "
          "configuração</button>");
  c.print("</form></div></div>");

  // Seção Teste DMX
  sendChunked(c, HTML_SEC_TEST);

  // Seção Roteamento
  char rbuf[900];
  snprintf(rbuf, sizeof(rbuf),
           "<div id='sec-route' class='section'>"
           "<div class='card'>"
           "<h2>&#8644; Roteamento DMX Interno</h2>"
           "<div class='banner-info' style='margin-bottom:14px'>Copia canais "
           "da <b>Entrada F&iacute;sica (U1 RX)</b> diretamente para a "
           "<b>Sa&iacute;da (U2 TX)</b> sem depender do QLC+. Lat&ecirc;ncia "
           "m&iacute;nima (&lt;1ms).</div>"
           "<div class='row' style='margin-bottom:12px'>"
           "<span style='font-size:.95em'>Roteamento ativo</span>"
           "<label class='toggle-sw'>"
           "<input type='checkbox' id='route-en' onchange='setRoute()' %s>"
           "<span class='toggle-sl'></span>"
           "</label>"
           "</div>"
           "<div class='g3' style='margin-bottom:14px'>"
           "<div>"
           "<div class='lbl'>Canal entrada inicial (1&ndash;512)</div>"
           "<input type='number' id='route-in-start' min='1' max='512' "
           "value='%d' oninput='setRoute()'>"
           "</div>"
           "<div>"
           "<div class='lbl'>Qtd. de canais (1&ndash;512)</div>"
           "<input type='number' id='route-in-count' min='1' max='512' "
           "value='%d' oninput='setRoute()'>"
           "</div>"
           "<div>"
           "<div class='lbl'>Canal sa&iacute;da inicial (1&ndash;512)</div>"
           "<input type='number' id='route-out-start' min='1' max='512' "
           "value='%d' oninput='setRoute()'>"
           "</div>"
           "</div>"
           "<div class='row'>"
           "<span>Status</span>"
           "<span id='route-status' class='val'>%s</span>"
           "</div>"
           "<div style='font-size:.8em;color:#888;margin-top:10px'>Exemplo: "
           "Entrada 1&ndash;32 &rarr; Sa&iacute;da 1&ndash;32 (espelho "
           "direto).<br>Ou: Entrada 1&ndash;16 &rarr; Sa&iacute;da 17&ndash;32 "
           "(mapeamento com offset).</div>"
           "</div>"
           "</div>",
           routeEnable ? "checked" : "", (int)routeInStart, (int)routeInCount,
           (int)routeOutStart, routeEnable ? "ok" : "err",
           routeEnable ? "Ativo ✔" : "Desativado");
  c.print(rbuf);

  c.print("</div></div></body></html>");
}

// ===========================================================================
//  JSON /data endpoint
// ===========================================================================
static void buildDataJson(char *buf, int bufSz, int chStart, int chCount,
                          int uni) {
  chStart = constrain(chStart, 1, 512);
  chCount = constrain(chCount, 1, 128);
  if (chStart + chCount - 1 > 512)
    chCount = 512 - chStart + 1;
  const uint8_t *chBuf = uRxMode[uni - 1] ? dmxIn[uni - 1] : dmxData[uni - 1];

  bool ethLnk = Ethernet.linkStatus() == LinkON;
  IPAddress ethIp = ethLnk ? Ethernet.localIP() : IPAddress(0, 0, 0, 0);
  char eIpS[16];
  snprintf(eIpS, sizeof(eIpS), "%d.%d.%d.%d", (int)ethIp[0], (int)ethIp[1],
           (int)ethIp[2], (int)ethIp[3]);

  int n = snprintf(
      buf, bufSz,
      "{\"running\":[%d,%d,%d,%d],"
      "\"ago\":[%lu,%lu,%lu,%lu],"
      "\"pktOk\":[%lu,%lu,%lu,%lu],"
      "\"pktFilt\":%lu,"
      "\"rxModes\":[%d,%d,%d,%d],"
      "\"artnetPort\":%d,\"artnetNet\":%d,\"artnetSubnet\":%d,"
      "\"universes\":[%d,%d,%d,%d],"
      "\"cont\":%d,\"val\":%d,"
      "\"ethLinked\":%d,\"ethIp\":\"%s\","
      "\"routeEnable\":%d,"
      "\"chStart\":%d,\"ch\":[",
      uSt[0].running ? 1 : 0, uSt[1].running ? 1 : 0, uSt[2].running ? 1 : 0,
      uSt[3].running ? 1 : 0, (millis() - uSt[0].lastPktMs) / 1000UL,
      (millis() - uSt[1].lastPktMs) / 1000UL,
      (millis() - uSt[2].lastPktMs) / 1000UL,
      (millis() - uSt[3].lastPktMs) / 1000UL, (unsigned long)uSt[0].pktOk,
      (unsigned long)uSt[1].pktOk, (unsigned long)uSt[2].pktOk,
      (unsigned long)uSt[3].pktOk, (unsigned long)uSt[0].pktFilt,
      uRxMode[0] ? 1 : 0, uRxMode[1] ? 1 : 0, uRxMode[2] ? 1 : 0,
      uRxMode[3] ? 1 : 0, artnetPort, artnetNet, artnetSubnet,
      artnetUniverse[0], artnetUniverse[1], artnetUniverse[2],
      artnetUniverse[3], testContinuous ? 1 : 0, testValue, ethLnk ? 1 : 0,
      eIpS, routeEnable ? 1 : 0, chStart);

  // Envia os canais DMX solicitados de forma segura
  for (int i = 0; i < chCount && n < bufSz - 6; i++) {
    if (i > 0)
      buf[n++] = ',';
    int chIdx = chStart + i;
    if (chIdx > 512)
      break; // Segurança contra estouro de buffer
    n += snprintf(buf + n, bufSz - n, "%d", chBuf[chIdx]);
  }
  if (n < bufSz - 2)
    buf[n++] = ']';
  if (n < bufSz - 2) {
    buf[n++] = '}';
    buf[n] = '\0';
  }
  // Serial.print("Sending JSON: "); // Removido para alta performance
  // Serial.println(buf);
}

// ===========================================================================
//  Painel Ethernet (servidor HTTP raw via W5500 SPI)
// ===========================================================================
static String ethParam(const String &q, const char *key) {
  String kEq = String(key) + "=";
  int idx = -1;

  // Busca a chave garantindo que ela esteja no início ou precedida por '&'
  // para evitar que "net=" case dentro de "subnet="
  int searchPos = 0;
  while (searchPos < (int)q.length()) {
    int found = q.indexOf(kEq, searchPos);
    if (found < 0)
      break;
    if (found == 0 || q[found - 1] == '&') {
      idx = found;
      break;
    }
    searchPos = found + 1;
  }

  if (idx < 0)
    return "";
  int s = idx + kEq.length(), e = q.indexOf('&', s);
  return e < 0 ? q.substring(s) : q.substring(s, e);
}

void handleEthHttp() {
  EthernetClient client = ethServer.accept();
  if (!client)
    return;
  unsigned long t = millis();
  while (!client.available() && client.connected() && millis() - t < 20) {
    dmxPollRx(0);
    dmxPollRx(1);
    dmxPollRx(2);
    dmxPollRx(3); // Mantém DMX vivo durante espera
    delay(1);
  }
  if (!client.available()) {
    client.stop();
    return;
  }

  String reqLine = client.readStringUntil('\n');
  reqLine.trim();
  String method, path, query;
  int s1 = reqLine.indexOf(' '), s2 = reqLine.lastIndexOf(' ');
  if (s1 > 0 && s2 > s1) {
    method = reqLine.substring(0, s1);
    String url = reqLine.substring(s1 + 1, s2);
    int qi = url.indexOf('?');
    if (qi >= 0) {
      path = url.substring(0, qi);
      query = url.substring(qi + 1);
    } else
      path = url;
  }
  if (path.isEmpty())
    path = "/";
  // Log apenas requisicoes que nao sejam /data (evita flood no Serial Monitor)
  if (path != "/data") {
    Serial.print("HTTP ");
    Serial.print(method);
    Serial.print(" ");
    Serial.println(path);
  }

  // Drena headers
  int contentLen = 0;
  unsigned long hdrT = millis();
  // Aumentado timeout para drenar headers
  while (client.connected() && millis() - hdrT < 200) {
    if (!client.available()) {
      delay(1);
      continue;
    }
    String hdr = client.readStringUntil('\n');
    hdr.trim();
    if (hdr.isEmpty())
      break;
    String lh = hdr;
    lh.toLowerCase();
    if (lh.startsWith("content-length:"))
      contentLen = hdr.substring(hdr.indexOf(':') + 1).toInt();
  }
  if (contentLen > 0 && method == "POST") {
    String body = "";
    body.reserve(contentLen);
    t = millis();
    // Aumentado timeout para 500ms para garantir leitura de pacotes lentos
    while ((int)body.length() < contentLen && millis() - t < 500) {
      if (client.available()) {
        body += (char)client.read();
      } else {
        dmxPollRx(0);
        dmxPollRx(1);
        dmxPollRx(2);
        dmxPollRx(3);
        delay(1);
      }
    }
    query = body;
    Serial.print("POST Body (");
    Serial.print(body.length());
    Serial.print("/");
    Serial.print(contentLen);
    Serial.print("): ");
    Serial.println(query);
  }

  // Roteamento
  if (path == "/" || path == "/index.html") {
    client.print("HTTP/1.1 200 OK\r\n"
                 "Content-Type: text/html; charset=UTF-8\r\n"
                 "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                 "Connection: close\r\n\r\n");
    sendPageStreaming(client);
    client.flush();
    client.stop();
    return;
  }
  if (path == "/data") {
    static char jbuf[2048];
    int cs = 1, cc = 16, uni = 1;
    String ss = ethParam(query, "start");
    if (ss.length())
      cs = constrain(ss.toInt(), 1, 512);
    String cs2 = ethParam(query, "count");
    if (cs2.length())
      cc = constrain(cs2.toInt(), 1, 128);
    String us = ethParam(query, "uni");
    if (us.length())
      uni = constrain(us.toInt(), 1, 4);
    buildDataJson(jbuf, sizeof(jbuf), cs, cc, uni);
    client.print("HTTP/1.1 200 OK\r\n"
                 "Content-Type: application/json\r\n"
                 "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                 "Connection: close\r\n\r\n");
    client.print(jbuf);
    client.flush();
    client.stop();
    return;
  }
  if (path == "/test-dmx-live") {
    testContinuous = false;
    String eu = ethParam(query, "u");
    if (!eu.length())
      eu = "both";
    bool doU[4] = {eu == "1" || eu == "both", eu == "2" || eu == "both",
                   eu == "3" || eu == "both", eu == "4" || eu == "both"};
    bool clr = ethParam(query, "clear") == "1";
    for (int u = 0; u < 4; u++)
      if (doU[u] && clr) {
        memset(dmxData[u] + 1, 0, DMX_CHANNELS);
        dmxData[u][0] = 0;
      }
    String ecsv = ethParam(query, "vals");
    if (ecsv.length()) {
      int st2 = 1;
      String ss2 = ethParam(query, "start");
      if (ss2.length())
        st2 = constrain(ss2.toInt(), 1, 512);
      int ech = st2, epos = 0;
      while (epos <= (int)ecsv.length() && ech <= DMX_CHANNELS) {
        int ec = ecsv.indexOf(',', epos);
        if (ec < 0)
          ec = ecsv.length();
        if (ec > epos) {
          uint8_t ev = constrain(ecsv.substring(epos, ec).toInt(), 0, 255);
          for (int u = 0; u < 4; u++)
            if (doU[u])
              dmxData[u][ech] = ev;
          ech++;
        }
        epos = ec + 1;
      }
    }
    for (int u = 0; u < 4; u++)
      if (doU[u] && !uRxMode[u]) {
        uSt[u].lastPktMs = millis();
        uSt[u].running = true;
      }
    liveMode =
        true; // mantem a saida DMX sem timeout enquanto faders estao ativos
    client.print(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: "
        "10\r\nConnection: close\r\n\r\n{\"ok\":true}");
    client.flush();
    client.stop();
    return;
  }
  if (path == "/test-dmx") {
    uint8_t val = testValue;
    String vs = ethParam(query, "val");
    if (vs.length())
      val = constrain(vs.toInt(), 0, 255);
    bool cont = ethParam(query, "cont") == "1";
    bool stop = ethParam(query, "cont") == "0";
    testValue = val;
    for (int u = 0; u < 4; u++) {
      memset(dmxData[u] + 1, val, DMX_CHANNELS);
      dmxData[u][0] = 0x00;
    }
    if (stop) {
      testContinuous = false;
      liveMode = false; // encerra modo faders, reativa timeout
      // Não chamamos dmxSendU() aqui pois bloqueia o SPI por ~22ms × 4 = ~88ms,
      // tempo suficiente para o browser perder a conexão TCP.
      // O loop() central envia o frame de zeros na próxima iteração (<25ms).
      for (int u = 0; u < 4; u++) {
        memset(dmxData[u] + 1, 0, DMX_CHANNELS);
        uSt[u].outLen = DMX_CHANNELS;
        uSt[u].lastPktMs = millis();
        uSt[u].running = true;
      }
    } else if (cont) {
      testContinuous = true;
      for (int u = 0; u < 4; u++) {
        uSt[u].outLen = DMX_CHANNELS;
        uSt[u].lastPktMs = millis();
        uSt[u].running = true;
      }
    } else {
      // Frame único: sinaliza para o loop() enviar
      liveMode = (val > 0); // impede timeout de 3s para o "1 Frame FULL"
      for (int u = 0; u < 4; u++) {
        uSt[u].outLen = DMX_CHANNELS;
        uSt[u].lastPktMs = millis();
        uSt[u].lastSendMs = millis() - 25; // força envio imediato
        uSt[u].running = true;
      }
    }
    for (int u = 0; u < 4; u++) {
      uSt[u].lastPktMs = millis();
      uSt[u].running = (val > 0 || testContinuous || liveMode);
    }
    client.print("HTTP/1.1 303 See Other\r\nLocation: /\r\nContent-Length: "
                 "0\r\nConnection: close\r\n\r\n");
    client.flush();
    client.stop();
    return;
  }
  if (path == "/artnet" && method == "POST") {
    String ps = ethParam(query, "port");
    if (ps.length()) {
      uint16_t p = constrain(ps.toInt(), 1, 65535);
      if (p != artnetPort) {
        artnetPort = p;
        udp_eth.stop();
        udp_eth.begin(artnetPort);
      }
    }
    String ns = ethParam(query, "net");
    if (ns.length())
      artnetNet = constrain(ns.toInt(), 0, 127);
    String ss2 = ethParam(query, "subnet");
    if (ss2.length())
      artnetSubnet = constrain(ss2.toInt(), 0, 15);
    Serial.print("--- POST /artnet START ---\n");
    Serial.print("Query: ");
    Serial.println(query);
    for (int i = 0; i < 4; i++) {
      String ku = "uni" + String(i + 1), kr = "rx" + String(i + 1);
      String kv = ethParam(query, ku.c_str()), rv = ethParam(query, kr.c_str());
      if (kv.length())
        artnetUniverse[i] = constrain(kv.toInt(), 0, 15);
      if (rv.length()) {
        uRxMode[i] = (rv == "1");
        applyDePin(i);
      }
      Serial.printf("  Porta %d: uni_key=%s val=%s (ParsedUni=%d), rx_key=%s "
                    "val=%s (Mode=%s)\n",
                    i + 1, ku.c_str(), kv.c_str(), artnetUniverse[i],
                    kr.c_str(), rv.c_str(), uRxMode[i] ? "RX" : "TX");
    }
    saveConfig();
    Serial.print("--- POST /artnet END ---\n");
    sendPollReply();
    delay(500); // Pequeno delay para garantir que o browser não atropela a
                // resposta
    client.print("HTTP/1.1 303 See Other\r\n"
                 "Location: /\r\n"
                 "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                 "Content-Length: 0\r\n"
                 "Connection: close\r\n\r\n");
    client.flush();
    client.stop();
    return;
  }
  if (path == "/set-rx-mode") {
    int uni = constrain(ethParam(query, "uni").toInt() - 1, 0, 3);
    bool rx = ethParam(query, "rx") == "1";
    uRxMode[uni] = rx;
    applyDePin(uni);
    saveConfig();
    client.print(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: "
        "12\r\nConnection: close\r\n\r\n{\"ok\":true}");
    client.flush();
    client.stop();
    return;
  }
  if (path == "/set-route") {
    String en2 = ethParam(query, "en"), inS = ethParam(query, "inStart"),
           inC = ethParam(query, "inCount"), outS = ethParam(query, "outStart");
    if (en2.length())
      routeEnable = en2 == "1";
    if (inS.length())
      routeInStart = constrain(inS.toInt(), 1, 512);
    if (inC.length())
      routeInCount = constrain(inC.toInt(), 1, 512);
    if (outS.length())
      routeOutStart = constrain(outS.toInt(), 1, 512);
    saveConfig();
    client.print(
        "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: "
        "12\r\nConnection: close\r\n\r\n{\"ok\":true}");
    client.flush();
    client.stop();
    return;
  }
  if (path == "/description.xml") {
    IPAddress ip = Ethernet.localIP();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/xml");
    client.println("Connection: close");
    client.println();
    client.printf(
        "<?xml version=\"1.0\"?>\r\n"
        "<root xmlns=\"urn:schemas-upnp-org:device-1-0\">\r\n"
        " <specVersion><major>1</major><minor>0</minor></specVersion>\r\n"
        " <device>\r\n"
        "  <deviceType>urn:schemas-upnp-org:device:Basic:1</deviceType>\r\n"
        "  <friendlyName>%s</friendlyName>\r\n"
        "  <manufacturer>OneBit</manufacturer>\r\n"
        "  <modelName>NexusProV1</modelName>\r\n"
        "  <modelNumber>1.0</modelNumber>\r\n"
        "  <serialNumber>%02x%02x%02x%02x%02x%02x</serialNumber>\r\n"
        "  <UDN>uuid:8a32d1e0-da50-4d3e-9005-%02x%02x%02x%02x%02x%02x</UDN>\r\n"
        "  <presentationURL>/</presentationURL>\r\n"
        " </device>\r\n"
        "</root>\r\n",
        NODE_SHORT_NAME, ethMac[0], ethMac[1], ethMac[2], ethMac[3], ethMac[4],
        ethMac[5], ethMac[0], ethMac[1], ethMac[2], ethMac[3], ethMac[4],
        ethMac[5]);
    return;
  }
  // 404
  client.print("HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: "
               "close\r\n\r\n");
  client.flush();
  client.stop();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n*****************************************");
  Serial.println("*    REBOOT DETECTED - ONEBIT NEXUS    *");
  Serial.println("*****************************************");
  Serial.println("Starting Node (RP2040 Ethernet-only)...");

  // DE pinos começam em LOW (RX/Escuta) por segurança para evitar
  // braço-de-ferro elétrico no boot
  for (int u = 0; u < 4; u++) {
    pinMode(DMX_DE[u], OUTPUT);
    digitalWrite(DMX_DE[u], LOW);
  }
  // TX pinos HIGH (idle)
  for (int u = 0; u < 4; u++) {
    pinMode(DMX_TX[u], OUTPUT);
    digitalWrite(DMX_TX[u], HIGH);
  }

  // Carrega configurações guardadas na EEPROM
  loadConfig();

  // Inicializa UARTs hardware (U1, U2)
  Serial1.setTX(DMX_TX[0]);
  Serial1.setRX(DMX_RX[0]);
  Serial1.begin(250000, SERIAL_8N2);
  // ESSENCIAL: habilita pull-up interno no pino RX após begin().
  // Sem isso, o GPIO em modo UART_FUNC ativa um pull-down interno que
  // puxa a saída RO do MAX485 para 0V (linha fica em LOW constante),
  // impedindo o UART de detectar start bits e receber qualquer dado.
  gpio_pull_up(DMX_RX[0]); // GP1 — UART0 RX (U1)

  Serial2.setTX(DMX_TX[1]);
  Serial2.setRX(DMX_RX[1]);
  Serial2.begin(250000, SERIAL_8N2);
  gpio_pull_up(DMX_RX[1]); // GP5 — UART1 RX (U2)

  // CRITICO: Desabilita os IRQs de RX do arduino-pico para uart0/uart1.
  // O SerialUART usa um IRQ para mover bytes do FIFO hardware para um ring
  // buffer interno. Isso compete com nossa leitura direta em dmxPollRx(),
  // impedindo a verificacao dos flags BE/FE de cada byte.
  // Com o IRQ desabilitado, nosso codigo tem acesso exclusivo ao FIFO e pode
  // detectar bytes de framing error do BREAK sem race condition.
  // TX continua funcionando via uart_putc_raw() e uart_tx_wait_blocking()
  // que sao baseados em polling (nao dependem de IRQ).
  uart_set_irq_enables(uart0, false, false); // U1: desabilita RX e TX IRQ
  irq_set_enabled(UART0_IRQ, false);
  uart_set_irq_enables(uart1, false, false); // U2: desabilita RX e TX IRQ
  irq_set_enabled(UART1_IRQ, false);

  // Inicializa SerialPIO (U3, U4) — o PIO ja configura pull-up por padrao
  Serial3.begin(250000, SERIAL_8N2);
  Serial4.begin(250000, SERIAL_8N2);

  // Aplica modos DE/RE APOS Serial.begin() para garantir que
  // gpio_set_function() das UARTs nao sobreponha o estado do pino DE carregado
  // da EEPROM
  for (int u = 0; u < 4; u++)
    applyDePin(u);

  // Inicializa buffers DMX
  for (int u = 0; u < 4; u++) {
    memset(dmxData[u], 0, sizeof(dmxData[u]));
    memset(dmxIn[u], 0, sizeof(dmxIn[u]));
  }

  // Ethernet W5500 via SPI0
  SPI.setRX(ETH_MISO);
  SPI.setSCK(ETH_SCK);
  SPI.setTX(ETH_MOSI);
  SPI.begin();

  // Confirma que o Chip Select do SPI p/ Ethernet não será acionado por puro
  // ruído no boot
  pinMode(ETH_CS, OUTPUT);
  digitalWrite(ETH_CS, HIGH);

  // Realiza um Hard Reset rigoroso no módulo Wiznet W5500
  pinMode(ETH_RST, OUTPUT);
  digitalWrite(ETH_RST, LOW);
  delay(50); // Mínimo de 2ms requeridos pelo W5500 para detectar low pulse
  digitalWrite(ETH_RST, HIGH);
  delay(1000); // Dá tempo (1 segundo completo) pro PHY negociar o link com o
               // Switch/Computador antes de ler

  Ethernet.init(ETH_CS);

  Serial.println("Initializing Ethernet module (W5500)...");
  if (useDHCP) {
    Serial.println("DHCP Mode enabled. Requesting IP...");
    if (Ethernet.begin(ethMac, 5000) != 0) {
      ethLinked = true;
      udp_eth.begin(artnetPort);
      ethServer.begin();

      if (mdns.begin(Ethernet.localIP(), NODE_MDNS_NAME)) {
        mdns.addServiceRecord("http", 8080, MDNSServiceTCP);
        Serial.println("mDNS responder started: http://" +
                       String(NODE_MDNS_NAME) + ".local:8080");
      } else {
        Serial.println("Error: mDNS failed to start.");
      }

      // Inicializa SSDP Manual
      initSSDP();

      Serial.print("DHCP Success. Assigned IP: ");
      Serial.println(Ethernet.localIP());
    } else {
      Serial.println("DHCP failed. Network not ready or no DHCP server found.");
    }
  } else {
    Serial.println("Static IP Mode enabled.");
    Ethernet.begin(ethMac, staticIP, Ethernet.dnsServerIP(), staticGateway,
                   staticSubnet);
    ethLinked = true;
    udp_eth.begin(artnetPort);
    ethServer.begin();

    if (mdns.begin(Ethernet.localIP(), NODE_MDNS_NAME)) {
      mdns.addServiceRecord("http", 8080, MDNSServiceTCP);
      Serial.println("mDNS responder started: http://" +
                     String(NODE_MDNS_NAME) + ".local:8080");
    } else {
      Serial.println("Error: mDNS failed to start.");
    }

    // Inicializa SSDP Manual
    initSSDP();

    Serial.print("Static IP set to: ");
    Serial.println(Ethernet.localIP());
  }

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Error: W5500 module was not found! Check SPI connections.");
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Warning: Ethernet cable is not connected.");
  }

  sendPollReply();
}

// ===========================================================================
//  CORE 1 — DMX TX exclusivo (não bloqueia Core 0 / Ethernet / Web)
// ===========================================================================
// O RP2040 tem dois núcleos. Core 0 roda setup()/loop() com rede+web.
// Core 1 roda setup1()/loop1() e cuida APENAS do envio DMX físico.
// Os buffers dmxData[] são compartilhados (leitura/escrita simples de bytes,
// sem necessidade de mutex no protocolo DMX que tolera frames perdidos).
static unsigned long lastTestSendMs = 0;

void setup1() {
  // Core 1 aguarda o Core 0 terminar a inicialização das UARTs/PIO
  delay(2000);
}

void loop1() {
  unsigned long now = millis();

  // DMX TX — 40 FPS por universo (quando há dados ArtNet ou Teste)
  for (int u = 0; u < 4; u++) {
    if (uRxMode[u])
      continue;

    bool needsSend = false;
    int lenToSend = uSt[u].outLen;

    if (testContinuous) {
      // Modo teste contínuo: 10 FPS com frame de 512 canais
      if ((now - uSt[u].lastSendMs) >= 100) {
        needsSend = true;
        lenToSend = DMX_CHANNELS;
      }
    } else if (uSt[u].running) {
      // Dados ArtNet ou faders: 40 FPS
      if ((now - uSt[u].lastSendMs) >= 25) {
        needsSend = true;
      }
      // Timeout 3s: desativado quando faders estao em uso (liveMode)
      // para que o ultimo valor enviado seja mantido na saida DMX.
      if (!liveMode && (now - uSt[u].lastPktMs) > 3000) {
        uSt[u].running = false;
      }
    }

    if (needsSend) {
      dmxSendU(u, lenToSend);
      uSt[u].lastSendMs = now;
    }
  }
}

// ===========================================================================
//  LOOP (Core 0) — Rede, Web, ArtNet RX, DMX RX
//  O DMX TX esta no loop1() (Core 1) para nao bloquear a rede.
// ===========================================================================
static unsigned long lastPollReplyMs = 0;

void loop() {
  // 1. DMX RX polling (todas as UARTs)
  for (int u = 0; u < 4; u++) {
    dmxPollRx(u);
    processRxFrame(u);
  }

  // 2. ArtNet RX
  handleArtNetEth();

  unsigned long now = millis();

  // 3. ArtPollReply periodico (3s)
  if ((now - lastPollReplyMs) >= 3000) {
    lastPollReplyMs = now;
    sendPollReply();
  }

  // 6. Web server Ethernet (Painel HTML e API JSON)
  handleEthHttp();

  // 7. Retry DHCP Ethernet
  if (useDHCP) {
    if (!ethLinked && (now - lastEthRetryMs) >= ETH_RETRY_MS) {
      lastEthRetryMs = now;
      if (Ethernet.begin(ethMac, 5000) != 0) {
        ethLinked = true;
        udp_eth.begin(artnetPort);
        ethServer.begin();

        mdns.begin(Ethernet.localIP(), NODE_MDNS_NAME);
        mdns.addServiceRecord("http", 8080, MDNSServiceTCP);

        sendPollReply();
      }
    }
  }
  if (ethLinked) {
    Ethernet.maintain();
    mdns.run();
    handleSSDP();
  }
}