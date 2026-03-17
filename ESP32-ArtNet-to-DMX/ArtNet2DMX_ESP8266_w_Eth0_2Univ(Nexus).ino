/*
  ArtNet2DMX_ESP8266_w_Eth0.ino - ESP8266 com suporte dual: WiFi + Ethernet e 2 Universos (módulo W5500, biblioteca Ethernet 2.x).
  ===================

  # Recebe um universo ArtNet via WiFi e transmite DMX512 via UART → MAX487 (RS485).
  # Recebe um universo ArtNet via Ethernet e transmite DMX512 via UART → MAX487 (RS485).
  # Inclusão de mais um universo

  Pinagem:
    GPIO1  (UART0 TX) ──► DI   do MAX487  (dados DMX)
    GPIO4             ──► DE + RE do MAX487  (nível HIGH = transmitir)
    GPIO2  (Serial1)  ──► debug (opcional, conecte ao RX do adaptador USB-Serial)

  Universo 1: UART0 TX → GPIO1 (D10), DE/RE → GPIO16 (D0)
  Universo 2: UART1 TX → GPIO2 (D4), RX (UART0 RX) → GPIO3 (RX), DE/RE → GPIO4 (D2)

  MAX485 — Universo 2 (bidirecional):
    Pino 1 (RO)      → RX (GPIO3) — UART0 RX (hardware)
    Pino 2 (/RE)     → D2 (GPIO4) — DE/RE compartilhado (LOW=RX, HIGH=TX)
    Pino 3 (DE)      → D2 (GPIO4) — DE/RE compartilhado (LOW=RX, HIGH=TX)
    Pino 4 (DI)      → D4 (GPIO2) — UART1 TX dados DMX
    Pino 5 (GND)     → GND
    Pino 6 (A/Y)     → linha RS-485 +
    Pino 7 (B/Z)     → linha RS-485 −
    Pino 8 (VCC)     → 3V3


  Bibliotecas necessárias (Gerenciador de Bibliotecas):
    - WiFiManager  (tzapu/WiFiManager)
    - Ethernet     (arduino-libraries/Ethernet >= 2.0.0)

  Primeiro boot sem WiFi salvo: cria AP "ArtNet2DMX" para configurar a rede.
  Após conectar, acesse http://artnet2dmx.local:8080 (mDNS) ou http://<ip>:8080 para o painel.
*/

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <AddrList.h>      // iteração de endereços de rede (IPv4 + IPv6 via lwIP)
#include <EEPROM.h>        // persistência de configurações

// ===========================================================================
//  ETHERNET W5500 — PINAGEM SPI
// ===========================================================================
const int ETH_CS_PIN  = 15;   // GPIO15 (D8) — SPI Chip Select  (requer pull-down 10 kΩ p/ boot ESP8266)
const int ETH_RST_PIN =  5;   // GPIO5  (D1) — Reset (ativo-baixo)
// Barramento SPI — pinos fixos do ESP8266 (hardware HSPI):
//   MOSI = GPIO13 (D7)  |  MISO = GPIO12 (D6)  |  SCLK = GPIO14 (D5)

// ===========================================================================
//  CONFIGURAÇÕES
// ===========================================================================

// --- WiFi ---
const char* WIFI_AP_NAME = "OneBitNexusDMX";
const char* WIFI_AP_PASS = "";           // "" = AP aberto

// --- ArtNet (editáveis pelo painel web) ---
uint16_t artnetPort     = 6454;
uint8_t  artnetNet      = 0;
uint8_t  artnetSubnet   = 0;
uint8_t  artnetUniverse  = 0;
uint8_t  artnetUniverse2 = 1;         // Universo 2 (padrão: Net.Sub.1)
bool     u1RxMode       = false;      // Universo 1: false=TX (ArtNet→DMX), true=RX (DMX→ArtNet)

// --- Roteamento interno DMX (U1 RX → U2 TX sem passar pelo QLC+) ---
bool    routeEnable   = false;  // ativa passagem direta U1 entrada → U2 saída
uint16_t routeInStart  = 1;     // canal de entrada inicial (1-512)
uint16_t routeInCount  = 512;   // quantidade de canais a rotear (1-512)
uint16_t routeOutStart = 1;     // canal de saída inicial (1-512)

// ===========================================================================
//  EEPROM — persistência de configurações
//  Layout (total 16 bytes, offset 0):
//   [0..3]  magic "OBAN"  (OneBit ArtNet Node)
//   [4..5]  artnetPort (uint16_t, little-endian)
//   [6]     artnetNet
//   [7]     artnetSubnet
//   [8]     artnetUniverse
//   [9]     artnetUniverse2
//   [10]    u1RxMode (0 ou 1)
//   [11]    routeEnable (0 ou 1)
//   [12..13] routeInStart (uint16_t, little-endian)
//   [14]    routeInCount (1-255, 0=256-512 não cabe em 1 byte → guardamos count-1)
//   [15]    routeOutStart-1 (canal 1-255; para >255 usa offset)
// ===========================================================================
#define CFG_EEPROM_SIZE  32
#define CFG_MAGIC0  'O'
#define CFG_MAGIC1  'B'
#define CFG_MAGIC2  'A'
#define CFG_MAGIC3  'N'

void saveConfig() {
  EEPROM.begin(CFG_EEPROM_SIZE);
  EEPROM.write(0, CFG_MAGIC0); EEPROM.write(1, CFG_MAGIC1);
  EEPROM.write(2, CFG_MAGIC2); EEPROM.write(3, CFG_MAGIC3);
  EEPROM.write(4, (uint8_t)(artnetPort & 0xFF));
  EEPROM.write(5, (uint8_t)(artnetPort >> 8));
  EEPROM.write(6, artnetNet);
  EEPROM.write(7, artnetSubnet);
  EEPROM.write(8, artnetUniverse);
  EEPROM.write(9, artnetUniverse2);
  EEPROM.write(10, u1RxMode ? 1 : 0);
  EEPROM.write(11, routeEnable ? 1 : 0);
  // routeInStart: 1-512, guardamos em 2 bytes
  EEPROM.write(12, (uint8_t)((routeInStart - 1) & 0xFF));
  EEPROM.write(13, (uint8_t)((routeInStart - 1) >> 8));
  // routeInCount: 1-512, guardamos count-1 em 2 bytes
  uint16_t cnt = (routeInCount < 1) ? 0 : (routeInCount > 512 ? 511 : routeInCount - 1);
  EEPROM.write(14, (uint8_t)(cnt & 0xFF));
  EEPROM.write(15, (uint8_t)(cnt >> 8));
  // routeOutStart: 1-512, igual ao inStart
  EEPROM.write(16, (uint8_t)((routeOutStart - 1) & 0xFF));
  EEPROM.write(17, (uint8_t)((routeOutStart - 1) >> 8));
  EEPROM.commit();
  EEPROM.end();
}

void loadConfig() {
  EEPROM.begin(CFG_EEPROM_SIZE);
  bool valid = (EEPROM.read(0) == CFG_MAGIC0 && EEPROM.read(1) == CFG_MAGIC1 &&
                EEPROM.read(2) == CFG_MAGIC2 && EEPROM.read(3) == CFG_MAGIC3);
  if (valid) {
    artnetPort     = (uint16_t)EEPROM.read(4) | ((uint16_t)EEPROM.read(5) << 8);
    artnetNet      = EEPROM.read(6);
    artnetSubnet   = EEPROM.read(7);
    artnetUniverse  = EEPROM.read(8);
    artnetUniverse2 = EEPROM.read(9);
    u1RxMode       = EEPROM.read(10) == 1;
    routeEnable    = EEPROM.read(11) == 1;
    routeInStart   = (uint16_t)EEPROM.read(12) | ((uint16_t)EEPROM.read(13) << 8);
    routeInStart   = constrain(routeInStart + 1, 1, 512);
    uint16_t cnt   = (uint16_t)EEPROM.read(14) | ((uint16_t)EEPROM.read(15) << 8);
    routeInCount   = constrain((uint16_t)(cnt + 1), 1, 512);
    routeOutStart  = (uint16_t)EEPROM.read(16) | ((uint16_t)EEPROM.read(17) << 8);
    routeOutStart  = constrain(routeOutStart + 1, 1, 512);
  }
  // Se não há magic válido, mantém os valores padrão definidos nas variáveis globais.
  EEPROM.end();
}

// --- DMX ---
const int     DMX_TX_PIN     = 1;     // GPIO1 = UART0 TX  → DMX Universo 1
const int     DMX_DE_PIN     = 16;    // GPIO16 = D0 = DE/RE do MAX487 (Universo 1)
const int     DMX2_TX_PIN    = 2;     // GPIO2 = UART1 TX  → DMX Universo 2
const int     DMX2_RX_PIN    = 3;     // GPIO3 = RX = UART0 RX ← MAX485 pino 1 (RO)
const int     DMX2_DE_PIN    = 4;     // GPIO4 = D2 = DE/RE do MAX485 (LOW=RX, HIGH=TX)
const int     DMX_CHANNELS   = 512;
const int     DMX_BREAK_US   = 100;   // duração do BREAK  (mínimo 92 µs)
const int     DMX_MAB_US     = 12;    // duração do MAB    (mínimo 8 µs)

// --- Node info ---
const char* NODE_SHORT_NAME = "OneBitNexusDMX";
const char* NODE_LONG_NAME  = "OneBit Nexus DMX to ArtNet and ArtNet to DMX Node converter";
const char* NODE_MDNS_NAME  = "onebitdmx";  // → http://onebitdmx.local:8080

// ===========================================================================
//  ESTADO
// ===========================================================================
static uint8_t  dmxData[DMX_CHANNELS + 1];   // [0]=start code, [1..512]=canais (Universo 1)
static uint8_t  dmxData2[DMX_CHANNELS + 1];  // Universo 2
static uint16_t dmxOut1Len = DMX_CHANNELS;   // nCh real a transmitir no U1 (ArtNet ou padrão)
static uint16_t dmxOut2Len = DMX_CHANNELS;   // nCh real a transmitir no U2 (ArtNet/roteamento)
static uint8_t pollReplyBuf[239];
static uint8_t udpBuf[640];

uint8_t  artnetSeq     = 0;
uint32_t lastPacketMs  = 0;          // último ArtDmx Universo 1
uint32_t lastPacketMs2 = 0;          // último ArtDmx Universo 2
bool     dmxRunning    = false;      // recebendo ArtNet? (Universo 1)
bool     dmxRunning2   = false;      // recebendo ArtNet? (Universo 2)
uint32_t pktOk         = 0;          // pacotes aceitos (Universo 1)
uint32_t pktFilt       = 0;          // pacotes filtrados (nenhum universo correspondeu)
uint32_t pktOk2        = 0;          // pacotes aceitos (Universo 2)

bool     testContinuous = false;    // modo teste contínuo (~40fps)
uint8_t  testValue      = 255;      // valor do modo teste contínuo
unsigned long lastTestSend = 0;
unsigned long lastDmxSend  = 0;     // reenvio DMX Universo 1
unsigned long lastDmxSend2 = 0;     // reenvio DMX Universo 2

unsigned long lastPollReply = 0;
const unsigned long POLL_REPLY_INTERVAL_MS = 3000;

// --- Ethernet ---
uint8_t  ethMac[]    = { 0xDE, 0xAD, 0xBE, 0xEF, 0xAB, 0x01 };  // MAC único — altere se houver múltiplos nós na rede
bool     ethLinked   = false;   // W5500 com link ativo e IP DHCP válido
bool     lastPktEth  = false;   // true = último ArtDmx recebido via Ethernet
unsigned long lastEthRetryMs = 0;          // millis do último retry DHCP
const unsigned long ETH_RETRY_MS = 30000;  // intervalo entre retries (30 s)

// --- mDNS Ethernet (responder manual na porta 5353) ---
static const IPAddress MDNS_MCAST(224, 0, 0, 251);
static const uint16_t  MDNS_PORT = 5353;

WiFiUDP          udp;
EthernetUDP      udp_eth;
EthernetUDP      mdns_udp_eth;
EthernetServer   ethServer(8080);  // painel web acessível pelo IP Ethernet
ESP8266WebServer server(8080);
WiFiManager      wm;
bool             wmPortalRunning = false;

// Modo de operação Universo 1 (U2 é fixo como saída — hardware DE/RE ligado ao 3V3)
// (declarado no bloco de configuração acima, junto com artnetPort/Net/Sub/Uni)

// Recepção DMX via UART0 RX (GPIO3) — apenas Universo 1
static uint8_t  dmxInBuf[513];     // buffer bruto da ISR
static uint8_t  dmxIn1[513];       // frame completo — Universo 1 entrada
static uint8_t  dmxIn1Prev[DMX_CHANNELS + 1]; // frame anterior para detecção de mudança
static uint16_t dmxIn1Len = DMX_CHANNELS;     // nCh real do último frame recebido
static uint32_t lastArtNetSendMs = 0;         // keepalive: envia mesmo sem mudança a cada 1s
static uint32_t dmxInLastByteUs = 0;
int      dmxInPos      = 0;
uint32_t dmxInPktOk1   = 0;
enum DmxRxState { WAIT_BREAK, WAIT_SC, COLLECTING };
static DmxRxState dmxRxState = WAIT_BREAK;
// Volatile — compartilhadas entre ISR dmxUartISR() e loop()
static volatile bool dmxIsrReady  = false;   // frame completo disponível
static volatile int  dmxIsrLen    = 0;       // tamanho do último frame
static volatile int  dmxIsrPos2   = 0;       // posição de escrita do ISR
static volatile bool dmxIsrInFrm  = false;   // ISR coletando frame?
bool     dmxInRunning1 = false;
uint32_t lastDmxInMs1  = 0;
static volatile bool dmxNewFrame = false;  // sinaliza que dmxIn1 tem frame novo para enviar via ArtNet

// ===========================================================================
//  HELPERS
// ===========================================================================
IPAddress getNodeIP() {
  IPAddress ip = WiFi.localIP();
  if (ip == IPAddress(0, 0, 0, 0)) ip = WiFi.softAPIP();
  return ip;
}

IPAddress getDirectedBroadcast() {
  IPAddress ip   = getNodeIP();
  IPAddress mask = (WiFi.status() == WL_CONNECTED) ? WiFi.subnetMask()
                                                    : IPAddress(255, 255, 255, 0);
  return IPAddress((uint8_t)(ip[0] | ~mask[0]), (uint8_t)(ip[1] | ~mask[1]),
                   (uint8_t)(ip[2] | ~mask[2]), (uint8_t)(ip[3] | ~mask[3]));
}

// Retorna o endereço IPv6 da interface WiFi: prefere global/site-local, cai para link-local (fe80::)
// Retorna "" se IPv6 não estiver ativo ou nenhum endereço atribuído ainda.
// Nota: W5500 (Ethernet) é hardware IPv4-only — não aparece aqui.
String getWiFiIPv6Str() {
  for (auto& a : addrList) {
    if (a.isV6() && !a.isLocal()) return a.toString();  // routable: DHCPv6/SLAAC
  }
  for (auto& a : addrList) {
    if (a.isV6()) return a.toString();                  // link-local fallback
  }
  return "";
}

// ===========================================================================
//  DMX OUTPUT via UART0 → MAX487
// ===========================================================================
void dmxSend(int nCh = DMX_CHANNELS) {
  if (u1RxMode) return;  // U1 em modo entrada — não transmite
  if (nCh < 1)            nCh = 1;
  if (nCh > DMX_CHANNELS) nCh = DMX_CHANNELS;

  // Aguarda FIFO TX esvaziar antes de gerar BREAK
  Serial.flush();

  // BREAK e MAB usando o bit UCBRK do registrador UART_CONF0:
  //   UCBRK=1 → força TXD LOW (espaço = sinal de BREAK DMX)
  //   UCBRK=0 → libera TXD de volta ao estado UART (HIGH = mark = MAB)
  // UART0 RX (GPIO3) não é afetado — continua recebendo normalmente.
  noInterrupts();
  USC0(0) |=  (1 << UCBRK);          // força GPIO1 LOW = BREAK
  delayMicroseconds(DMX_BREAK_US);   // ≥ 92 µs
  USC0(0) &= ~(1 << UCBRK);          // libera GPIO1 HIGH = MAB
  delayMicroseconds(DMX_MAB_US);     // ≥ 8 µs
  interrupts();

  // Envia start code (0x00) + nCh canais — apenas o necessário
  Serial.write(dmxData, nCh + 1);

  yield();
}

// ===========================================================================
//  ISR UART0 — recepção DMX em tempo real (ICACHE_RAM_ATTR = roda de IRAM).
//  Instalada via ETS_UART_INTR_ATTACH em applyModes().
//  Como a ISR processa each byte à medida que chega, os flags FRM_ERR/BRK_DET
//  em USIS(0) são sempre frescos para o byte atual — sem o problema de
//  acumulação de FIFO que afeta a abordagem de polling no loop().
//
//  Registradores UART0 ESP8266 (base 0x60000000):
//   USF(0)  = FIFO        USS(0)  = STATUS (bits[7:0]=RXFIFO_CNT)
//   USIR(0) = INT_RAW     USIS(0) = INT_ST (= INT_RAW & INT_ENA)
//   USIE(0) = INT_ENA     USIC(0) = INT_CLR
//  Bits relevantes: 0=RXFIFO_FULL  3=FRM_ERR  7=BRK_DET  8=RXFIFO_TOUT
// ===========================================================================
static bool _serialReady = false;  // true após Serial.begin()

static void ICACHE_RAM_ATTR dmxUartISR(void*) {
  uint32_t st = USIS(0);   // INT_ST = INT_RAW & INT_ENA

  // -- BREAK detectado (FRM_ERR ou BRK_DET) -----------------------------------
  if (st & ((1<<3)|(1<<7))) {
    USIC(0) = (1<<3)|(1<<7);
    // CRÍTICO: processa bytes ainda no FIFO ANTES de salvar/resetar o estado.
    // Para frames curtos (< 112 canais = threshold do FIFO_FULL), nenhum FIFO_FULL
    // disparou durante o frame — os bytes ficam no FIFO até o BREAK chegar.
    // Sem este processamento, o frame inteiro seria descartado. Também cobre o
    // caso em que TOUT + BRK_DET disparam no mesmo ISR call (quando noInterrupts()
    // estava ativo durante os 100µs do BREAK+MAB no dmxSend2), onde BRK_DET rodaria
    // primeiro com o código antigo e descartaria o FIFO antes que o TOUT salvasse.
    while (USS(0) & 0xFF) {
      uint8_t b = (uint8_t)USF(0);
      if (!dmxIsrInFrm) {
        if (b == 0x00) { dmxInBuf[0] = 0x00; dmxIsrPos2 = 1; dmxIsrInFrm = true; }
      } else if (dmxIsrPos2 < 513) {
        dmxInBuf[dmxIsrPos2++] = b;
      }
    }
    // Salva frame se temos start code + pelo menos 1 canal
    if (dmxIsrInFrm && dmxIsrPos2 > 1) {
      dmxIsrLen   = dmxIsrPos2;
      dmxIsrReady = true;
    }
    dmxIsrInFrm = false;
    dmxIsrPos2  = 0;
  }

  // -- Bytes de dados no FIFO (FIFO_FULL ou timeout de linha parada) ---------
  if (st & ((1<<0)|(1<<8))) {
    while (USS(0) & 0xFF) {
      uint8_t b = (uint8_t)USF(0);
      if (!dmxIsrInFrm) {
        if (b == 0x00) {            // start code DMX correto
          dmxInBuf[0] = 0x00;
          dmxIsrPos2  = 1;
          dmxIsrInFrm = true;
        }
      } else {
        if (dmxIsrPos2 < 513) dmxInBuf[dmxIsrPos2++] = b;
        if (dmxIsrPos2 >= DMX_CHANNELS + 1) {  // frame completo (512 canais)
          dmxIsrLen   = dmxIsrPos2;
          dmxIsrReady = true;
          dmxIsrInFrm = false;
          dmxIsrPos2  = 0;
        }
      }
    }
    // TOUT com frame parcial: silêncio na linha = fim de frame curto
    if ((st & (1<<8)) && dmxIsrInFrm && dmxIsrPos2 > 1) {
      dmxIsrLen   = dmxIsrPos2;
      dmxIsrReady = true;
      dmxIsrInFrm = false;
      dmxIsrPos2  = 0;
    }
    USIC(0) = (1<<0)|(1<<8);
  }

  USIC(0) = 0xFFFF;  // limpa quaisquer flags residuais
}

// ===========================================================================
//  Aplica modo de operação do MAX485 conforme u1RxMode
//  U1: DE/RE=GPIO16  HIGH=TX (driver habilitado)  LOW=RX (receiver habilitado)
//  U2: DE/RE=GPIO4   fixo HIGH (hardware ligado ao 3V3) — sempre TX
// ===========================================================================
void applyModes() {
  static bool _dmxIsrInstalled = false;  // true enquanto nossa ISR custom está ativa
  digitalWrite(DMX_DE_PIN,  u1RxMode ? LOW : HIGH);
  // DMX2_DE_PIN hardware-fixo ao 3V3 — sem controle por software

  if (!_serialReady) return;  // UART não inicializado ainda — apenas pinos DE/RE

  if (u1RxMode) {
    dmxIsrInFrm = false;
    dmxIsrPos2  = 0;
    dmxIsrReady = false;
    // Limpa buffers de entrada para não exibir dados antigos/lixo
    memset(dmxIn1, 0, sizeof(dmxIn1)); dmxInRunning1 = false; dmxInPktOk1 = 0;
    memset(dmxInBuf, 0, sizeof(dmxInBuf));
    if (!_dmxIsrInstalled) {           // instala apenas na primeira transição TX→RX
      ETS_UART_INTR_DISABLE();
      ETS_UART_INTR_ATTACH(dmxUartISR, NULL);
      USIC(0) = 0xFFFF;
      // RXFIFO_TOUT threshold = 2 baudclocks (~8µs a 250kbps) — mínimo para frames curtos
      USC1(0) = (USC1(0) & ~(0x7F << 24)) | (2 << 24);
      USIE(0) = (1<<0)|(1<<3)|(1<<7)|(1<<8);  // RXFIFO_FULL|FRM_ERR|BRK_DET|RXFIFO_TOUT
      ETS_UART_INTR_ENABLE();
      _dmxIsrInstalled = true;
    }
  } else {
    if (_dmxIsrInstalled) {            // restaura ISR Arduino apenas se nossa ISR estava ativa
      ETS_UART_INTR_DISABLE();
      USIC(0) = 0xFFFF;
      USIE(0) = 0;
      Serial.end();
      delay(2);
      Serial.setRxBufferSize(1030);
      Serial.begin(250000, SERIAL_8N2);
      _dmxIsrInstalled = false;
    }
  }
}

// ===========================================================================
//  DMX OUTPUT via UART1 → MAX485 (Universo 2)
// ===========================================================================
void dmxSend2(int nCh = DMX_CHANNELS) {
  // U2 é sempre saída (DE/RE hardware-fixo ao 3V3)
  if (nCh < 1)            nCh = 1;
  if (nCh > DMX_CHANNELS) nCh = DMX_CHANNELS;
  Serial1.flush();
  noInterrupts();
  USC0(1) |=  (1 << UCBRK);          // força GPIO2 LOW = BREAK
  delayMicroseconds(DMX_BREAK_US);   // ≥ 92 µs
  USC0(1) &= ~(1 << UCBRK);          // libera GPIO2 HIGH = MAB
  delayMicroseconds(DMX_MAB_US);     // ≥ 8 µs
  interrupts();

  // Envia start code + nCh canais — apenas o necessário
  Serial1.write(dmxData2, nCh + 1);

  yield();
}

// ===========================================================================
//  Envia frame ArtDmx a partir de dados DMX recebidos fisicamente (RX mode)
//  data[0] = start code, data[1..DMX_CHANNELS] = canais
//  uni = universo ArtNet a usar no pacote
// ===========================================================================
static void sendArtDmxInput(const uint8_t* data, uint8_t uni, bool toEth, int nCh = DMX_CHANNELS) {
  // ArtNet spec: Length deve ser par, mínimo 2
  if (nCh < 2)             nCh = 2;
  if (nCh & 1)             nCh++;          // arredonda para par
  if (nCh > DMX_CHANNELS)  nCh = DMX_CHANNELS;
  static uint8_t artBuf[18 + DMX_CHANNELS];
  memcpy(artBuf, "Art-Net", 7);
  artBuf[7]  = 0x00;
  artBuf[8]  = 0x00; artBuf[9]  = 0x50;  // OpCode ArtDmx = 0x5000
  artBuf[10] = 0x00; artBuf[11] = 0x0E;  // ProtVer = 14
  artBuf[12] = artnetSeq++;
  artBuf[13] = 0x00;                      // Physical
  artBuf[14] = (uint8_t)((artnetSubnet & 0x0F) << 4) | (uni & 0x0F);
  artBuf[15] = artnetNet & 0x7F;
  artBuf[16] = (uint8_t)(nCh >> 8);       // Length high byte (big-endian)
  artBuf[17] = (uint8_t)(nCh & 0xFF);     // Length low byte
  memcpy(artBuf + 18, data + 1, nCh);
  int pktLen = 18 + nCh;

  if (toEth && ethLinked) {
    IPAddress ip   = Ethernet.localIP();
    IPAddress mask = Ethernet.subnetMask();
    IPAddress bcast((uint8_t)(ip[0]|~mask[0]),(uint8_t)(ip[1]|~mask[1]),
                    (uint8_t)(ip[2]|~mask[2]),(uint8_t)(ip[3]|~mask[3]));
    udp_eth.beginPacket(bcast, artnetPort);
    udp_eth.write(artBuf, pktLen);
    udp_eth.endPacket();
  } else {
    IPAddress bcast = getDirectedBroadcast();
    udp.beginPacket(bcast, artnetPort);
    udp.write(artBuf, pktLen);
    udp.endPacket();
  }
}

// ===========================================================================
//  Recepção DMX via UART0 RX (GPIO3) — ativa quando u1RxMode
//
//  Detecção de BREAK por gap de tempo entre bytes:
//   • bytes consecutivos dentro do frame chegam a cada ~44 µs (250 kbaud, 8N2)
//   • gap > 300 µs = BREAK + MAB = fim/início de frame
//   • primeiro byte após gap com valor 0x00 = start code DMX
//
//  u1RxMode: frame vai para dmxIn1 → rebroadcast ArtNet no universo 1
//  O roteamento DMX (ex: espelhar para U2 TX) é responsabilidade do software externo.
//  U2 é sempre saída — sem modo RX
// ===========================================================================
// Flush e salva frame de dmxInBuf para dmxIn1 — rebroadcast ArtNet apenas.
// O software externo decide o roteamento (ex: enviar de volta como DMX pelo U2).
// nBytes = número de bytes em dmxInBuf (inclui start code em [0]).
static void dmxSaveFrame(int nBytes) {
  if (nBytes < 2) return;   // mínimo: start code + 1 canal
  int nCh = nBytes - 1;
  if (nCh > DMX_CHANNELS) nCh = DMX_CHANNELS;

  if (u1RxMode) {
    memcpy(dmxIn1, dmxInBuf, 1 + nCh);
    memset(dmxIn1 + 1 + nCh, 0, DMX_CHANNELS - nCh);
    dmxIn1Len = (uint16_t)nCh;
    dmxInPktOk1++;
    dmxInRunning1 = true;
    lastDmxInMs1  = millis();
    // Só sinaliza envio se os dados mudaram OU keepalive de 1s expirou
    bool changed   = (memcmp(dmxIn1 + 1, dmxIn1Prev + 1, nCh) != 0);
    bool keepalive = ((millis() - lastArtNetSendMs) >= 1000UL);
    if (changed || keepalive) {
      memcpy(dmxIn1Prev, dmxIn1, 1 + nCh);
      dmxNewFrame = true;
    }
  }
}

// ===========================================================================
//  dmxReceive() — chamado no loop(). Apenas verifica se o ISR sinalizou
//  um frame completo e o processa via dmxSaveFrame().
// ===========================================================================
void dmxReceive() {
  if (!u1RxMode) {
    while (Serial.available()) Serial.read();
    return;
  }
  if (!dmxIsrReady) return;
  noInterrupts();
  int len = dmxIsrLen;
  dmxIsrReady = false;
  interrupts();
  dmxSaveFrame(len);
}

// Recebe frame DMX e, se houver frame novo, envia imediatamente via ArtNet.
// Substitui todas as chamadas isoladas a dmxReceive() no loop() para garantir
// que cada frame seja encaminhado sem aguardar o próximo ciclo completo do loop.
static void dmxReceiveAndSend() {
  dmxReceive();
  if (!dmxNewFrame) return;
  dmxNewFrame = false;

  // Broadcast ArtNet (para QLC+ e outros listeners)
  sendArtDmxInput(dmxIn1, artnetUniverse, false, dmxIn1Len);  // via WiFi
  sendArtDmxInput(dmxIn1, artnetUniverse, true,  dmxIn1Len);  // via Ethernet
  lastArtNetSendMs = millis();

  // --- Roteamento interno: U1 entrada → U2 saída (sem QLC+) ---
  // Copia os canais configurados de dmxIn1 para dmxData2 e dispara imediatamente dmxSend2().
  // Latência: apenas o tempo de transmissão UART (~23ms para 512ch, proporcional para menos).
  if (routeEnable) {
    uint16_t inEnd  = routeInStart + routeInCount - 1;
    if (inEnd  > 512) inEnd  = 512;
    uint16_t nCh = inEnd - routeInStart + 1;
    uint16_t outEnd = routeOutStart + nCh - 1;
    if (outEnd > 512) nCh = 512 - routeOutStart + 1;
    // Copia canais mapeados: dmxIn1[routeInStart..] → dmxData2[routeOutStart..]
    memcpy(dmxData2 + routeOutStart, dmxIn1 + routeInStart, nCh);
    dmxData2[0] = 0x00;  // start code
    // Envia apenas até o último canal roteado — evita transmitir zeros desnecessários.
    // Ex: outStart=1, nCh=32 → 33 bytes (~1.5 ms) em vez de 513 (~22.6 ms).
    uint16_t sendLen = (uint16_t)(routeOutStart - 1 + nCh);
    if (sendLen > DMX_CHANNELS) sendLen = DMX_CHANNELS;
    dmxOut2Len = sendLen;
    dmxSend2(sendLen);
    lastDmxSend2  = millis();
    dmxRunning2   = true;
    lastPacketMs2 = millis();
  } else if (artnetUniverse == artnetUniverse2) {
    // Comportamento anterior: espelha U1→U2 quando universos são iguais
    memcpy(dmxData2, dmxIn1, 1 + dmxIn1Len);
    dmxOut2Len = dmxIn1Len;
    dmxSend2(dmxIn1Len);
    lastDmxSend2  = millis();
    dmxRunning2   = true;
    lastPacketMs2 = millis();
  }
}

// ===========================================================================
//  ArtPollReply
// ===========================================================================
void buildPollReply(uint8_t* buf, IPAddress ip, const uint8_t* mac) {
  memset(buf, 0, 239);
  memcpy(buf, "Art-Net", 7);
  buf[8]  = 0x00; buf[9]  = 0x21;          // OpCode ArtPollReply
  buf[10] = ip[0]; buf[11] = ip[1];
  buf[12] = ip[2]; buf[13] = ip[3];
  buf[14] = artnetPort & 0xFF;
  buf[15] = (artnetPort >> 8) & 0xFF;
  buf[16] = 0x00; buf[17] = 0x01;           // versão firmware
  buf[18] = artnetNet    & 0x7F;             // NetSwitch
  buf[19] = artnetSubnet & 0x0F;             // SubSwitch
  buf[20] = 0xFF; buf[21] = 0xFF;            // OEM
  buf[23] = 0xD2;                            // Status1
  strncpy((char*)&buf[26], NODE_SHORT_NAME, 17);
  strncpy((char*)&buf[44], NODE_LONG_NAME,  63);
  strncpy((char*)&buf[108], "#0001 [0000] OK", 63);
  buf[173] = 0x02;                                          // NumPorts = 2
  // U1: bidirecional quando em modo RX, saída somente caso contrário
  // U2: sempre saída (hardware fixo)
  buf[174] = u1RxMode ? 0xC0 : 0x40;                       // PortTypes[0] (Universo 1)
  buf[175] = 0x40;                                          // PortTypes[1] (Universo 2 — sempre TX)
  buf[178] = u1RxMode ? 0x80 : 0x00;                       // GoodInput[0]
  buf[179] = 0x00;                                          // GoodInput[1] (U2 sem RX)
  buf[182] = u1RxMode ? 0x80 : 0x00;                       // GoodOutput[0]
  buf[183] = 0x80;                                          // GoodOutput[1] (U2 sempre TX)
  buf[186] = artnetUniverse  & 0x0F;                        // SwOut[0]
  buf[187] = artnetUniverse2 & 0x0F;                        // SwOut[1]
  buf[190] = artnetUniverse  & 0x0F;                        // SwIn[0]
  buf[191] = artnetUniverse2 & 0x0F;                        // SwIn[1]
  memcpy(&buf[201], mac, 6);
  buf[207] = ip[0]; buf[208] = ip[1];
  buf[209] = ip[2]; buf[210] = ip[3];
  buf[211] = 0x01;                           // BindIndex
  buf[212] = 0x08;                           // Status2
}

void sendArtPollReply() {
  uint8_t mac[6]; WiFi.macAddress(mac);
  buildPollReply(pollReplyBuf, getNodeIP(), mac);
  IPAddress bcast = getDirectedBroadcast();
  udp.beginPacket(bcast, artnetPort);
  udp.write(pollReplyBuf, 239);
  udp.endPacket();
}

void sendArtPollReplyTo(IPAddress dest) {
  uint8_t mac[6]; WiFi.macAddress(mac);
  buildPollReply(pollReplyBuf, getNodeIP(), mac);
  udp.beginPacket(dest, artnetPort);
  udp.write(pollReplyBuf, 239);
  udp.endPacket();
}

void sendArtPollReplyEth() {
  if (!ethLinked) return;
  IPAddress ip   = Ethernet.localIP();
  IPAddress mask = Ethernet.subnetMask();
  IPAddress bcast((uint8_t)(ip[0] | ~mask[0]), (uint8_t)(ip[1] | ~mask[1]),
                  (uint8_t)(ip[2] | ~mask[2]), (uint8_t)(ip[3] | ~mask[3]));
  buildPollReply(pollReplyBuf, ip, ethMac);
  udp_eth.beginPacket(bcast, artnetPort);
  udp_eth.write(pollReplyBuf, 239);
  udp_eth.endPacket();
}

void sendArtPollReplyToEth(IPAddress dest) {
  if (!ethLinked) return;
  buildPollReply(pollReplyBuf, Ethernet.localIP(), ethMac);
  udp_eth.beginPacket(dest, artnetPort);
  udp_eth.write(pollReplyBuf, 239);
  udp_eth.endPacket();
}

// ===========================================================================
//  Recepção ArtNet (dual-interface)
// ===========================================================================

// Processa um buffer ArtNet já lido — usado por WiFi e Ethernet
void processArtNetPacket(uint8_t* buf, int len, IPAddress sender, bool isEth) {
  if (memcmp(buf, "Art-Net", 7) != 0 || buf[7] != 0x00) return;

  uint16_t opcode = buf[8] | ((uint16_t)buf[9] << 8);
  const char* src = isEth ? "ETH" : "WiFi";

  // --- ArtPoll → responde pela mesma interface que recebeu ---
  if (opcode == 0x2000) {
    if (isEth) { sendArtPollReplyToEth(sender); sendArtPollReplyEth(); }
    else        { sendArtPollReplyTo(sender);    sendArtPollReply();    }
    return;
  }

  // --- ArtDmx ---
  if (opcode == 0x5000 && len >= 18) {
    uint8_t pktNet      = buf[15] & 0x7F;
    uint8_t pktSubUni   = buf[14];
    uint8_t pktSubnet   = (pktSubUni >> 4) & 0x0F;
    uint8_t pktUniverse = pktSubUni & 0x0F;

    bool matchU1 = (pktNet == artnetNet && pktSubnet == artnetSubnet && pktUniverse == artnetUniverse);
    bool matchU2 = (pktNet == artnetNet && pktSubnet == artnetSubnet && pktUniverse == artnetUniverse2);
    if (!matchU1 && !matchU2) { pktFilt++; return; }

    uint16_t dataLen = ((uint16_t)buf[16] << 8) | buf[17];
    if (dataLen > DMX_CHANNELS) dataLen = DMX_CHANNELS;

    if (matchU1 && !u1RxMode) {
      dmxData[0] = 0x00;
      memcpy(dmxData + 1, buf + 18, dataLen);
      // Detecta canais alterados
      int alteredCount = 0;
      for (int i = 1; i <= dataLen; i++) {
        if (dmxData[i] != 0) alteredCount++;
      }
      if (alteredCount == 0) alteredCount = 1; // Garante pelo menos 1 canal
      if (alteredCount <= 8) {
        dmxOut1Len = 8;
      } else {
        dmxOut1Len = alteredCount;
      }
      if (dmxOut1Len > DMX_CHANNELS) dmxOut1Len = DMX_CHANNELS;
      // Não chama dmxSend() aqui — o timer do loop() dispara na próxima iteração.
      // Subtraímos 25 ms para que (millis()-lastDmxSend) >= 25 na próxima checagem.
      lastDmxSend  = millis() - 25;
      lastPacketMs = millis();
      dmxRunning   = true;
      lastPktEth   = isEth;
      pktOk++;
    }
    if (matchU2) {  // U2 é sempre TX
      dmxData2[0] = 0x00;
      memcpy(dmxData2 + 1, buf + 18, dataLen);
      dmxOut2Len    = (uint16_t)dataLen;  // guarda comprimento real para dmxSend2()
      lastDmxSend2  = millis() - 25;
      lastPacketMs2 = millis();
      dmxRunning2   = true;
      pktOk2++;
    }
    if ((!matchU1 || u1RxMode) && !matchU2) pktFilt++;
  }
}

// Lê pacote da interface WiFi e processa
void handleArtNetWiFi() {
  // Drena TODOS os pacotes UDP pendentes — evita acúmulo quando o loop foi lento
  while (true) {
    int pktSize = udp.parsePacket();
    if (pktSize < 10) break;
    IPAddress sender = udp.remoteIP();
    int len = udp.read(udpBuf, sizeof(udpBuf));
    if (len < 10) break;
    processArtNetPacket(udpBuf, len, sender, false);
  }
}

// Lê pacote da interface Ethernet (W5500) e processa
void handleArtNetEth() {
  if (!ethLinked) return;
  // Drena TODOS os pacotes UDP pendentes — evita acúmulo quando o loop foi lento
  while (true) {
    int pktSize = udp_eth.parsePacket();
    if (pktSize < 10) break;
    IPAddress sender = udp_eth.remoteIP();
    int len = udp_eth.read(udpBuf, sizeof(udpBuf));
    if (len < 10) break;
    processArtNetPacket(udpBuf, len, sender, true);
  }
}

// ===========================================================================
//  Painel Web — streaming direto sem alocação de String grande
//  Todo HTML estático fica em PROGMEM (flash); apenas os valores dinâmicos
//  são formatados em pequenos buffers de stack e enviados em seguida.
//  Isso elimina a necessidade de alocar ~15 KB de heap contígua.
// ===========================================================================

// Helper: envia string literal de PROGMEM pelo cliente WiFi
static void wSend(Print& c, const __FlashStringHelper* s) { c.print(s); }
// Helper: envia string de RAM pelo cliente
static void wSend(Print& c, const String& s)              { c.print(s); }
static void wSend(Print& c, const char* s)                 { c.print(s); }

// Seções estáticas em PROGMEM — separadas para caber em blocos razoáveis
static const char HTML_HEAD[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OneBit Nexus DMX</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;display:flex;flex-direction:column;min-height:100vh}
/* ── Topbar ── */
.topbar{display:flex;align-items:center;gap:12px;padding:10px 16px;background:#10192e;border-bottom:1px solid #1e2a4a;flex-shrink:0}
.logo{flex-shrink:0}.title-wrap{flex:1;min-width:0}
h1{color:#e94560;font-size:1.25em;white-space:nowrap}
.sub{color:#aaa;font-size:.75em;line-height:1.5}.sub b{color:#eee}
.mdns-link{display:block;color:#7ec8e3;font-size:.75em;text-decoration:none;margin-top:2px}
#dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#555;margin-left:6px;vertical-align:middle}
/* ── Layout principal ── */
.main-wrap{display:flex;flex:1;min-height:0}
/* ── Sidebar ── */
.sidebar{width:200px;min-width:200px;background:#10192e;border-right:1px solid #1e2a4a;display:flex;flex-direction:column;transition:width .2s;overflow:hidden;flex-shrink:0}
.sidebar.collapsed{width:52px;min-width:52px}
.sb-toggle{display:flex;align-items:center;justify-content:flex-end;padding:8px 10px;border-bottom:1px solid #1e2a4a;cursor:pointer;color:#7ec8e3;gap:6px;white-space:nowrap;overflow:hidden}
.sb-toggle:hover{background:#162040}
.sb-toggle svg{flex-shrink:0}
.sb-label{font-size:.78em;overflow:hidden;white-space:nowrap;transition:opacity .15s}
.sidebar.collapsed .sb-label{opacity:0;width:0}
.sidebar.collapsed .sb-toggle{justify-content:center}
.sb-nav{list-style:none;flex:1}
.sb-nav li a{display:flex;align-items:center;gap:10px;padding:11px 14px;color:#bbb;text-decoration:none;font-size:.88em;white-space:nowrap;overflow:hidden;border-left:3px solid transparent;transition:background .15s,color .15s}
.sb-nav li a:hover{background:#162040;color:#eee}
.sb-nav li a.active{background:#0f3460;color:#7ec8e3;border-left-color:#7ec8e3}
.sb-nav li a svg{flex-shrink:0}
.sb-nav li a span{overflow:hidden;white-space:nowrap;transition:opacity .15s}
.sidebar.collapsed .sb-nav li a span{opacity:0;width:0}
.sidebar.collapsed .sb-nav li a{padding:11px 0;justify-content:center}
/* ── Content area ── */
.content{flex:1;padding:20px;overflow-y:auto;min-width:0}
.section{display:none}.section.active{display:block}
/* ── Card ── */
.card{background:#16213e;border-radius:8px;padding:20px;margin-bottom:16px}
.card h2{font-size:1em;color:#7ec8e3;margin-bottom:14px;padding-bottom:8px;border-bottom:1px solid #1e2a4a}
/* ── Grid layouts ── */
.g2{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:14px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:14px}
@media(max-width:700px){.g2,.g3,.g4{grid-template-columns:1fr}}
@media(max-width:1000px){.g3,.g4{grid-template-columns:1fr 1fr}}
/* ── Rows / fields ── */
.row{display:flex;justify-content:space-between;align-items:center;font-size:.88em;padding:6px 0;border-bottom:1px solid #1e2a4a}
.row:last-child{border:none}
.val{color:#e94560;font-weight:bold}
.v6{color:#7ec8e3;font-weight:normal;font-size:.75em;word-break:break-all;display:block;text-align:right;margin-top:1px}
.lbl{font-size:.72em;color:#aaa;margin-bottom:4px}
.sec-title{font-size:.72em;color:#7ec8e3;padding:8px 0 4px;border-bottom:1px solid #1e2a4a;margin:8px 0 4px;font-weight:bold;letter-spacing:.05em;text-transform:uppercase}
/* ── Form elements ── */
input[type=number],select{width:100%;padding:7px 9px;border-radius:4px;border:1px solid #444;background:#0f3460;color:#eee;font-size:.93em}
.btn{display:block;width:100%;padding:10px;border:none;border-radius:4px;cursor:pointer;font-size:.95em;margin-top:10px;color:#fff;background:#e94560}
.btn:hover{background:#c73652}
/* ── Toggle switch ── */
.toggle-sw{position:relative;display:inline-block;width:46px;height:26px;flex-shrink:0}
.toggle-sw input{opacity:0;width:0;height:0}
.toggle-sl{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#444;border-radius:26px;transition:.3s}
.toggle-sl:before{position:absolute;content:"";height:20px;width:20px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.3s}
.toggle-sw input:checked+.toggle-sl{background:#2196f3}
.toggle-sw input:checked+.toggle-sl:before{transform:translateX(20px)}
.btn-sm{display:inline-block;padding:8px 14px;border:none;border-radius:4px;cursor:pointer;font-size:.88em;color:#fff;background:#e94560;text-decoration:none;width:100%;text-align:center}
.btn-sm:hover{background:#c73652}
/* ── Status colors ── */
.ok{color:#4caf50}.warn{color:#ff9800}.err{color:#f44336}
/* ── Channel grid ── */
.ch-cell{background:#0f2040;border-radius:4px;padding:6px 8px;text-align:center}
.ch-cell .lbl{font-size:.65em;margin-bottom:2px}
.ch-cell .val{font-size:1.1em}
#ch-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(58px,1fr));gap:6px;margin-top:8px}
/* ── Banner info ── */
.banner-info{background:#1a3a5c;border-left:3px solid #7ec8e3;padding:7px 10px;border-radius:4px;font-size:.82em;margin-bottom:10px}
.banner-ok{background:#0d2c1e;border-left:3px solid #4caf50;padding:7px 10px;border-radius:4px;font-size:.82em;margin-bottom:10px}
</style>)HTML";

static const char HTML_SCRIPT[] PROGMEM = R"HTML(<script>
var T=null,U=1;
// ── Sidebar collapse ──────────────────────────────────────────────────────
function toggleSidebar(){
  var sb=document.getElementById('sidebar');
  sb.classList.toggle('collapsed');
  localStorage.setItem('sbCollapsed',sb.classList.contains('collapsed')?'1':'0');
}
// ── Section navigation ────────────────────────────────────────────────────
function showSection(id){
  document.querySelectorAll('.section').forEach(function(s){s.classList.remove('active');});
  document.querySelectorAll('.sb-nav a').forEach(function(a){a.classList.remove('active');});
  var s=document.getElementById('sec-'+id);
  if(s)s.classList.add('active');
  var a=document.querySelector('.sb-nav a[data-sec="'+id+'"]');
  if(a)a.classList.add('active');
  localStorage.setItem('activeSec',id);
}
// ── Live data update ───────────────────────────────────────────────────────
function setUni(u){U=u;sched();}
// Auto-scheduling: upd() se reagenda a cada 50ms após a resposta chegar.
// Não usa setInterval — evita acumular requisições se o ESP8266 estiver ocupado.
function sched(){if(T)clearTimeout(T);T=setTimeout(upd,0);}  // disparo imediato
function upd(){
  if(T){clearTimeout(T);T=null;}
  if(_fetching)return;  // já tem fetch em andamento — próximo ciclo pegará os dados
  _fetching=true;
  var st=parseInt(document.getElementById('ch-start').value)||1;
  var cnt=parseInt(document.getElementById('ch-count').value)||16;
  if(st<1)st=1;if(st>512)st=512;if(cnt<1)cnt=1;if(cnt>128)cnt=128;
  var ist=parseInt((document.getElementById('in1-start')||{value:'1'}).value)||1;
  var icnt=parseInt((document.getElementById('in1-count')||{value:'32'}).value)||32;
  fetch('/data?start='+st+'&count='+cnt+'&uni='+U+'&inStart='+ist+'&inCount='+icnt).then(r=>r.json()).then(function(d){
    // U1 ArtNet status
    var s=document.getElementById('rx-status');
    if(s){if(!d.running){s.className='val err';s.textContent='Aguardando...';}
      else if(d.ago<3){s.className='val ok';s.textContent='Ativo';}
      else{s.className='val warn';s.textContent='Sem pacotes';}}
    _t('rx-ago',d.running?(d.ago+'s atr\u00e1s'):'\u2014');
    _t('rx-ch123',d.s1+' / '+d.s2+' / '+d.s3);
    _t('rx-ok',d.pktOk);_t('rx-filt',d.pktFilt);
    var iv=document.getElementById('rx-interval');
    if(iv){var ms=d.interval;iv.textContent=ms===0?'M\u00e1x':(ms<1000?(ms+'ms'):(+(ms/1000).toFixed(1)+'s'));}
    // Channel grid — reconstrói só quando mudam start/count, caso contrário atualiza apenas os textos
    var g=document.getElementById('ch-grid');
    if(g){
      var ns=d.chStart,nl=d.ch.length;
      if(ns!==_chStart||nl!==_chCount){
        _chStart=ns;_chCount=nl;
        var h='';
        for(var i=0;i<nl;i++)
          h+='<div class="ch-cell"><div class="lbl">Ch '+(ns+i)+'</div>'
            +'<div class="val" id="chv-'+(ns+i)+'">'+d.ch[i]+'</div></div>';
        g.innerHTML=h;
      } else {
        for(var i=0;i<nl;i++){
          var cv=document.getElementById('chv-'+(ns+i));
          if(cv){var nv=d.ch[i]+'';if(cv.textContent!==nv)cv.textContent=nv;}
        }
      }
    }
    // Card channel title
    var ct=document.getElementById('card-ch-title');
    var ul='U'+U+(U===1&&d.u1rx?' (RX)':' (TX)');
    if(ct)ct.textContent=d.cont?('Canais DMX (teste ativo \u2014 val='+d.val+')'):'Canais DMX '+ul+' ('+(d.chStart)+'\u2013'+(d.chStart+d.ch.length-1)+')';
    // Universe labels
    var l1=document.getElementById('lbl-uni1');if(l1)l1.textContent='U1 '+(d.u1rx?'(\u25c4 Entrada)':'(\u25ba Sa\u00edda)');
    var l2=document.getElementById('lbl-uni2');if(l2)l2.textContent='U2 (\u25ba Sa\u00edda)';  // U2 fixo como sa\u00edda
    // RX banner on channel card
    var rxb=document.getElementById('ch-rx-banner');
    if(rxb){var isRx=(U===1&&d.u1rx);rxb.style.display=isRx?'block':'none';
      if(isRx)rxb.innerHTML='&#9664; Exibindo canais <b>recebidos</b> (DMX\u2192ArtNet) &mdash; U1 modo Entrada';}
    // Test continuous
    var sb=document.getElementById('cont-status'),ss=document.getElementById('cont-start');
    if(sb&&ss){if(d.cont){sb.style.display='block';sb.querySelector('span').textContent=d.val;ss.style.display='none';}
      else{sb.style.display='none';ss.style.display='flex';}}
    // Ethernet
    var es=document.getElementById('eth-status');
    if(es){if(d.ethLinked){es.className='val ok';es.textContent=d.ethIp;}
      else if(!d.ethHw){es.className='val err';es.textContent='Sem hardware';}
      else if(!d.ethLink){es.className='val err';es.textContent='Sem cabo/link';}
      else{es.className='val warn';es.textContent='Sem IP (DHCP)';}}
    // WiFi
    var ws=document.getElementById('wifi-status');
    if(ws){var wip=d.wifiIp||'';ws.className=(wip&&wip!=='0.0.0.0')?'val ok':'val err';
      if(wip&&wip!=='0.0.0.0')ws.innerHTML=wip+(d.wifiIpv6?'<span class="v6">'+d.wifiIpv6+'</span>':'');
      else ws.textContent='Sem link';}
    _t('rx-src',d.running?(d.lastSrc==='eth'?'Ethernet':'WiFi'):'\u2014');
    // U2 TX status section (sempre visível, U2 é sempre TX)
    var u2tx=document.getElementById('sec-u2tx');if(u2tx)u2tx.style.display='block';
    var s2=document.getElementById('rx2-status');
    if(s2){
      if(!d.running2){s2.className='val err';s2.textContent='Aguardando...';}
      else if(d.ago2<3){s2.className='val ok';s2.textContent='Ativo';}
      else{s2.className='val warn';s2.textContent='Sem pacotes';}
      _t('rx2-ago',d.running2?(d.ago2+'s atr\u00e1s'):'\u2014');
      _t('rx2-ch123',d.s1_2+' / '+d.s2_2+' / '+d.s3_2);_t('rx2-ok',d.pktOk2);}
    // Monitor Entrada DMX U1
    var in1off=document.getElementById('in1-off'),in1on=document.getElementById('in1-on');
    if(in1off)in1off.style.display=d.u1rx?'none':'block';
    if(in1on)in1on.style.display=d.u1rx?'block':'none';
    // Sincroniza select e banner da tela ArtNet com o estado atual,
    // mas nunca sobrescreve enquanto o usuário estiver com o select aberto/focado.
    var artSel=document.getElementById('artnet-u1rx-sel');
    if(artSel){
      var srv=d.u1rx?'1':'0';
      // Limpa _dirty quando o servidor já confirmou o novo valor salvo
      if(artSel._dirty&&artSel.value===srv)artSel._dirty=false;
      // Só sincroniza se o usuário não fez nenhuma alteração pendente
      if(!artSel._dirty)artSel.value=srv;
    }
    var artBan=document.getElementById('artnet-u1-banner');
    if(artBan){artBan.className=d.u1rx?'banner-info':'banner-ok';
      artBan.innerHTML=d.u1rx?'&#9664; <b>U1 Entrada ativa<\/b> &mdash; GPIO3 lendo DMX &rarr; ArtNet. Espelho &rarr; U2 TX.':'&#9654; <b>U1 Sa\u00edda<\/b> &mdash; U1 (GPIO1) e U2 (GPIO2) transmitindo DMX.';}
    if(d.u1rx){
      var d1=document.getElementById('dmxin1-status');
      if(d1){if(!d.dmxInRunning1){d1.className='val err';d1.textContent='Aguardando...';}
        else if(d.dmxInAgo1<3){d1.className='val ok';d1.textContent='Ativo';}
        else{d1.className='val warn';d1.textContent='Sem sinal';}}
      _t('dmxin1-ago',d.dmxInRunning1?(d.dmxInAgo1+'s atr\u00e1s'):'\u2014');
      _t('dmxin1-ok',d.dmxInPktOk1);
      // Atualiza grid de entrada direto do /data — sempre presente quando u1RxMode
      if(d.inCh){
        var ig=document.getElementById('in1-grid');if(ig){
          if(!d.dmxInRunning1){
            ig.innerHTML='<div style="color:#888;font-size:.85em;padding:8px 0">Aguardando sinal DMX...</div>';
            _inChStart=0;_inChCount=0;
          } else if(d.inChStart!==_inChStart||d.inCh.length!==_inChCount){
            _inChStart=d.inChStart;_inChCount=d.inCh.length;
            var h='';
            for(var i=0;i<d.inCh.length;i++)
              h+='<div class="ch-cell"><div class="lbl">Ch '+(d.inChStart+i)+'</div>'
                +'<div class="val" id="ichv-'+(d.inChStart+i)+'">'+d.inCh[i]+'</div></div>';
            ig.innerHTML=h;
          } else {
            for(var i=0;i<d.inCh.length;i++){
              var cv=document.getElementById('ichv-'+(d.inChStart+i));
              if(cv){var nv=d.inCh[i]+'';if(cv.textContent!==nv)cv.textContent=nv;}
            }
          }
        }
      }
    }
    var dot=document.getElementById('dot');
    if(dot)dot.style.background=dot.style.background==='#4caf50'?'#555':'#4caf50';
    // Sincroniza status do roteamento interno
    var rs=document.getElementById('route-status');
    if(rs){rs.textContent=d.routeEnable?'Ativo \u2714':'Desativado';rs.className=d.routeEnable?'val ok':'val err';}
    var ren=document.getElementById('route-en');
    if(ren&&!ren._dirty)ren.checked=!!d.routeEnable;
    // Sincroniza sliders do painel com os valores recebidos (quando não está arrastando)
    // Permite que os faders físicos do DMX se reflitam visualmente no painel.
    if(!_dragging){
      for(var i=0;i<d.ch.length;i++){
        var ch=d.chStart+i;
        var fi=ch-_liveStart+1; // índice 1-based no painel de faders
        if(fi>=1&&fi<=_liveCount){
          _liveVals[fi-1]=d.ch[i];
          var sld=document.getElementById('fsl-'+fi);
          var fvl=document.getElementById('fval-'+fi);
          if(sld)sld.value=d.ch[i];
          if(fvl)fvl.textContent=d.ch[i];
        }
      }
    }
    _fetching=false;
    T=setTimeout(upd,20);  // poll a cada 20ms — resposta visual mais rápida
  }).catch(function(){_fetching=false;T=setTimeout(upd,50);});
}
function _t(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}
var _liveTimer=null,_liveVals=[],_liveStart=1,_liveCount=16,_chStart=0,_chCount=0,_inChStart=0,_inChCount=0;
var _fetching=false;  // controle do loop de poll auto-scheduling
var _dragging=0;      // contador de pointers ativos nos sliders (>0 = usuário está arrastando)
function refreshChFromLive(){
  if(!_liveTimer)return;
  for(var i=0;i<_liveCount;i++){
    var cv=document.getElementById('chv-'+(_liveStart+i));
    if(cv)cv.textContent=_liveVals[i];
  }
}
function buildFaders(n){
  _liveCount=Math.max(1,Math.min(32,n));
  _liveVals=new Array(_liveCount).fill(0);
  var strip=document.getElementById('fader-strip');
  if(!strip)return;
  var h='<div class="fader-col">'
    +'<div class="fader-val" style="color:#eee;background:#1a3a5a">M</div>'
    +'<input type="range" orient="vertical" min="0" max="255" value="0" id="master-sl" class="fader-sl" oninput="masterSlide(this.value)">'
    +'<div class="fader-ch" style="color:#7ec8e3;font-weight:bold">ALL</div></div>'
    +'<div class="fader-div"></div>';
  for(var i=1;i<=_liveCount;i++){
    h+='<div class="fader-col">'
      +'<div class="fader-val" id="fval-'+i+'">0</div>'
      +'<input type="range" orient="vertical" min="0" max="255" value="0" class="fader-sl" id="fsl-'+i+'" oninput="chSlide('+i+',this.value)" onpointerdown="++_dragging" onpointerup="setTimeout(function(){if(_dragging>0)--_dragging;},100)" ontouchstart="++_dragging" ontouchend="setTimeout(function(){if(_dragging>0)--_dragging;},100)">'
      +'<div class="fader-ch" id="flbl-'+i+'">CH'+(_liveStart+i-1)+'</div></div>';
  }
  strip.innerHTML=h;
  _t('master-val','0');
  _t('lbl-last',_liveStart+_liveCount-1);
}
function updateChLabels(){
  var max=Math.max(1,513-_liveCount);
  var s=Math.max(1,Math.min(max,parseInt((document.getElementById('fader-start')||{value:'1'}).value)||1));
  _liveStart=s;
  _t('lbl-first',s);_t('lbl-last',s+_liveCount-1);
  for(var i=1;i<=_liveCount;i++){var el=document.getElementById('flbl-'+i);if(el)el.textContent='CH'+(s+i-1);}
}
function updateChCount(){
  var el=document.getElementById('fader-count');
  var n=Math.max(1,Math.min(32,parseInt((el||{value:'16'}).value)||16));
  if(el)el.value=n;
  buildFaders(n);
  updateChLabels();
}
function masterSlide(v){
  _t('master-val',v);v=parseInt(v);
  for(var i=1;i<=_liveCount;i++){_liveVals[i-1]=v;
    var s=document.getElementById('fsl-'+i),d=document.getElementById('fval-'+i);
    if(s)s.value=v;if(d)d.textContent=v;}
  schedulePush();
}
var _pushTimer=null,_lastPush=0;
function schedulePush(){
  if(!_liveTimer||_pushTimer)return;
  refreshChFromLive();  // atualiza o monitor imediatamente, sem esperar o próximo poll
  var delay=_lastPush?Math.max(0,30-(Date.now()-_lastPush)):0;
  _pushTimer=setTimeout(function(){_pushTimer=null;_lastPush=Date.now();pushLive(false);},delay);
}
function chSlide(i,v){_liveVals[i-1]=parseInt(v);_t('fval-'+i,v);schedulePush();}
function pushLive(clearAll){
  var u=document.querySelector('input[name="test-uni"]:checked');
  var url='/test-dmx-live?u='+(u?u.value:'both')+'&start='+_liveStart+'&vals='+_liveVals.slice(0,_liveCount).join(',');
  if(clearAll)url+='&clear=1';
  fetch(url).catch(function(){});
}
function startLive(){
  if(_liveTimer)return;
  document.getElementById('live-start-btn').style.display='none';
  document.getElementById('live-stop-btn').style.display='';
  document.getElementById('live-banner').style.display='block';
  _liveTimer=true;  // flag: modo fader ativo
  // Para testContinuous e zera o buffer completo antes de iniciar
  fetch('/test-dmx?val=0&cont=0')
    .then(function(){pushLive(true);})
    .catch(function(){pushLive(true);});
}
function stopLive(){
  _liveTimer=null;
  if(_pushTimer){clearTimeout(_pushTimer);_pushTimer=null;}
  document.getElementById('live-start-btn').style.display='';
  document.getElementById('live-stop-btn').style.display='none';
  document.getElementById('live-banner').style.display='none';
  _liveVals.fill(0);
  for(var i=1;i<=_liveCount;i++){var s=document.getElementById('fsl-'+i),d=document.getElementById('fval-'+i);if(s)s.value=0;if(d)d.textContent=0;}
  var ms=document.getElementById('master-sl');if(ms)ms.value=0;_t('master-val','0');
  fetch('/test-dmx-live?u=both&clear=1').catch(function(){});
}
function setRxMode(uni,rx){
  fetch('/set-rx-mode?uni='+uni+'&rx='+rx)
    .then(function(){upd();})
    .catch(function(){});
}
// Envia configuração de roteamento interno para o firmware
var _routeTimer=null;
function setRoute(){
  var en=document.getElementById('route-en');
  var is=document.getElementById('route-in-start');
  var ic=document.getElementById('route-in-count');
  var os=document.getElementById('route-out-start');
  if(!en)return;
  if(en)en._dirty=true;
  if(_routeTimer)clearTimeout(_routeTimer);
  _routeTimer=setTimeout(function(){
    var enV=en.checked?1:0;
    var isV=Math.max(1,Math.min(512,parseInt((is||{value:'1'}).value)||1));
    var icV=Math.max(1,Math.min(512,parseInt((ic||{value:'512'}).value)||512));
    var osV=Math.max(1,Math.min(512,parseInt((os||{value:'1'}).value)||1));
    fetch('/set-route?en='+enV+'&inStart='+isV+'&inCount='+icV+'&outStart='+osV)
      .then(function(r){return r.json();})
      .then(function(d){
        var rs=document.getElementById('route-status');
        if(rs){rs.textContent=d.routeEnable?'Ativo \u2714':'Desativado';rs.className=d.routeEnable?'val ok':'val err';}
        if(en)en._dirty=false;
        sched();
      }).catch(function(){});
  },300); // debounce 300ms para não spammar durante digitação
}
// schedIn: mudança nos controles do monitor de entrada apenas força um novo upd() geral
function schedIn(uni){sched();}
function updIn(uni){
  var s=parseInt((document.getElementById('in'+uni+'-start')||{value:'1'}).value)||1;
  var c=parseInt((document.getElementById('in'+uni+'-count')||{value:'32'}).value)||32;
  fetch('/dmx-input?uni='+uni+'&start='+s+'&count='+c)
    .then(function(r){return r.json();})
    .then(function(d){
      var g=document.getElementById('in'+uni+'-grid');
      if(!g)return;
      if(!d.active){
        g.innerHTML='<div style="color:#888;font-size:.85em;padding:8px 0">Aguardando sinal DMX...</div>';
        return;
      }
      if(!d.ch)return;
      var h='';
      for(var i=0;i<d.ch.length;i++)
        h+='<div class="ch-cell"><div class="lbl">Ch '+(d.chStart+i)+'</div><div class="val">'+d.ch[i]+'</div></div>';
      g.innerHTML=h;
    }).catch(function(){});
}
window.onload=function(){
  var sb=document.getElementById('sidebar');
  if(localStorage.getItem('sbCollapsed')==='1')sb.classList.add('collapsed');
  var sec=localStorage.getItem('activeSec')||'status';
  showSection(sec);
  buildFaders(16);
  upd();  // inicia o loop de auto-scheduling
};
</script>)HTML";

static const char HTML_LOGO[] PROGMEM =
  "</head><body>"
  "<div class='topbar'>"
  "<div class='logo'>"
  "<svg width='180' height='46' viewBox='0 0 212 54' fill='none' xmlns='http://www.w3.org/2000/svg'>"
  "<polygon points='27,2 49,14 49,40 27,52 5,40 5,14' fill='#06111f'/>"
  "<polygon points='27,5 47,16.5 47,37.5 27,49 7,37.5 7,16.5' fill='#0d2345'/>"
  "<polygon points='27,8 44,18.5 44,35.5 27,46 10,35.5 10,18.5' fill='none' stroke='#1b5fa0' stroke-width='1.4'/>"
  "<polyline points='13,19 40,19 13,35 40,35' fill='none' stroke='#2a85d0' stroke-width='2.2' stroke-linecap='round' stroke-linejoin='round'/>"
  "<circle cx='13' cy='19' r='2' fill='#4aaae8'/><circle cx='40' cy='19' r='2' fill='#4aaae8'/>"
  "<circle cx='13' cy='35' r='2' fill='#4aaae8'/><circle cx='40' cy='35' r='2' fill='#4aaae8'/>"
  "<polygon points='54,27 45,21.5 45,25 51,25 51,29 45,29 45,32.5' fill='#1e6ab8'/>"
  "<text x='62' y='25' font-family='Arial,sans-serif' font-weight='bold' font-size='19' fill='#ffffff' letter-spacing='0.5'>ONEBIT</text>"
  "<line x1='62' y1='30' x2='208' y2='30' stroke='#1e6ab8' stroke-width='1.5'/>"
  "<text x='62' y='46' font-family='Arial,sans-serif' font-size='11' fill='#5ab2eb' letter-spacing='2.8'>ENGENHARIA</text>"
  "</svg></div>"
  "<div class='title-wrap'><h1>Nexus DMX <span id='dot'></span></h1>";

// ── Seção: Status ─────────────────────────────────────────────────────────
static const char HTML_SEC_STATUS[] PROGMEM =
  "<div id='sec-status' class='section'>"
  // Linha 1: U1 ArtNet + Rede
  "<div class='g2'>"
  "<div class='card'><h2>&#9654; Universo 1 &mdash; ArtNet</h2>"
  "<div class='row'><span>Recep&ccedil;&atilde;o ArtNet</span><span id='rx-status' class='val err'>Aguardando...</span></div>"
  "<div class='row'><span>&Uacute;ltimo pacote</span><span id='rx-ago' class='val'>&mdash;</span></div>"
  "<div class='row'><span>Ch 1 / 2 / 3</span><span id='rx-ch123' class='val'>0 / 0 / 0</span></div>"
  "<div class='row'><span>Aceitos / Filtrados</span><span class='val'><span id='rx-ok'>0</span> / <span id='rx-filt'>0</span></span></div>"
  "<div class='row'><span>Intervalo reenvio</span><span id='rx-interval' class='val'>&mdash;</span></div>"
  "<div class='row'><span>Origem &uacute;ltimo pacote</span><span id='rx-src' class='val'>&mdash;</span></div>"
  "</div>"
  "<div class='card'><h2>&#127760; Rede</h2>"
  "<div class='row'><span>Ethernet (W5500)</span><span id='eth-status' class='val'>&mdash;</span></div>"
  "<div class='row'><span>WiFi</span><span id='wifi-status' class='val'>&mdash;</span></div>"
  "</div>"
  "</div>"
  // Linha 2: U2 TX (condicional)
  "<div id='sec-u2tx'><div class='card'><h2>&#9654; Universo 2 &mdash; ArtNet Sa&iacute;da</h2>"
  "<div class='g3'>"
  "<div><div class='row'><span>Recep&ccedil;&atilde;o ArtNet</span><span id='rx2-status' class='val err'>Aguardando...</span></div>"
  "<div class='row'><span>&Uacute;ltimo pacote</span><span id='rx2-ago' class='val'>&mdash;</span></div></div>"
  "<div><div class='row'><span>Ch 1 / 2 / 3</span><span id='rx2-ch123' class='val'>0 / 0 / 0</span></div>"
  "<div class='row'><span>Pacotes aceitos</span><span id='rx2-ok' class='val'>0</span></div></div>"
  "</div></div></div>"
  // Linha 3: Monitor Entrada DMX U1 (sempre visível)
  "<div id='sec-dmxin1'><div class='card'><h2>&#9664; Monitor Entrada DMX &mdash; U1 (RS-485)</h2>"
  "<div id='in1-off'>"
  "<div class='banner-info'>U1 em modo <b>Sa&iacute;da (TX)</b>. Para monitorar o sinal recebido da mesa DMX512, ative o modo Entrada:</div>"
  "<button class='btn' style='background:#2196f3;margin-top:8px' onclick='setRxMode(1,1)'>&#9664; Ativar U1 como Entrada (DMX512 &#8594; ArtNet)</button>"
  "</div>"
  "<div id='in1-on' style='display:none'>"
  "<div class='g3' style='margin-bottom:10px;align-items:end'>"
  "<div><div class='row'><span>Sinal DMX</span><span id='dmxin1-status' class='val err'>Aguardando...</span></div>"
  "<div class='row'><span>&Uacute;ltimo frame</span><span id='dmxin1-ago' class='val'>&mdash;</span></div>"
  "<div class='row'><span>Frames recebidos</span><span id='dmxin1-ok' class='val'>0</span></div></div>"
  "<div><div class='lbl'>Canal inicial</div>"
  "<input type='number' id='in1-start' min='1' max='512' value='1' oninput='schedIn(1)' style='margin-bottom:8px'>"
  "<div class='lbl'>Quantidade (1&ndash;128)</div>"
  "<input type='number' id='in1-count' min='1' max='128' value='32' oninput='schedIn(1)'></div>"
  "<button class='btn' style='background:#555;margin-top:0' onclick='setRxMode(1,0)'>&#9654; Voltar Sa&iacute;da (TX)</button>"
  "</div>"
  "<div id='in1-grid' style='margin-top:8px;display:grid;grid-template-columns:repeat(auto-fill,minmax(58px,1fr));gap:6px'></div>"
  "</div></div></div>"
  "</div>";

// ── Seção: Canais DMX ─────────────────────────────────────────────────────
static const char HTML_SEC_CHANNELS[] PROGMEM =
  "<div id='sec-channels' class='section'><div class='card'>"
  "<h2 id='card-ch-title'>Canais DMX U1 (TX) (1&ndash;16)</h2>"
  "<div style='display:flex;gap:20px;flex-wrap:wrap;margin-bottom:12px'>"
  "<label style='font-size:.88em;cursor:pointer'><input type='radio' name='uni-sel' value='1' checked onchange='setUni(1)'> <span id='lbl-uni1'>U1 (&raquo; Sa&iacute;da)</span></label>"
  "<label style='font-size:.88em;cursor:pointer'><input type='radio' name='uni-sel' value='2' onchange='setUni(2)'> <span id='lbl-uni2'>U2 (&raquo; Sa&iacute;da)</span></label>"
  "</div>"
  "<div class='g2' style='margin-bottom:12px'>"
  "<div><div class='lbl'>Canal inicial (1&ndash;512)</div><input type='number' id='ch-start' min='1' max='512' value='1' oninput='sched()'></div>"
  "<div><div class='lbl'>Quantidade (1&ndash;128)</div><input type='number' id='ch-count' min='1' max='128' value='32' oninput='sched()'></div>"
  "</div>"
  "<div id='ch-rx-banner' class='banner-info' style='display:none'></div>"
  "<div id='ch-grid'></div>"
  "</div></div>";

// ── Seção: Teste DMX ──────────────────────────────────────────────────────
static const char HTML_SEC_TEST[] PROGMEM =
  "<style>"
  ".fader-wrap{display:flex;gap:0;overflow-x:auto;padding:12px 10px 10px;border:1px solid #1e2a4a;border-radius:6px;background:#080e1c;margin-bottom:16px;align-items:flex-end;min-height:210px}"
  ".fader-col{display:flex;flex-direction:column;align-items:center;min-width:44px;flex-shrink:0}"
  ".fader-val{font-size:.75em;font-weight:bold;color:#7ec8e3;margin-bottom:6px;min-width:32px;text-align:center;background:#0f2040;border-radius:3px;padding:1px 2px}"
  ".fader-sl{-webkit-appearance:slider-vertical;appearance:slider-vertical;writing-mode:vertical-lr;direction:rtl;width:26px;height:150px;cursor:ns-resize;outline:none;background:transparent;padding:0;border:none}"
  ".fader-sl::-webkit-slider-runnable-track{background:#1a3a5a;border-radius:4px}"
  ".fader-sl::-webkit-slider-thumb{-webkit-appearance:none;width:30px;height:14px;background:#4aaae8;border-radius:4px;cursor:grab}"
  ".fader-sl:active::-webkit-slider-thumb{background:#7ec8e3}"
  ".fader-ch{font-size:.70em;color:#aaa;margin-top:6px;text-align:center;white-space:nowrap}"
  ".fader-div{width:1px;background:#1e2a4a;align-self:stretch;margin:0 6px;flex-shrink:0}"
  "</style>"
  "<div id='sec-test' class='section'><div class='card'>"
  "<h2>Teste DMX (sem ArtNet)</h2>"
  // Linha de controles
  "<div class='g3' style='margin-bottom:8px;align-items:end'>"
  "<div><div class='lbl'>Canal inicial (1&ndash;512)</div>"
  "<input type='number' id='fader-start' min='1' max='512' value='1' oninput='updateChLabels()' style='width:90px'></div>"
  "<div><div class='lbl'>N&ordm; de canais (1&ndash;32)</div>"
  "<input type='number' id='fader-count' min='1' max='32' value='16' oninput='updateChCount()' style='width:80px'></div>"
  "<div style='display:flex;gap:16px;align-items:center;flex-wrap:wrap'>"
  "<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='1'> U1</label>"
  "<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='2'> U2</label>"
  "<label style='cursor:pointer;font-size:.9em'><input type='radio' name='test-uni' value='both' checked> Ambos</label>"
  "</div></div>"
  "<div style='display:flex;gap:8px;margin-bottom:12px'>"
  "<button id='live-start-btn' class='btn' style='background:#2196f3' onclick='startLive()'>&#9654; Iniciar envio cont&iacute;nuo</button>"
  "<button id='live-stop-btn' class='btn' style='background:#f44336;display:none' onclick='stopLive()'>&#9646;&#9646; Parar e zerar</button>"
  "</div>"
  "<div id='live-banner' class='banner-info' style='display:none;margin-bottom:10px'>&#9679; Enviando cont&iacute;nuo ~10fps &mdash; CH <span id='lbl-first'>1</span>&ndash;<span id='lbl-last'>16</span> &nbsp;|&nbsp; Master: <span id='master-val'>0</span></div>"
  "<div class='fader-wrap' id='fader-strip'></div>"
  // Botões preset
  "<div class='sec-title' style='border-top:1px solid #1e2a4a;padding-top:14px;margin-top:4px'>Quadros pr&eacute;-definidos</div>"
  "<div class='g2' style='margin-bottom:10px'>"
  "<a href='/test-dmx?val=255'><button class='btn' style='background:#4caf50'>&#9646;&#9646; 1 Frame FULL (255)</button></a>"
  "<a href='/test-dmx?val=0'><button class='btn' style='background:#555'>&#9646;&#9646; 1 Frame ZERO (0)</button></a>"
  "</div>"
  "<div id='cont-start' class='g2'>"
  "<a href='/test-dmx?val=255&amp;cont=1'><button class='btn' style='background:#2196f3'>&#9654; Cont&iacute;nuo FULL (255)</button></a>"
  "<a href='/test-dmx?val=128&amp;cont=1'><button class='btn' style='background:#ff9800'>&#9654; Cont&iacute;nuo HALF (128)</button></a>"
  "</div>"
  "<div id='cont-status' style='display:none;margin-top:10px'>"
  "<div class='banner-ok'>&#9654; Enviando cont&iacute;nuo val=<span>0</span> (~10fps)</div>"
  "<a href='/test-dmx?val=0&amp;cont=0'><button class='btn' style='background:#f44336'>&#9646;&#9646; PARAR cont&iacute;nuo</button></a>"
  "</div></div></div>";
// (HTML_CARD_CHANNELS replaced by HTML_SEC_CHANNELS above)

// (HTML_CARD_TEST replaced by HTML_SEC_TEST above)

// ── Seção: WiFi ───────────────────────────────────────────────────────────
static const char HTML_FOOT[] PROGMEM =
  "<div id='sec-wifi' class='section'><div class='card'>"
  "<h2>&#8635; Ger&ecirc;nciamento WiFi</h2>"
  "<div class='g2'>"
  "<a href='/reconnect-wifi' onclick=\"return confirm('Reconectar ao WiFi salvo?')\"><button class='btn' style='background:#2d6a4f'>&#8635; Reconectar WiFi salvo</button></a>"
  "<a href='/reset-wifi' onclick=\"return confirm('Apagar credenciais WiFi e reiniciar?')\"><button class='btn' style='background:#7a1c2e'>&#9888; Apagar WiFi e abrir portal</button></a>"
  "</div></div></div>"
  "</div></div></body></html>";

// Sidebar SVG icons (inline, kept tiny)
#define ICO_STATUS  "<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'><circle cx='12' cy='12' r='10'/><polyline points='12 6 12 12 16 14'/></svg>"
#define ICO_CHANNELS "<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'><rect x='3' y='3' width='18' height='18' rx='2'/><line x1='3' y1='9' x2='21' y2='9'/><line x1='3' y1='15' x2='21' y2='15'/><line x1='9' y1='9' x2='9' y2='21'/></svg>"
#define ICO_ARTNET  "<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'><polygon points='12 2 22 8.5 22 15.5 12 22 2 15.5 2 8.5'/><line x1='12' y1='2' x2='12' y2='22'/><line x1='2' y1='8.5' x2='22' y2='8.5'/><line x1='2' y1='15.5' x2='22' y2='15.5'/></svg>"
#define ICO_TEST    "<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'><path d='M9 3H5a2 2 0 0 0-2 2v4m6-6h10a2 2 0 0 1 2 2v4M9 3v18m0 0h10a2 2 0 0 0 2-2V9M9 21H5a2 2 0 0 1-2-2V9m0 0h18'/></svg>"
#define ICO_WIFI    "<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'><path d='M5 12.55a11 11 0 0 1 14.08 0'/><path d='M1.42 9a16 16 0 0 1 21.16 0'/><path d='M8.53 16.11a6 6 0 0 1 6.95 0'/><line x1='12' y1='20' x2='12.01' y2='20'/></svg>"

void sendPageStreaming(Print& c) {
  char buf[160];

  // HEAD + SCRIPT
  c.print(FPSTR(HTML_HEAD)); yield();
  c.print(FPSTR(HTML_SCRIPT)); yield();

  // Logo + topbar (estático)
  c.print(FPSTR(HTML_LOGO));

  // Linha de status dinâmica no topbar
  String ip      = getNodeIP().toString();
  String mode    = (WiFi.status() == WL_CONNECTED) ? F("STA") : F("AP");
  String ethIpStr = ethLinked ? Ethernet.localIP().toString() : String("-");
  c.print(F("<div class='sub'>WiFi: <b>")); c.print(ip);
  c.print(F("</b> (")); c.print(mode);
  c.print(F(") &nbsp;&middot;&nbsp; ETH: <b>")); c.print(ethIpStr);
  snprintf(buf, sizeof(buf),
    "</b> &nbsp;&middot;&nbsp; Net:<b>%d</b> Sub:<b>%d</b> U1:<b>%d</b> U2:<b>%d</b> Porta:<b>%d</b></div>",
    artnetNet, artnetSubnet, artnetUniverse, artnetUniverse2, artnetPort);
  c.print(buf);
  c.print(F("<a class='mdns-link' href='http://onebitdmx.local:8080'>&#127760; onebitdmx.local:8080</a>"));
  c.print(F("</div></div>"));  // fecha title-wrap e topbar

  // ── Layout: sidebar + content ──────────────────────────────────────────
  c.print(F("<div class='main-wrap'>"));

  // Sidebar
  c.print(F("<nav class='sidebar' id='sidebar'>"));
  c.print(F("<div class='sb-toggle' onclick='toggleSidebar()'>"
            "<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'>"
            "<line x1='3' y1='12' x2='21' y2='12'/><line x1='3' y1='6' x2='21' y2='6'/><line x1='3' y1='18' x2='21' y2='18'/></svg>"
            "<span class='sb-label'>Recolher</span></div>"));
  c.print(F("<ul class='sb-nav'>"
    "<li><a href='#' data-sec='status' onclick=\"showSection('status');return false;\">" ICO_STATUS "<span>Status</span></a></li>"
    "<li><a href='#' data-sec='channels' onclick=\"showSection('channels');return false;\">" ICO_CHANNELS "<span>Canais DMX</span></a></li>"
    "<li><a href='#' data-sec='artnet' onclick=\"showSection('artnet');return false;\">" ICO_ARTNET "<span>ArtNet</span></a></li>"
    "<li><a href='#' data-sec='route' onclick=\"showSection('route');return false;\"><svg xmlns='http://www.w3.org/2000/svg' width='16' height='16' fill='none' stroke='currentColor' stroke-width='2' viewBox='0 0 24 24'><polyline points='17 1 21 5 17 9'/><path d='M3 11V9a4 4 0 0 1 4-4h14'/><polyline points='7 23 3 19 7 15'/><path d='M21 13v2a4 4 0 0 1-4 4H3'/></svg><span>Roteamento</span></a></li>"
    "<li><a href='#' data-sec='test' onclick=\"showSection('test');return false;\">" ICO_TEST "<span>Teste DMX</span></a></li>"
    "<li><a href='#' data-sec='wifi' onclick=\"showSection('wifi');return false;\">" ICO_WIFI "<span>WiFi</span></a></li>"
    "</ul></nav>"));

  // Content area
  c.print(F("<div class='content'>"));
  yield();

  // ── Seção Status ───────────────────────────────────────────────────────
  c.print(FPSTR(HTML_SEC_STATUS));
  yield();

  // ── Seção Canais ───────────────────────────────────────────────────────
  c.print(FPSTR(HTML_SEC_CHANNELS));
  yield();

  // ── Seção Roteamento (dinâmica) ──────────────────────────────────────────
  {
    char rbuf[900];
    snprintf(rbuf, sizeof(rbuf),
      "<div id='sec-route' class='section'>"
      "<div class='card'><h2>&#8644; Roteamento DMX Interno</h2>"
      "<div class='banner-info' style='margin-bottom:14px'>Copia canais da <b>Entrada F&iacute;sica (U1 RX)</b> diretamente para a <b>Sa&iacute;da (U2 TX)</b> sem depender do QLC+. Lat&ecirc;ncia m&iacute;nima (&lt;1ms).</div>"
      "<div class='row' style='margin-bottom:12px'><span style='font-size:.95em'>Roteamento ativo</span>"
      "<label class='toggle-sw'><input type='checkbox' id='route-en' onchange='setRoute()' %s><span class='toggle-sl'></span></label></div>"
      "<div class='g3' style='margin-bottom:14px'>"
      "<div><div class='lbl'>Canal entrada inicial (1&ndash;512)</div><input type='number' id='route-in-start' min='1' max='512' value='%d' oninput='setRoute()'></div>"
      "<div><div class='lbl'>Qtd. de canais (1&ndash;512)</div><input type='number' id='route-in-count' min='1' max='512' value='%d' oninput='setRoute()'></div>"
      "<div><div class='lbl'>Canal sa&iacute;da inicial (1&ndash;512)</div><input type='number' id='route-out-start' min='1' max='512' value='%d' oninput='setRoute()'></div>"
      "</div>"
      "<div class='row'><span>Status</span><span id='route-status' class='val'>%s</span></div>"
      "<div style='font-size:.8em;color:#888;margin-top:10px'>Exemplo: Entrada 1&ndash;32 &rarr; Sa&iacute;da 1&ndash;32 (espelho direto).<br>Ou: Entrada 1&ndash;16 &rarr; Sa&iacute;da 17&ndash;32 (mapeamento com offset).</div>"
      "</div></div></div>", // <-- fecha todos os divs do card de roteamento
      routeEnable ? "checked" : "",
      (int)routeInStart, (int)routeInCount, (int)routeOutStart,
      routeEnable ? "Ativo" : "Desativado");
    c.print(rbuf);
  }
  yield();

  // ── Seção ArtNet (dinâmica) ────────────────────────────────────────────
  c.print(F("<div id='sec-artnet' class='section'><div class='card'><h2>Endere&ccedil;o ArtNet</h2>"
            "<form action='/artnet' method='POST'>"));

  // Linha de inputs: porta, net, subnet, U1, U2
  c.print(F("<div class='g3' style='margin-bottom:14px'>"));
  snprintf(buf, sizeof(buf), "<div><div class='lbl'>Porta UDP</div><input type='number' name='port' min='1' max='65535' value='%d'></div>", artnetPort);
  c.print(buf);
  snprintf(buf, sizeof(buf), "<div><div class='lbl'>Net (0-127)</div><input type='number' name='net' min='0' max='127' value='%d'></div>", artnetNet);
  c.print(buf);
  snprintf(buf, sizeof(buf), "<div><div class='lbl'>Sub-Net (0-15)</div><input type='number' name='subnet' min='0' max='15' value='%d'></div>", artnetSubnet);
  c.print(buf);
  snprintf(buf, sizeof(buf), "<div><div class='lbl'>Universo 1 (0-15)</div><input type='number' name='universe' min='0' max='15' value='%d'></div>", artnetUniverse);
  c.print(buf);
  snprintf(buf, sizeof(buf), "<div><div class='lbl'>Universo 2 (0-15)</div><input type='number' name='universe2' min='0' max='15' value='%d'></div>", artnetUniverse2);
  c.print(buf);
  c.print(F("</div>"));

  // Banner modo atual
  if (u1RxMode)
    c.print(F("<div id='artnet-u1-banner' class='banner-info' style='margin-bottom:12px'>&#9664; <b>U1 Entrada ativa</b> &mdash; GPIO3 lendo DMX &rarr; ArtNet. Espelho &rarr; U2 TX.</div>"));
  else
    c.print(F("<div id='artnet-u1-banner' class='banner-ok' style='margin-bottom:12px'>&#9654; <b>U1 Sa&iacute;da</b> &mdash; U1 (GPIO1) e U2 (GPIO2) transmitindo DMX.</div>"));

  // Select modo U1 (U2 é fixo como saída — hardware)
  c.print(F("<div class='lbl' style='margin-bottom:8px'>Modo de opera&ccedil;&atilde;o U1</div>"
            "<div class='g2' style='margin-bottom:14px'>"));
  c.print(F("<div><div class='lbl'>Universo 1 &nbsp;<small>(TX GPIO1 | DE GPIO16)</small></div><select name='u1rx' id='artnet-u1rx-sel' onchange='this._dirty=true'>"));
  c.print(u1RxMode
    ? F("<option value='0'>&#9654; Sa&iacute;da (ArtNet&rarr;DMX)</option><option value='1' selected>&#9664; Entrada (DMX&rarr;ArtNet)</option>")
    : F("<option value='0' selected>&#9654; Sa&iacute;da (ArtNet&rarr;DMX)</option><option value='1'>&#9664; Entrada (DMX&rarr;ArtNet)</option>"));
  c.print(F("</select></div>"
            "<div><div class='lbl'>Universo 2 &nbsp;<small>(TX GPIO2 | DE fixo 3V3)</small></div>"
            "<div class='banner-info' style='margin-top:4px'>&#9654; <b>Sempre Sa&iacute;da (TX)</b> &mdash; hardware fixo.</div></div></div>"));
  c.print(F("<button class='btn' type='submit'>Aplicar configura&ccedil;&atilde;o</button></form></div></div></div>")); // <-- fecha todos os divs do card de artnet
  yield();

  // ── Seção Teste ────────────────────────────────────────────────────────
  c.print(FPSTR(HTML_SEC_TEST));
  yield();

  // ── Seção WiFi (footer) ────────────────────────────────────────────────
  c.print(FPSTR(HTML_FOOT));
}

void handleRoot() {
  // Pega o cliente raw ANTES de qualquer header para escrever HTTP manualmente.
  // Não usa chunked encoding — browser lê até Connection:close (HTTP/1.1 válido).
  WiFiClient cl = server.client();
  cl.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\nConnection: close\r\n\r\n"));
  sendPageStreaming(cl);
  cl.flush();
  cl.stop();
}



// Monta o JSON de status (usado por WiFi e Ethernet)
void buildDataJson(char* buf, int bufSize, int chStart, int chCount, int uni = 1, int inStart = 1, int inCount = 32) {
  if (chStart < 1) chStart = 1;
  if (chCount < 1) chCount = 16;
  if (chStart + chCount - 1 > 512) chCount = 512 - chStart + 1;

  uint32_t dmxIntervalMs = 25;  // intervalo fixo de reenvio DMX (ms)
  IPAddress ethIpAddr = ethLinked ? Ethernet.localIP() : IPAddress(0, 0, 0, 0);
  char ethIpBuf[16];
  snprintf(ethIpBuf, sizeof(ethIpBuf), "%d.%d.%d.%d",
           (int)ethIpAddr[0], (int)ethIpAddr[1], (int)ethIpAddr[2], (int)ethIpAddr[3]);
  IPAddress wifiIpAddr = getNodeIP();
  char wifiIpBuf[16];
  snprintf(wifiIpBuf, sizeof(wifiIpBuf), "%d.%d.%d.%d",
           (int)wifiIpAddr[0], (int)wifiIpAddr[1], (int)wifiIpAddr[2], (int)wifiIpAddr[3]);
  String wv6s = getWiFiIPv6Str();
  const char* wifiIpv6 = wv6s.c_str();

  int n = snprintf(buf, bufSize,
    "{\"running\":%d,\"cont\":%d,\"val\":%d,\"ago\":%lu,"
    "\"pktOk\":%lu,\"pktFilt\":%lu,"
    "\"net\":%d,\"sub\":%d,\"uni\":%d,\"port\":%d,"
    "\"interval\":%lu,"
    "\"s1\":%d,\"s2\":%d,\"s3\":%d,"
    "\"running2\":%d,\"ago2\":%lu,"
    "\"pktOk2\":%lu,"
    "\"uni2\":%d,"
    "\"s1_2\":%d,\"s2_2\":%d,\"s3_2\":%d,"
    "\"u1rx\":%d,"
    "\"routeEnable\":%d,\"routeInStart\":%d,\"routeInCount\":%d,\"routeOutStart\":%d,"
    "\"dmxInRunning1\":%d,\"dmxInAgo1\":%lu,\"dmxInPktOk1\":%lu,"
    "\"dmxIn1_1\":%d,\"dmxIn2_1\":%d,\"dmxIn3_1\":%d,"
    "\"ethHw\":%d,\"ethLink\":%d,"
    "\"ethLinked\":%d,\"ethIp\":\"%s\",\"wifiIp\":\"%s\",\"wifiIpv6\":\"%s\",\"lastSrc\":\"%s\","
    "\"chStart\":%d,\"ch\":[",
    dmxRunning ? 1 : 0,
    testContinuous ? 1 : 0,
    testValue,
    (millis() - lastPacketMs) / 1000UL,
    (unsigned long)pktOk,
    (unsigned long)pktFilt,
    artnetNet, artnetSubnet, artnetUniverse, artnetPort,
    (unsigned long)dmxIntervalMs,
    dmxData[1], dmxData[2], dmxData[3],
    dmxRunning2 ? 1 : 0,
    (millis() - lastPacketMs2) / 1000UL,
    (unsigned long)pktOk2,
    artnetUniverse2,
    dmxData2[1], dmxData2[2], dmxData2[3],
    u1RxMode ? 1 : 0,
    routeEnable ? 1 : 0, (int)routeInStart, (int)routeInCount, (int)routeOutStart,
    dmxInRunning1 ? 1 : 0,
    (millis() - lastDmxInMs1) / 1000UL,
    (unsigned long)dmxInPktOk1,
    dmxIn1[1], dmxIn1[2], dmxIn1[3],
    (Ethernet.hardwareStatus() != EthernetNoHardware) ? 1 : 0,
    (Ethernet.linkStatus() == LinkON) ? 1 : 0,
    ethLinked ? 1 : 0, ethIpBuf, wifiIpBuf, wifiIpv6,
    dmxRunning ? (lastPktEth ? "eth" : "wifi") : "none",
    chStart);

  // Escolhe buffer conforme universo selecionado:
  //   U1 RX mode → dmxIn1, TX mode → dmxData/dmxData2  (U2 não tem RX)
  const uint8_t* chBuf = (uni == 2) ? dmxData2
                                     : (u1RxMode ? dmxIn1 : dmxData);
  for (int i = 0; i < chCount && n < bufSize - 6; i++) {
    if (i > 0) buf[n++] = ',';
    n += snprintf(buf + n, bufSize - n, "%d", chBuf[chStart + i]);
  }
  if (n < bufSize - 2) buf[n++] = ']';  // fecha ch[]

  // Canal de entrada embutido — sempre presente quando u1RxMode, igual ao comportamento de ch[]
  if (u1RxMode) {
    if (inStart < 1) inStart = 1;
    if (inCount < 1) inCount = 1;
    if (inStart + inCount - 1 > 512) inCount = 512 - inStart + 1;
    n += snprintf(buf + n, bufSize - n, ",\"inChStart\":%d,\"inCh\":[", inStart);
    for (int i = 0; i < inCount && n < bufSize - 6; i++) {
      if (i > 0) buf[n++] = ',';
      n += snprintf(buf + n, bufSize - n, "%d", dmxIn1[inStart + i]);
    }
    if (n < bufSize - 2) buf[n++] = ']';
  }

  if (n < bufSize - 2) { buf[n++] = '}'; buf[n] = '\0'; }
}

// Endpoint JSON leve para atualizações ao vivo via fetch()
void handleData() {
  static char buf[2048];
  int chStart = 1, chCount = 16, uni = 1, inStart = 1, inCount = 32;
  if (server.hasArg("start"))   chStart = constrain(server.arg("start").toInt(),   1, 512);
  if (server.hasArg("count"))   chCount = constrain(server.arg("count").toInt(),   1, 128);
  if (server.hasArg("uni"))     uni     = constrain(server.arg("uni").toInt(),     1,   2);
  if (server.hasArg("inStart")) inStart = constrain(server.arg("inStart").toInt(), 1, 512);
  if (server.hasArg("inCount")) inCount = constrain(server.arg("inCount").toInt(), 1, 128);
  buildDataJson(buf, sizeof(buf), chStart, chCount, uni, inStart, inCount);
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", buf);
}

// GET /dmx-input?uni=1&start=N&count=N  → canais do buffer de entrada (dmxIn1)
// U2 é sempre saída, por isso /dmx-input só serve dados do Universo 1.
void handleDmxInput() {
  int start = server.hasArg("start") ? constrain(server.arg("start").toInt(), 1, 512) : 1;
  int count = server.hasArg("count") ? constrain(server.arg("count").toInt(), 1, 128) : 32;
  if (start + count - 1 > 512) count = 512 - start + 1;
  const uint8_t* buf = dmxIn1;
  bool     active = dmxInRunning1;
  uint32_t pkt    = (uint32_t)dmxInPktOk1;
  uint32_t ago    = (millis() - lastDmxInMs1) / 1000UL;
  static char res[1600]; int n = 0;
  n += snprintf(res + n, sizeof(res) - n,
    "{\"active\":%d,\"rxMode\":%d,\"pkt\":%lu,\"ago\":%lu,\"chStart\":%d,\"ch\":[",
    active ? 1 : 0, u1RxMode ? 1 : 0, (unsigned long)pkt, active ? (unsigned long)ago : 0UL, start);
  for (int i = 0; i < count && n < (int)sizeof(res) - 8; i++) {
    if (i > 0) res[n++] = ',';
    n += snprintf(res + n, sizeof(res) - n, "%d", buf[start + i]);
  }
  if (n < (int)sizeof(res) - 3) { res[n++] = ']'; res[n++] = '}'; res[n] = '\0'; }
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", res);
}

// GET /set-route?en=0|1&inStart=1&inCount=512&outStart=1  → configura roteamento interno
void handleSetRoute() {
  String enS      = server.arg("en");
  String inStartS = server.arg("inStart");
  String inCountS = server.arg("inCount");
  String outStartS= server.arg("outStart");
  if (enS.length())      routeEnable   = (enS == "1");
  if (inStartS.length()) routeInStart  = (uint16_t)constrain(inStartS.toInt(), 1, 512);
  if (inCountS.length()) routeInCount  = (uint16_t)constrain(inCountS.toInt(), 1, 512);
  if (outStartS.length())routeOutStart = (uint16_t)constrain(outStartS.toInt(), 1, 512);
  saveConfig();
  server.send(200, "application/json",
    String("{\"ok\":1,\"routeEnable\":") + (routeEnable?"1":"0") +
    ",\"routeInStart\":" + routeInStart +
    ",\"routeInCount\":" + routeInCount +
    ",\"routeOutStart\":" + routeOutStart + "}");
}

// GET /set-rx-mode?uni=1&rx=0|1  → altera u1RxMode (U2 é sempre TX)
void handleSetRxMode() {
  int  uni = server.hasArg("uni") ? constrain(server.arg("uni").toInt(), 1, 2) : 0;
  bool rx  = server.hasArg("rx")  && server.arg("rx") == "1";
  if (uni == 1) { u1RxMode = rx; }
  // uni == 2 ignorado: U2 é sempre saída (hardware fixo)
  saveConfig();  // persiste na EEPROM
  applyModes();
  char out[48];
  snprintf(out, sizeof(out), "{\"ok\":true,\"u1rx\":%d}",
           u1RxMode ? 1 : 0);
  server.send(200, "application/json", out);
}

// Envia um frame DMX de teste com valor fixo em todos os canais
// GET /test-dmx-live?u=1|2|both&start=N&vals=v1,...,vN  → bulk CSV (JSON)
// GET /test-dmx-live?u=...&clear=1                       → zera tudo
void handleTestDmxLive() {
  testContinuous = false;  // sempre interrompe modo contínuo ao usar faders
  String u = server.hasArg("u") ? server.arg("u") : "both";
  bool doU1 = (u == "1" || u == "both");
  bool doU2 = (u == "2" || u == "both");
  bool clearAll = server.hasArg("clear") && server.arg("clear") == "1";
  // Força TX na universe alvo se ela estiver em modo RX (evita dmxSend retornar early)
  bool modeChanged = false;
  if (doU1 && u1RxMode) { u1RxMode = false; modeChanged = true; }
  // doU2: U2 é sempre TX, não precisa forcão de modo
  if (modeChanged) applyModes();
  if (clearAll) {
    if (doU1) { memset(dmxData  + 1, 0, DMX_CHANNELS); dmxData[0]  = 0; }
    if (doU2) { memset(dmxData2 + 1, 0, DMX_CHANNELS); dmxData2[0] = 0; }
  }
  if (server.hasArg("vals")) {
    int start = server.hasArg("start") ? constrain(server.arg("start").toInt(), 1, 512) : 1;
    String csv = server.arg("vals");
    int ch = start, pos = 0;
    while (pos <= (int)csv.length() && ch <= DMX_CHANNELS) {
      int comma = csv.indexOf(',', pos);
      if (comma < 0) comma = csv.length();
      if (comma > pos) {
        uint8_t v = (uint8_t)constrain(csv.substring(pos, comma).toInt(), 0, 255);
        if (doU1) dmxData[ch]  = v;
        if (doU2) dmxData2[ch] = v;
        ch++;
      }
      pos = comma + 1;
    }
  } else if (server.hasArg("val")) {
    uint8_t val = (uint8_t)constrain(server.arg("val").toInt(), 0, 255);
    int ch = server.hasArg("ch") ? server.arg("ch").toInt() : 0;
    if (ch == 0) {
      if (doU1) memset(dmxData  + 1, val, DMX_CHANNELS);
      if (doU2) memset(dmxData2 + 1, val, DMX_CHANNELS);
    } else if (ch >= 1 && ch <= DMX_CHANNELS) {
      if (doU1) dmxData[ch]  = val;
      if (doU2) dmxData2[ch] = val;
    } else { server.send(400, "application/json", "{\"err\":\"bad ch\"}"); return; }
  } else if (!clearAll) {
    server.send(400, "application/json", "{\"err\":\"no vals\"}"); return;
  }
  // dmxSend()/dmxSend2() removidos daqui — o loop() envia a cada 25ms (40fps).
  // Chamar dmxSend() no handler bloqueava ~34ms por requisição, causando fila TCP e latência de segundos.
  lastPacketMs = millis();
  dmxRunning   = true;
  server.send(200, "application/json", "{\"ok\":true}");
}

// GET /test-dmx?val=255        → envia 1 frame
// GET /test-dmx?val=255&cont=1 → ativa envio contínuo a ~40fps
// GET /test-dmx?val=0&cont=0   → desativa envio contínuo
void handleTestDmx() {
  uint8_t val  = testValue;
  if (server.hasArg("val"))  val = (uint8_t)constrain(server.arg("val").toInt(), 0, 255);
  bool    cont = server.hasArg("cont") && server.arg("cont") == "1";
  bool    stop = server.hasArg("cont") && server.arg("cont") == "0";

  testValue = val;
  memset(dmxData  + 1, val, DMX_CHANNELS); dmxData[0]  = 0x00;
  memset(dmxData2 + 1, val, DMX_CHANNELS); dmxData2[0] = 0x00;

  if (stop) {
    testContinuous = false;
    memset(dmxData  + 1, 0, DMX_CHANNELS);
    memset(dmxData2 + 1, 0, DMX_CHANNELS);
    dmxSend(); dmxSend2();
  } else if (cont) {
    testContinuous = true;
  } else {
    dmxSend(); dmxSend2();  // 1 frame único
  }

  lastPacketMs = millis();
  dmxRunning   = (val > 0 || testContinuous);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleArtnetCfg() {
  if (server.hasArg("port")) {
    uint16_t p = (uint16_t)constrain(server.arg("port").toInt(), 1, 65535);
    if (p != artnetPort) {
      artnetPort = p;
      udp.stop();  udp.begin(artnetPort);
      if (ethLinked) { udp_eth.stop(); udp_eth.begin(artnetPort); }
    }
  }
  if (server.hasArg("net"))       artnetNet       = (uint8_t)constrain(server.arg("net").toInt(),       0, 127);
  if (server.hasArg("subnet"))    artnetSubnet    = (uint8_t)constrain(server.arg("subnet").toInt(),    0,  15);
  if (server.hasArg("universe"))  artnetUniverse  = (uint8_t)constrain(server.arg("universe").toInt(),  0,  15);
  if (server.hasArg("universe2")) artnetUniverse2 = (uint8_t)constrain(server.arg("universe2").toInt(), 0,  15);
  if (server.hasArg("u1rx")) {
    u1RxMode = server.arg("u1rx") == "1";
    // u2rx ignorado: U2 é sempre saída (hardware fixo)
    applyModes();
  }
  saveConfig();  // persiste na EEPROM
  sendArtPollReply();
  sendArtPollReplyEth();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleReconnectWifi() {
  server.send(200, "text/html",
    "<html><body style='font-family:Arial;background:#1a1a2e;color:#eee;text-align:center;padding:40px'>"
    "<h2>Reconectando...</h2>"
    "<p>Tentando conectar ao WiFi salvo. Aguarde 30 s e volte ao painel.</p>"
    "<script>setTimeout(function(){window.location='/';},35000);</script>"
    "</body></html>");
  delay(100);
  WiFi.disconnect(false);  // desconecta mas mantém credenciais
  delay(500);
  WiFi.begin();            // reconecta usando credenciais salvas
}

void handleResetWifi() {
  server.send(200, "text/html",
    "<html><body style='font-family:Arial;background:#1a1a2e;color:#eee;text-align:center;padding:40px'>"
    "<h2 style='color:#e94560'>Credenciais apagadas</h2>"
    "<p>Conecte ao AP <b>OneBit - Core - V2</b> e acesse <b>192.168.4.1</b></p>"
    "</body></html>");
  delay(500);
  wm.resetSettings();
  ESP.restart();
}

// ===========================================================================
//  Painel Web por Ethernet (servidor HTTP independente na porta 8080)
// ===========================================================================

// Extrai valor de parâmetro de uma query string (ex: "port=6454&net=0")
static String ethParam(const String& q, const char* key) {
  String kEq = String(key) + "=";
  int idx = q.indexOf(kEq);
  if (idx < 0) return "";
  int s = idx + kEq.length();
  int e = q.indexOf('&', s);
  return e < 0 ? q.substring(s) : q.substring(s, e);
}

void handleEthHttp() {
  // accept() retorna o cliente assim que a conexão TCP é estabelecida,
  // sem esperar dados — correto para servidores HTTP.
  EthernetClient client = ethServer.accept();
  if (!client) return;

  // Aguarda a primeira linha da requisição HTTP chegar (máx 20ms — LAN local responde em <2ms)
  unsigned long t = millis();
  while (!client.available() && client.connected() && millis() - t < 20) { dmxReceiveAndSend(); yield(); }
  if (!client.available()) { client.stop(); return; }

  // Lê linha de requisição: "METHOD /path?query HTTP/1.x"
  String reqLine = client.readStringUntil('\n');
  reqLine.trim();
  String method = "", path = "", query = "";
  {
    int s1 = reqLine.indexOf(' ');
    int s2 = reqLine.lastIndexOf(' ');
    if (s1 > 0 && s2 > s1) {
      method = reqLine.substring(0, s1);
      String url = reqLine.substring(s1 + 1, s2);
      int qi = url.indexOf('?');
      if (qi >= 0) { path = url.substring(0, qi); query = url.substring(qi + 1); }
      else path = url;
    }
  }
  if (path.isEmpty()) path = "/";

  // Drena cabeçalhos, captura Content-Length (30ms — suficiente para LAN local)
  int contentLen = 0;
  unsigned long hdrT = millis();
  while (client.connected() && millis() - hdrT < 30) {
    if (!client.available()) { dmxReceiveAndSend(); yield(); continue; }
    String hdr = client.readStringUntil('\n');
    hdr.trim();
    if (hdr.isEmpty()) break;
    String lhdr = hdr; lhdr.toLowerCase();
    if (lhdr.startsWith("content-length:"))
      contentLen = hdr.substring(hdr.indexOf(':') + 1).toInt();
  }

  // Lê corpo para POST (formulários de configuração)
  if (contentLen > 0 && method == "POST") {
    String body = "";
    t = millis();
    while ((int)body.length() < contentLen && millis() - t < 50) {
      if (client.available()) body += (char)client.read();
      else { dmxReceiveAndSend(); yield(); }
    }
    query = body;
  }

  // --- Roteamento ---

  if (path == "/reconnect-wifi") {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
                   "<html><body style='font-family:Arial;background:#1a1a2e;color:#eee;"
                   "text-align:center;padding:40px'><h2>Reconectando...</h2>"
                   "<p>Tentando conectar ao WiFi salvo. Aguarde 30 s e volte ao painel.</p>"
                   "<script>setTimeout(function(){window.location='/';},35000);</script>"
                   "</body></html>"));
    client.stop();
    delay(100);
    WiFi.disconnect(false);
    delay(500);
    WiFi.begin();
    return;
  }

  if (path == "/reset-wifi") {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
                   "<html><body style='font-family:Arial;background:#1a1a2e;color:#eee;"
                   "text-align:center;padding:40px'><h2 style='color:#e94560'>Credenciais "
                   "apagadas</h2><p>Conecte ao AP <b>OneBit - Core - V2</b> e acesse "
                   "<b>192.168.4.1</b></p></body></html>"));
    client.stop();
    delay(500); wm.resetSettings(); ESP.restart();
    return;
  }

  if (path == "/test-dmx") {
    String vs = ethParam(query, "val");
    uint8_t val = testValue;
    if (vs.length()) val = (uint8_t)constrain(vs.toInt(), 0, 255);
    bool cont_ = ethParam(query, "cont") == "1";
    bool stop_ = ethParam(query, "cont") == "0";
    testValue = val;
    memset(dmxData  + 1, val, DMX_CHANNELS); dmxData[0]  = 0x00;
    memset(dmxData2 + 1, val, DMX_CHANNELS); dmxData2[0] = 0x00;
    if (stop_) { testContinuous = false; memset(dmxData + 1, 0, DMX_CHANNELS); memset(dmxData2 + 1, 0, DMX_CHANNELS); dmxSend(); dmxSend2(); }
    else if (cont_) testContinuous = true;
    else { dmxSend(); dmxSend2(); }
    lastPacketMs = millis();
    dmxRunning = (val > 0 || testContinuous);
    client.print(F("HTTP/1.1 303 See Other\r\nLocation: /\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"));
    client.flush();
    client.stop(); return;
  }

  if (path == "/test-dmx-live") {
    testContinuous = false;
    String eu = ethParam(query, "u"); if (!eu.length()) eu = "both";
    bool dU1 = (eu == "1" || eu == "both");
    bool dU2 = (eu == "2" || eu == "both");
    bool eClear = ethParam(query, "clear") == "1";
    if (eClear) {
      if (dU1) { memset(dmxData  + 1, 0, DMX_CHANNELS); dmxData[0]  = 0; }
      if (dU2) { memset(dmxData2 + 1, 0, DMX_CHANNELS); dmxData2[0] = 0; }
    }
    String ecsv = ethParam(query, "vals");
    if (ecsv.length()) {
      String startStr = ethParam(query, "start");
      int estart = startStr.length() ? constrain(startStr.toInt(), 1, 512) : 1;
      int ech = estart, epos = 0;
      while (epos <= (int)ecsv.length() && ech <= DMX_CHANNELS) {
        int ec = ecsv.indexOf(',', epos); if (ec < 0) ec = ecsv.length();
        if (ec > epos) {
          uint8_t ev = (uint8_t)constrain(ecsv.substring(epos, ec).toInt(), 0, 255);
          if (dU1) dmxData[ech]  = ev;
          if (dU2) dmxData2[ech] = ev;
          ech++;
        }
        epos = ec + 1;
      }
    } else if (ethParam(query, "val").length()) {
      uint8_t val2 = (uint8_t)constrain(ethParam(query, "val").toInt(), 0, 255);
      int ch2 = ethParam(query, "ch").toInt();
      if (ch2 == 0) {
        if (dU1) memset(dmxData  + 1, val2, DMX_CHANNELS);
        if (dU2) memset(dmxData2 + 1, val2, DMX_CHANNELS);
      } else if (ch2 >= 1 && ch2 <= DMX_CHANNELS) {
        if (dU1) dmxData[ch2] = val2;
        if (dU2) dmxData2[ch2] = val2;
      }
    } else if (!eClear) {
      client.print(F("HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"));
      client.stop(); return;
    }
    // dmxSend()/dmxSend2() removidos — o loop() envia a cada 25ms
    lastPacketMs = millis(); dmxRunning = true;
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 10\r\nConnection: close\r\n\r\n{\"ok\":true}"));
    client.flush(); client.stop(); return;
  }

  if (path == "/artnet") {
    String ps = ethParam(query, "port");
    if (ps.length()) {
      uint16_t p = (uint16_t)constrain(ps.toInt(), 1, 65535);
      if (p != artnetPort) {
        artnetPort = p;
        udp.stop(); udp.begin(artnetPort);
        if (ethLinked) { udp_eth.stop(); udp_eth.begin(artnetPort); }
      }
    }
    String ns  = ethParam(query, "net");       if (ns.length())  artnetNet       = (uint8_t)constrain(ns.toInt(),  0, 127);
    String ss  = ethParam(query, "subnet");    if (ss.length())  artnetSubnet    = (uint8_t)constrain(ss.toInt(),  0,  15);
    String us  = ethParam(query, "universe");  if (us.length())  artnetUniverse  = (uint8_t)constrain(us.toInt(),  0,  15);
    String us2 = ethParam(query, "universe2"); if (us2.length()) artnetUniverse2 = (uint8_t)constrain(us2.toInt(), 0,  15);
    String u1rs = ethParam(query, "u1rx");
    if (u1rs.length()) {
      u1RxMode = u1rs == "1";
      // u2rx ignorado: U2 é sempre saída (hardware fixo)
      applyModes();
    }
    sendArtPollReply(); sendArtPollReplyEth();
    client.print(F("HTTP/1.1 303 See Other\r\nLocation: /\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"));
    client.flush();
    client.stop(); return;
  }

  if (path == "/data") {
    int chStart = 1, chCount = 16, uni = 1;
    String ss = ethParam(query, "start"); if (ss.length()) chStart = constrain(ss.toInt(), 1, 512);
    String cs = ethParam(query, "count"); if (cs.length()) chCount = constrain(cs.toInt(), 1, 128);
    String us = ethParam(query, "uni");   if (us.length()) uni     = constrain(us.toInt(), 1, 2);
    static char jsonBuf[1400];
    buildDataJson(jsonBuf, sizeof(jsonBuf), chStart, chCount, uni);
    int jLen = strlen(jsonBuf);
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-cache\r\nContent-Length: "));
    client.print(jLen);
    client.print(F("\r\nConnection: close\r\n\r\n"));
    client.write((const uint8_t*)jsonBuf, jLen);
    client.flush();
    client.stop(); return;
  }

  if (path == "/dmx-input") {
    int eUni = 2, eStart = 1, eCount = 32;
    String s_ = ethParam(query, "uni");   if (s_.length()) eUni   = constrain(s_.toInt(), 1,   2);
    String t_ = ethParam(query, "start"); if (t_.length()) eStart = constrain(t_.toInt(), 1, 512);
    String u_ = ethParam(query, "count"); if (u_.length()) eCount = constrain(u_.toInt(), 1, 128);
    if (eStart + eCount - 1 > 512) eCount = 512 - eStart + 1;
    // U2 nunca tem RX; sempre serve dmxIn1
    (void)eUni;
    const uint8_t* ib = dmxIn1;
    bool     ia  = dmxInRunning1;
    uint32_t ip_ = (uint32_t)dmxInPktOk1;
    uint32_t ia_ = (millis() - lastDmxInMs1) / 1000UL;
    bool irx = u1RxMode;
    static char inJ[1600]; int in_ = 0;
    in_ += snprintf(inJ + in_, sizeof(inJ) - in_,
      "{\"active\":%d,\"rxMode\":%d,\"pkt\":%lu,\"ago\":%lu,\"chStart\":%d,\"ch\":[",
      ia ? 1 : 0, irx ? 1 : 0, (unsigned long)ip_, ia ? (unsigned long)ia_ : 0UL, eStart);
    for (int ii = 0; ii < eCount && in_ < (int)sizeof(inJ) - 8; ii++) {
      if (ii > 0) inJ[in_++] = ',';
      in_ += snprintf(inJ + in_, sizeof(inJ) - in_, "%d", ib[eStart + ii]);
    }
    if (in_ < (int)sizeof(inJ) - 3) { inJ[in_++] = ']'; inJ[in_++] = '}'; inJ[in_] = '\0'; }
    int il = strlen(inJ);
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-cache\r\nContent-Length: "));
    client.print(il); client.print(F("\r\nConnection: close\r\n\r\n"));
    client.write((const uint8_t*)inJ, il);
    client.flush(); client.stop(); return;
  }

  if (path == "/set-route") {
    String enS       = ethParam(query, "en");
    String inStartS  = ethParam(query, "inStart");
    String inCountS  = ethParam(query, "inCount");
    String outStartS = ethParam(query, "outStart");
    if (enS.length())       routeEnable   = (enS == "1");
    if (inStartS.length())  routeInStart  = (uint16_t)constrain(inStartS.toInt(),  1, 512);
    if (inCountS.length())  routeInCount  = (uint16_t)constrain(inCountS.toInt(),  1, 512);
    if (outStartS.length()) routeOutStart = (uint16_t)constrain(outStartS.toInt(), 1, 512);
    saveConfig();
    char rB[96];
    snprintf(rB, sizeof(rB), "{\"ok\":1,\"routeEnable\":%d,\"routeInStart\":%d,\"routeInCount\":%d,\"routeOutStart\":%d}",
             routeEnable?1:0, (int)routeInStart, (int)routeInCount, (int)routeOutStart);
    int rl = strlen(rB);
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: "));
    client.print(rl); client.print(F("\r\nConnection: close\r\n\r\n"));
    client.write((const uint8_t*)rB, rl);
    client.flush(); client.stop(); return;
  }

  if (path == "/set-rx-mode") {
    int  rUni = ethParam(query, "uni").toInt();
    bool rRx  = ethParam(query, "rx") == "1";
    if (rUni == 1) { u1RxMode = rRx; }
    // rUni == 2 ignorado: U2 é sempre saída (hardware fixo)
    applyModes();
    char rB[48];
    snprintf(rB, sizeof(rB), "{\"ok\":true,\"u1rx\":%d}",
             u1RxMode ? 1 : 0);
    int rl = strlen(rB);
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: "));
    client.print(rl); client.print(F("\r\nConnection: close\r\n\r\n"));
    client.write((const uint8_t*)rB, rl);
    client.flush(); client.stop(); return;
  }

  // GET / — painel principal via streaming (sem alocar String grande)
  // Usa Connection: close sem Content-Length — o browser aceita e lê até fechar o socket.
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\nConnection: close\r\n\r\n"));
  sendPageStreaming(client);
  client.flush();
  client.stop();
}

// ===========================================================================
//  mDNS Ethernet — responder para onebitdmx.local (W5500)
// ===========================================================================

// Envia resposta mDNS tipo A: artnet2dmx.local → IP Ethernet
static void sendEthMdnsResponse() {
  // Nome codificado DNS: [9]onebitdmx[5]local[0]
  static const uint8_t kHostLabel[] = {
    9,'o','n','e','b','i','t','d','m','x',
    5,'l','o','c','a','l',0
  };
  uint8_t resp[60];
  int n = 0;
  resp[n++]=0;    resp[n++]=0;     // ID = 0
  resp[n++]=0x84; resp[n++]=0x00;  // QR=1 AA=1 Opcode=0
  resp[n++]=0;    resp[n++]=0;     // QDCOUNT = 0
  resp[n++]=0;    resp[n++]=1;     // ANCOUNT = 1
  resp[n++]=0;    resp[n++]=0;     // NSCOUNT = 0
  resp[n++]=0;    resp[n++]=0;     // ARCOUNT = 0
  memcpy(resp + n, kHostLabel, sizeof(kHostLabel)); n += sizeof(kHostLabel);
  resp[n++]=0;    resp[n++]=1;     // TYPE A (1)
  resp[n++]=0x80; resp[n++]=0x01;  // CLASS IN + cache-flush
  resp[n++]=0;    resp[n++]=0;  resp[n++]=0; resp[n++]=120; // TTL 120 s
  resp[n++]=0;    resp[n++]=4;     // RDLENGTH = 4
  IPAddress ip = Ethernet.localIP();
  resp[n++]=ip[0]; resp[n++]=ip[1]; resp[n++]=ip[2]; resp[n++]=ip[3];
  mdns_udp_eth.beginPacket(MDNS_MCAST, MDNS_PORT);
  mdns_udp_eth.write(resp, n);
  mdns_udp_eth.endPacket();
}

// Escuta o multicast mDNS e responde consultas sobre artnet2dmx.local
void handleEthMdns() {
  if (!ethLinked) return;
  int pktSize = mdns_udp_eth.parsePacket();
  if (pktSize < 12) return;
  static uint8_t pkt[300];
  int len = mdns_udp_eth.read(pkt, sizeof(pkt));
  if (len < 12 || (pkt[2] & 0x80)) return;  // ignora não-queries
  // Busca a sequência exata do hostname codificado em labels DNS
  static const uint8_t kName[] = {
    8,'o','n','e','b','i','t','d','m','x',
    5,'l','o','c','a','l',0
  };
  for (int i = 12; i <= len - (int)sizeof(kName); i++) {
    if (memcmp(pkt + i, kName, sizeof(kName)) == 0) {
      sendEthMdnsResponse();
      return;
    }
  }
}

// ===========================================================================
//  Setup
// ===========================================================================
void setup() {
  // Universo 1: DE/RE HIGH = modo transmissão
  pinMode(DMX_DE_PIN, OUTPUT);
  digitalWrite(DMX_DE_PIN, HIGH);

  // Universo 2: DE/RE fixo HIGH no hardware (ligado ao 3V3) — sem controle de software.
  // IMPORTANTE: definir HIGH antes de OUTPUT para que o pino nunca fique LOW pulsando
  // o 3V3 externo (~130 mA) e afetando o rail de alimentação.
  digitalWrite(DMX2_DE_PIN, HIGH);     // nível HIGH primeiro
  pinMode(DMX2_DE_PIN, OUTPUT);        // depois liga o driver — sai em HIGH
  loadConfig();  // restaura artnetPort/Net/Sub/Uni/u1RxMode da EEPROM (ou mantém padrões)
  applyModes();  // aplica HIGH/LOW nos pinos DE/RE conforme os modos

  // Inicializa UART0 (250 kbaud, 8N2) uma única vez.
  // dmxSend() usa o bit UCBRK para gerar BREAK sem reinicializar a UART,
  // preservando o FIFO RX (GPIO3) intacto durante a transmissão U1.
  // Buffer RX ampliado para 2 frames completos: evita overflow entre polls.
  Serial.setRxBufferSize(1030);
  Serial.begin(250000, SERIAL_8N2);
  _serialReady = true;  // libera applyModes() para tocar nos registradores UART
  applyModes();         // reinstala ISR, agora com UART pronta (u1RxMode pode ter sido carregado da EEPROM)
  Serial1.begin(250000, SERIAL_8N2);  // UART1 para DMX U2 (GPIO2) — init único, sem reinit por frame

  // Inicializa buffers DMX zerados (blackout)
  memset(dmxData,  0, sizeof(dmxData));
  memset(dmxData2, 0, sizeof(dmxData2));

  // --- WiFi ---
  WiFi.mode(WIFI_STA);
  // Nota: IPv6 requer lwIP compilado com LWIP_IPV6=1. Esta build usa LWIP_IPV6=0,
  // portanto getWiFiIPv6Str() retornará "" e o campo wifiIpv6 ficará vazio no painel.
  String savedSSID = WiFi.SSID();

  if (savedSSID.length() > 0) {
    wm.setConfigPortalTimeout(1);
    wm.setConnectTimeout(30);
    wm.autoConnect(WIFI_AP_NAME, WIFI_AP_PASS);
  }

  if (WiFi.status() != WL_CONNECTED) {
    wm.setConfigPortalBlocking(false);
    wm.setConfigPortalTimeout(0);
    wm.startConfigPortal(WIFI_AP_NAME, WIFI_AP_PASS[0] ? WIFI_AP_PASS : nullptr);
    wmPortalRunning = true;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (MDNS.begin(NODE_MDNS_NAME)) {
      MDNS.addService("http", "tcp", 8080);
    }
  }

  // --- Ethernet W5500 ---
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);
  delay(10);
  digitalWrite(ETH_RST_PIN, HIGH);
  delay(200);
  Ethernet.init(ETH_CS_PIN);
  if (Ethernet.begin(ethMac, 4000, 1000)) {
    ethLinked = true;
    udp_eth.begin(artnetPort);
    ethServer.begin();
    mdns_udp_eth.beginMulticast(MDNS_MCAST, MDNS_PORT);
  }

  // --- UDP WiFi ---
  udp.begin(artnetPort);

  // --- Web server ---
  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/data",        HTTP_GET,  handleData);
  server.on("/dmx-input",   HTTP_GET,  handleDmxInput);
  server.on("/set-route",   HTTP_GET,  handleSetRoute);
  server.on("/set-rx-mode", HTTP_GET,  handleSetRxMode);
  server.on("/artnet",     HTTP_POST, handleArtnetCfg);
  server.on("/test-dmx",      HTTP_GET,  handleTestDmx);
  server.on("/test-dmx-live", HTTP_GET,  handleTestDmxLive);
  server.on("/reconnect-wifi", HTTP_GET,  handleReconnectWifi);
  server.on("/reset-wifi",     HTTP_GET,  handleResetWifi);
  server.begin();

  // Anuncia o node em ambas as interfaces
  sendArtPollReply();
  sendArtPollReplyEth();
}

// ===========================================================================
//  Loop
// ===========================================================================
void loop() {
  unsigned long now = millis();

  // --- Recepção DMX + envio ArtNet imediato — antes de qualquer operação bloqueante ---
  dmxReceiveAndSend();

  // --- Recepção ArtNet (WiFi + Ethernet) — ANTES dos timers de saída DMX ---
  // Fundamental para o caminho de feedback QLC+: o pacote de retorno do QLC+ chega aqui,
  // atualiza dmxData2, e o timer de dmxSend2() dispara IMEDIATAMENTE logo abaixo
  // (na mesma iteração do loop), sem esperar uma volta completa.
  handleArtNetWiFi();
  handleArtNetEth();

  // --- Ethernet: sempre ativo, independente do estado WiFi ---
  if (ethLinked) {
    Ethernet.maintain();
    handleEthHttp();
    handleEthMdns();
  } else if ((unsigned long)(now - lastEthRetryMs) >= ETH_RETRY_MS) {
    // Tenta reconectar Ethernet.
    // IMPORTANTE: só chama Ethernet.begin() (que bloqueia ~3s aguardando DHCP) se
    // o cabo estiver conectado. Sem link → retorna em ~110ms sem interromper o DMX.
    lastEthRetryMs = now;
    digitalWrite(ETH_RST_PIN, LOW);
    delay(10);
    digitalWrite(ETH_RST_PIN, HIGH);
    delay(100);            // W5500 startup mínimo (100ms)
    Ethernet.init(ETH_CS_PIN);
    if (Ethernet.hardwareStatus() != EthernetNoHardware
        && Ethernet.linkStatus() == LinkON) {
      // Cabo detectado — tenta DHCP (pode bloquear até 4s, mas há link real)
      if (Ethernet.begin(ethMac, 4000, 1000)) {
        ethLinked = true;
        udp_eth.begin(artnetPort);
        ethServer.begin();
        mdns_udp_eth.beginMulticast(MDNS_MCAST, MDNS_PORT);
        sendArtPollReplyEth();
      }
    }
    // Se não há hardware ou não há link, só gastamos ~110ms (delay acima) e retornamos.
  }

  // --- Saída DMX: sempre ativa (teste contínuo ou reenvio por intervalo) ---
  if (testContinuous) {
    if (now - lastTestSend >= 25) {  // 25ms = ~40fps (igual ao refresh normal)
      lastTestSend  = now;
      lastPacketMs  = now;
      lastPacketMs2 = now;
      if (!u1RxMode) dmxSend();
      dmxSend2();   // U2 é sempre TX
      dmxReceiveAndSend(); // processa e envia frame que chegou durante os sends
    }
  } else {
    // Timer 25 ms — único ponto de envio DMX (processArtNetPacket só atualiza os dados).
    // Como handleArtNetWiFi/Eth já rodaram acima, qualquer ArtNet recebido neste ciclo
    // dispara o send IMEDIATAMENTE aqui (na mesma iteração), sem loop extra.
    if (!u1RxMode && (uint32_t)(millis() - lastDmxSend) >= 25UL) {
      dmxSend(dmxOut1Len);  // comprimento real do último pacote ArtNet recebido
      lastDmxSend = millis();
      dmxReceiveAndSend(); // processa e envia frame que chegou durante dmxSend()
      server.handleClient(); // serve requisições HTTP pendentes sem esperar o fim do loop
    }

    if ((uint32_t)(millis() - lastDmxSend2) >= 25UL) {  // U2 sempre TX
      dmxSend2(dmxOut2Len);  // comprimento real: ArtNet recebido ou roteamento
      lastDmxSend2 = millis();
      dmxReceiveAndSend(); // processa e envia frame que chegou durante dmxSend2() (~17ms)
      server.handleClient(); // serve requisições HTTP pendentes sem esperar o fim do loop
    }
  }

  // --- Painel Web WiFi: sempre ativo (funciona em modo AP e STA) ---
  server.handleClient();
  dmxReceiveAndSend();  // processa e envia frame que chegou durante handleClient()

  // --- Portal WiFiManager: bloqueia apenas as tarefas exclusivas do WiFi ---
  if (wmPortalRunning) {
    wm.process();
    server.handleClient();  // painel web continua respondendo durante o portal AP
    if (WiFi.status() == WL_CONNECTED) {
      delay(1000);
      ESP.restart();
    }
    delay(2);
    return;
  }

  // --- WiFi STA: mDNS ---
  MDNS.update();
  dmxReceiveAndSend();  // processa e envia frame que chegou durante MDNS/ArtNet handling

  if (now - lastPollReply >= POLL_REPLY_INTERVAL_MS) {
    lastPollReply = now;
    sendArtPollReply();
    sendArtPollReplyEth();
  }
}