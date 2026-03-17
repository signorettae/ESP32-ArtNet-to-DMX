/*
  ArtNetSimpleNode — ESP8266 WiFi
  ================================
  Envia ArtPollReply e ArtDmx via WiFi.
  Painel de controle: http://<ip-dispositivo>:8080

  Hardware: ESP8266 (NodeMCU, Wemos D1 Mini, etc.)

  Bibliotecas necessárias (instale via Gerenciador de Bibliotecas):
    - WiFiManager  (tzapu/WiFiManager)

  Primeiro boot: cria AP "ArtNetSimple" para configurar WiFi.
  Após conectar, acesse http://<ip>:8080
*/

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>

// ===========================================================================
//  CONFIGURAÇÕES
// ===========================================================================

// --- WiFi ---
const char*    WIFI_AP_NAME   = "ArtNetSimple";
const char*    WIFI_AP_PASS   = "";            // "" = AP aberto
const int      PORTAL_TIMEOUT = 180;           // s

// --- ArtNet (editáveis pelo painel web) ---
uint16_t artnetPort     = 6454;  // porta UDP
uint8_t  artnetNet      = 0;     // Net (0-127)
uint8_t  artnetSubnet   = 0;     // Sub-Net (0-15)
uint8_t  artnetUniverse = 0;     // Universo (0-15) — usado no ArtDmx e no ArtPollReply

// --- Node info ---
const char* NODE_SHORT_NAME = "ArtNetSimple";
const char* NODE_LONG_NAME  = "ESP8266 ArtNet Simple Node";

// ===========================================================================
//  ESTADO — editável pelo painel web
// ===========================================================================
bool     testEnabled  = false;
uint16_t testInterval = 1000;
int      testChannels = 1;
uint8_t  testValue    = 255;

// ===========================================================================
//  BUFFERS GLOBAIS (evita consumo de stack com arrays grandes)
// ===========================================================================
static uint8_t dmxData[513];          // [0]=start code, [1..512]=canais
static uint8_t artDmxPacket[530];     // 18 header + 512 dados
static uint8_t pollReplyBuf[239];

uint8_t  artnetSeq = 0;

unsigned long lastDmxSend   = 0;
unsigned long lastPollReply = 0;
const unsigned long POLL_REPLY_INTERVAL_MS = 3000;

WiFiUDP          udp;
ESP8266WebServer server(8080);
WiFiManager      wm;
bool             wmPortalRunning = false;  // portal de config ativo?

// ===========================================================================
//  ArtPollReply
// ===========================================================================
// Retorna o IP correto tanto em modo STA quanto em modo AP
IPAddress getNodeIP() {
  IPAddress ip = WiFi.localIP();
  if (ip == IPAddress(0, 0, 0, 0)) {
    ip = WiFi.softAPIP();  // AP mode
  }
  return ip;
}

// Calcula o broadcast direcionado da sub-rede (ex: 192.168.4.255)
IPAddress getDirectedBroadcast() {
  IPAddress ip   = getNodeIP();
  // Em modo AP, a máscara é sempre /24 (255.255.255.0)
  // Em modo STA, pergunta ao WiFi
  IPAddress mask = (WiFi.status() == WL_CONNECTED) ? WiFi.subnetMask()
                                                   : IPAddress(255, 255, 255, 0);
  return IPAddress((uint8_t)(ip[0] | ~mask[0]), (uint8_t)(ip[1] | ~mask[1]),
                   (uint8_t)(ip[2] | ~mask[2]), (uint8_t)(ip[3] | ~mask[3]));
}

void sendArtPollReply() {
  IPAddress ip = getNodeIP();
  uint8_t mac[6];
  WiFi.macAddress(mac);

  memset(pollReplyBuf, 0, 239);
  memcpy(pollReplyBuf, "Art-Net", 7);
  pollReplyBuf[8]  = 0x00; pollReplyBuf[9]  = 0x21;  // OpCode ArtPollReply
  pollReplyBuf[10] = ip[0]; pollReplyBuf[11] = ip[1];
  pollReplyBuf[12] = ip[2]; pollReplyBuf[13] = ip[3];
  pollReplyBuf[14] = artnetPort & 0xFF;
  pollReplyBuf[15] = (artnetPort >> 8) & 0xFF;
  pollReplyBuf[16] = 0x00; pollReplyBuf[17] = 0x01;   // versão firmware
  pollReplyBuf[18] = artnetNet    & 0x7F;              // NetSwitch
  pollReplyBuf[19] = artnetSubnet & 0x0F;              // SubSwitch
  pollReplyBuf[20] = 0xFF; pollReplyBuf[21] = 0xFF;   // OEM não registrado
  pollReplyBuf[23] = 0xD2;                             // Status1
  strncpy((char*)&pollReplyBuf[26],  NODE_SHORT_NAME, 17);
  strncpy((char*)&pollReplyBuf[44],  NODE_LONG_NAME,  63);
  strncpy((char*)&pollReplyBuf[108], "#0001 [0000] OK", 63);
  pollReplyBuf[173] = 0x01;  // NumPorts = 1
  pollReplyBuf[174] = 0x80;  // PortTypes[0] = saída DMX
  pollReplyBuf[182] = 0x80;  // GoodOutput[0]
  pollReplyBuf[190] = artnetUniverse & 0x0F;          // SwOut[0]
  memcpy(&pollReplyBuf[201], mac, 6);
  pollReplyBuf[207] = ip[0]; pollReplyBuf[208] = ip[1];
  pollReplyBuf[209] = ip[2]; pollReplyBuf[210] = ip[3];
  pollReplyBuf[211] = 0x01;  // BindIndex
  pollReplyBuf[212] = 0x08;  // Status2: end. 15-bit

  // Envia como broadcast direcionado (funciona em modo AP e STA)
  IPAddress bcast = getDirectedBroadcast();
  udp.beginPacket(bcast, artnetPort);
  udp.write(pollReplyBuf, 239);
  udp.endPacket();
}

// Versão unicast: responde diretamente ao remetente do ArtPoll
void sendArtPollReplyTo(IPAddress dest) {
  // Monta o buffer igual
  IPAddress ip = getNodeIP();
  uint8_t mac[6];
  WiFi.macAddress(mac);

  memset(pollReplyBuf, 0, 239);
  memcpy(pollReplyBuf, "Art-Net", 7);
  pollReplyBuf[8]  = 0x00; pollReplyBuf[9]  = 0x21;
  pollReplyBuf[10] = ip[0]; pollReplyBuf[11] = ip[1];
  pollReplyBuf[12] = ip[2]; pollReplyBuf[13] = ip[3];
  pollReplyBuf[14] = artnetPort & 0xFF;
  pollReplyBuf[15] = (artnetPort >> 8) & 0xFF;
  pollReplyBuf[16] = 0x00; pollReplyBuf[17] = 0x01;
  pollReplyBuf[18] = artnetNet    & 0x7F;
  pollReplyBuf[19] = artnetSubnet & 0x0F;
  pollReplyBuf[20] = 0xFF; pollReplyBuf[21] = 0xFF;
  pollReplyBuf[23] = 0xD2;
  strncpy((char*)&pollReplyBuf[26],  NODE_SHORT_NAME, 17);
  strncpy((char*)&pollReplyBuf[44],  NODE_LONG_NAME,  63);
  strncpy((char*)&pollReplyBuf[108], "#0001 [0000] OK", 63);
  pollReplyBuf[173] = 0x01;
  pollReplyBuf[174] = 0x80;
  pollReplyBuf[182] = 0x80;
  pollReplyBuf[190] = artnetUniverse & 0x0F;
  memcpy(&pollReplyBuf[201], mac, 6);
  pollReplyBuf[207] = ip[0]; pollReplyBuf[208] = ip[1];
  pollReplyBuf[209] = ip[2]; pollReplyBuf[210] = ip[3];
  pollReplyBuf[211] = 0x01;
  pollReplyBuf[212] = 0x08;

  udp.beginPacket(dest, artnetPort);
  udp.write(pollReplyBuf, 239);
  udp.endPacket();
}

// ===========================================================================
//  ArtDmx
// ===========================================================================
void sendArtDmx() {
  const uint16_t dataLen = 512;
  memset(artDmxPacket, 0, 18 + dataLen);
  memcpy(artDmxPacket, "Art-Net", 7);
  artDmxPacket[8]  = 0x00; artDmxPacket[9]  = 0x50;   // OpCode ArtDmx
  artDmxPacket[10] = 0x00; artDmxPacket[11] = 0x0e;   // Protocol v14
  artDmxPacket[12] = ++artnetSeq;
  artDmxPacket[13] = 0x00;
  artDmxPacket[14] = ((artnetSubnet & 0x0F) << 4) | (artnetUniverse & 0x0F); // SubUni
  artDmxPacket[15] = artnetNet & 0x7F;                                         // Net
  artDmxPacket[16] = (dataLen >> 8) & 0xFF;
  artDmxPacket[17] = dataLen & 0xFF;
  memcpy(artDmxPacket + 18, dmxData + 1, dataLen);

  udp.beginPacket(getDirectedBroadcast(), artnetPort);
  udp.write(artDmxPacket, 18 + dataLen);
  udp.endPacket();
}

// ===========================================================================
//  Painel Web — HTML
// ===========================================================================
String buildPage() {
  String wifiIP = getNodeIP().toString();

  String p = R"(<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ArtNet Simple Node</title>
<style>
  *{box-sizing:border-box}
  body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;margin:0;padding:16px}
  h1{color:#e94560;text-align:center;margin-bottom:4px;font-size:1.4em}
  .sub{text-align:center;color:#aaa;font-size:.85em;margin-bottom:16px}
  .card{background:#16213e;border-radius:8px;padding:18px;margin:0 auto 14px;max-width:520px}
  .card h2{margin:0 0 14px;font-size:1em;color:#7ec8e3}
  label{display:flex;justify-content:space-between;align-items:center;margin:10px 0 2px;font-size:.9em}
  label span{font-weight:bold;color:#e94560;min-width:32px;text-align:right}
  input[type=range]{width:100%;accent-color:#e94560;margin:4px 0 10px}
  input[type=number]{width:100%;padding:6px 8px;border-radius:4px;border:1px solid #444;
    background:#0f3460;color:#eee;font-size:.95em}
  .btn{display:block;width:100%;padding:10px;border:none;border-radius:4px;
    cursor:pointer;font-size:1em;margin-top:8px;color:#fff}
  .btn-on{background:#e94560}.btn-on:hover{background:#c73652}
  .btn-pause{background:#555}.btn-pause:hover{background:#444}
  .btn-send{background:#0f3460}.btn-send:hover{background:#1a4a8a}
  .status{text-align:center;padding:8px 12px;border-radius:4px;margin-bottom:10px;font-weight:bold}
  .on{background:#1a5c38}.off{background:#5c1a1a}
  .ch-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:10px}
  .ch-cell{text-align:center}
  .ch-label{font-size:.7em;color:#aaa;margin-bottom:2px}
  .ch-cell input{text-align:center;padding:4px 2px}
  .indicator{display:inline-block;width:12px;height:12px;border-radius:50%;
    background:#e94560;margin-right:8px;animation:pulse 1s infinite alternate}
  @keyframes pulse{from{opacity:.4}to{opacity:1}}
</style>
</head><body>
<h1>ArtNet Simple Node</h1>
)";

  p += "<div class='sub'>IP: <b>" + wifiIP + "</b>&nbsp;|&nbsp;"
       "Net:<b>" + String(artnetNet) + "</b> "
       "Sub:<b>" + String(artnetSubnet) + "</b> "
       "Uni:<b>" + String(artnetUniverse) + "</b>&nbsp;|&nbsp;"
       "Porta:<b>" + String(artnetPort) + "</b></div>";

  // Card de status + controles de teste
  p += "<div class='card'>";
  p += "<div class='status " + String(testEnabled ? "on" : "off") + "'>";
  if (testEnabled) {
    p += "<span class='indicator'></span>Enviando ArtNet via WiFi...";
  } else {
    p += "<span class='indicator' style='background:#555;animation:none'></span>Envio pausado";
  }
  p += "</div>";

  p += "<form action='/set' method='POST'>";
  p += "<label>Canais de teste (1-512) <span id='sv'>" + String(testChannels) + "</span></label>";
  p += "<input type='range' name='channels' min='1' max='512' value='" + String(testChannels) +
       "' oninput=\"document.getElementById('sv').textContent=this.value\">";
  p += "<label>Valor DMX (0-255) <span id='vv'>" + String(testValue) + "</span></label>";
  p += "<input type='range' name='value' min='0' max='255' value='" + String(testValue) +
       "' oninput=\"document.getElementById('vv').textContent=this.value\">";
  p += "<label>Intervalo de envio (ms)</label>";
  p += "<input type='number' name='interval' min='50' max='10000' value='" + String(testInterval) + "'>";
  p += "<button class='btn btn-on'    type='submit' name='enable' value='1'>Aplicar e Ativar</button>";
  p += "<button class='btn btn-pause' type='submit' name='enable' value='0'>Pausar</button>";
  p += "</form></div>";

  // Card canais individuais 1–16
  p += "<div class='card'><h2>Canais individuais (1-16)</h2>";
  p += "<form action='/channels' method='POST'>";
  p += "<div class='ch-grid'>";
  for (int i = 1; i <= 16; i++) {
    p += "<div class='ch-cell'><div class='ch-label'>Ch " + String(i) + "</div>";
    p += "<input type='number' name='ch" + String(i) +
         "' min='0' max='255' value='" + String(dmxData[i]) + "'></div>";
  }
  p += "</div>";
  p += "<button class='btn btn-send' type='submit'>Enviar agora</button>";
  p += "</form></div>";

  // Card endereço ArtNet
  p += "<div class='card'><h2>Endereço ArtNet</h2>";
  p += "<form action='/artnet' method='POST'>";
  p += "<div style='display:grid;grid-template-columns:1fr 1fr;gap:10px'>";
  p += "<div><div class='ch-label' style='text-align:left;margin-bottom:4px'>Porta UDP</div>";
  p += "<input type='number' name='port' min='1' max='65535' value='" + String(artnetPort) + "'></div>";
  p += "<div><div class='ch-label' style='text-align:left;margin-bottom:4px'>Net (0-127)</div>";
  p += "<input type='number' name='net' min='0' max='127' value='" + String(artnetNet) + "'></div>";
  p += "<div><div class='ch-label' style='text-align:left;margin-bottom:4px'>Sub-Net (0-15)</div>";
  p += "<input type='number' name='subnet' min='0' max='15' value='" + String(artnetSubnet) + "'></div>";
  p += "<div><div class='ch-label' style='text-align:left;margin-bottom:4px'>Universo (0-15)</div>";
  p += "<input type='number' name='universe' min='0' max='15' value='" + String(artnetUniverse) + "'></div>";
  p += "</div>";
  p += "<button class='btn btn-send' type='submit' style='margin-top:12px'>Aplicar</button>";
  p += "</form></div>";

  // Card reset WiFi
  p += "<div class='card' style='text-align:center'>";
  p += "<a href='/reset-wifi' style='color:#e94560;font-size:.85em' "
       "onclick=\"return confirm('Apagar credenciais WiFi e reiniciar?')\">"
       "&#9888; Apagar WiFi salvo e abrir portal de configuração</a>";
  p += "</div>";

  p += "</body></html>";
  return p;
}

// ===========================================================================
//  Handlers Web
// ===========================================================================
void handleRoot() {
  server.send(200, "text/html", buildPage());
}

void handleSet() {
  if (server.hasArg("enable"))   testEnabled  = (server.arg("enable") == "1");
  if (server.hasArg("channels")) testChannels = constrain(server.arg("channels").toInt(), 1, 512);
  if (server.hasArg("value"))    testValue    = (uint8_t)constrain(server.arg("value").toInt(), 0, 255);
  if (server.hasArg("interval")) testInterval = (uint16_t)constrain(server.arg("interval").toInt(), 50, 10000);

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleChannels() {
  // Desativa modo de teste ao definir canais manualmente
  testEnabled = false;

  for (int i = 1; i <= 16; i++) {
    String key = "ch" + String(i);
    if (server.hasArg(key)) {
      dmxData[i] = (uint8_t)constrain(server.arg(key).toInt(), 0, 255);
    }
  }
  sendArtDmx();

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleArtnet() {
  if (server.hasArg("port")) {
    uint16_t p = (uint16_t)constrain(server.arg("port").toInt(), 1, 65535);
    if (p != artnetPort) {
      artnetPort = p;
      udp.stop();
      udp.begin(artnetPort);
    }
  }
  if (server.hasArg("net"))      artnetNet      = (uint8_t)constrain(server.arg("net").toInt(),      0, 127);
  if (server.hasArg("subnet"))   artnetSubnet   = (uint8_t)constrain(server.arg("subnet").toInt(),   0,  15);
  if (server.hasArg("universe")) artnetUniverse = (uint8_t)constrain(server.arg("universe").toInt(), 0,  15);
  sendArtPollReply();  // re-anuncia com o novo endereço
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleResetWifi() {
  server.send(200, "text/html",
    "<html><body style='font-family:Arial;background:#1a1a2e;color:#eee;text-align:center;padding:40px'>"
    "<h2 style='color:#e94560'>Credenciais WiFi apagadas</h2>"
    "<p>Conecte ao AP <b>ArtNetSimple</b> e acesse <b>192.168.4.1</b> para configurar.</p>"
    "</body></html>");
  delay(500);
  wm.resetSettings();
  ESP.restart();
}

// ===========================================================================
//  Setup
// ===========================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ArtNetSimpleNode (WiFi) ===");

  memset(dmxData, 0, sizeof(dmxData));

  // --- WiFi: tenta STA se houver credenciais salvas, senão abre portal não-bloqueante ---
  WiFi.mode(WIFI_STA);
  String savedSSID = WiFi.SSID();

  if (savedSSID.length() > 0) {
    // Rede salva: tenta conectar (bloqueia até 15 s, sem abrir portal)
    Serial.print("Tentando conectar em: ");
    Serial.println(savedSSID);
    wm.setConfigPortalTimeout(1);    // não abre portal se falhar
    wm.setConnectTimeout(15);
    wm.autoConnect(WIFI_AP_NAME, WIFI_AP_PASS);
  }

  if (WiFi.status() != WL_CONNECTED) {
    // Sem rede salva ou falha: abre portal de configuração não-bloqueante
    // O usuário conecta ao AP "ArtNetSimple" e acessa 192.168.4.1 para configurar
    Serial.println("Sem WiFi — abrindo portal de configuração (não-bloqueante)...");
    wm.setConfigPortalBlocking(false);
    wm.setConfigPortalTimeout(0);    // 0 = nunca expira
    wm.startConfigPortal(WIFI_AP_NAME, WIFI_AP_PASS[0] ? WIFI_AP_PASS : nullptr);
    wmPortalRunning = true;
  }

  Serial.print("IP: ");
  Serial.println(getNodeIP());
  Serial.print("Broadcast: ");
  Serial.println(getDirectedBroadcast());

  // --- UDP ---
  udp.begin(artnetPort);

  // --- Web server na porta 8080 (evita conflito com WiFiManager na 80) ---
  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/set",       HTTP_POST, handleSet);
  server.on("/channels",  HTTP_POST, handleChannels);
  server.on("/artnet",    HTTP_POST, handleArtnet);
  server.on("/reset-wifi",HTTP_GET,  handleResetWifi);
  server.begin();
  Serial.print("Painel em: http://");
  Serial.print(getNodeIP());
  Serial.println(":8080");

  // Anuncia o node
  sendArtPollReply();

  Serial.println("Pronto.");
}

// ===========================================================================
//  Loop
// ===========================================================================
void loop() {
  server.handleClient();

  // Portal WiFiManager não-bloqueante: processa requisições do portal de configuração
  if (wmPortalRunning) {
    wm.process();
    if (WiFi.status() == WL_CONNECTED) {
      // Usuário configurou a rede — reinicia para conectar como STA
      Serial.println("WiFi configurado! Reiniciando...");
      delay(1000);
      ESP.restart();
    }
  }

  unsigned long now = millis();

  // Responde ArtPoll recebido (software faz Refresh → envia ArtPoll → node deve responder)
  int pktSize = udp.parsePacket();
  if (pktSize >= 10) {
    IPAddress sender = udp.remoteIP();
    uint8_t buf[12];
    udp.read(buf, sizeof(buf));
    // Detecta pacote Art-Net: ID "Art-Net\0" + OpCode ArtPoll (0x2000)
    if (memcmp(buf, "Art-Net", 7) == 0 && buf[8] == 0x00 && buf[9] == 0x20) {
      Serial.print("[ArtPoll] de "); Serial.println(sender);
      sendArtPollReplyTo(sender);   // unicast ao solicitante
      sendArtPollReply();           // broadcast para outros na rede
    }
  }

  // Re-anuncia periodicamente
  if (now - lastPollReply >= POLL_REPLY_INTERVAL_MS) {
    lastPollReply = now;
    sendArtPollReply();
  }

  // Modo de teste automático
  if (testEnabled && (now - lastDmxSend >= testInterval)) {
    lastDmxSend = now;

    memset(dmxData + 1, 0, 512);
    for (int i = 1; i <= testChannels; i++)
      dmxData[i] = testValue;

    sendArtDmx();
    Serial.printf("[TEST] Ch 1-%d = %d\n", testChannels, testValue);
  }
}
