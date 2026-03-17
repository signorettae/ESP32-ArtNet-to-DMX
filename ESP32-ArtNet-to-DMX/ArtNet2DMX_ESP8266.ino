/*
  ArtNet2DMX_ESP8266
  ===================
  Recebe um universo ArtNet via WiFi e transmite DMX512 via UART → MAX487 (RS485).

  Pinagem:
    GPIO1  (UART0 TX) ──► DI   do MAX487  (dados DMX)
    GPIO4             ──► DE + RE do MAX487  (nível HIGH = transmitir)
    GPIO2  (Serial1)  ──► debug (opcional, conecte ao RX do adaptador USB-Serial)

  Bibliotecas necessárias (Gerenciador de Bibliotecas):
    - WiFiManager  (tzapu/WiFiManager)

  Primeiro boot sem WiFi salvo: cria AP "ArtNet2DMX" para configurar a rede.
  Após conectar, acesse http://<ip>:8080 para o painel.
*/

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>

// ===========================================================================
//  CONFIGURAÇÕES
// ===========================================================================

// --- WiFi ---
const char* WIFI_AP_NAME = "ArtNet2DMX";
const char* WIFI_AP_PASS = "";           // "" = AP aberto

// --- ArtNet (editáveis pelo painel web) ---
uint16_t artnetPort     = 6454;
uint8_t  artnetNet      = 0;
uint8_t  artnetSubnet   = 0;
uint8_t  artnetUniverse = 0;

// --- DMX ---
const int     DMX_TX_PIN     = 1;     // GPIO1 = UART0 TX
const int     DMX_DE_PIN     = 16;    // GPIO16 = D0 = DE/RE do MAX487
const int     DMX_CHANNELS   = 512;
const int     DMX_BREAK_US   = 100;   // duração do BREAK  (mínimo 92 µs)
const int     DMX_MAB_US     = 12;    // duração do MAB    (mínimo 8 µs)

// --- Node info ---
const char* NODE_SHORT_NAME = "ArtNet2DMX";
const char* NODE_LONG_NAME  = "ESP8266 ArtNet to DMX Node";

// ===========================================================================
//  ESTADO
// ===========================================================================
static uint8_t dmxData[DMX_CHANNELS + 1];  // [0]=start code(0x00), [1..512]=canais
static uint8_t pollReplyBuf[239];
static uint8_t udpBuf[640];

uint8_t  artnetSeq    = 0;
uint32_t lastPacketMs = 0;          // quando chegou o último ArtDmx
bool     dmxRunning   = false;      // recebendo ArtNet?
uint32_t pktOk        = 0;          // pacotes ArtDmx aceitos
uint32_t pktFilt      = 0;          // pacotes ArtDmx filtrados (universo errado)

bool     testContinuous = false;    // modo teste contínuo (~40fps)
uint8_t  testValue      = 255;      // valor do modo teste contínuo
unsigned long lastTestSend = 0;

unsigned long lastPollReply = 0;
const unsigned long POLL_REPLY_INTERVAL_MS = 3000;

WiFiUDP          udp;
ESP8266WebServer server(8080);
WiFiManager      wm;
bool             wmPortalRunning = false;

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

// ===========================================================================
//  DMX OUTPUT via UART0 → MAX487
// ===========================================================================
void dmxSend() {
  // Garante que a UART esvaziou antes de parar
  Serial.flush();
  Serial.end();

  // BREAK e MAB com interrupts desabilitados para garantir timing preciso
  // (o stack WiFi do ESP8266 usa interrupts e corromperia o sinal)
  noInterrupts();
  pinMode(DMX_TX_PIN, OUTPUT);
  digitalWrite(DMX_TX_PIN, LOW);
  delayMicroseconds(DMX_BREAK_US);   // BREAK ≥ 92 µs
  digitalWrite(DMX_TX_PIN, HIGH);
  delayMicroseconds(DMX_MAB_US);     // MAB ≥ 8 µs
  interrupts();

  // Religa UART0 no modo DMX (250 kbaud, 8N2)
  Serial.begin(250000, SERIAL_8N2);
  delayMicroseconds(10);             // deixa UART estabilizar

  // Envia start code (0x00) + 512 canais
  // Não chama Serial.flush() aqui: o write já bloqueia ~22ms enchendo o FIFO;
  // um flush extra atrasaria ainda mais o WiFi.
  Serial.write(dmxData, DMX_CHANNELS + 1);

  // Cede tempo ao stack WiFi após envio
  yield();
}

// ===========================================================================
//  ArtPollReply
// ===========================================================================
void buildPollReply(uint8_t* buf) {
  IPAddress ip = getNodeIP();
  uint8_t mac[6];
  WiFi.macAddress(mac);

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
  buf[173] = 0x01;                           // NumPorts = 1
  buf[174] = 0x40;                           // PortTypes[0] = entrada DMX (recebe ArtNet)
  buf[178] = 0x80;                           // GoodInput[0]
  buf[190] = artnetUniverse & 0x0F;          // SwIn[0]
  memcpy(&buf[201], mac, 6);
  buf[207] = ip[0]; buf[208] = ip[1];
  buf[209] = ip[2]; buf[210] = ip[3];
  buf[211] = 0x01;                           // BindIndex
  buf[212] = 0x08;                           // Status2
}

void sendArtPollReply() {
  buildPollReply(pollReplyBuf);
  IPAddress bcast = getDirectedBroadcast();
  udp.beginPacket(bcast, artnetPort);
  udp.write(pollReplyBuf, 239);
  udp.endPacket();
}

void sendArtPollReplyTo(IPAddress dest) {
  buildPollReply(pollReplyBuf);
  udp.beginPacket(dest, artnetPort);
  udp.write(pollReplyBuf, 239);
  udp.endPacket();
}

// ===========================================================================
//  Recepção ArtNet
// ===========================================================================
void handleArtNet() {
  int pktSize = udp.parsePacket();
  if (pktSize < 10) return;

  IPAddress sender = udp.remoteIP();
  int len = udp.read(udpBuf, sizeof(udpBuf));
  if (len < 10) return;

  // Valida ID "Art-Net\0"
  if (memcmp(udpBuf, "Art-Net", 7) != 0 || udpBuf[7] != 0x00) return;

  uint16_t opcode = udpBuf[8] | ((uint16_t)udpBuf[9] << 8);

  // --- ArtPoll → responde imediatamente ---
  if (opcode == 0x2000) {
    Serial1.print("[ArtPoll] de "); Serial1.println(sender);
    sendArtPollReplyTo(sender);
    sendArtPollReply();
    return;
  }

  // --- ArtDmx ---
  if (opcode == 0x5000 && len >= 18) {
    // Filtra por Net.SubNet.Universe
    uint8_t pktNet      = udpBuf[15] & 0x7F;
    uint8_t pktSubUni   = udpBuf[14];
    uint8_t pktSubnet   = (pktSubUni >> 4) & 0x0F;
    uint8_t pktUniverse = pktSubUni & 0x0F;

    Serial1.printf("[ArtDmx] recebido Net%d.Sub%d.Uni%d (filtro: %d.%d.%d)\n",
                   pktNet, pktSubnet, pktUniverse,
                   artnetNet, artnetSubnet, artnetUniverse);

    if (pktNet != artnetNet || pktSubnet != artnetSubnet || pktUniverse != artnetUniverse) {      pktFilt++;      Serial1.println("[ArtDmx] FILTRADO — universo não corresponde");
      return;
    }

    uint16_t dataLen = ((uint16_t)udpBuf[16] << 8) | udpBuf[17];
    if (dataLen > DMX_CHANNELS) dataLen = DMX_CHANNELS;

    dmxData[0] = 0x00;  // start code
    memcpy(dmxData + 1, udpBuf + 18, dataLen);

    dmxSend();

    lastPacketMs = millis();
    dmxRunning   = true;
    pktOk++;

    Serial1.printf("[ArtDmx] Net%d.Sub%d.Uni%d ch1=%d ch2=%d ch3=%d\n",
                   pktNet, pktSubnet, pktUniverse,
                   dmxData[1], dmxData[2], dmxData[3]);
  }
}

// ===========================================================================
//  Painel Web
// ===========================================================================
String buildPage() {
  String ip   = getNodeIP().toString();
  String mode = (WiFi.status() == WL_CONNECTED) ? "STA" : "AP";

  String p = R"(<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ArtNet2DMX</title>
<style>
  *{box-sizing:border-box}
  body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;margin:0;padding:16px}
  h1{color:#e94560;text-align:center;margin-bottom:4px;font-size:1.4em}
  .sub{text-align:center;color:#aaa;font-size:.85em;margin-bottom:16px}
  .card{background:#16213e;border-radius:8px;padding:18px;margin:0 auto 14px;max-width:520px}
  .card h2{margin:0 0 12px;font-size:1em;color:#7ec8e3}
  .row{display:flex;justify-content:space-between;font-size:.88em;padding:4px 0;border-bottom:1px solid #1e2a4a}
  .row:last-child{border:none}
  .val{color:#e94560;font-weight:bold}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .lbl{font-size:.7em;color:#aaa;margin-bottom:3px}
  input[type=number]{width:100%;padding:6px 8px;border-radius:4px;border:1px solid #444;
    background:#0f3460;color:#eee;font-size:.95em}
  .btn{display:block;width:100%;padding:10px;border:none;border-radius:4px;
    cursor:pointer;font-size:1em;margin-top:10px;color:#fff;background:#e94560}
  .btn:hover{background:#c73652}
  .ok{color:#4caf50}.warn{color:#ff9800}.err{color:#f44336}
  #dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#555;margin-left:6px;vertical-align:middle}
</style>
<script>
function upd(){
  fetch('/data').then(r=>r.json()).then(d=>{
    // status
    var s=document.getElementById('rx-status');
    if(!d.running){s.className='val err';s.textContent='Aguardando...';}
    else if(d.ago<3){s.className='val ok';s.textContent='Ativo';}
    else{s.className='val warn';s.textContent='Sem pacotes';}
    document.getElementById('rx-ago').textContent=d.running?(d.ago+'s atr\u00e1s'):'\u2014';
    document.getElementById('rx-ch123').textContent=d.ch[1]+' / '+d.ch[2]+' / '+d.ch[3];
    document.getElementById('rx-ok').textContent=d.pktOk;
    document.getElementById('rx-filt').textContent=d.pktFilt;
    // canais 1-16
    for(var i=1;i<=16;i++) document.getElementById('ch'+i).textContent=d.ch[i];
    // card titulo e modo continuo
    var ct=document.getElementById('card-ch-title');
    ct.textContent=d.cont?('Canais DMX (teste ativo \u2014 val='+d.val+')'):'Canais DMX recebidos (1\u201316)';
    // barra de status continuo
    var sb=document.getElementById('cont-status');
    var ss=document.getElementById('cont-start');
    if(d.cont){sb.style.display='block';sb.querySelector('span').textContent=d.val;ss.style.display='none';}
    else{sb.style.display='none';ss.style.display='flex';}
    // dot piscando
    var dot=document.getElementById('dot');
    dot.style.background=dot.style.background==='#4caf50'?'#555':'#4caf50';
  }).catch(function(){});
}
setInterval(upd,2000);
window.onload=upd;
</script>
</head><body>
<h1>ArtNet2DMX_ESP8266 <span id='dot'></span></h1>
)";

  p += "<div class='sub'>IP: <b>" + ip + "</b> (" + mode + ")"
       "&nbsp;|&nbsp;Net:<b>" + String(artnetNet) +
       "</b> Sub:<b>" + String(artnetSubnet) +
       "</b> Uni:<b>" + String(artnetUniverse) +
       "</b>&nbsp;|&nbsp;Porta:<b>" + String(artnetPort) + "</b></div>";

  // Status
  p += "<div class='card'><h2>Status</h2>";
  p += "<div class='row'><span>Recep\u00e7\u00e3o ArtNet</span>";
  p += "<span id='rx-status' class='val err'>Aguardando...</span></div>";
  p += "<div class='row'><span>\u00daltimo pacote</span>";
  p += "<span id='rx-ago' class='val'>\u2014</span></div>";
  p += "<div class='row'><span>Ch 1 / 2 / 3</span>";
  p += "<span id='rx-ch123' class='val'>0 / 0 / 0</span></div>";
  p += "<div class='row'><span>Pacotes aceitos / filtrados</span>";
  p += "<span class='val'><span id='rx-ok'>0</span> / <span id='rx-filt'>0</span></span></div>";
  p += "</div>";

  // Canais 1-16
  p += "<div class='card'><h2 id='card-ch-title'>Canais DMX recebidos (1\u201316)</h2>";
  p += "<div class='grid2'>";
  for (int i = 1; i <= 16; i++) {
    p += "<div><div class='lbl'>Ch " + String(i) + "</div>"
         "<div id='ch" + String(i) + "' class='val' style='font-size:1.1em'>0</div></div>";
  }
  p += "</div></div>";

  // Card de teste DMX direto
  p += "<div class='card'><h2>Teste DMX (sem ArtNet)</h2>";
  p += "<div style='display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px'>";
  p += "<a href='/test-dmx?val=255' style='flex:1'><button class='btn' style='background:#4caf50'>1 Frame FULL</button></a>";
  p += "<a href='/test-dmx?val=0'   style='flex:1'><button class='btn' style='background:#555'>1 Frame ZERO</button></a>";
  p += "</div>";
  // botoes continuo — visibilidade controlada por JS
  p += "<div id='cont-start' style='display:flex;gap:8px;flex-wrap:wrap'>";
  p += "<a href='/test-dmx?val=255&cont=1' style='flex:1'><button class='btn' style='background:#2196f3'>&#9654; Cont\u00ednuo FULL (255)</button></a>";
  p += "<a href='/test-dmx?val=128&cont=1' style='flex:1'><button class='btn' style='background:#ff9800'>&#9654; Cont\u00ednuo HALF (128)</button></a>";
  p += "</div>";
  p += "<div id='cont-status' style='display:none'>";
  p += "<div style='background:#1a5c38;padding:8px;border-radius:4px;text-align:center;margin-bottom:8px'>";
  p += "&#9654; Enviando cont\u00ednuo val=<span>0</span> (~10fps)</div>";
  p += "<a href='/test-dmx?val=0&cont=0'><button class='btn' style='background:#f44336'>&#9646;&#9646; PARAR cont\u00ednuo</button></a>";
  p += "</div>";
  p += "<div style='margin-top:8px;font-size:.75em;color:#aaa'>";
  p += "Para oscilosc&oacute;pio: ative Cont&iacute;nuo, Trigger Edge Falling ~1.5V, Mode Normal, 500&micro;s/div. (10fps = frame a cada 100ms)</div>";
  p += "</div>";

  // Config ArtNet
  p += "<div class='card'><h2>Endereço ArtNet</h2>";
  p += "<form action='/artnet' method='POST'>";
  p += "<div class='grid2'>";
  p += "<div><div class='lbl'>Porta UDP</div>"
       "<input type='number' name='port' min='1' max='65535' value='" + String(artnetPort) + "'></div>";
  p += "<div><div class='lbl'>Net (0-127)</div>"
       "<input type='number' name='net' min='0' max='127' value='" + String(artnetNet) + "'></div>";
  p += "<div><div class='lbl'>Sub-Net (0-15)</div>"
       "<input type='number' name='subnet' min='0' max='15' value='" + String(artnetSubnet) + "'></div>";
  p += "<div><div class='lbl'>Universo (0-15)</div>"
       "<input type='number' name='universe' min='0' max='15' value='" + String(artnetUniverse) + "'></div>";
  p += "</div>";
  p += "<button class='btn' type='submit'>Aplicar</button>";
  p += "</form></div>";

  // Reset WiFi
  p += "<div class='card' style='text-align:center'>";
  p += "<a href='/reset-wifi' style='color:#e94560;font-size:.85em' "
       "onclick=\"return confirm('Apagar credenciais WiFi e reiniciar?')\">"
       "&#9888; Apagar WiFi salvo e abrir portal de configuração</a>";
  p += "</div>";

  p += "</body></html>";
  return p;
}

void handleRoot() { server.send(200, "text/html", buildPage()); }

// Endpoint JSON leve para atualizações ao vivo via fetch()
void handleData() {
  // Usa buffer est\u00e1tico para evitar fragmenta\u00e7\u00e3o do heap do ESP8266
  // (String += 512x causa falhas quando h\u00e1 tr\u00e1fego UDP simult\u00e2neo)
  static char buf[512];
  int n = snprintf(buf, sizeof(buf),
    "{\"running\":%d,\"cont\":%d,\"val\":%d,\"ago\":%lu,"
    "\"pktOk\":%lu,\"pktFilt\":%lu,"
    "\"net\":%d,\"sub\":%d,\"uni\":%d,\"port\":%d,"
    "\"ch\":[0",
    dmxRunning ? 1 : 0,
    testContinuous ? 1 : 0,
    testValue,
    (millis() - lastPacketMs) / 1000UL,
    (unsigned long)pktOk,
    (unsigned long)pktFilt,
    artnetNet, artnetSubnet, artnetUniverse, artnetPort);

  // 32 canais s\u00e3o suficientes para o painel (exibe s\u00f3 1-16)
  for (int i = 1; i <= 32 && n < (int)sizeof(buf) - 6; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, ",%d", dmxData[i]);
  }
  if (n < (int)sizeof(buf) - 3) { buf[n++] = ']'; buf[n++] = '}'; buf[n] = '\0'; }

  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", buf);
}

// Envia um frame DMX de teste com valor fixo em todos os canais
// GET /test-dmx?val=255        → envia 1 frame
// GET /test-dmx?val=255&cont=1 → ativa envio contínuo a ~40fps
// GET /test-dmx?val=0&cont=0   → desativa envio contínuo
void handleTestDmx() {
  uint8_t val  = testValue;
  if (server.hasArg("val"))  val = (uint8_t)constrain(server.arg("val").toInt(), 0, 255);
  bool    cont = server.hasArg("cont") && server.arg("cont") == "1";
  bool    stop = server.hasArg("cont") && server.arg("cont") == "0";

  testValue = val;
  memset(dmxData + 1, val, DMX_CHANNELS);
  dmxData[0] = 0x00;

  if (stop) {
    testContinuous = false;
    memset(dmxData + 1, 0, DMX_CHANNELS);  // blackout ao parar
    dmxSend();
    Serial1.println("[TEST-DMX] contínuo PARADO");
  } else if (cont) {
    testContinuous = true;
    Serial1.printf("[TEST-DMX] contínuo INICIADO val=%d\n", val);
  } else {
    dmxSend();  // 1 frame único
    Serial1.printf("[TEST-DMX] 1 frame val=%d\n", val);
  }

  lastPacketMs = millis();
  dmxRunning   = (val > 0 || testContinuous);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleArtnetCfg() {
  if (server.hasArg("port")) {
    uint16_t p = (uint16_t)constrain(server.arg("port").toInt(), 1, 65535);
    if (p != artnetPort) { artnetPort = p; udp.stop(); udp.begin(artnetPort); }
  }
  if (server.hasArg("net"))      artnetNet      = (uint8_t)constrain(server.arg("net").toInt(),      0, 127);
  if (server.hasArg("subnet"))   artnetSubnet   = (uint8_t)constrain(server.arg("subnet").toInt(),   0,  15);
  if (server.hasArg("universe")) artnetUniverse = (uint8_t)constrain(server.arg("universe").toInt(), 0,  15);
  sendArtPollReply();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleResetWifi() {
  server.send(200, "text/html",
    "<html><body style='font-family:Arial;background:#1a1a2e;color:#eee;text-align:center;padding:40px'>"
    "<h2 style='color:#e94560'>Credenciais apagadas</h2>"
    "<p>Conecte ao AP <b>ArtNet2DMX</b> e acesse <b>192.168.4.1</b></p>"
    "</body></html>");
  delay(500);
  wm.resetSettings();
  ESP.restart();
}

// ===========================================================================
//  Setup
// ===========================================================================
void setup() {
  // Serial1 = GPIO2, somente TX, usado para debug (não interfere no DMX)
  Serial1.begin(115200);
  Serial1.println("\n=== ArtNet2DMX_ESP8266 ===");

  // MAX487: DE/RE HIGH = modo transmissão
  pinMode(DMX_DE_PIN, OUTPUT);
  digitalWrite(DMX_DE_PIN, HIGH);

  // Inicializa buffer DMX zerado (blackout)
  memset(dmxData, 0, sizeof(dmxData));

  // --- WiFi ---
  WiFi.mode(WIFI_STA);
  String savedSSID = WiFi.SSID();

  if (savedSSID.length() > 0) {
    Serial1.print("Conectando em: "); Serial1.println(savedSSID);
    wm.setConfigPortalTimeout(1);
    wm.setConnectTimeout(15);
    wm.autoConnect(WIFI_AP_NAME, WIFI_AP_PASS);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial1.println("Sem WiFi — portal não-bloqueante...");
    wm.setConfigPortalBlocking(false);
    wm.setConfigPortalTimeout(0);
    wm.startConfigPortal(WIFI_AP_NAME, WIFI_AP_PASS[0] ? WIFI_AP_PASS : nullptr);
    wmPortalRunning = true;
  }

  Serial1.print("IP: ");       Serial1.println(getNodeIP());
  Serial1.print("Broadcast: "); Serial1.println(getDirectedBroadcast());

  // --- UDP ---
  udp.begin(artnetPort);

  // --- Web server ---
  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/data",        HTTP_GET,  handleData);
  server.on("/artnet",     HTTP_POST, handleArtnetCfg);
  server.on("/test-dmx",  HTTP_GET,  handleTestDmx);
  server.on("/reset-wifi", HTTP_GET,  handleResetWifi);
  server.begin();
  Serial1.print("Painel: http://"); Serial1.print(getNodeIP()); Serial1.println(":8080");

  // Anuncia o node
  sendArtPollReply();
  Serial1.println("Pronto — aguardando ArtNet...");
}

// ===========================================================================
//  Loop
// ===========================================================================
void loop() {
  unsigned long now = millis();

  server.handleClient();

  if (wmPortalRunning) {
    wm.process();
    if (WiFi.status() == WL_CONNECTED) {
      Serial1.println("WiFi configurado! Reiniciando...");
      delay(1000);
      ESP.restart();
    }
  }

  // Processa pacotes ArtNet (apenas se não estiver em modo teste contínuo)
  if (!testContinuous) handleArtNet();

  // Modo teste contínuo: envia DMX a ~10fps (100ms) para deixar WiFi respirar
  if (testContinuous && (now - lastTestSend) >= 100) {
    lastTestSend = now;
    lastPacketMs  = now;  // impede o blackout de timeout apagar os dados
    dmxSend();
  }

  // Re-anuncia periodicamente
  if (now - lastPollReply >= POLL_REPLY_INTERVAL_MS) {
    lastPollReply = now;
    sendArtPollReply();
  }

  // Timeout: se não receber pacotes ArtNet por 5s, zera os canais (blackout de segurança)
  // Não aplica durante teste contínuo (lastPacketMs é atualizado acima)
  if (dmxRunning && !testContinuous && (now - lastPacketMs) > 5000) {
    dmxRunning = false;
    memset(dmxData + 1, 0, DMX_CHANNELS);
    dmxSend();
    Serial1.println("[WARN] ArtNet timeout — blackout DMX");
  }
}
