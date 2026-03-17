/*
  ArtNet2DMX_ESP8266_w_Eth0.ino - ESP8266 com suporte dual: WiFi + Ethernet (módulo W5500, biblioteca Ethernet 2.x).
  ===================
  Recebe um universo ArtNet via WiFi e transmite DMX512 via UART → MAX487 (RS485).
  ou 
  Recebe um universo ArtNet via Ethernet e transmite DMX512 via UART → MAX487 (RS485).

  Pinagem:
    GPIO1  (UART0 TX) ──► DI   do MAX487  (dados DMX)
    GPIO4             ──► DE + RE do MAX487  (nível HIGH = transmitir)
    GPIO2  (Serial1)  ──► debug (opcional, conecte ao RX do adaptador USB-Serial)

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
const char* WIFI_AP_NAME = "OneBitCoreV2DMX";
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
const char* NODE_SHORT_NAME = "OneBitCoreV2DMX";
const char* NODE_LONG_NAME  = "ESP8266 OneBitCore V2 ArtNet to DMX Node";
const char* NODE_MDNS_NAME  = "onebitdmx";  // → http://onebitdmx.local:8080

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
unsigned long lastDmxSend  = 0;     // controla o loop de reenvio DMX (canal 1 → 0-30s)

unsigned long lastPollReply = 0;
const unsigned long POLL_REPLY_INTERVAL_MS = 3000;

// --- Ethernet ---
uint8_t  ethMac[]    = { 0xDE, 0xAD, 0xBE, 0xEF, 0xAB, 0x01 };  // MAC único — altere se houver múltiplos nós na rede
bool     ethLinked   = false;   // W5500 com link ativo e IP DHCP válido
bool     lastPktEth  = false;   // true = último ArtDmx recebido via Ethernet

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
    Serial1.printf("[ArtPoll/%s] de %d.%d.%d.%d\n", src,
                   sender[0], sender[1], sender[2], sender[3]);
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

    Serial1.printf("[ArtDmx/%s] Net%d.Sub%d.Uni%d (filtro: %d.%d.%d)\n",
                   src, pktNet, pktSubnet, pktUniverse,
                   artnetNet, artnetSubnet, artnetUniverse);

    if (pktNet != artnetNet || pktSubnet != artnetSubnet || pktUniverse != artnetUniverse) {
      pktFilt++;
      Serial1.println("[ArtDmx] FILTRADO — universo não corresponde");
      return;
    }

    uint16_t dataLen = ((uint16_t)buf[16] << 8) | buf[17];
    if (dataLen > DMX_CHANNELS) dataLen = DMX_CHANNELS;

    dmxData[0] = 0x00;
    memcpy(dmxData + 1, buf + 18, dataLen);

    dmxSend();

    lastPacketMs = millis();
    lastDmxSend  = lastPacketMs;
    dmxRunning   = true;
    lastPktEth   = isEth;
    pktOk++;

    Serial1.printf("[ArtDmx/%s] ch1=%d ch2=%d ch3=%d\n",
                   src, dmxData[1], dmxData[2], dmxData[3]);
  }
}

// Lê pacote da interface WiFi e processa
void handleArtNetWiFi() {
  int pktSize = udp.parsePacket();
  if (pktSize < 10) return;
  IPAddress sender = udp.remoteIP();
  int len = udp.read(udpBuf, sizeof(udpBuf));
  if (len < 10) return;
  processArtNetPacket(udpBuf, len, sender, false);
}

// Lê pacote da interface Ethernet (W5500) e processa
void handleArtNetEth() {
  if (!ethLinked) return;
  int pktSize = udp_eth.parsePacket();
  if (pktSize < 10) return;
  IPAddress sender = udp_eth.remoteIP();
  int len = udp_eth.read(udpBuf, sizeof(udpBuf));
  if (len < 10) return;
  processArtNetPacket(udpBuf, len, sender, true);
}

// ===========================================================================
//  Painel Web
// ===========================================================================
String buildPage() {
  String ip   = getNodeIP().toString();
  String mode = (WiFi.status() == WL_CONNECTED) ? "STA" : "AP";

  String p = R"HTML(<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OneBit CoreV2 DMX</title>
<style>
  *{box-sizing:border-box}
  body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;margin:0;padding:12px 16px}
  .topbar{display:flex;align-items:center;gap:12px;margin-bottom:6px}
  .logo{flex-shrink:0}
  .title-wrap{flex:1;min-width:0}
  h1{color:#e94560;margin:0;font-size:1.35em;white-space:nowrap}
  .sub{color:#aaa;font-size:.78em;margin:2px 0 0;line-height:1.5}
  .sub b{color:#eee}
  .mdns-link{display:block;color:#7ec8e3;font-size:.78em;text-decoration:none;margin-top:2px}
  .page{max-width:1200px;margin:0 auto}
  .deck{display:grid;grid-template-columns:1fr;gap:14px}
  @media(min-width:700px){.deck{grid-template-columns:1fr 1fr}}
  @media(min-width:1050px){.deck{grid-template-columns:1fr 1fr 1fr}}
  .card{background:#16213e;border-radius:8px;padding:18px}
  .card h2{margin:0 0 12px;font-size:1em;color:#7ec8e3}
  .row{display:flex;justify-content:space-between;font-size:.88em;padding:4px 0;border-bottom:1px solid #1e2a4a}
  .row:last-child{border:none}
  .val{color:#e94560;font-weight:bold}
  .v6{color:#7ec8e3;font-weight:normal;font-size:.75em;word-break:break-all;display:block;text-align:right;margin-top:1px}
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
var updTimer=null;
function schedUpd(){if(updTimer)clearTimeout(updTimer);updTimer=setTimeout(upd,200);}
function upd(){
  var start=parseInt(document.getElementById('ch-start').value)||1;
  var count=parseInt(document.getElementById('ch-count').value)||16;
  if(start<1)start=1;if(start>512)start=512;
  if(count<1)count=1;if(count>128)count=128;
  fetch('/data?start='+start+'&count='+count).then(r=>r.json()).then(d=>{
    // status
    var s=document.getElementById('rx-status');
    if(!d.running){s.className='val err';s.textContent='Aguardando...';}
    else if(d.ago<3){s.className='val ok';s.textContent='Ativo';}
    else{s.className='val warn';s.textContent='Sem pacotes';}
    document.getElementById('rx-ago').textContent=d.running?(d.ago+'s atr\u00e1s'):'\u2014';
    document.getElementById('rx-ch123').textContent=d.s1+' / '+d.s2+' / '+d.s3;
    document.getElementById('rx-ok').textContent=d.pktOk;
    document.getElementById('rx-filt').textContent=d.pktFilt;
    // intervalo canal 1
    var iv=document.getElementById('rx-interval');
    if(iv){var ms=d.interval;iv.textContent=ms===0?'M\u00e1ximo (sem espera)':(ms<1000?(ms+'ms'):(+(ms/1000).toFixed(1)+'s'));}
    // grid din\u00e2mico de canais
    var grid=document.getElementById('ch-grid');
    var html='';
    for(var i=0;i<d.ch.length;i++){
      html+='<div><div class=\'lbl\'>Ch '+(d.chStart+i)+'</div>'
           +'<div class=\'val\' style=\'font-size:1.1em\'>'+d.ch[i]+'</div></div>';
    }
    grid.innerHTML=html;
    // card titulo e modo continuo
    var ct=document.getElementById('card-ch-title');
    var last=d.chStart+d.ch.length-1;
    ct.textContent=d.cont?('Canais DMX (teste ativo \u2014 val='+d.val+')'):'Canais DMX recebidos ('+d.chStart+'\u2013'+last+')';
    // barra de status continuo
    var sb=document.getElementById('cont-status');
    var ss=document.getElementById('cont-start');
    if(d.cont){sb.style.display='block';sb.querySelector('span').textContent=d.val;ss.style.display='none';}
    else{sb.style.display='none';ss.style.display='flex';}
    // ethernet e fonte do sinal
    var es=document.getElementById('eth-status');
    if(es){es.className=d.ethLinked?'val ok':'val err';es.textContent=d.ethLinked?d.ethIp:'Sem link';}
    var ws=document.getElementById('wifi-status');
    if(ws){var wip=d.wifiIp||'';ws.className=(wip&&wip!=='0.0.0.0')?'val ok':'val err';
      if(wip&&wip!=='0.0.0.0'){ws.innerHTML=wip+(d.wifiIpv6?'<span class=\'v6\'>'+d.wifiIpv6+'</span>':'');}
      else ws.textContent='Sem link';}
    var rs=document.getElementById('rx-src');
    if(rs){rs.textContent=d.running?(d.lastSrc==='eth'?'Ethernet':'WiFi'):'\u2014';}
    // dot piscando
    var dot=document.getElementById('dot');
    dot.style.background=dot.style.background==='#4caf50'?'#555':'#4caf50';
  }).catch(function(){});
}
setInterval(upd,300);
window.onload=upd;
</script>
</head><body>
<div class='page'>
<div class='topbar'>
  <div class='logo'>
    <svg width='212' height='54' viewBox='0 0 212 54' fill='none' xmlns='http://www.w3.org/2000/svg'>
      <!-- shadow / outer layer -->
      <polygon points='27,2 49,14 49,40 27,52 5,40 5,14' fill='#06111f'/>
      <!-- main hex body -->
      <polygon points='27,5 47,16.5 47,37.5 27,49 7,37.5 7,16.5' fill='#0d2345'/>
      <!-- inner hex border -->
      <polygon points='27,8 44,18.5 44,35.5 27,46 10,35.5 10,18.5' fill='none' stroke='#1b5fa0' stroke-width='1.4'/>
      <!-- Z / lightning stroke -->
      <polyline points='13,19 40,19 13,35 40,35' fill='none' stroke='#2a85d0' stroke-width='2.2' stroke-linecap='round' stroke-linejoin='round'/>
      <!-- corner circuit dots -->
      <circle cx='13' cy='19' r='2' fill='#4aaae8'/>
      <circle cx='40' cy='19' r='2' fill='#4aaae8'/>
      <circle cx='13' cy='35' r='2' fill='#4aaae8'/>
      <circle cx='40' cy='35' r='2' fill='#4aaae8'/>
      <!-- right-pointing arrow out of hex -->
      <polygon points='54,27 45,21.5 45,25 51,25 51,29 45,29 45,32.5' fill='#1e6ab8'/>
      <!-- ONEBIT -->
      <text x='62' y='25' font-family='Arial,sans-serif' font-weight='bold' font-size='19' fill='#ffffff' letter-spacing='0.5'>ONEBIT</text>
      <!-- separator line -->
      <line x1='62' y1='30' x2='208' y2='30' stroke='#1e6ab8' stroke-width='1.5'/>
      <!-- ENGENHARIA -->
      <text x='62' y='46' font-family='Arial,sans-serif' font-size='11' fill='#5ab2eb' letter-spacing='2.8'>ENGENHARIA</text>
    </svg>
  </div>
  <div class='title-wrap'>
    <h1>CoreV2 DMX <span id='dot'></span></h1>
)HTML";

  String ethIpStr = ethLinked ? Ethernet.localIP().toString() : String("-");
  String wv6 = getWiFiIPv6Str();
  p += "<div class='sub'>WiFi: <b>" + ip + "</b>";
  if (wv6.length()) p += " / <b style='color:#7ec8e3'>" + wv6 + "</b>";
  p += " (" + mode + ") &nbsp;·&nbsp; ETH: <b>" + ethIpStr + "</b> <small style='color:#888'>(IPv4)</small>"
       " &nbsp;·&nbsp; Net:<b>" + String(artnetNet) +
       "</b> Sub:<b>" + String(artnetSubnet) +
       "</b> Uni:<b>" + String(artnetUniverse) +
       "</b> Porta:<b>" + String(artnetPort) + "</b></div>";
  p += "<a class='mdns-link' href='http://onebitdmx.local:8080'>"
       "&#127760; onebitdmx.local:8080</a>";
  p += "</div></div>";  // fecha title-wrap e topbar

  p += "<div class='deck'>";  // abre grade de cards

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
  p += "<div class='row'><span>Intervalo reenvio (canal 1)</span>";
  p += "<span id='rx-interval' class='val'>\u2014</span></div>";
  p += "<div class='row'><span>Origem \u00faltimo pacote</span>";
  p += "<span id='rx-src' class='val'>\u2014</span></div>";
  p += "<div class='row'><span>Ethernet (W5500)</span>";
  p += "<span id='eth-status' class='val'>\u2014</span></div>";
  p += "<div class='row'><span>WiFi</span>";
  p += "<span id='wifi-status' class='val'>\u2014</span></div>";
  p += "</div>";

  // Canais din\u00e2micos
  p += "<div class='card'><h2 id='card-ch-title'>Canais DMX recebidos (1\u201316)</h2>";
  p += "<div class='grid2' style='margin-bottom:12px'>";
  p += "<div><div class='lbl'>Canal inicial (1\u2013512)</div>"
       "<input type='number' id='ch-start' min='1' max='512' value='1' oninput='schedUpd()'></div>";
  p += "<div><div class='lbl'>Quantidade (1\u2013128)</div>"
       "<input type='number' id='ch-count' min='1' max='128' value='16' oninput='schedUpd()'></div>";
  p += "</div>";
  p += "<div class='grid2' id='ch-grid'></div>";
  p += "</div>";

  // Card de teste DMX direto
  p += "<div class='card'><h2>Teste DMX (sem ArtNet)</h2>";
  p += "<div style='display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px'>";
  p += "<a href='/test-dmx?val=255' style='flex:1'><button class='btn' style='background:#4caf50'>1 Frame FULL</button></a>";
  p += "<a href='/test-dmx?val=0'   style='flex:1'><button class='btn' style='background:#555'>1 Frame ZERO</button></a>";
  p += "</div>";
  // botoes continuo
  p += "<div id='cont-start' style='display:flex;gap:8px;flex-wrap:wrap'>";
  p += "<a href='/test-dmx?val=255&cont=1' style='flex:1'><button class='btn' style='background:#2196f3'>&#9654; Cont\u00ednuo FULL (255)</button></a>";
  p += "<a href='/test-dmx?val=128&cont=1' style='flex:1'><button class='btn' style='background:#ff9800'>&#9654; Cont\u00ednuo HALF (128)</button></a>";
  p += "</div>";
  p += "<div id='cont-status' style='display:none'>";
  p += "<div style='background:#1a5c38;padding:8px;border-radius:4px;text-align:center;margin-bottom:8px'>";
  p += "&#9654; Enviando cont\u00ednuo val=<span>0</span> (~10fps)</div>";
  p += "<a href='/test-dmx?val=0&cont=0'><button class='btn' style='background:#f44336'>&#9646;&#9646; PARAR cont\u00ednuo</button></a>";
  p += "</div>";   // fecha cont-status
  p += "</div>";   // fecha card Teste DMX

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
  p += "</div>";  // fecha card WiFi

  p += "</div>";  // fecha .deck
  p += "</div>";  // fecha .page
  p += "</body></html>";
  return p;
}

void handleRoot() { server.send(200, "text/html", buildPage()); }

// Monta o JSON de status (usado por WiFi e Ethernet)
void buildDataJson(char* buf, int bufSize, int chStart, int chCount) {
  if (chStart < 1) chStart = 1;
  if (chCount < 1) chCount = 16;
  if (chStart + chCount - 1 > 512) chCount = 512 - chStart + 1;

  uint32_t dmxIntervalMs = (uint32_t)dmxData[1] * 30000UL / 255;
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
    ethLinked ? 1 : 0, ethIpBuf, wifiIpBuf, wifiIpv6,
    dmxRunning ? (lastPktEth ? "eth" : "wifi") : "none",
    chStart);

  for (int i = 0; i < chCount && n < bufSize - 6; i++) {
    if (i > 0) buf[n++] = ',';
    n += snprintf(buf + n, bufSize - n, "%d", dmxData[chStart + i]);
  }
  if (n < bufSize - 3) { buf[n++] = ']'; buf[n++] = '}'; buf[n] = '\0'; }
}

// Endpoint JSON leve para atualizações ao vivo via fetch()
void handleData() {
  static char buf[1200];
  int chStart = 1, chCount = 16;
  if (server.hasArg("start")) chStart = constrain(server.arg("start").toInt(), 1, 512);
  if (server.hasArg("count")) chCount = constrain(server.arg("count").toInt(), 1, 128);
  buildDataJson(buf, sizeof(buf), chStart, chCount);
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
  } else if (cont) {
    testContinuous = true;
  } else {
    dmxSend();  // 1 frame único
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
  if (server.hasArg("net"))      artnetNet      = (uint8_t)constrain(server.arg("net").toInt(),      0, 127);
  if (server.hasArg("subnet"))   artnetSubnet   = (uint8_t)constrain(server.arg("subnet").toInt(),   0,  15);
  if (server.hasArg("universe")) artnetUniverse = (uint8_t)constrain(server.arg("universe").toInt(), 0,  15);
  sendArtPollReply();
  sendArtPollReplyEth();
  server.sendHeader("Location", "/");
  server.send(303);
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

  // Aguarda a primeira linha da requisição HTTP chegar
  unsigned long t = millis();
  while (!client.available() && client.connected() && millis() - t < 1500) yield();
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

  // Drena cabeçalhos, captura Content-Length (com timeout para não bloquear)
  int contentLen = 0;
  unsigned long hdrT = millis();
  while (client.connected() && millis() - hdrT < 2000) {
    if (!client.available()) { yield(); continue; }
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
    while ((int)body.length() < contentLen && millis() - t < 800) {
      if (client.available()) body += (char)client.read();
      else yield();
    }
    query = body;
  }

  // --- Roteamento ---

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
    memset(dmxData + 1, val, DMX_CHANNELS); dmxData[0] = 0x00;
    if (stop_) { testContinuous = false; memset(dmxData + 1, 0, DMX_CHANNELS); dmxSend(); }
    else if (cont_) testContinuous = true;
    else dmxSend();
    lastPacketMs = millis();
    dmxRunning = (val > 0 || testContinuous);
    client.print(F("HTTP/1.1 303 See Other\r\nLocation: /\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"));
    client.flush();
    client.stop(); return;
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
    String ns = ethParam(query, "net");      if (ns.length()) artnetNet      = (uint8_t)constrain(ns.toInt(),  0, 127);
    String ss = ethParam(query, "subnet");   if (ss.length()) artnetSubnet   = (uint8_t)constrain(ss.toInt(),  0,  15);
    String us = ethParam(query, "universe"); if (us.length()) artnetUniverse = (uint8_t)constrain(us.toInt(),  0,  15);
    sendArtPollReply(); sendArtPollReplyEth();
    client.print(F("HTTP/1.1 303 See Other\r\nLocation: /\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"));
    client.flush();
    client.stop(); return;
  }

  if (path == "/data") {
    int chStart = 1, chCount = 16;
    String ss = ethParam(query, "start"); if (ss.length()) chStart = constrain(ss.toInt(), 1, 512);
    String cs = ethParam(query, "count"); if (cs.length()) chCount = constrain(cs.toInt(), 1, 128);
    static char jsonBuf[1200];
    buildDataJson(jsonBuf, sizeof(jsonBuf), chStart, chCount);
    int jLen = strlen(jsonBuf);
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-cache\r\nContent-Length: "));
    client.print(jLen);
    client.print(F("\r\nConnection: close\r\n\r\n"));
    client.write((const uint8_t*)jsonBuf, jLen);
    client.flush();
    client.stop(); return;
  }

  // GET / — painel principal
  // A página tem ~6 KB mas o buffer TX do W5500 é de apenas 2 KB/socket.
  // Enviamos em chunks de 512 bytes, aguardando cada um drenar antes de continuar.
  String page = buildPage();
  client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\nConnection: close\r\n\r\n"));
  {
    const int CHUNK = 512;
    int total = page.length();
    int sent  = 0;
    while (sent < total) {
      int sz = min(CHUNK, total - sent);
      client.write((const uint8_t*)page.c_str() + sent, sz);
      client.flush();
      sent += sz;
      yield();  // cede ao stack WiFi/SPI entre chunks
    }
  }
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
  // Serial1 = GPIO2, somente TX, usado para debug (não interfere no DMX)
  Serial1.begin(115200);

  // MAX487: DE/RE HIGH = modo transmissão
  pinMode(DMX_DE_PIN, OUTPUT);
  digitalWrite(DMX_DE_PIN, HIGH);

  // Inicializa buffer DMX zerado (blackout)
  memset(dmxData, 0, sizeof(dmxData));

  // --- WiFi ---
  WiFi.mode(WIFI_STA);
  // Nota: IPv6 requer lwIP compilado com LWIP_IPV6=1. Esta build usa LWIP_IPV6=0,
  // portanto getWiFiIPv6Str() retornará "" e o campo wifiIpv6 ficará vazio no painel.
  String savedSSID = WiFi.SSID();

  if (savedSSID.length() > 0) {
    wm.setConfigPortalTimeout(1);
    wm.setConnectTimeout(15);
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
      Serial1.print("[mDNS] WiFi: onebitdmx.local ativo — "); Serial1.println(getNodeIP());
    }
  }

  // --- Ethernet W5500 ---
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);
  delay(10);
  digitalWrite(ETH_RST_PIN, HIGH);
  delay(200);
  Ethernet.init(ETH_CS_PIN);
  Serial1.print("[ETH] Iniciando (DHCP)...");
  if (Ethernet.begin(ethMac, 4000, 1000)) {
    ethLinked = true;
    udp_eth.begin(artnetPort);
    ethServer.begin();
    mdns_udp_eth.beginMulticast(MDNS_MCAST, MDNS_PORT);  // entra no grupo mDNS 224.0.0.251:5353
    Serial1.print(" OK — IP: "); Serial1.println(Ethernet.localIP());
    Serial1.println("[mDNS] ETH: onebitdmx.local ativo");
  } else {
    Serial1.println(" sem link ou DHCP indispon\xEDvel");
  }

  // --- UDP WiFi ---
  udp.begin(artnetPort);

  // --- Web server ---
  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/data",        HTTP_GET,  handleData);
  server.on("/artnet",     HTTP_POST, handleArtnetCfg);
  server.on("/test-dmx",   HTTP_GET,  handleTestDmx);
  server.on("/reset-wifi",  HTTP_GET,  handleResetWifi);
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

  // --- Ethernet: sempre ativo, independente do estado WiFi ---
  if (ethLinked) Ethernet.maintain();
  if (ethLinked) handleEthHttp();
  if (ethLinked) handleEthMdns();

  // --- Saída DMX: sempre ativa (teste contínuo ou reenvio por intervalo) ---
  if (testContinuous) {
    if (now - lastTestSend >= 100) {
      lastTestSend = now;
      lastPacketMs  = now;
      dmxSend();
    }
  } else {
    // ArtNet via Ethernet funciona mesmo sem WiFi
    if (ethLinked) handleArtNetEth();

    uint32_t dmxIntervalMs = (uint32_t)dmxData[1] * 30000UL / 255UL;
    if (dmxIntervalMs == 0) dmxIntervalMs = 25;
    if ((uint32_t)(now - lastDmxSend) >= dmxIntervalMs) {
      dmxSend();
      lastDmxSend = now;
    }
  }

  // --- Portal WiFiManager: bloqueia apenas as tarefas exclusivas do WiFi ---
  if (wmPortalRunning) {
    wm.process();
    MDNS.update();
    if (WiFi.status() == WL_CONNECTED) {
      delay(1000);
      ESP.restart();
    }
    delay(2);
    return;
  }

  // --- WiFi: painel, ArtNet e mDNS (só quando conectado) ---
  server.handleClient();
  MDNS.update();
  handleArtNetWiFi();

  if (now - lastPollReply >= POLL_REPLY_INTERVAL_MS) {
    lastPollReply = now;
    sendArtPollReply();
    sendArtPollReplyEth();
  }
}