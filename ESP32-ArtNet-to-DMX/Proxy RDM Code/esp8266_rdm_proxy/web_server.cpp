/**
 * web_server.cpp — Servidor web síncrono (ESP8266WebServer)
 *
 * USA APENAS ESP8266WebServer do core ESP8266.
 * NÃO inclui ESPAsyncWebServer (causaria conflito de enum com WiFiManager).
 *
 * Diferença de API em relação ao AsyncWebServer:
 *   Async: server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){ req->send(...); })
 *   Sync:  server.on("/", HTTP_GET, [](){  server.send(200, ...);  })
 *          server.handleClient() chamado no loop()
 *
 * Rotas disponíveis:
 *   GET  /           → Dashboard HTML
 *   GET  /config     → Página de configuração HTML
 *   POST /config     → Salvar configuração (body JSON)
 *   GET  /api/dmx    → JSON buffers DMX (entrada + saída)
 *   GET  /api/status → JSON status do sistema
 *   GET  /api/rdmlog → JSON log RDM
 *   POST /api/reset  → Reset credenciais Wi-Fi
 *   POST /api/factory→ Reset de fábrica
 */

// WiFiManager.h DEVE ter sido incluído antes (no .ino) para que
// ESP8266WebServer.h já esteja declarado — evita dupla inclusão.
#include "web_server.h"
#include "storage.h"
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Arduino.h>

// ─── Instância do servidor na porta 80 ───────────────────────
static ESP8266WebServer server(80);

static ProxyConfig* gCfg = nullptr;
static DMXState*    gDMX = nullptr;
static RDMState*    gRDM = nullptr;

// ─── HTML: Dashboard ─────────────────────────────────────────
static const char HTML_DASHBOARD[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RDM Proxy</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',Arial,sans-serif;background:#1a1a2e;color:#eee;min-height:100vh}
header{background:#16213e;padding:14px 20px;display:flex;align-items:center;justify-content:space-between;border-bottom:2px solid #0f3460}
h1{font-size:1.3rem;color:#e94560}
nav a{color:#a8b2d8;text-decoration:none;margin-left:14px;font-size:.88rem}
nav a:hover{color:#e94560}
.wrap{padding:16px;max-width:1100px;margin:auto}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px;margin-bottom:20px}
.card{background:#16213e;border-radius:8px;padding:14px;border:1px solid #0f3460}
.card h3{font-size:.72rem;color:#a8b2d8;text-transform:uppercase;letter-spacing:1px;margin-bottom:5px}
.card .v{font-size:1.5rem;font-weight:700;color:#e94560}
.card .s{font-size:.7rem;color:#7a8199;margin-top:3px}
.sec{font-size:.9rem;color:#a8b2d8;margin:16px 0 7px;border-bottom:1px solid #0f3460;padding-bottom:4px}
.sbar{display:flex;gap:12px;background:#0f3460;padding:6px 14px;border-radius:6px;margin-bottom:12px;font-size:.76rem;flex-wrap:wrap}
.si{display:flex;align-items:center;gap:5px}
.dot{width:7px;height:7px;border-radius:50%;display:inline-block}
.dg{background:#5cb85c;animation:p 2s infinite}
.dx{background:#555}
@keyframes p{0%,100%{opacity:1}50%{opacity:.3}}
.tabs{display:flex;gap:7px;margin-bottom:10px}
.tab{padding:6px 13px;border-radius:5px;cursor:pointer;font-size:.8rem;border:1px solid #0f3460;color:#a8b2d8;background:transparent}
.tab.on{background:#e94560;color:#fff;border-color:#e94560}
.grid{display:grid;grid-template-columns:repeat(16,1fr);gap:2px;margin-bottom:18px}
.ch{position:relative;background:#0f3460;border-radius:2px;height:42px;overflow:hidden;cursor:default}
.bar{background:linear-gradient(#e94560,#f5a623);width:100%;position:absolute;bottom:0;transition:height .2s}
.cn{position:absolute;bottom:1px;left:50%;transform:translateX(-50%);font-size:7px;color:rgba(255,255,255,.45);pointer-events:none}
.ch:hover .tt{display:block}
.tt{display:none;position:absolute;top:-20px;left:50%;transform:translateX(-50%);background:#000;color:#fff;padding:1px 5px;border-radius:3px;font-size:9px;white-space:nowrap;z-index:9}
table{width:100%;border-collapse:collapse;font-size:.78rem}
th{background:#0f3460;padding:6px 10px;text-align:left;color:#a8b2d8}
td{padding:5px 10px;border-bottom:1px solid #0f3460;color:#ccd6f6}
tr:hover td{background:#1a2a4a}
.uid{font-family:monospace;color:#f5a623}
footer{text-align:center;padding:12px;font-size:.7rem;color:#444;margin-top:24px}
</style>
</head>
<body>
<header>
  <h1>&#9889; ESP8266 RDM Proxy</h1>
  <nav><a href="/">Dashboard</a><a href="/config">Config</a></nav>
</header>
<div class="wrap">
  <div class="sbar">
    <div class="si"><span class="dot dx" id="ddmx"></span>DMX In</div>
    <div class="si"><span class="dot dx" id="dan"></span>Art-Net</div>
    <div class="si"><span id="lm" style="color:#a8b2d8">&#8212;</span></div>
    <div class="si" style="margin-left:auto"><span id="lu" style="color:#444">&#8212;</span></div>
  </div>
  <div class="cards">
    <div class="card"><h3>Channel Offset</h3><div class="v" id="vo">&#8212;</div><div class="s">Deslocamento entrada</div></div>
    <div class="card"><h3>RDM UID</h3><div class="v uid" id="vu" style="font-size:.95rem">&#8212;</div><div class="s">Identidade do proxy</div></div>
    <div class="card"><h3>DMX Start Addr</h3><div class="v" id="vd">&#8212;</div><div class="s">Endere&ccedil;o RDM simulado</div></div>
    <div class="card"><h3>RDM Cmds</h3><div class="v" id="vr">0</div><div class="s">Desde o boot</div></div>
    <div class="card"><h3>Free Heap</h3><div class="v" id="vh">&#8212;</div><div class="s">bytes livres</div></div>
  </div>
  <div class="tabs">
    <div class="tab on" onclick="tab('i')">DMX Input (512ch)</div>
    <div class="tab"    onclick="tab('o')">DMX Output (mapeado)</div>
  </div>
  <div id="ti"><div class="sec">Buffer de Entrada &#8212; Canais 1&#8211;512</div><div class="grid" id="gi"></div></div>
  <div id="to" style="display:none"><div class="sec">Buffer de Sa&iacute;da &#8212; Ap&oacute;s offset</div><div class="grid" id="go"></div></div>
  <div class="sec">Log de Atividade RDM</div>
  <table><thead><tr><th>Tempo</th><th>UID</th><th>PID</th><th>CMD</th><th>Descri&ccedil;&atilde;o</th></tr></thead>
  <tbody id="rl"><tr><td colspan="5" style="color:#555;text-align:center">Nenhuma atividade RDM ainda</td></tr></tbody></table>
</div>
<footer>ESP8266 RDM Proxy &mdash; <span id="ip">&#8212;</span></footer>
<script>
let curTab='i';
function tab(t){
  curTab=t;
  document.getElementById('ti').style.display=t==='i'?'':'none';
  document.getElementById('to').style.display=t==='o'?'':'none';
  document.querySelectorAll('.tab').forEach((e,i)=>e.classList.toggle('on',(i===0&&t==='i')||(i===1&&t==='o')));
}
function buildGrid(id,data){
  const g=document.getElementById(id);
  if(!g.children.length){
    for(let i=0;i<512;i++){
      const d=document.createElement('div');
      d.className='ch';
      d.innerHTML=`<div class="tt">ch${i+1}:0</div><div class="bar" style="height:0"></div><div class="cn">${i+1}</div>`;
      g.appendChild(d);
    }
  }
  const ch=g.children;
  for(let i=0;i<512;i++){
    const v=data[i]||0;
    ch[i].querySelector('.bar').style.height=(v/255*100)+'%';
    ch[i].querySelector('.tt').textContent=`ch${i+1}:${v}`;
  }
}
async function poll(){
  try{
    const s=await(await fetch('/api/status')).json();
    document.getElementById('vo').textContent=s.channelOffset??'&#8212;';
    document.getElementById('vu').textContent=s.uid??'&#8212;';
    document.getElementById('vd').textContent=s.dmxStartAddr??'&#8212;';
    document.getElementById('vh').textContent=s.freeHeap??'&#8212;';
    document.getElementById('lm').textContent='Mode: '+(s.mode??'&#8212;');
    document.getElementById('ip').textContent=s.ip??'';
    const up=s.uptime||0,sec=Math.floor(up/1000),m=Math.floor(sec/60),h=Math.floor(m/60);
    document.getElementById('lu').textContent='Up: '+(h>0?`${h}h${m%60}m`:`${m}m${sec%60}s`);
    document.getElementById('ddmx').className='dot '+(s.dmxActive?'dg':'dx');
    document.getElementById('dan').className='dot '+(s.artnetActive?'dg':'dx');
  }catch(e){}
  try{
    const d=await(await fetch('/api/dmx')).json();
    buildGrid('gi',d.input||[]);
    buildGrid('go',d.output||[]);
  }catch(e){}
  try{
    const l=await(await fetch('/api/rdmlog')).json();
    const entries=l.log||[];
    if(!entries.length)return;
    document.getElementById('vr').textContent=l.total||entries.length;
    const tb=document.getElementById('rl');
    tb.innerHTML='';
    [...entries].reverse().forEach(e=>{
      const tr=document.createElement('tr');
      tr.innerHTML=`<td>${(e.ts/1000).toFixed(1)}s</td><td class="uid">${e.uid}</td><td>0x${(e.pid>>>0).toString(16).toUpperCase().padStart(4,'0')}</td><td>${e.cc}</td><td>${e.desc}</td>`;
      tb.appendChild(tr);
    });
  }catch(e){}
}
poll();setInterval(poll,1000);
</script>
</body>
</html>
)rawhtml";

// ─── HTML: Configuração ──────────────────────────────────────
static const char HTML_CONFIG[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RDM Proxy &#8212; Config</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',Arial,sans-serif;background:#1a1a2e;color:#eee;min-height:100vh}
header{background:#16213e;padding:14px 20px;display:flex;align-items:center;justify-content:space-between;border-bottom:2px solid #0f3460}
h1{font-size:1.3rem;color:#e94560}
nav a{color:#a8b2d8;text-decoration:none;margin-left:14px;font-size:.88rem}
nav a:hover{color:#e94560}
.wrap{padding:16px;max-width:680px;margin:auto}
.card{background:#16213e;border-radius:8px;padding:20px;border:1px solid #0f3460;margin-bottom:16px}
h2{font-size:.88rem;color:#a8b2d8;text-transform:uppercase;letter-spacing:1px;margin-bottom:14px}
.row{margin-bottom:13px}
label{display:block;font-size:.78rem;color:#a8b2d8;margin-bottom:5px}
input,select{width:100%;background:#0f3460;border:1px solid #1a4080;border-radius:5px;padding:8px 11px;color:#eee;font-size:.87rem;outline:none}
input:focus,select:focus{border-color:#e94560}
.hint{font-size:.7rem;color:#555;margin-top:3px}
.btn{padding:9px 20px;border-radius:5px;border:none;cursor:pointer;font-size:.87rem;font-weight:600}
.bp{background:#e94560;color:#fff}.bp:hover{background:#c73652}
.bd{background:#6b0000;color:#fff;margin-left:8px}.bd:hover{background:#900}
.msg{padding:9px 13px;border-radius:5px;margin-bottom:14px;font-size:.82rem;display:none}
.ok{background:#1a472a;color:#5cb85c}
.er{background:#3d1a1a;color:#d9534f}
</style>
</head>
<body>
<header>
  <h1>&#9889; ESP8266 RDM Proxy</h1>
  <nav><a href="/">Dashboard</a><a href="/config">Config</a></nav>
</header>
<div class="wrap">
  <div id="msg" class="msg"></div>
  <div class="card">
    <h2>&#128256; Mapeamento de Canais</h2>
    <div class="row">
      <label>Channel Offset (0 = sem offset)</label>
      <input type="number" id="co" min="0" max="511" value="0">
      <div class="hint">Offset=130 &#8594; canais 131&#8211;512 da entrada viram canais 1&#8211;382 na sa&iacute;da</div>
    </div>
  </div>
  <div class="card">
    <h2>&#128225; Fonte de Sinal</h2>
    <div class="row">
      <label>Modo de entrada</label>
      <select id="im">
        <option value="0">Apenas DMX f&iacute;sico</option>
        <option value="1">Apenas Art-Net</option>
        <option value="2">Merge (DMX + Art-Net)</option>
      </select>
    </div>
    <div class="row">
      <label>Modo Merge</label>
      <select id="mm">
        <option value="0">HTP &#8212; Maior valor vence</option>
        <option value="1">DMX f&iacute;sico prioridade (Art-Net fallback)</option>
        <option value="2">Art-Net prioridade (DMX fallback)</option>
      </select>
    </div>
    <div class="row">
      <label>Universo Art-Net (0&#8211;15)</label>
      <input type="number" id="au" min="0" max="15" value="0">
    </div>
  </div>
  <div class="card">
    <h2>&#128223; Identidade RDM</h2>
    <div class="row">
      <label>Device Label (m&aacute;x 32 chars)</label>
      <input type="text" id="dl" maxlength="32">
    </div>
    <div class="row">
      <label>Manufacturer Label (m&aacute;x 32 chars)</label>
      <input type="text" id="ml" maxlength="32">
    </div>
    <div class="row">
      <label>DMX Start Address (1&#8211;512)</label>
      <input type="number" id="da" min="1" max="512" value="1">
      <div class="hint">Alterar isto tamb&eacute;m atualiza o offset acima.</div>
    </div>
    <div class="row">
      <label>Device Model ID (hex)</label>
      <input type="text" id="mi" maxlength="6" value="0x0001">
    </div>
  </div>
  <div style="margin-bottom:20px">
    <button class="btn bp" onclick="save()">&#128190; Salvar</button>
    <button class="btn bd" onclick="resetWifi()">&#128260; Reset WiFi</button>
    <button class="btn bd" onclick="factory()" style="margin-left:8px">&#128465; Factory Reset</button>
  </div>
</div>
<script>
async function load(){
  try{
    const s=await(await fetch('/api/status')).json();
    document.getElementById('co').value=s.channelOffset??0;
    document.getElementById('im').value=s.inputMode??0;
    document.getElementById('mm').value=s.mergeMode??0;
    document.getElementById('au').value=s.artnetUniverse??0;
    document.getElementById('dl').value=s.deviceLabel??'';
    document.getElementById('ml').value=s.mfrLabel??'';
    document.getElementById('da').value=s.dmxStartAddr??1;
    document.getElementById('mi').value='0x'+(s.modelId||1).toString(16).toUpperCase().padStart(4,'0');
  }catch(e){msg('Erro ao carregar config',false);}
}
async function save(){
  const body=JSON.stringify({
    channelOffset:parseInt(document.getElementById('co').value),
    inputMode:parseInt(document.getElementById('im').value),
    mergeMode:parseInt(document.getElementById('mm').value),
    artnetUniverse:parseInt(document.getElementById('au').value),
    rdmDeviceLabel:document.getElementById('dl').value,
    rdmManufacturerLabel:document.getElementById('ml').value,
    rdmDmxStartAddress:parseInt(document.getElementById('da').value),
    rdmDeviceModelId:parseInt(document.getElementById('mi').value,16)
  });
  try{
    const r=await(await fetch('/config',{method:'POST',headers:{'Content-Type':'application/json'},body})).json();
    msg(r.ok?'&#10003; Salvo com sucesso!':'Erro: '+(r.error||'?'),r.ok);
  }catch(e){msg('Falha ao salvar: '+e,false);}
}
async function resetWifi(){
  if(!confirm('Resetar credenciais WiFi?'))return;
  await fetch('/api/reset',{method:'POST'});
  msg('WiFi resetado. Conecte ao AP RDM-Proxy-AP.',true);
}
async function factory(){
  if(!confirm('Reset de fabrica? Todas as configuracoes serao apagadas.'))return;
  await fetch('/api/factory',{method:'POST'});
  msg('Reset feito. Reiniciando...',true);
}
function msg(t,ok){
  const e=document.getElementById('msg');
  e.className='msg '+(ok?'ok':'er');
  e.style.display='block';
  e.innerHTML=t;
  setTimeout(()=>{e.style.display='none';},5000);
}
load();
</script>
</body>
</html>
)rawhtml";

// ─── Handlers das rotas ──────────────────────────────────────

static void handleRoot() {
  server.send_P(200, "text/html", HTML_DASHBOARD);
}

static void handleConfigPage() {
  server.send_P(200, "text/html", HTML_CONFIG);
}

static void handleApiDmx() {
  // Constrói JSON manualmente para economizar heap
  String json;
  json.reserve(2200);
  json = F("{\"input\":[");
  for (int i = 0; i < 512; i++) {
    json += gDMX->inputBuffer[i];
    if (i < 511) json += ',';
  }
  json += F("],\"output\":[");
  for (int i = 0; i < 512; i++) {
    json += gDMX->outputBuffer[i];
    if (i < 511) json += ',';
  }
  json += F("]}");
  server.send(200, F("application/json"), json);
}

static void handleApiStatus() {
  static const char* MODES[] = { "DMX only", "Art-Net only", "Merge" };

  char uid[20];
  snprintf(uid, sizeof(uid), "%02X:%02X:%02X:%02X:%02X:%02X",
           gRDM->uid[0], gRDM->uid[1], gRDM->uid[2],
           gRDM->uid[3], gRDM->uid[4], gRDM->uid[5]);

  StaticJsonDocument<512> doc;
  doc[F("channelOffset")]  = gCfg->channelOffset;
  doc[F("inputMode")]      = gCfg->inputMode;
  doc[F("mergeMode")]      = gCfg->mergeMode;
  doc[F("artnetUniverse")] = gCfg->artnetUniverse;
  doc[F("uid")]            = uid;
  doc[F("dmxStartAddr")]   = gCfg->rdmDmxStartAddress;
  doc[F("deviceLabel")]    = gCfg->rdmDeviceLabel;
  doc[F("mfrLabel")]       = gCfg->rdmManufacturerLabel;
  doc[F("modelId")]        = gCfg->rdmDeviceModelId;
  doc[F("mode")]           = MODES[constrain((int)gCfg->inputMode, 0, 2)];
  doc[F("dmxActive")]      = gDMX->dmxActive;
  doc[F("artnetActive")]   = gDMX->artnetActive;
  doc[F("uptime")]         = millis();
  doc[F("ip")]             = WiFi.localIP().toString();
  doc[F("identify")]       = gCfg->rdmIdentifyActive;
  doc[F("freeHeap")]       = ESP.getFreeHeap();

  String out;
  serializeJson(doc, out);
  server.send(200, F("application/json"), out);
}

static void handleApiRDMLog() {
  String json;
  json.reserve(512);
  json = F("{\"total\":");
  json += gRDM->logCount;
  json += F(",\"log\":[");

  for (uint8_t i = 0; i < gRDM->logCount; i++) {
    uint8_t idx = (gRDM->logHead - gRDM->logCount + i + RDM_LOG_MAX) % RDM_LOG_MAX;
    const RDMLogEntry& e = gRDM->log[idx];

    char uid[20];
    snprintf(uid, sizeof(uid), "%02X:%02X:%02X:%02X:%02X:%02X",
             e.uid[0], e.uid[1], e.uid[2],
             e.uid[3], e.uid[4], e.uid[5]);

    if (i > 0) json += ',';
    json += F("{\"ts\":");  json += e.timestamp;
    json += F(",\"uid\":\""); json += uid; json += F("\"");
    json += F(",\"pid\":");   json += e.pid;
    json += F(",\"cc\":\"");
    json += (e.cc == RDM_CC_GET_CMD  ? "GET"  :
             e.cc == RDM_CC_SET_CMD  ? "SET"  :
             e.cc == RDM_CC_DISC_CMD ? "DISC" : "RSP");
    json += F("\",\"desc\":\""); json += e.description; json += F("\"}");
  }
  json += F("]}");
  server.send(200, F("application/json"), json);
}

static void handleConfigPost() {
  if (!server.hasArg("plain")) {
    server.send(400, F("application/json"), F("{\"ok\":false,\"error\":\"No body\"}"));
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, F("application/json"), F("{\"ok\":false,\"error\":\"JSON invalido\"}"));
    return;
  }

  if (doc.containsKey("channelOffset"))
    gCfg->channelOffset = constrain((int)doc["channelOffset"], 0, 511);
  if (doc.containsKey("inputMode"))
    gCfg->inputMode = constrain((int)doc["inputMode"], 0, 2);
  if (doc.containsKey("mergeMode"))
    gCfg->mergeMode = constrain((int)doc["mergeMode"], 0, 2);
  if (doc.containsKey("artnetUniverse"))
    gCfg->artnetUniverse = constrain((int)doc["artnetUniverse"], 0, 15);
  if (doc.containsKey("rdmDmxStartAddress")) {
    gCfg->rdmDmxStartAddress = constrain((int)doc["rdmDmxStartAddress"], 1, 512);
    gCfg->channelOffset = gCfg->rdmDmxStartAddress - 1;
  }
  if (doc.containsKey("rdmDeviceLabel"))
    strncpy(gCfg->rdmDeviceLabel, doc["rdmDeviceLabel"] | "", 32);
  if (doc.containsKey("rdmManufacturerLabel"))
    strncpy(gCfg->rdmManufacturerLabel, doc["rdmManufacturerLabel"] | "", 32);
  if (doc.containsKey("rdmDeviceModelId"))
    gCfg->rdmDeviceModelId = (uint16_t)(int)doc["rdmDeviceModelId"];

  Storage_save(gCfg);
  server.send(200, F("application/json"), F("{\"ok\":true}"));
}

static void handleApiReset() {
  server.send(200, F("application/json"), F("{\"ok\":true}"));
  delay(400);
  WiFiManager wm;
  wm.resetSettings();
  ESP.restart();
}

static void handleApiFactory() {
  server.send(200, F("application/json"), F("{\"ok\":true}"));
  delay(400);
  Storage_reset();
  WiFiManager wm;
  wm.resetSettings();
  ESP.restart();
}

static void handleNotFound() {
  server.send(404, F("text/plain"), F("Not found"));
}

// ─── Inicialização ───────────────────────────────────────────
void WebServer_init(ProxyConfig* cfg, DMXState* dmxState, RDMState* rdmState) {
  gCfg = cfg;
  gDMX = dmxState;
  gRDM = rdmState;

  server.on("/",            HTTP_GET,  handleRoot);
  server.on("/config",      HTTP_GET,  handleConfigPage);
  server.on("/config",      HTTP_POST, handleConfigPost);
  server.on("/api/dmx",     HTTP_GET,  handleApiDmx);
  server.on("/api/status",  HTTP_GET,  handleApiStatus);
  server.on("/api/rdmlog",  HTTP_GET,  handleApiRDMLog);
  server.on("/api/reset",   HTTP_POST, handleApiReset);
  server.on("/api/factory", HTTP_POST, handleApiFactory);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println(F("[WebServer] Iniciado na porta 80"));
}

// Deve ser chamada no loop() principal
void WebServer_loop() {
  server.handleClient();
}
