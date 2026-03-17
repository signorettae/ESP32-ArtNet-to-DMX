/*
  ESP8266 DMX/RDM Proxy + Channel Remap + Art-Net + Web UI

  FEATURES:
  - Receives DMX from physical RS485 and/or Art-Net
  - Applies offset remap: OUT[1..512] comes from IN[(1+offset)..(512+offset)] with wrap/zero
  - Hybrid modes: DMX only, Art-Net only, Merge (HTP), Priority DMX, Priority Art-Net
  - WiFiManager captive portal + button reset
  - Async Web UI + WebSocket telemetry
  - RDM "virtual device" (simulated): responds as one RDM device, stores params,
    can pretend downstream DMX-only fixtures are manageable (basic RDM)

  WIRING (NodeMCU):
  - RS485 DI  -> TX0 (GPIO1)
  - RS485 RO  -> RX0 (GPIO3)
  - RS485 DE&/RE -> GPIO5 (D1)  (Direction pin)
  - WiFi config button -> GPIO0 (D3) to GND (INPUT_PULLUP)
  - LED (optional) -> LED_BUILTIN (GPIO2)

  LIBS:
  - WiFiManager (tzapu)
  - ESPAsyncWebServer (me-no-dev) + ESPAsyncTCP
  - ArduinoJson
  - EEPROM
  - ArtNet by hideakitai
  - DmxRdmLib_esp8266 by JonasArnold (or equivalent ESP8266 DMX/RDM lib)
*/

#include <Arduino.h>
#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFiManager.h>

#include <ArduinoJson.h>

#include <WiFiUdp.h>
'#include <ArtnetWiFi.h> // hideakitai ArtNet

// ===== DMX/RDM library (ESP8266) =====
// JonasArnold/DmxRdmLib_esp8266 (example include name may differ)
// Adjust includes according to the library installation.
#include <DmxRdmLib.h> // TODO (library API): confirm header name

// ===================== Config =====================
static const uint16_t EEPROM_SIZE = 1024;
static const uint16_t CFG_MAGIC = 0xBEEF;

enum InputMode : uint8_t {
  MODE_DMX_ONLY = 0,
  MODE_ARTNET_ONLY = 1,
  MODE_MERGE_HTP = 2,
  MODE_PRIORITY_DMX = 3,
  MODE_PRIORITY_ARTNET = 4
};

struct Config {
  uint16_t magic;
  uint16_t offset;         // 0..511 (virtual start channel = offset+1)
  uint8_t  mode;           // InputMode
  uint16_t artnetUniverse; // 0..32767
  bool     artnetEnabled;
  bool     dmxEnabled;

  // RDM virtual device params
  uint8_t uid_manufacturer[2]; // 2 bytes (ESTA manufacturer ID)
  uint8_t uid_device[4];       // 4 bytes device ID
  char    deviceLabel[32];     // RDM DEVICE_LABEL
  uint16_t dmxStartAddress;    // RDM DMX_START_ADDRESS (1..512)

  uint8_t reserved[64];
};

Config g_cfg;

// ===================== Globals =====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiManager wm;
WiFiUDP udp;
ArtnetWiFiReceiver artnet;

// DMX buffers
static uint8_t dmx_in_phys[512];
static uint8_t dmx_in_artnet[512];
static uint8_t dmx_out[512];

// State
static volatile bool got_phys_dmx = false;
static volatile bool got_artnet = false;
static uint32_t last_phys_ms = 0;
static uint32_t last_artnet_ms = 0;

// Timers
static uint32_t last_ws_push_ms = 0;
static uint32_t last_led_blink_ms = 0;

// Pins
static const uint8_t PIN_RS485_DIR = 5;   // D1 GPIO5
static const uint8_t PIN_BTN_WIFI  = 0;   // D3 GPIO0
static const uint8_t PIN_LED       = LED_BUILTIN;

// ====== DMX/RDM Objects ======
// TODO (library API): create objects per library docs.
// Example placeholders:
DmxRdm dmx; // TODO: adjust type/class

// ===================== Utility: Logging =====================
String g_log_ring[32];
uint8_t g_log_idx = 0;

void logLine(const String& s) {
  g_log_ring[g_log_idx] = s;
  g_log_idx = (g_log_idx + 1) % 32;
  Serial.println(s);
}

// ===================== EEPROM Config =====================
void cfgDefaults() {
  memset(&g_cfg, 0, sizeof(g_cfg));
  g_cfg.magic = CFG_MAGIC;
  g_cfg.offset = 0;
  g_cfg.mode = MODE_PRIORITY_DMX;
  g_cfg.artnetUniverse = 0;
  g_cfg.artnetEnabled = true;
  g_cfg.dmxEnabled = true;

  // Example manufacturer ID (0x7A70 is a placeholder - use a real ESTA ID if you have one)
  g_cfg.uid_manufacturer[0] = 0x7A;
  g_cfg.uid_manufacturer[1] = 0x70;
  g_cfg.uid_device[0] = 0x00;
  g_cfg.uid_device[1] = 0x00;
  g_cfg.uid_device[2] = 0x00;
  g_cfg.uid_device[3] = 0x01;

  strncpy(g_cfg.deviceLabel, "ESP8266 RDM Proxy", sizeof(g_cfg.deviceLabel) - 1);
  g_cfg.dmxStartAddress = 1;
}

void cfgLoad() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, g_cfg);
  if (g_cfg.magic != CFG_MAGIC) {
    cfgDefaults();
    EEPROM.put(0, g_cfg);
    EEPROM.commit();
  }
}

void cfgSave() {
  g_cfg.magic = CFG_MAGIC;
  EEPROM.put(0, g_cfg);
  EEPROM.commit();
}

// ===================== DMX Remap =====================
inline uint8_t getInMerged(uint16_t ch1to512) {
  // ch1to512 in 1..512
  uint16_t idx = ch1to512 - 1;

  const uint8_t a = dmx_in_phys[idx];
  const uint8_t b = dmx_in_artnet[idx];

  switch (g_cfg.mode) {
    case MODE_DMX_ONLY:
      return a;
    case MODE_ARTNET_ONLY:
      return b;
    case MODE_MERGE_HTP:
      return (a > b) ? a : b; // HTP
    case MODE_PRIORITY_DMX:
      // if physical is "fresh" use it, else fallback to artnet
      if (millis() - last_phys_ms < 1000) return a;
      return b;
    case MODE_PRIORITY_ARTNET:
      if (millis() - last_artnet_ms < 1000) return b;
      return a;
    default:
      return a;
  }
}

void applyOffsetRemap() {
  // offset means: OUT[ch] = IN[ch + offset]
  // Example: offset=130 => OUT[1] = IN[131]
  uint16_t offset = g_cfg.offset % 512;

  for (uint16_t outCh = 1; outCh <= 512; outCh++) {
    uint16_t inCh = outCh + offset; // 1-based
    if (inCh > 512) {
      // beyond range -> could wrap or zero; choose zero for safety.
      // If you prefer wrap: inCh = ((inCh - 1) % 512) + 1;
      dmx_out[outCh - 1] = 0;
    } else {
      dmx_out[outCh - 1] = getInMerged(inCh);
    }
  }
}

// ===================== DMX Physical IO =====================
void dmxInit() {
  pinMode(PIN_RS485_DIR, OUTPUT);
  digitalWrite(PIN_RS485_DIR, LOW); // start receiving

  memset(dmx_in_phys, 0, sizeof(dmx_in_phys));
  memset(dmx_out, 0, sizeof(dmx_out));

  // TODO (library API):
  // - Configure DMX/RDM library to use Serial (UART0) and direction pin.
  // - Set mode to "receive + transmit" (half duplex) as supported.
  // - Setup callbacks for DMX frame received and RDM packets.

  // Placeholder:
  // dmx.begin(Serial, PIN_RS485_DIR);
  // dmx.setReceiveCallback(onDmxFrame);
  // dmx.setRdmCallback(onRdmPacket);

  logLine("[DMX] init done (check library API wiring).");
}

// Called often
void dmxPoll() {
  if (!g_cfg.dmxEnabled) return;

  // TODO (library API):
  // - poll receiver state
  // - if a full DMX frame arrived, copy to dmx_in_phys[512]
  // Example:
  // if (dmx.available()) { dmx.readFrame(dmx_in_phys, 512); ... }

  // Placeholder “no-op”
}

// Transmit dmx_out to bus
void dmxSendOut() {
  // TODO (library API):
  // - switch to transmit, write 512 slots, return to receive quickly
  // Example:
  // dmx.writeFrame(dmx_out, 512);

  // Placeholder “no-op”
}

// ===================== RDM Virtual Device (Simulated) =====================
//
// Minimal: Respond as ONE RDM device with UID and some PIDs.
// You can extend to emulate multiple "virtual fixtures" by creating a UID table and per-UID state.
//
// PIDs of interest (E1.20):
// - DISC_UNIQUE_BRANCH, MUTE, UN_MUTE (discovery)
// - DEVICE_INFO, DEVICE_LABEL
// - DMX_START_ADDRESS (GET/SET)
//
// NOTE: Implementing full discovery timing is non-trivial. The library may handle lower-level encoding.
// If your library doesn't support RDM well, you can still simulate at a higher layer and skip physical RDM.
// (Many controllers require proper discovery on the wire.)

void rdmInitVirtual() {
  // TODO (library API):
  // dmx.rdmSetUid(g_cfg.uid_manufacturer, g_cfg.uid_device);
  // dmx.rdmSetDeviceLabel(g_cfg.deviceLabel);
  // dmx.rdmEnable(true);

  logLine("[RDM] virtual device configured.");
}

void rdmHandlePacket(/* params depend on library */) {
  // TODO: parse packet and respond.
  // When a controller asks:
  // - GET DEVICE_INFO => reply with model, footprint, start addr etc
  // - GET/SET DMX_START_ADDRESS => update g_cfg.dmxStartAddress and save.
  // - GET/SET DEVICE_LABEL => update g_cfg.deviceLabel and save.
  //
  // Also: log requests for the Web UI ring buffer.
}

// ===================== Art-Net =====================
void artnetInit() {
  memset(dmx_in_artnet, 0, sizeof(dmx_in_artnet));

  // hideakitai ArtNet Receiver usage:
  artnet.begin(udp);

  artnet.subscribe(g_cfg.artnetUniverse, [&](const uint8_t* data, uint16_t size, const art_net::PacketMetadata& metadata) {
    // DMX payload is up to 512 bytes
    uint16_t n = min<uint16_t>(size, 512);
    memcpy(dmx_in_artnet, data, n);
    if (n < 512) memset(dmx_in_artnet + n, 0, 512 - n);
    got_artnet = true;
    last_artnet_ms = millis();
  });

  logLine("[ArtNet] receiver ready.");
}

void artnetPoll() {
  if (!g_cfg.artnetEnabled) return;
  artnet.parse(); // pump UDP packets
}

// ===================== WiFi (WiFiManager) =====================
void wifiStart() {
  WiFi.mode(WIFI_STA);

  wm.setConfigPortalTimeout(180);
  wm.setConnectTimeout(20);

  // Auto connect; if fails -> captive portal AP
  bool ok = wm.autoConnect("ESP-RDM-PROXY");
  if (ok) {
    logLine("[WiFi] Connected: " + WiFi.SSID() + " IP=" + WiFi.localIP().toString());
  } else {
    logLine("[WiFi] Portal timeout / not connected (still running).");
  }
}

// Hold button 3s to reset WiFi settings
void wifiButtonTask() {
  static uint32_t pressedAt = 0;
  bool pressed = (digitalRead(PIN_BTN_WIFI) == LOW);

  if (pressed && pressedAt == 0) pressedAt = millis();
  if (!pressed && pressedAt != 0) pressedAt = 0;

  if (pressedAt != 0 && (millis() - pressedAt) > 3000) {
    logLine("[WiFi] Reset settings + reboot");
    wm.resetSettings();
    delay(200);
    ESP.restart();
  }
}

// ===================== Web UI =====================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pt-br">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>ESP8266 RDM Proxy</title>
<style>
  body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:0;background:#0b0f14;color:#e8eef6}
  header{padding:14px 16px;background:#111824;position:sticky;top:0}
  .wrap{padding:14px 16px;display:grid;gap:12px;grid-template-columns:1fr}
  .card{background:#111824;border:1px solid #1c2940;border-radius:12px;padding:12px}
  .row{display:flex;gap:10px;flex-wrap:wrap}
  .pill{padding:6px 10px;border-radius:999px;background:#0b1220;border:1px solid #1c2940;font-size:12px}
  .grid2{display:grid;gap:12px;grid-template-columns:1fr}
  @media(min-width:900px){.grid2{grid-template-columns:1fr 1fr}}
  table{width:100%;border-collapse:collapse;font-size:12px}
  th,td{border-bottom:1px solid #1c2940;padding:6px 8px;text-align:left}
  input,select,button{background:#0b1220;color:#e8eef6;border:1px solid #1c2940;border-radius:10px;padding:10px}
  button{cursor:pointer}
  .log{white-space:pre-wrap;font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12px;max-height:220px;overflow:auto}
</style>
</head>
<body>
<header>
  <div class="row" style="align-items:center;justify-content:space-between">
    <div><b>ESP8266 RDM Proxy</b> <span class="pill" id="status">offline</span></div>
    <div class="row">
      <a class="pill" href="/config">Config</a>
    </div>
  </div>
</header>

<div class="wrap">
  <div class="card">
    <div class="row">
      <span class="pill" id="ip">IP: -</span>
      <span class="pill" id="mode">Mode: -</span>
      <span class="pill" id="offset">Offset: -</span>
      <span class="pill" id="u">Universe: -</span>
    </div>
  </div>

  <div class="grid2">
    <div class="card">
      <h3 style="margin:0 0 10px">DMX IN (amostra)</h3>
      <table id="tin"><thead><tr><th>Ch</th><th>Val</th></tr></thead><tbody></tbody></table>
    </div>
    <div class="card">
      <h3 style="margin:0 0 10px">DMX OUT (amostra)</h3>
      <table id="tout"><thead><tr><th>Ch</th><th>Val</th></tr></thead><tbody></tbody></table>
    </div>
  </div>

  <div class="card">
    <h3 style="margin:0 0 10px">Log</h3>
    <div class="log" id="log"></div>
  </div>
</div>

<script>
let ws;
function el(id){return document.getElementById(id)}
function fillTable(tblId, arr){
  const tb = document.querySelector(`#${tblId} tbody`);
  tb.innerHTML = "";
  // show channels 1..64 as sample
  for(let i=0;i<64;i++){
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${i+1}</td><td>${arr[i] ?? 0}</td>`;
    tb.appendChild(tr);
  }
}
function connect(){
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen=()=>{el("status").textContent="online";}
  ws.onclose=()=>{el("status").textContent="offline"; setTimeout(connect,1200);}
  ws.onmessage=(ev)=>{
    const msg = JSON.parse(ev.data);
    if(msg.type==="telemetry"){
      el("ip").textContent="IP: "+msg.ip;
      el("mode").textContent="Mode: "+msg.mode;
      el("offset").textContent="Offset: "+msg.offset;
      el("u").textContent="Universe: "+msg.universe;
      fillTable("tin", msg.in);
      fillTable("tout", msg.out);
      el("log").textContent = msg.log.join("\n");
    }
  };
}
connect();
</script>
</body>
</html>
)HTML";

String modeToStr(uint8_t m) {
  switch (m) {
    case MODE_DMX_ONLY: return "DMX_ONLY";
    case MODE_ARTNET_ONLY: return "ARTNET_ONLY";
    case MODE_MERGE_HTP: return "MERGE_HTP";
    case MODE_PRIORITY_DMX: return "PRIORITY_DMX";
    case MODE_PRIORITY_ARTNET: return "PRIORITY_ARTNET";
    default: return "UNKNOWN";
  }
}

void wsSendTelemetry() {
  // push at ~10Hz
  if (millis() - last_ws_push_ms < 100) return;
  last_ws_push_ms = millis();

  StaticJsonDocument<2048> doc;
  doc["type"] = "telemetry";
  doc["ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : String("-");
  doc["mode"] = modeToStr(g_cfg.mode);
  doc["offset"] = g_cfg.offset;
  doc["universe"] = g_cfg.artnetUniverse;

  JsonArray ain = doc.createNestedArray("in");
  JsonArray aout = doc.createNestedArray("out");

  // sample first 64 channels
  for (int i = 0; i < 64; i++) {
    ain.add(getInMerged(i + 1));
    aout.add(dmx_out[i]);
  }

  JsonArray alog = doc.createNestedArray("log");
  // send ring buffer in order (oldest->newest)
  for (int i = 0; i < 32; i++) {
    int idx = (g_log_idx + i) % 32;
    if (g_log_ring[idx].length()) alog.add(g_log_ring[idx]);
  }

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

const char CONFIG_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pt-br">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Config - ESP8266 RDM Proxy</title>
<style>
  body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:0;background:#0b0f14;color:#e8eef6}
  header{padding:14px 16px;background:#111824}
  .wrap{padding:14px 16px;max-width:980px;margin:0 auto}
  .card{background:#111824;border:1px solid #1c2940;border-radius:12px;padding:12px;margin:12px 0}
  label{display:block;margin:10px 0 6px;font-size:12px;color:#b9c7dd}
  input,select,button{width:100%;background:#0b1220;color:#e8eef6;border:1px solid #1c2940;border-radius:10px;padding:10px}
  button{cursor:pointer;margin-top:10px}
  .row{display:grid;grid-template-columns:1fr;gap:12px}
  @media(min-width:800px){.row{grid-template-columns:1fr 1fr}}
  .hint{font-size:12px;color:#b9c7dd}
</style>
</head>
<body>
<header><b>Config</b> <span class="hint">/config</span></header>
<div class="wrap">
  <div class="card">
    <form id="f">
      <div class="row">
        <div>
          <label>Offset (0..511)</label>
          <input name="offset" type="number" min="0" max="511" value="0"/>
          <div class="hint">Ex: offset=130 => OUT[1]=IN[131]</div>
        </div>
        <div>
          <label>Modo</label>
          <select name="mode">
            <option value="0">DMX only</option>
            <option value="1">Art-Net only</option>
            <option value="2">Merge (HTP)</option>
            <option value="3">Priority DMX (fallback Art-Net)</option>
            <option value="4">Priority Art-Net (fallback DMX)</option>
          </select>
        </div>
        <div>
          <label>Art-Net Universe</label>
          <input name="universe" type="number" min="0" max="32767" value="0"/>
        </div>
        <div>
          <label>DMX físico</label>
          <select name="dmxEnabled">
            <option value="1">Habilitado</option>
            <option value="0">Desabilitado</option>
          </select>
        </div>
        <div>
          <label>Art-Net</label>
          <select name="artnetEnabled">
            <option value="1">Habilitado</option>
            <option value="0">Desabilitado</option>
          </select>
        </div>
        <div>
          <label>RDM Device Label</label>
          <input name="label" maxlength="31" value="ESP8266 RDM Proxy"/>
        </div>
        <div>
          <label>RDM DMX Start Address (1..512)</label>
          <input name="start" type="number" min="1" max="512" value="1"/>
          <div class="hint">Armazenado internamente (para simular SET via RDM).</div>
        </div>
      </div>

      <button type="submit">Salvar</button>
    </form>

    <button onclick="fetch('/wifi/reset',{method:'POST'}).then(()=>alert('Reset Wi-Fi: reiniciando...'))">
      Reset Wi-Fi (portal)
    </button>
    <button onclick="fetch('/reboot',{method:'POST'}).then(()=>alert('Reiniciando...'))">
      Reboot
    </button>
  </div>

  <div class="card">
    <a href="/" style="color:#9ecbff">← Voltar</a>
  </div>
</div>

<script>
async function load(){
  const r = await fetch('/api/config');
  const c = await r.json();
  const f = document.getElementById('f');
  f.offset.value = c.offset;
  f.mode.value = c.mode;
  f.universe.value = c.universe;
  f.dmxEnabled.value = c.dmxEnabled ? 1 : 0;
  f.artnetEnabled.value = c.artnetEnabled ? 1 : 0;
  f.label.value = c.label;
  f.start.value = c.start;
}
document.getElementById('f').addEventListener('submit', async (e)=>{
  e.preventDefault();
  const fd = new FormData(e.target);
  const obj = Object.fromEntries(fd.entries());
  obj.offset = Number(obj.offset);
  obj.mode = Number(obj.mode);
  obj.universe = Number(obj.universe);
  obj.dmxEnabled = obj.dmxEnabled === "1";
  obj.artnetEnabled = obj.artnetEnabled === "1";
  obj.start = Number(obj.start);
  const r = await fetch('/api/config', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(obj)});
  if(r.ok) alert("Salvo!");
  else alert("Erro ao salvar");
});
load();
</script>
</body>
</html>
)HTML";

void webInit() {
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      logLine("[WS] client connected");
    } else if (type == WS_EVT_DISCONNECT) {
      logLine("[WS] client disconnected");
    }
  });
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", CONFIG_HTML);
  });

  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *req) {
    StaticJsonDocument<512> doc;
    doc["offset"] = g_cfg.offset;
    doc["mode"] = g_cfg.mode;
    doc["universe"] = g_cfg.artnetUniverse;
    doc["dmxEnabled"] = g_cfg.dmxEnabled;
    doc["artnetEnabled"] = g_cfg.artnetEnabled;
    doc["label"] = g_cfg.deviceLabel;
    doc["start"] = g_cfg.dmxStartAddress;

    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *req) {},
    NULL,
    [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<512> doc;
      DeserializationError err = deserializeJson(doc, data, len);
      if (err) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return;
      }

      g_cfg.offset = (uint16_t) constrain((int)doc["offset"], 0, 511);
      g_cfg.mode = (uint8_t) constrain((int)doc["mode"], 0, 4);
      g_cfg.artnetUniverse = (uint16_t) constrain((int)doc["universe"], 0, 32767);
      g_cfg.dmxEnabled = doc["dmxEnabled"] | true;
      g_cfg.artnetEnabled = doc["artnetEnabled"] | true;

      const char* label = doc["label"] | "ESP8266 RDM Proxy";
      memset(g_cfg.deviceLabel, 0, sizeof(g_cfg.deviceLabel));
      strncpy(g_cfg.deviceLabel, label, sizeof(g_cfg.deviceLabel)-1);

      g_cfg.dmxStartAddress = (uint16_t) constrain((int)doc["start"], 1, 512);

      cfgSave();
      logLine("[CFG] saved: offset=" + String(g_cfg.offset) +
              " mode=" + modeToStr(g_cfg.mode) +
              " universe=" + String(g_cfg.artnetUniverse));

      // Re-init ArtNet subscription if universe changed:
      artnetInit();
      // Re-init RDM virtual device label/start:
      rdmInitVirtual();

      req->send(200, "application/json", "{\"ok\":true}");
    }
  );

  server.on("/wifi/reset", HTTP_POST, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", "{\"ok\":true}");
    logLine("[WiFi] reset requested via web");
    wm.resetSettings();
    delay(200);
    ESP.restart();
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", "{\"ok\":true}");
    logLine("[SYS] reboot requested via web");
    delay(200);
    ESP.restart();
  });

  server.begin();
  logLine("[WEB] server started.");
}

// ===================== Status LED =====================
void ledTask() {
  // blink when not connected, steady when connected
  uint32_t now = millis();
  if (WiFi.isConnected()) {
    digitalWrite(PIN_LED, LOW); // many boards: LOW = ON
  } else {
    if (now - last_led_blink_ms > 250) {
      last_led_blink_ms = now;
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
  }
}

// ===================== Setup/Loop =====================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();

  pinMode(PIN_BTN_WIFI, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // OFF on many boards

  cfgLoad();
  logLine("[BOOT] ESP8266 RDM Proxy starting...");

  wifiStart();
  webInit();

  dmxInit();
  rdmInitVirtual();
  artnetInit();

  logLine("[BOOT] ready.");
}

void loop() {
  wifiButtonTask();
  ledTask();

  // Input pumps
  dmxPoll();
  artnetPoll();

  // Compute output
  applyOffsetRemap();

  // Send to DMX bus
  dmxSendOut();

  // Websocket telemetry
  ws.cleanupClients();
  wsSendTelemetry();
}
