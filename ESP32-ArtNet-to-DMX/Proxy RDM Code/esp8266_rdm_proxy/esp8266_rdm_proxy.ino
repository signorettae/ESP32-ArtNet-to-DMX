/**
 * ESP8266 RDM Proxy Firmware v1.1
 * ============================================================
 * CORREÇÃO: removido ESPAsyncWebServer que conflitava com WiFiManager.
 * Agora usa apenas ESP8266WebServer (inclusa no core ESP8266).
 *
 * PIN CONNECTIONS (NodeMCU / Wemos D1 Mini + MAX485):
 * ─────────────────────────────────────────────────────
 *  MAX485 DI  (TX)  │  GPIO1  (TX)   │  Dados DMX Saída
 *  MAX485 RO  (RX)  │  GPIO3  (RX)   │  Dados DMX Entrada
 *  MAX485 DE+RE     │  GPIO5  (D1)   │  Controle de direção
 *  Botão Reset      │  GPIO0  (D3)   │  Resetar WiFi (segurar 3s)
 *  LED Wi-Fi        │  GPIO2  (D4)   │  Status (built-in, ativo LOW)
 *  LED DMX          │  GPIO4  (D2)   │  Atividade DMX
 *  LED Erro         │  GPIO14 (D5)   │  Erros + RDM Identify
 * ─────────────────────────────────────────────────────
 *
 * BIBLIOTECAS NECESSÁRIAS (Gerenciador de Bibliotecas):
 *   - WiFiManager   (tzapu)
 *   - ArduinoJson   (bblanchon) v6.x
 *   ESP8266WebServer, ArduinoOTA, Ticker → já incluídas no core ESP8266
 */

// ─── ATENÇÃO: WiFiManager DEVE ser incluído ANTES de qualquer ───
// ─── coisa que também inclua ESP8266WebServer, para garantir  ───
// ─── que apenas UMA declaração dos enums HTTP_GET etc. exista ───
#include <WiFiManager.h>        // inclui ESP8266WebServer internamente
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>   // já inclusa via WiFiManager, mas explicitamos
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Ticker.h>

// Módulos do projeto (não incluem ESPAsyncWebServer)
#include "config.h"
#include "storage.h"
#include "dmx_handler.h"
#include "rdm_handler.h"
#include "artnet_handler.h"
#include "web_server.h"

// ─── Estado Global ───────────────────────────────────────────
ProxyConfig gConfig;
DMXState    gDMXState;
RDMState    gRDMState;

Ticker      statusTicker;
bool        wifiConnected = false;

// ─── Protótipos ──────────────────────────────────────────────
void setupWiFi();
void setupOTA();
void handleResetButton();
void statusBlink();

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_RS485_DIR, OUTPUT);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_WIFI,  OUTPUT);
  pinMode(PIN_LED_DMX,   OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);

  // LEDs OFF (GPIO2 built-in é ativo LOW)
  digitalWrite(PIN_LED_WIFI,  HIGH);
  digitalWrite(PIN_LED_DMX,   HIGH);
  digitalWrite(PIN_LED_ERROR, HIGH);

  Serial.println(F("\n=== ESP8266 RDM Proxy v1.1 ==="));

  Storage_init();
  Storage_load(&gConfig);
  Serial.printf("[Config] offset=%d, mode=%d\n",
                gConfig.channelOffset, gConfig.inputMode);

  RDM_init(&gRDMState, &gConfig);
  Serial.printf("[RDM] UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
    gRDMState.uid[0], gRDMState.uid[1], gRDMState.uid[2],
    gRDMState.uid[3], gRDMState.uid[4], gRDMState.uid[5]);

  DMX_init(&gDMXState);

  setupWiFi();

  if (wifiConnected) {
    ArtNet_init(&gConfig, &gDMXState);
    WebServer_init(&gConfig, &gDMXState, &gRDMState);
    setupOTA();
    Serial.printf("[Web] http://%s\n", WiFi.localIP().toString().c_str());
  }

  DMX_begin(&gDMXState, &gConfig);
  statusTicker.attach(1.0f, statusBlink);
  Serial.println(F("[Boot] OK"));
}

// ─── Loop ────────────────────────────────────────────────────
void loop() {
  handleResetButton();

  if (gConfig.inputMode == INPUT_DMX || gConfig.inputMode == INPUT_MERGE) {
    DMX_receive(&gDMXState);
  }

  RDM_process(&gRDMState, &gDMXState, &gConfig);

  if (wifiConnected) {
    ArtNet_loop();
    WebServer_loop();        // handleClient() do servidor síncrono
    ArduinoOTA.handle();
  }

  DMX_applyOffsetAndTransmit(&gDMXState, &gConfig);

  yield();
}

// ─── WiFi Setup ──────────────────────────────────────────────
void setupWiFi() {
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);

  WiFiManagerParameter p_offset("offset", "Channel Offset (0-511)",
                                 String(gConfig.channelOffset).c_str(), 4);
  wm.addParameter(&p_offset);

  Serial.println(F("[WiFi] Conectando..."));
  digitalWrite(PIN_LED_WIFI, LOW);

  bool ok = wm.autoConnect("RDM-Proxy-AP", "rdmproxy");

  if (ok) {
    wifiConnected = true;
    digitalWrite(PIN_LED_WIFI, HIGH);
    Serial.printf("[WiFi] Conectado: %s\n", WiFi.localIP().toString().c_str());

    int newOffset = String(p_offset.getValue()).toInt();
    if (newOffset != gConfig.channelOffset) {
      gConfig.channelOffset = constrain(newOffset, 0, 511);
      Storage_save(&gConfig);
    }
  } else {
    Serial.println(F("[WiFi] Não configurado. Modo offline."));
    digitalWrite(PIN_LED_ERROR, LOW);
  }
}

// ─── OTA ─────────────────────────────────────────────────────
void setupOTA() {
  ArduinoOTA.setHostname("rdm-proxy");
  ArduinoOTA.setPassword("rdmproxy2024");
  ArduinoOTA.onStart([]() { DMX_stop(); });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("[OTA] Erro %u\n", e);
    digitalWrite(PIN_LED_ERROR, LOW);
  });
  ArduinoOTA.begin();
}

// ─── Botão de Reset ──────────────────────────────────────────
static unsigned long btnPressStart = 0;
void handleResetButton() {
  if (digitalRead(PIN_RESET_BTN) == LOW) {
    if (btnPressStart == 0) btnPressStart = millis();
    if (millis() - btnPressStart > 3000) {
      Serial.println(F("[Reset] Limpando WiFi..."));
      WiFiManager wm;
      wm.resetSettings();
      delay(300);
      ESP.restart();
    }
  } else {
    btnPressStart = 0;
  }
}

// ─── LED de Status ───────────────────────────────────────────
void statusBlink() {
  if (wifiConnected) {
    digitalWrite(PIN_LED_WIFI, !digitalRead(PIN_LED_WIFI));
  }
}
