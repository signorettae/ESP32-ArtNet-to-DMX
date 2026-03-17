/**
 * web_server.h — Servidor web usando ESP8266WebServer (síncrono)
 *
 * IMPORTANTE: NÃO incluir ESPAsyncWebServer aqui nem em nenhum
 * arquivo do projeto. ESPAsyncWebServer redefine HTTP_GET, HTTP_POST
 * etc. como enum WebRequestMethod, enquanto ESP8266WebServer (incluída
 * via WiFiManager) já os define como enum HTTPMethod — conflito fatal.
 *
 * Solução: usar APENAS ESP8266WebServer, que já vem no core ESP8266
 * e é a mesma biblioteca que WiFiManager usa internamente.
 *
 * WebServer_loop() deve ser chamada no loop() principal.
 */
#pragma once

// WiFiManager já inclui ESP8266WebServer — não precisamos incluir de novo,
// mas o pragma above garante que este header seja incluído após o .ino
// que inclui WiFiManager primeiro.
#include "config.h"

void WebServer_init(ProxyConfig* cfg, DMXState* dmxState, RDMState* rdmState);
void WebServer_loop();
