/*

  ESP32-ArtNet-to-DMX

  This sketch allows you to receive 2 ArtNet universes from a WiFi network
  and to send the received data in 2 physical DMX universes.

  Based on the DMX_write sketch by Mitch Weisbord (https://github.com/someweisguy/esp_dmx)
  and on ArtNetWifiNeoPixel sketch by rstephan (https://github.com/rstephan/ArtnetWifi)

  2023 - Emanuele Signoretta

*/

#include <Arduino.h>
#include <esp_dmx.h>
#include "ArtnetWifi.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//#define DHCP_DISABLED

#ifdef DHCP_DISABLED

IPAddress local_IP(192, 168, 1, 154);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 1, 1);  //optional
IPAddress secondaryDNS(1, 1, 1, 1);    //optional

#endif

WiFiUDP UdpSend;
ArtnetWifi artnet;

const char* ssid = "MyArtNetNetwork";
const char* password = "MyArtNetNetwork";

/* First, lets define the hardware pins that we are using with our ESP32. We
  need to define which pin is transmitting data and which pin is receiving data.
  DMX circuits also often need to be told when we are transmitting and when we
  are receiving data. We can do this by defining an enable pin. */
int transmitPinA = 17;
int receivePinA = 16;  //Not connected
int enablePinA = 4;

int transmitPinB = 21;
int receivePinB = 16;  //Not connected
int enablePinB = 19;


/* Make sure to double-check that these pins are compatible with your ESP32!
  Some ESP32s, such as the ESP32-WROVER series, do not allow you to read or
  write data on pins 16 or 17, so it's always good to read the manuals. */

/* Next, lets decide which DMX port to use. The ESP32 has either 2 or 3 ports.
  Port 0 is typically used to transmit serial data back to your Serial Monitor,
  so we shouldn't use that port. Lets use port 1! */
dmx_port_t dmxPortA = 1;
dmx_port_t dmxPortB = 2;

/* Now we want somewhere to store our DMX data. Since a single packet of DMX
  data can be up to 513 bytes long, we want our array to be at least that long.
  This library knows that the max DMX packet size is 513, so we can fill in the
  array size with `DMX_PACKET_SIZE`. */
byte dataA[DMX_PACKET_SIZE];
byte dataB[DMX_PACKET_SIZE];

//ArtNet settings

const int startUniverse = 0;  // CHANGE FOR YOUR SETUP most software this is 1, some software send out artnet first universe as 0.
const int maxUniverses = 2;
const int numberOfChannels = 1024;
bool universesReceived[maxUniverses];
bool sendFrame = 1;
int previousDataLength = 0;

void setup() {
  /* Start the serial connection back to the computer so that we can log
    messages to the Serial Monitor. Lets set the baud rate to 115200. */
  Serial.begin(115200);

  // Setup wifi
  WiFi.mode(WIFI_STA);

#ifdef DHCP_DISABLED

  //Comment to use DHCP instead of static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

#endif

  WiFi.begin(ssid, password);
  delay(1000);
  Serial.println("\nConnecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("ESP32-ArtNet-to-DMX-Converter");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else  // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  artnet.setArtDmxCallback(onArtNetFrame);

  artnet.begin("ESP32-ArtNet-to-DMX-Converter");

  /* Set the DMX hardware pins to the pins that we want to use. */
  dmx_set_pin(dmxPortA, transmitPinA, receivePinA, enablePinA);
  dmx_set_pin(dmxPortB, transmitPinB, receivePinB, enablePinB);


  /* Now we can install the DMX driver! We'll tell it which DMX port to use and
    which interrupt priority it should have. If you aren't sure which interrupt
    priority to use, you can use the macro `DMX_DEFAULT_INTR_FLAG` to set the
    interrupt to its default settings.*/
  dmx_driver_install(dmxPortA, DMX_DEFAULT_INTR_FLAGS);
  dmx_driver_install(dmxPortB, DMX_DEFAULT_INTR_FLAGS);
}

void onArtNetFrame(uint16_t universe, uint16_t numberOfChannels, uint8_t sequence, uint8_t* dmxData) {


  sendFrame = 1;

  // Store which universe has got in
  if ((universe - startUniverse) < maxUniverses)
    universesReceived[universe - startUniverse] = 1;

  for (int i = 0; i < maxUniverses; i++) {
    if (universesReceived[i] == 0) {
      //Serial.println("Broke");
      sendFrame = 0;
      break;
    }
  }
  // read universe and put into the right array of data
  for (int i = 0; i < numberOfChannels; i++) {
    if (universe == startUniverse)
      dataA[i + 1] = dmxData[i];
    else if (universe == startUniverse + 1)
      dataB[i + 1] = dmxData[i];
  }
  previousDataLength = numberOfChannels;

  dmx_write(dmxPortA, dataA, DMX_MAX_PACKET_SIZE);
  dmx_write(dmxPortB, dataB, DMX_MAX_PACKET_SIZE);
  dmx_send(dmxPortA, DMX_PACKET_SIZE);
  dmx_send(dmxPortB, DMX_PACKET_SIZE);
  dmx_wait_sent(dmxPortA, DMX_TIMEOUT_TICK);
  dmx_wait_sent(dmxPortB, DMX_TIMEOUT_TICK);
  // Reset universeReceived to 0
  memset(universesReceived, 0, maxUniverses);
}

void loop() {

  if ((WiFi.status() == WL_CONNECTED)) {
    artnet.read();
  }
}
