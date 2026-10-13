#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ---- Access point settings ----
const char* AP_SSID = "WallBot";   // network name you will see on your phone/laptop
const char* AP_PASS = "wallbot123";    // MUST be at least 8 characters
const int   AP_CHANNEL = 6;
const int   AP_MAX_CLIENTS = 4;

// ---- OTA settings ----
const char* OTA_HOSTNAME = "wallbot";
const char* OTA_PASS     = "ota12345";

// void stopMotors() {
//   // TODO: put your ESC outputs to minimum throttle here.
//   // Called before an OTA update starts so the props can't spin during flashing.
// }

void setup() {
  Serial.begin(115200);

  // Start the ESP32 as its own Wi-Fi network
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  Serial.print("AP started. IP: ");
  Serial.println(WiFi.softAPIP());   // normally 192.168.4.1

  // ArduinoOTA
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASS);

  ArduinoOTA.onStart([]() {
    // stopMotors();
    Serial.println("OTA start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA end");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", progress * 100 / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error[%u]\n", error);
  });

  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

void loop() {
  ArduinoOTA.handle();   // must be called often; never block loop() with long delay()

  // Your robot code goes here (WebSocket, ESC/servo control, failsafe...)
}
