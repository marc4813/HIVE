#include <WiFi.h> // ESP32 WiFi Library
#include <WebServer.h> // WebServer Library for ESP32
#include <WebSocketsClient.h> // WebSocket Client Library for WebSocket
#include <ArduinoJson.h> // Arduino JSON Library

const char* ssid = "Konstantin"; // Wifi SSID
const char* password = "hive1234"; // Wi-FI Password
const char* webSocketServerIP = "172.20.10.3"; // IP Address of server (Raspberry Pi)
const int webSocketPort = 80;
WebSocketsClient webSocket; // WebSocket client class instance
StaticJsonDocument<100> doc; // Allocate a static JSON document
const String WStypes[11] = {
  "WStype_ERROR",
  "WStype_DISCONNECTED",
  "WStype_CONNECTED",
  "WStype_TEXT",
  "WStype_BIN",
  "WStype_FRAGMENT_TEXT_START",
  "WStype_FRAGMENT_BIN_START",
  "WStype_FRAGMENT",
  "WStype_FRAGMENT_FIN",
  "WStype_PING",
  "WStype_PONG"
};

void setup() {
  WiFi.begin(ssid, password); 
  Serial.begin(115200);
  Serial.print("Establishing connection");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // Print local IP address
  // address, port, URL path
  webSocket.begin(webSocketServerIP, webSocketPort, "/");
  // WebSocket event handler
  webSocket.onEvent(webSocketEvent);
  // if connection failed, retry every 5s
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop(); // Keep the socket alive
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  Serial.print("Websocket Event Triggered. ");
  Serial.print("type: ");
  Serial.println(WStypes[type]);
  if (type == WStype_TEXT) {
    // Deserialize incoming JSON String
    DeserializationError error = deserializeJson(doc, payload); 
    if (error) { // Print error msg if incoming String is not JSON formated
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }
    const String test_string = doc["Test"];
    Serial.println(String(test_string));
    webSocket.sendTXT("OK"); // Send acknowledgement
  }
}
