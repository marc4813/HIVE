#include <WiFi.h> // ESP32 WiFi Library
#include <WebServer.h> // WebServer Library for ESP32
#include <WebSocketsClient.h> // WebSocket Client Library for WebSocket
#include <ArduinoJson.h> // Arduino JSON Library

const char* ssid = "UCF_Guest"; // Wifi SSID
const char* password = ""; //Wi-FI Password
WebSocketsClient webSocket; // Websocket client class instance
StaticJsonDocument<100> doc; // Allocate a static JSON document

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
  webSocket.begin("10.37.61.149", 80, "/");
  // WebSocket event handler
  webSocket.onEvent(webSocketEvent);
  // if connection failed, retry every 5s
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop(); // Keep the socket alive
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  Serial.println("Websocket Event Triggered");
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
