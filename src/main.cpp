#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Replace with your GPS module RX and TX pins
#define RXpin A0
#define TXpin A0

// Replace with your SSID and password
const char* ssid = "";
const char* password = "";

// Replace with your UDP server address and port
const char* udpAddress = "";
const int udpPort = 0;

WiFiUDP udp;

SoftwareSerial GPS(RXpin, TXpin);
TinyGPSPlus gps;

void processGPSData() {
  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();

    StaticJsonDocument<200> doc;

    doc["device"] = "rover";
    doc["time"] = gps.time.value();
    doc["lat_error"] = String(lat, 8);
    doc["lon_error"] = String(lon, 8);

    String jsonString;
    serializeJson(doc, jsonString);

    udp.beginPacket(udpAddress, udpPort);
    udp.print(jsonString);
    udp.endPacket();

    Serial.print(jsonString);
  }
}

void setup() {
  GPS.begin(115200);
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to WiFi");

  udp.begin(udpPort);
}

void loop() {
  while (GPS.available()) {
    gps.encode(GPS.read());
  }

  processGPSData();
}