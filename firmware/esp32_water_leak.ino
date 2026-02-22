#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <Firebase_ESP_Client.h>

// ================= 1. CONFIGURATION =================
struct Config {
  const char* firmwareVersion = "1.2";
  // WiFi
  const char* ssid = "DripCheck";
  const char* wifiPassword = "123456789";
  // Hardware
  const uint8_t pinS1 = 27;
  const uint8_t pinS2 = 26;
  const float calibrationFactor = 7.5; 
  const float leakThreshold = 0.5;
  // Network
  const uint16_t discoveryPort = 41234; 
  // Firebase
  const char* apiKey = "AIzaSyBfWWsLoOLjFSN6ulNpOcSDIP7b3lasles"; 
  const char* dbUrl = "https://dripcheckiot-default-rtdb.asia-southeast1.firebasedatabase.app"; 
  const char* userEmail = "christianviente29@gmail.com";
  const char* userPass = "DripCheck";
  const char* dbPath = "/FlowLogs"; 
  // OTA
  const char* otaUrl = "https://raw.githubusercontent.com/Dhirk07/proto-dripwater/refs/heads/main/version.json"; 
};

Config config;

// ================= 2. GLOBALS & OBJECTS =================
WebSocketsServer webSocket(81);
WiFiUDP Udp;

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig fbConfig;

unsigned long lastMeasureTime = 0;
unsigned long lastDbUpdate = 0;
unsigned long lastDiscovery = 0;

// ================= 3. CLASS: FLOW SENSOR =================
class FlowSensor {
  private:
    uint8_t _pin;
    volatile uint32_t _pulseCount;
    float _flowRate;
    float _currentVolume;
    
  public:
    FlowSensor(uint8_t pin) : _pin(pin), _pulseCount(0), _flowRate(0.0), _currentVolume(0.0) {}

    void begin(void (*isr)()) { 
      pinMode(_pin, INPUT_PULLUP); 
      attachInterrupt(digitalPinToInterrupt(_pin), isr, FALLING);
    }
    
    void IRAM_ATTR increment() { 
      _pulseCount++; 
    }

    void calculateFlow(unsigned long durationMs) {
      if (durationMs == 0) return;
      
      // Safely grab the pulse count by briefly pausing interrupts
      noInterrupts();
      uint32_t pulses = _pulseCount;
      _pulseCount = 0; 
      interrupts();

      // Calculate Rate (L/min) and Volume (Liters)
      _flowRate = ((1000.0 / durationMs) * pulses) / config.calibrationFactor;
      _currentVolume = _flowRate * (durationMs / 60000.0);
    }

    float getFlowRate() { return _flowRate; }
    float getTotalVolume() { return _currentVolume; }
};

FlowSensor sensor1(config.pinS1);
FlowSensor sensor2(config.pinS2);

void IRAM_ATTR isrS1() { sensor1.increment(); }
void IRAM_ATTR isrS2() { sensor2.increment(); }

// ================= 4. NETWORK & OTA MANAGERS =================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.ssid, config.wifiPassword);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi Connected. IP: " + WiFi.localIP().toString());
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
}

void setupFirebase() {
  Serial.println("[Firebase] Initializing...");
  fbConfig.api_key = config.apiKey;
  fbConfig.database_url = config.dbUrl;
  auth.user.email = config.userEmail;
  auth.user.password = config.userPass;
  
  fbdo.setResponseSize(2048);
  Firebase.begin(&fbConfig, &auth);
  Firebase.reconnectWiFi(true);
}

void checkAndRunUpdate() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("[OTA] Checking for updates...");
  
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  http.begin(client, config.otaUrl);
  if (http.GET() == 200) {
    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, http.getString())) {
      String remoteVer = doc["version"].as<String>();
      if (remoteVer != config.firmwareVersion) {
        Serial.println("[OTA] New version found! Updating...");
        httpUpdate.update(client, doc["bin_url"].as<String>());
      } else {
        Serial.println("[OTA] Up to date.");
      }
    }
  }
  http.end();
}

// ================= 5. MAIN SYSTEM =================
void setup() {
  Serial.begin(115200);
  
  connectWiFi(); 
  setupFirebase(); 
  Udp.begin(config.discoveryPort);

  sensor1.begin(isrS1);
  sensor2.begin(isrS2);

  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
      String msg = (char*)payload;
      if (msg.indexOf("\"command\":\"ota\"") > 0 || msg == "{\"command\":\"ota\"}") {
        Serial.println("[WebSocket] OTA Command Received!");
        checkAndRunUpdate();
      }
    }
  });

  webSocket.begin();

  // Check for OTA updates once on boot
  checkAndRunUpdate(); 
}

void loop() {
  webSocket.loop();

  unsigned long now = millis();

  // --- SENSOR MEASUREMENT & WEBSOCKETS (Every 500ms) ---
  if (now - lastMeasureTime > 500) {
    unsigned long duration = now - lastMeasureTime;
    
    sensor1.calculateFlow(duration);
    sensor2.calculateFlow(duration);
    lastMeasureTime = now;

    // -- flow_in(L/min) & total_in(volume) = S1(sensor 1) input --
    // -- flow_out(L/min) & total_out(volume) = S2(sensor 2) output --

    float f1 = sensor1.getFlowRate();
    float f2 = sensor2.getFlowRate();
    float t1 = sensor1.getTotalVolume();
    float t2 = sensor2.getTotalVolume();
    float diff = f1 - f2;
    bool leak = (f1 > 0.5 && diff > config.leakThreshold);

    // Broadcast WebSocket Data
    StaticJsonDocument<256> doc;
    doc["flow_in"] = f1;
    doc["flow_out"] = f2;
    doc["total_in"] = t1;
    doc["total_out"] = t2;
    doc["diff"] = diff;
    doc["leak"] = leak;
    doc["fw_ver"] = config.firmwareVersion;
    doc["ip_address"] = WiFi.localIP().toString();

    String jsonStr;
    serializeJson(doc, jsonStr);
    webSocket.broadcastTXT(jsonStr);

    // --- FIREBASE UPDATE (Every 1000ms) ---
    if (now - lastDbUpdate > 1000 && Firebase.ready()) {
      FirebaseJson fbJson;
      fbJson.set("flow_in", f1);
      fbJson.set("flow_out", f2);
      fbJson.set("total_in", t1);
      fbJson.set("total_out", t2);
      fbJson.set("diff", diff);
      fbJson.set("leak", leak);
      fbJson.set("fw_ver", config.firmwareVersion);
      fbJson.set("ip_address", WiFi.localIP().toString());
      fbJson.set("ts/.sv", "timestamp"); 
      
      Firebase.RTDB.pushJSON(&fbdo, config.dbPath, &fbJson);
      lastDbUpdate = now;
    }
  }

  // --- UDP DISCOVERY (Every 5000ms) ---
  if (now - lastDiscovery >= 5000 && WiFi.status() == WL_CONNECTED) {
      String payload = "DRIPCHECK_WS:ws://" + WiFi.localIP().toString() + ":81";
      Udp.beginPacket("255.255.255.255", config.discoveryPort);
      Udp.write((uint8_t*)payload.c_str(), payload.length());
      Udp.endPacket();
      lastDiscovery = now;
  }
}