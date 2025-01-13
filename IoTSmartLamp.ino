#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <EEPROM.h>

#include "config.h" // IP address, Auth Token, etc.

// --------------------------- Variables <-> Master System  --------------------------- //

// EEPROM settings
#define EEPROM_SIZE 24
#define EEPROM_ADDR_START 100
#define EEPROM_ADDR_VALID (EEPROM_ADDR_START + 0)
#define EEPROM_ADDR_LED_STATE (EEPROM_ADDR_START + 1)
#define EEPROM_ADDR_PATTERN (EEPROM_ADDR_START + 5)
#define EEPROM_ADDR_INTERVAL (EEPROM_ADDR_START + 9)
#define EEPROM_ADDR_RESET (EEPROM_ADDR_START + 13)
#define EEPROM_RESET_TIMEOUT 10000

struct Settings {
  uint32_t ledState;
  int patternType;
  unsigned long patternInterval;
  unsigned int savedPowerOnCounter;
};

unsigned long lastPowerOnTimestamp;
unsigned int powerOnCounter;

// --------------------------- Variables <-> Lighting System --------------------------- //

// Shift Register Pins
const int dataPin = D7;  // Serial data input
const int latchPin = D6; // Register clock
const int clockPin = D5; // Shift register clock

// LED Pattern Variables
unsigned long lastLedUpdate = 0;
const long ledUpdateInterval = 50; // Update LEDs every 50ms
uint32_t ledState = 0;

enum PatternType {
  NONE,
  PAIR,
  OUT_TO_IN,
  IN_TO_OUT,
  LEFT_TO_RIGHT,
  RIGHT_TO_LEFT,
  G1,G2,G3,G4,G5,G6,G7,G8,G0
};

// Define the LED groups
const uint32_t G0_MASK = 0x0000;
const uint32_t G1_MASK = 0xC000;
const uint32_t G2_MASK = 0x2000;
const uint32_t G3_MASK = 0x1000;
const uint32_t G4_MASK = 0x0C30;
const uint32_t G5_MASK = 0x0008;
const uint32_t G6_MASK = 0x0004;
const uint32_t G7_MASK = 0x0003;
const uint32_t G8_MASK = 0xFFFF;

int NUM_STATES = 7;
int currentState = 0;
PatternType activePattern = PAIR;
unsigned long lastPatternUpdate = 0;
unsigned long patternUpdateInterval = 1000; 


// --------------------------- Variables <-> Ambiental System --------------------------- //

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// --------------------------- Variables <-> Web Server System --------------------------- //

// Wifi Server - Constants and Objects
ESP8266WebServer server(80);
WiFiManager wifiManager;
unsigned long totalRequests = 0;
unsigned long lastCheckTime = 0;
unsigned long wifiConnectStartTime = 0;
const unsigned long WIFI_CONNECT_TIMEOUT = 600000; 


// ---------------------------- Functions <-> Master System  --------------------------- //

void saveSettings() {
  Settings settings;
  settings.ledState = ledState;
  settings.patternType = static_cast<int>(activePattern);
  settings.patternInterval = patternUpdateInterval;
  settings.savedPowerOnCounter = powerOnCounter;

  EEPROM.begin(EEPROM_SIZE + EEPROM_ADDR_START);
  EEPROM.write(EEPROM_ADDR_VALID, 0xAA);
  EEPROM.put(EEPROM_ADDR_LED_STATE, settings);
  EEPROM.commit();
  EEPROM.end();
}

void loadSettings() {

  EEPROM.begin(EEPROM_SIZE + EEPROM_ADDR_START);
  
  if (EEPROM.read(EEPROM_ADDR_VALID) != 0xAA) {

    // Reset EEPROM and WIFI Config
    ledState = 0x0000;
    patternUpdateInterval = 1000;
    NUM_STATES = 1;
    setPattern(PAIR);
    powerOnCounter = 0;
    updatePatternState();
    saveSettings();
    resetWifiSettings();

    EEPROM.end();
    return;
  }

  Settings settings;
  EEPROM.get(EEPROM_ADDR_LED_STATE, settings);
  EEPROM.end();

  ledState = settings.ledState;
  activePattern = static_cast<PatternType>(settings.patternType);
  patternUpdateInterval = settings.patternInterval;
  if(settings.savedPowerOnCounter > 4) {
    powerOnCounter = 1;
  } else if(settings.savedPowerOnCounter >= 3) {
    Serial.println("Resetting EEPROM");

    // Reset EEPROM and WIFI Config
    ledState = 0x0000;
    setPattern(PAIR);
    powerOnCounter = 0;

    updatePatternState();
    saveSettings();
    resetWifiSettings();

    ledState = 0x0000;
    setPattern(LEFT_TO_RIGHT);
    patternUpdateInterval = 1000;
    updatePatternState();
    saveSettings();
  } else {
    powerOnCounter = settings.savedPowerOnCounter;
    powerOnCounter = powerOnCounter + 1;
    saveSettings();
  }


  switch (activePattern) {
    case OUT_TO_IN:
    case IN_TO_OUT:
      NUM_STATES = 4;
      break;
    case LEFT_TO_RIGHT:
    case RIGHT_TO_LEFT:
      NUM_STATES = 7;
      break;
    default:
      NUM_STATES = 1;
      break;
  }
  updatePatternState();
}

// ---------------------------- Functions <-> Lighting System  --------------------------- //

void setupShiftRegisters() {
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
}

void setPattern(PatternType pattern) {
  activePattern = pattern;
  currentState = 0;
}

String getPatternName(PatternType pattern) {
  switch (pattern) {
    case OUT_TO_IN: return "Out to In";
    case IN_TO_OUT: return "In to Out";
    case LEFT_TO_RIGHT: return "Left to Right";
    case RIGHT_TO_LEFT: return "Right to Left";
    default:
      String activeGs = "";
      if (ledState == G0_MASK) return "All Off";
      if (ledState == G8_MASK) return "All On";
      if (ledState & G1_MASK) activeGs += "G1 ";
      if (ledState & G2_MASK) activeGs += "G2 ";
      if (ledState & G3_MASK) activeGs += "G3 ";
      if (ledState & G4_MASK) activeGs += "G4 ";
      if (ledState & G5_MASK) activeGs += "G5 ";
      if (ledState & G6_MASK) activeGs += "G6 ";
      if (ledState & G7_MASK) activeGs += "G7 ";
     
      if (activeGs.length() > 0) {
        activeGs.trim(); // Remove trailing space
        return activeGs;
      } else {
        return "None";
      }
  }
}

void  updatePatternState() {
  switch (activePattern) {
    case G0: ledState = 0x0000; break;
    case G8: ledState = 0xFFFF; break;
    case G1:
    case G2:
    case G3:
    case G4:
    case G5:
    case G6:
    case G7:
      switch (activePattern) {
        case G1: ledState |= G1_MASK; break;
        case G2: ledState |= G2_MASK; break;
        case G3: ledState |= G3_MASK; break;
        case G4: ledState |= G4_MASK; break;
        case G5: ledState |= G5_MASK; break;
        case G6: ledState |= G6_MASK; break;
        case G7: ledState |= G7_MASK; break;
      }
      break;
    case PAIR:
      switch(currentState){
        case 0: ledState = 0x0002;break;
      }
      break;
    case OUT_TO_IN:
      switch (currentState) {
        case 0: ledState = 0xC003; break;
        case 1: ledState = 0x2004; break;
        case 2: ledState = 0x1008; break;
        case 3: ledState = 0x0C30; break;
      }
      break;
    case IN_TO_OUT:
      switch (currentState) {
        case 0: ledState = 0x0C30; break;
        case 1: ledState = 0x1008; break;
        case 2: ledState = 0x2004; break;
        case 3: ledState = 0xC003; break;
      }
      break;
    case RIGHT_TO_LEFT:
      switch (currentState) {
        case 0: ledState = 0xC000; break;
        case 1: ledState = 0x2000; break;
        case 2: ledState = 0x1000; break;
        case 3: ledState = 0x0C30; break;
        case 4: ledState = 0x0008; break;
        case 5: ledState = 0x0004; break;
        case 6: ledState = 0x0003; break;
      }
      break;
    case LEFT_TO_RIGHT:
      switch (currentState) {
        case 6: ledState = 0xC000; break;
        case 5: ledState = 0x2000; break;
        case 4: ledState = 0x1000; break;
        case 3: ledState = 0x0C30; break;
        case 2: ledState = 0x0008; break;
        case 1: ledState = 0x0004; break;
        case 0: ledState = 0x0003; break;
      }
      break;
    default:
      ledState = 0x0000;
      activePattern = NONE;
      break;
  }
  updateLEDs();
}

void updateLEDs() {
  digitalWrite(latchPin, LOW);
  for (int i = 23; i >= 0; i--) {
    digitalWrite(clockPin, LOW);
    digitalWrite(dataPin, bitRead(ledState, i));
    digitalWrite(clockPin, HIGH);
  }
  digitalWrite(latchPin, HIGH);
}

// ---------------------------- Functions <-> Web Server System  --------------------------- //

bool isAuthenticated() {
  // Check if request comes from allowed IP
  if (server.client().remoteIP() != ALLOWED_IP ) {
   return false;
  }
  
  if (server.hasHeader("Authorization")) {
    String token = server.header("Authorization");
    if (token == AUTH_TOKEN) {
      lastCheckTime = millis();
      return true;
    }
  }
  if (server.hasArg("token")) {
    String token = server.arg("token");
    if (token == AUTH_TOKEN) {
      return true;
    }
  }
  return false;
}

void resetWifiSettings() {
  server.stop();  // Stop the main server
 
  // Force the configuration portal to start
  wifiManager.resetSettings();  // Optional: uncomment to clear saved settings
  bool portalRunning = wifiManager.startConfigPortal("NodeMCU_Reconfig");
 
  server.begin();

  lastCheckTime = millis();
}

bool connectToSavedWiFi() {
  WiFi.mode(WIFI_STA);
  String savedSSID = WiFi.SSID();
  Serial.println(savedSSID);

  if(savedSSID.length() == 0){
    delay(1000);
    return false;
  }

  WiFi.begin();

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_CONNECT_TIMEOUT) {
    delay(100);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  } else {
    return false;
  }

}

void handleRoot() {
  if (!isAuthenticated()) {
    server.send(401, "application/json", "{\"error\":\"Unauthorized\"}");
    return;
  }
  server.send(200, "text/plain", "NodeMCU Device");
}

void handleStatus() {
  if (!isAuthenticated()) {
    server.send(401, "application/json", "{\"error\":\"Unauthorized\"}");
    return;
  }
 
  String response = "{\"status\":\"ok\", \"uptime\":" + String(millis() / 1000);
 
  sensors_event_t event;
  if (bmp.getEvent(&event)) {
    float temperature;
    bmp.getTemperature(&temperature);
    response += ", \"sensor_status\":\"operational\"";
    response += ", \"temperature\":" + String(temperature);
    response += ", \"pressure\":" + String(event.pressure);
  } else {
    response += ", \"sensor_status\":\"NOK Please check wiring !\"";
  }

  response += ", \"led_pattern\":\"" + String(getPatternName(activePattern)) + "\"";
  response += ", \"led_pattern_interval\":" + String(patternUpdateInterval);
  response += ", \"led_state\":\"0x" + String(ledState, HEX) + "\"";
  response += ", \"current_state\":" + String(currentState);
 
  response += ", \"ip_address\":\"" + WiFi.localIP().toString() + "\"";
  response += ", \"total_requests\":" + String(totalRequests);
  response += ", \"free_heap\":" + String(ESP.getFreeHeap());
  response += ", \"wifi_rssi\":" + String(WiFi.RSSI());
  response += ", \"cpu_freq\":" + String(ESP.getCpuFreqMHz());
  response += ", \"mac_address\":\"" + WiFi.macAddress() + "\"";
  response += ", \"wifi_ssid\":\"" + WiFi.SSID() + "\"";
  response += ", \"flash_size\":" + String(ESP.getFlashChipSize());
  response += ", \"sdk_version\":\"" + String(ESP.getSdkVersion()) + "\"";
 
  response += "}";
  server.send(200, "application/json", response);
}

void handleReconfigure() {
  if (!isAuthenticated()) {
    server.send(401, "application/json", "{\"error\":\"Unauthorized\"}");
    return;
  }
  server.send(200, "application/json", "{\"message\":\"Reconfiguring WiFi\"}");
  delay(1000);  // Give the server time to send the response
  resetWifiSettings();
}

void handleLEDs() {
  if (!isAuthenticated()) {
    server.send(401, "application/json", "{\"error\":\"Unauthorized\"}");
    return;
  }
 
  bool settingsChanged = false;

  if (server.hasArg("pattern")) {
    String patternArg = server.arg("pattern");
    
    if (server.hasArg("interval")) {
      int newInterval = max(50, (int)server.arg("interval").toInt());
      if (newInterval != patternUpdateInterval) {
        patternUpdateInterval = newInterval;
        settingsChanged = true;
      }
    }

    if ((patternArg.startsWith("g") && patternArg.length() == 2 &&
         patternArg[1] >= '1' && patternArg[1] <= '7') &&
        (activePattern < G1 || activePattern > G7)) {
      ledState = 0;
    }

        PatternType newPattern = NONE;
    if (patternArg == "out_to_in")          { newPattern = OUT_TO_IN;     NUM_STATES = 4;}
    else if (patternArg == "in_to_out")     { newPattern = IN_TO_OUT;     NUM_STATES = 4;}
    else if (patternArg == "left_to_right") { newPattern = LEFT_TO_RIGHT; NUM_STATES = 7;}
    else if (patternArg == "right_to_left") { newPattern = RIGHT_TO_LEFT; NUM_STATES = 7;}
    else if (patternArg == "g0")            { newPattern = G0;            NUM_STATES = 1;}
    else if (patternArg == "g1")            { newPattern = G1;            NUM_STATES = 1;}
    else if (patternArg == "g2")            { newPattern = G2;            NUM_STATES = 1;}
    else if (patternArg == "g3")            { newPattern = G3;            NUM_STATES = 1;}
    else if (patternArg == "g4")            { newPattern = G4;            NUM_STATES = 1;}
    else if (patternArg == "g5")            { newPattern = G5;            NUM_STATES = 1;}
    else if (patternArg == "g6")            { newPattern = G6;            NUM_STATES = 1;}
    else if (patternArg == "g7")            { newPattern = G7;            NUM_STATES = 1;}
    else if (patternArg == "g8")            { newPattern = G8;            NUM_STATES = 1;}
    else if (patternArg == "stop")          { newPattern = NONE; ledState = 0; }    
    else {
      newPattern = NONE;
      ledState = strtoul(patternArg.c_str(), NULL, 16);
    }

    if (newPattern != activePattern) {
      setPattern(newPattern);
      settingsChanged = true;
    }

    updatePatternState();
    
    if (settingsChanged) {
      saveSettings();
    }

    server.send(200, "application/json", "{\"status\":\"ok\",\"pattern\":\"" + getPatternName(activePattern) + "\",\"interval\":" + String(patternUpdateInterval) + "}");
  } else {
    server.send(400, "application/json", "{\"error\":\"Missing LED pattern\"}");
  }
}

// ---------------------------- Arduino Generic Functions  --------------------------- //


void setup() {
  Serial.begin(115200);
 
  setupShiftRegisters();

  loadSettings(); 
  
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  }

  if (!connectToSavedWiFi()) {
    resetWifiSettings();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.on("/", handleRoot);
    server.on("/api/status", handleStatus);
    server.on("/api/reconfigure", handleReconfigure);
    server.on("/api/leds", handleLEDs);

    server.begin();
    Serial.println("HTTP server started");
  } else {
    Serial.println("Failed to connect and configure WiFi. Restarting...");
    ESP.restart();
  }

  Serial.println(powerOnCounter);
  lastPowerOnTimestamp = millis();
}

void loop() {
  unsigned long currentMillis = millis();

 if ((powerOnCounter)&&(currentMillis - lastPowerOnTimestamp > EEPROM_RESET_TIMEOUT)) {
    powerOnCounter = 0;
    saveSettings();
    Serial.println("Reset Power on counter set to 0 after timeout.");
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (currentMillis - wifiConnectStartTime >= WIFI_CONNECT_TIMEOUT) {
      Serial.println("Failed to connect to WiFi within timeout. Starting config portal.");
      resetWifiSettings();
    }
  } else {
    server.handleClient();
  }

  if (activePattern != NONE && currentMillis - lastPatternUpdate >= patternUpdateInterval) {
    lastPatternUpdate = currentMillis;
    currentState = (currentState + 1) % NUM_STATES;
    updatePatternState();
  }
}

