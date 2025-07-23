#include <SPI.h>
#include <Wire.h>
#include "SparkFun_AS3935.h"
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// --- Constants ---
#define INDOOR 0x12
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01
#define BUZZER_PIN 13
#define LIGHTNING_INT_PIN 26
#define SPI_CS_PIN 27
#define ALERT_COOLDOWN 10UL * 60UL * 1000UL // 10 minutes
#define LIGHTNING_CLEAR_COOLDOWN 1UL * 60UL * 60UL * 1000UL // 1 hour
#define MQTT_PUBLISH_INTERVAL 5UL * 60UL * 1000UL // 5 minutes
#define DISPLAY_CYCLE_INTERVAL 4000 // 4 seconds
#define ALERT_DURATION 30000 // 30 seconds

// --- Sensor and Display Objects ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
SparkFun_AS3935 lightning;
RTC_DS3231 rtc;
Adafruit_BMP280 bmp;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiManager wm;
WebServer server(80);
Preferences preferences;

// --- Custom Characters ---
byte signalBarsFull[8] = {0b00000, 0b00001, 0b00011, 0b00111, 0b01111, 0b11111, 0b00000, 0b00000};
byte lightningBoltDiagonal[8] = {0b10000, 0b11000, 0b01000, 0b00100, 0b00010, 0b00110, 0b01000, 0b10000};
byte thermometer[8] = {0b00100, 0b01010, 0b01010, 0b01110, 0b01110, 0b01110, 0b11111, 0b01110};
byte droplet[8] = {0b00100, 0b00100, 0b01010, 0b01010, 0b10001, 0b10001, 0b10001, 0b01110};
byte pressure[8] = {0b00000, 0b01110, 0b10001, 0b10101, 0b10001, 0b01110, 0b00000, 0b00000};

// --- State Structure ---
struct WeatherStationState {
  float currentTempF;
  float currentPressureInHg;
  int lastDistanceMiles;
  int lastEnergy;
  unsigned long lastLightningTimeMillis;
  unsigned long lastAlertMillis;
  unsigned long lastCycle;
  unsigned long lastMQTTPublishMillis;
  int displayState;
  bool alertActive;
  unsigned long alertStartMillis;
  int lastClockMinute;
};

// --- Global State ---
WeatherStationState state = {0.0, 0.0, -1, -1, 0, 0, 0, 0, 0, false, 0, -1};

// --- MQTT Configuration ---
struct MQTTConfig {
  char host[64];
  int port;
  char username[32];
  char password[32];
  char topicTemp[40];
  char topicPressure[40];
  char tLightningDist[40]; // Shortened from topicLightningDist
  char tLightningEnergy[40]; // Shortened from topicLightningEnergy
  char topicStatus[40];
} mqttConfig;

// Update WiFiManager parameters
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqttConfig.host, 64);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", "1883", 6);
WiFiManagerParameter custom_mqtt_username("username", "MQTT Username", mqttConfig.username, 32);
WiFiManagerParameter custom_mqtt_password("password", "MQTT Password", mqttConfig.password, 32);
WiFiManagerParameter custom_mqtt_topic_temp("top_t", "Temp Topic", mqttConfig.topicTemp, 40);
WiFiManagerParameter custom_mqtt_topic_pressure("top_p", "Pressure Topic", mqttConfig.topicPressure, 40);
WiFiManagerParameter custom_mqtt_topic_lightning_dist("top_ld", "Lightning Dist Topic", mqttConfig.tLightningDist, 40);
WiFiManagerParameter custom_mqtt_topic_lightning_energy("top_le", "Lightning Energy Topic", mqttConfig.tLightningEnergy, 40);
WiFiManagerParameter custom_mqtt_topic_status("top_s", "Status Topic", mqttConfig.topicStatus, 40);

// --- Function Prototypes ---
void setupHardware();
void setupWiFi();
void setupWebServer();
void setupMQTT();
void handleSensors();
void handleDisplay();
void handleMQTT();
void playStartupChime();
void playAlertChime();
void haltAndCatchFire();
void displayClock();
void displayScrollingInfo();
void saveCustomParamsCallback();
bool testMqttConnection();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void reconnectMqtt();
void handleRoot();
void handleJsonStatus();
void handleConfig();
void handleMQTTConfigForm();
void handleMQTTConfigSubmit();
void loadMQTTConfig();
void saveMQTTConfig();

// --- Setup Function ---
void setup() {
  Serial.begin(115200);
  pinMode(LIGHTNING_INT_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  SPI.begin();

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, signalBarsFull);
  lcd.createChar(1, lightningBoltDiagonal);
  lcd.createChar(2, thermometer);
  lcd.createChar(3, droplet);
  lcd.createChar(4, pressure);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0);
  lcd.print(" System Start");
  delay(2000);

  setupHardware();
  setupWiFi();
  setupWebServer();
  setupMQTT();
  playStartupChime();
  lcd.clear();
}

// --- Loop Function ---
void loop() {
  unsigned long nowMillis = millis();

  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  if (strlen(mqttConfig.host) > 0 && !mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();

  handleSensors();
  handleDisplay();
  handleMQTT();
  delay(100); // Prevent CPU overload
}

// --- Hardware Setup ---
void setupHardware() {
  if (!lightning.beginSPI(SPI_CS_PIN)) {
    lcd.setCursor(0, 1);
    lcd.print("Sensor Failure");
    haltAndCatchFire();
  }
  lightning.setNoiseLevel(2);
  lightning.setIndoorOutdoor(OUTDOOR);

  if (!rtc.begin()) {
    lcd.setCursor(0, 1);
    lcd.print("Clock Failure");
    haltAndCatchFire();
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    lcd.setCursor(0, 1);
    lcd.print("BMP280 Failure");
    haltAndCatchFire();
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  lcd.setCursor(0, 1);
  lcd.print("Sensors OK!");
  delay(1500);
}

// --- WiFi Setup ---
void setupWiFi() {
  wm.setAPCallback([](WiFiManager *myWiFiManager) {
    Serial.println("Entered WiFi config mode");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("AP SSID: ");
    Serial.println(myWiFiManager->getConfigPortalSSID());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connect to AP:");
    lcd.setCursor(0, 1);
    lcd.print(myWiFiManager->getConfigPortalSSID());
    lcd.write((uint8_t)0);
    Serial.println("Web server status: " + String(server.active() ? "Running" : "Not Running"));
  });

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_username);
  wm.addParameter(&custom_mqtt_password);
  wm.addParameter(&custom_mqtt_topic_temp);
  wm.addParameter(&custom_mqtt_topic_pressure);
  wm.addParameter(&custom_mqtt_topic_lightning_dist);
  wm.addParameter(&custom_mqtt_topic_lightning_energy);
  wm.addParameter(&custom_mqtt_topic_status);
  wm.setSaveConfigCallback(saveCustomParamsCallback);

  if (!wm.autoConnect("WeatherStationSetup")) {
    Serial.println("WiFi connection failed!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Error!");
    lcd.setCursor(0, 1);
    lcd.print("Restarting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("WiFi connected!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0);
  lcd.print(" WiFi Connected");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(2000);
}

// --- Web Server Setup ---
void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/json", handleJsonStatus);
  server.on("/config", handleConfig);
  server.on("/mqtt-config", HTTP_GET, handleMQTTConfigForm);
  server.on("/mqtt-config", HTTP_POST, handleMQTTConfigSubmit);
  server.on("/factory-reset", HTTP_GET, []() {
    preferences.clear();
    wm.resetSettings();
    server.send(200, "text/html", "<html><body><h3>Factory reset complete. Rebooting...</h3></body></html>");
    delay(1000);
    ESP.restart();
  });
  server.begin();
  Serial.println("HTTP server started");
}

// --- MQTT Setup ---
void setupMQTT() {
  loadMQTTConfig();
  if (strlen(mqttConfig.host) > 0) {
    if (!testMqttConnection()) {
      Serial.println("MQTT test failed! Entering config mode.");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("MQTT Test FAIL!");
      lcd.setCursor(0, 1);
      lcd.print("Reconfigure AP");
      delay(3000);
      wm.startConfigPortal("WeatherStationSetup");
      // Do not restart immediately; allow configuration
    } else {
      mqttClient.setServer(mqttConfig.host, mqttConfig.port);
      mqttClient.setCallback(mqttCallback);
    }
  } else {
    Serial.println("No MQTT server configured. Entering config mode.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No MQTT config");
    lcd.setCursor(0, 1);
    lcd.print("Reconfigure AP");
    delay(3000);
    wm.startConfigPortal("WeatherStationSetup");
  }
}

// --- Sensor Handling ---
void handleSensors() {
  float tempC = bmp.readTemperature();
  state.currentTempF = (tempC * 9.0 / 5.0) + 32.0;
  float pressurePa = bmp.readPressure();
  state.currentPressureInHg = pressurePa / 3386.389;

  if (digitalRead(LIGHTNING_INT_PIN) == HIGH) {
    int intVal = lightning.readInterruptReg();
    if (intVal == LIGHTNING_INT) {
      byte distanceKm = lightning.distanceToStorm();
      state.lastDistanceMiles = round(distanceKm * 0.621371);
      state.lastEnergy = lightning.lightningEnergy();
      state.lastLightningTimeMillis = millis();

      if (state.lastDistanceMiles <= 10 && (state.lastAlertMillis == 0 || millis() - state.lastAlertMillis > ALERT_COOLDOWN)) {
        state.alertStartMillis = millis();
        state.alertActive = true;
        state.lastAlertMillis = millis();
        playAlertChime();
      }
    }
  }

  if (state.lastLightningTimeMillis > 0 && millis() - state.lastLightningTimeMillis > LIGHTNING_CLEAR_COOLDOWN) {
    state.lastDistanceMiles = -1;
    state.lastEnergy = -1;
    state.lastLightningTimeMillis = 0;
  }
}

// --- Display Handling ---
void handleDisplay() {
  unsigned long nowMillis = millis();
  DateTime now = rtc.now();

  if (state.alertActive) {
    if (nowMillis - state.alertStartMillis < ALERT_DURATION) {
      lcd.setCursor(0, 0);
      lcd.print("!!! LIGHTNING !!!");
      lcd.setCursor(0, 1);
      lcd.print("Within ");
      lcd.print(state.lastDistanceMiles);
      lcd.print(" miles!    ");
      if ((nowMillis / 500) % 2 == 0) {
        lcd.backlight();
      } else {
        lcd.noBacklight();
      }
    } else {
      state.alertActive = false;
      lcd.backlight();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write((uint8_t)0);
      displayClock();
      displayScrollingInfo();
    }
  } else {
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)0);
    if (now.minute() != state.lastClockMinute) {
      state.lastClockMinute = now.minute();
      displayClock();
    }
    if (nowMillis - state.lastCycle > DISPLAY_CYCLE_INTERVAL) {
      state.lastCycle = nowMillis;
      displayScrollingInfo();
    }
  }
}

// --- MQTT Handling ---
void handleMQTT() {
  if (strlen(mqttConfig.host) == 0 || !mqttClient.connected()) {
    return;
  }

  unsigned long nowMillis = millis();
  if (nowMillis - state.lastMQTTPublishMillis > MQTT_PUBLISH_INTERVAL) {
    state.lastMQTTPublishMillis = nowMillis;
    char payload[32];

    sprintf(payload, "%.1f", state.currentTempF);
    mqttClient.publish(mqttConfig.topicTemp, payload);
    Serial.printf("Published Temp: %s\n", payload);

    sprintf(payload, "%.2f", state.currentPressureInHg);
    mqttClient.publish(mqttConfig.topicPressure, payload);
    Serial.printf("Published Pressure: %s\n", payload);

    if (state.lastDistanceMiles >= 0) {
      sprintf(payload, "%d", state.lastDistanceMiles);
      mqttClient.publish(mqttConfig.tLightningDist, payload);
      Serial.printf("Published Lightning Distance: %s\n", payload);
    } else {
      mqttClient.publish(mqttConfig.tLightningDist, "none");
    }

    if (state.lastEnergy >= 0) {
      sprintf(payload, "%d", state.lastEnergy);
      mqttClient.publish(mqttConfig.tLightningEnergy, payload);
      Serial.printf("Published Lightning Energy: %s\n", payload);
    } else {
      mqttClient.publish(mqttConfig.tLightningEnergy, "none");
    }
  }
}
// --- Buzzer Functions ---
void playStartupChime() {
  tone(BUZZER_PIN, 523, 150);
  delay(200);
  tone(BUZZER_PIN, 659, 150);
  delay(200);
  noTone(BUZZER_PIN);
}

void playAlertChime() {
  tone(BUZZER_PIN, 523, 200);
  delay(250);
  tone(BUZZER_PIN, 440, 200);
  delay(250);
  tone(BUZZER_PIN, 659, 250);
  delay(300);
  noTone(BUZZER_PIN);
}

// --- Display Functions ---
void displayClock() {
  DateTime now = rtc.now();
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", now.hour(), now.minute());
  int startCol = 15 - strlen(timeStr);
  lcd.setCursor(1, 0);
  lcd.print("               ");
  lcd.setCursor(startCol, 0);
  lcd.print(timeStr);
}

void displayScrollingInfo() {
  lcd.setCursor(0, 1);
  switch (state.displayState) {
    case 0:
      lcd.write((uint8_t)2);
      lcd.print(" Temp: ");
      lcd.print(state.currentTempF, 1);
      lcd.print("F   ");
      break;
    case 1:
      lcd.write((uint8_t)4);
      lcd.print(" Pres:");
      lcd.print(state.currentPressureInHg, 2);
      lcd.print("inHg");
      break;
    case 2:
      lcd.write((uint8_t)1);
      lcd.print(" Last: ");
      if (state.lastDistanceMiles >= 0) {
        lcd.print(state.lastDistanceMiles);
        lcd.print("mi   ");
      } else {
        lcd.print("None     ");
      }
      break;
    case 3:
      lcd.write((uint8_t)1);
      lcd.print(" Energy: ");
      if (state.lastEnergy >= 0) {
        lcd.print(state.lastEnergy);
        lcd.print("      ");
      } else {
        lcd.print("None     ");
      }
      break;
    default:
      state.displayState = 0;
      return;
  }
  state.displayState = (state.displayState + 1) % 4;
}

void haltAndCatchFire() {
  while (true) {
    playAlertChime();
    delay(1500);
  }
}

// --- MQTT Utility Functions ---
void saveCustomParamsCallback() {
  strncpy(mqttConfig.host, custom_mqtt_server.getValue(), sizeof(mqttConfig.host));
  mqttConfig.port = atoi(custom_mqtt_port.getValue());
  strncpy(mqttConfig.username, custom_mqtt_username.getValue(), sizeof(mqttConfig.username));
  strncpy(mqttConfig.password, custom_mqtt_password.getValue(), sizeof(mqttConfig.password));
  strncpy(mqttConfig.topicTemp, custom_mqtt_topic_temp.getValue(), sizeof(mqttConfig.topicTemp));
  strncpy(mqttConfig.topicPressure, custom_mqtt_topic_pressure.getValue(), sizeof(mqttConfig.topicPressure));
  strncpy(mqttConfig.tLightningDist, custom_mqtt_topic_lightning_dist.getValue(), sizeof(mqttConfig.tLightningDist));
  strncpy(mqttConfig.tLightningEnergy, custom_mqtt_topic_lightning_energy.getValue(), sizeof(mqttConfig.tLightningEnergy));
  strncpy(mqttConfig.topicStatus, custom_mqtt_topic_status.getValue(), sizeof(mqttConfig.topicStatus));
  saveMQTTConfig();
}

bool testMqttConnection() {
  Serial.printf("Testing MQTT connection to %s:%d\n", mqttConfig.host, mqttConfig.port);
  lcd.setCursor(0, 0);
  lcd.print("Testing MQTT...");
  lcd.setCursor(0, 1);
  lcd.print("               ");

  mqttClient.setServer(mqttConfig.host, mqttConfig.port);
  String clientId = "ESP32Weather-" + String(micros(), HEX);

  bool connected = strlen(mqttConfig.username) > 0 ?
    mqttClient.connect(clientId.c_str(), mqttConfig.username, mqttConfig.password) :
    mqttClient.connect(clientId.c_str());

  if (connected) {
    Serial.println("MQTT Test CONNECTED");
    lcd.setCursor(0, 1);
    lcd.print("MQTT OK!      ");
    mqttClient.disconnect();
    delay(1000);
    return true;
  } else {
    Serial.printf("MQTT Test FAILED, rc=%d\n", mqttClient.state());
    lcd.setCursor(0, 1);
    lcd.print("MQTT Failed! rc:");
    lcd.print(mqttClient.state());
    delay(2000);
    return false;
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.printf("Message arrived [%s] ", topic);
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    lcd.setCursor(0, 1);
    lcd.print("MQTT Reconn...");
    String clientId = "ESP32Weather-" + WiFi.macAddress().substring(12);

    bool connected = strlen(mqttConfig.username) > 0 ?
      mqttClient.connect(clientId.c_str(), mqttConfig.username, mqttConfig.password) :
      mqttClient.connect(clientId.c_str());

    if (connected) {
      Serial.println("MQTT Connected!");
      lcd.setCursor(0, 1);
      lcd.print("MQTT Connected  ");
      mqttClient.publish(mqttConfig.topicStatus, "Weather Station Online");
      state.lastMQTTPublishMillis = millis();
    } else {
      Serial.printf("MQTT failed, rc=%d\n", mqttClient.state());
      lcd.setCursor(0, 1);
      lcd.print("MQTT Failed, ");
      lcd.print(mqttClient.state());
      delay(5000);
    }
  }
}

// --- Configuration Storage ---
void loadMQTTConfig() {
  preferences.begin("weather-station", false);
  preferences.getString("host", mqttConfig.host, sizeof(mqttConfig.host));
  mqttConfig.port = preferences.getInt("port", 1883);
  preferences.getString("username", mqttConfig.username, sizeof(mqttConfig.username));
  preferences.getString("password", mqttConfig.password, sizeof(mqttConfig.password));
  preferences.getString("topicTemp", mqttConfig.topicTemp, sizeof(mqttConfig.topicTemp));
  preferences.getString("topicPressure", mqttConfig.topicPressure, sizeof(mqttConfig.topicPressure));
  preferences.getString("tLightningDist", mqttConfig.tLightningDist, sizeof(mqttConfig.tLightningDist));
  preferences.getString("tLightningEnergy", mqttConfig.tLightningEnergy, sizeof(mqttConfig.tLightningEnergy));
  preferences.getString("topicStatus", mqttConfig.topicStatus, sizeof(mqttConfig.topicStatus));
  preferences.end();

  // Set default topics if not configured
  if (strlen(mqttConfig.topicTemp) == 0) strcpy(mqttConfig.topicTemp, "weather/temperature");
  if (strlen(mqttConfig.topicPressure) == 0) strcpy(mqttConfig.topicPressure, "weather/pressure");
  if (strlen(mqttConfig.tLightningDist) == 0) strcpy(mqttConfig.tLightningDist, "weather/lightning_distance");
  if (strlen(mqttConfig.tLightningEnergy) == 0) strcpy(mqttConfig.tLightningEnergy, "weather/lightning_energy");
  if (strlen(mqttConfig.topicStatus) == 0) strcpy(mqttConfig.topicStatus, "weather/status");
}

void saveMQTTConfig() {
  preferences.begin("weather-station", false);
  preferences.putString("host", mqttConfig.host);
  preferences.putInt("port", mqttConfig.port);
  preferences.putString("username", mqttConfig.username);
  preferences.putString("password", mqttConfig.password);
  preferences.putString("topicTemp", mqttConfig.topicTemp);
  preferences.putString("topicPressure", mqttConfig.topicPressure);
  preferences.putString("topicLightningDist", mqttConfig.topicLightningDist);
  preferences.putString("topicLightningEnergy", mqttConfig.topicLightningEnergy);
  preferences.putString("topicStatus", mqttConfig.topicStatus);
  preferences.end();
  Serial.println("MQTT config saved to NVS");
}

// --- Web Server Handlers ---
void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>ESP32 Weather Station</title>
      <style>
        body { font-family: Arial, sans-serif; text-align: center; }
        h1 { color: #333; }
        .container { max-width: 500px; margin: auto; padding: 20px; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        p { line-height: 1.6; }
        a { color: #007bff; text-decoration: none; }
        a:hover { text-decoration: underline; }
        code { background-color: #f4f4f4; padding: 2px 4px; border-radius: 4px; }
      </style>
    </head>
    <body>
      <div class="container">
        <h1>ESP32 Weather Station</h1>
        <p>Your weather station is up and running!</p>
        <p>View sensor data: <a href="/json">/json</a></p>
        <p><b><a href="/mqtt-config">Configure MQTT Settings</a></b></p>
        <p><b><a href="/config">Launch WiFiManager</a></b></p>
        <p><b><a href="/factory-reset" onclick="return confirm('Are you sure you want to wipe WiFi and MQTT settings?');">Factory Reset</a></b></p>
        <p>Current IP: <code>)rawliteral";
  html += WiFi.localIP().toString();
  html += R"rawliteral(</code></p>
      </div>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleJsonStatus() {
  JsonDocument doc;
  doc["temperature_F"] = state.currentTempF;
  doc["pressure_inHg"] = state.currentPressureInHg;
  if (state.lastDistanceMiles >= 0) {
    doc["last_lightning_distance_miles"] = state.lastDistanceMiles;
  } else {
    doc["last_lightning_distance_miles"] = "N/A";
  }
  if (state.lastEnergy >= 0) {
    doc["last_lightning_energy"] = state.lastEnergy;
  } else {
    doc["last_lightning_energy"] = "N/A";
  }
  DateTime now = rtc.now();
  char timeStr[20];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  doc["current_time"] = timeStr;
  char dateStr[20];
  sprintf(dateStr, "%04d-%02d-%02d", now.year(), now.month(), now.day());
  doc["current_date"] = dateStr;
  doc["wifi_status"] = WiFi.status() == WL_CONNECTED ? "connected" : "disconnected";
  doc["ip_address"] = WiFi.localIP().toString();
  doc["mqtt_status"] = mqttClient.connected() ? "connected" : "disconnected";

  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

void handleConfig() {
  server.send(200, "text/html", "Launching configuration portal...");
  Serial.println("Launching WiFiManager configuration portal...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Config Mode...");
  lcd.setCursor(0, 1);
  lcd.print("Connect to AP:");
  delay(100);
  wm.startConfigPortal("WeatherStationSetup");
}

void handleMQTTConfigForm() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>MQTT Config</title>
      <style>
        body { font-family: Arial, sans-serif; text-align: center; }
        .container { max-width: 500px; margin: auto; padding: 20px; }
        input { margin: 5px; padding: 5px; width: 200px; }
      </style>
    </head>
    <body>
      <div class="container">
        <h2>MQTT Configuration</h2>
        <form method="POST" action="/mqtt-config">
          Host: <input name="host" value=")rawliteral";
  html += mqttConfig.host;
  html += R"rawliteral("><br>
          Port: <input name="port" type="number" value=")rawliteral";
  html += String(mqttConfig.port);
  html += R"rawliteral("><br>
          Username: <input name="username" value=")rawliteral";
  html += mqttConfig.username;
  html += R"rawliteral("><br>
          Password: <input name="password" type="password" value=")rawliteral";
  html += mqttConfig.password;
  html += R"rawliteral("><br>
          Temp Topic: <input name="topicTemp" value=")rawliteral";
  html += mqttConfig.topicTemp;
  html += R"rawliteral("><br>
          Pressure Topic: <input name="topicPressure" value=")rawliteral";
  html += mqttConfig.topicPressure;
  html += R"rawliteral("><br>
          Lightning Dist Topic: <input name="tLightningDist" value=")rawliteral";
  html += mqttConfig.tLightningDist;
  html += R"rawliteral("><br>
          Lightning Energy Topic: <input name="tLightningEnergy" value=")rawliteral";
  html += mqttConfig.tLightningEnergy;
  html += R"rawliteral("><br>
          Status Topic: <input name="topicStatus" value=")rawliteral";
  html += mqttConfig.topicStatus;
  html += R"rawliteral("><br>
          <input type="submit" value="Save">
        </form>
      </div>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleMQTTConfigSubmit() {
  if (server.hasArg("host") && server.arg("host").length() < sizeof(mqttConfig.host)) {
    strncpy(mqttConfig.host, server.arg("host").c_str(), sizeof(mqttConfig.host));
  }
  if (server.hasArg("port")) {
    int port = server.arg("port").toInt();
    if (port > 0 && port <= 65535) {
      mqttConfig.port = port;
    }
  }
  if (server.hasArg("username") && server.arg("username").length() < sizeof(mqttConfig.username)) {
    strncpy(mqttConfig.username, server.arg("username").c_str(), sizeof(mqttConfig.username));
  }
  if (server.hasArg("password") && server.arg("password").length() < sizeof(mqttConfig.password)) {
    strncpy(mqttConfig.password, server.arg("password").c_str(), sizeof(mqttConfig.password));
  }
  if (server.hasArg("topicTemp") && server.arg("topicTemp").length() < sizeof(mqttConfig.topicTemp)) {
    strncpy(mqttConfig.topicTemp, server.arg("topicTemp").c_str(), sizeof(mqttConfig.topicTemp));
  }
  if (server.hasArg("topicPressure") && server.arg("topicPressure").length() < sizeof(mqttConfig.topicPressure)) {
    strncpy(mqttConfig.topicPressure, server.arg("topicPressure").c_str(), sizeof(mqttConfig.topicPressure));
  }
  if (server.hasArg("tLightningDist") && server.arg("tLightningDist").length() < sizeof(mqttConfig.tLightningDist)) {
    strncpy(mqttConfig.tLightningDist, server.arg("tLightningDist").c_str(), sizeof(mqttConfig.tLightningDist));
  }
  if (server.hasArg("tLightningEnergy") && server.arg("tLightningEnergy").length() < sizeof(mqttConfig.tLightningEnergy)) {
    strncpy(mqttConfig.tLightningEnergy, server.arg("tLightningEnergy").c_str(), sizeof(mqttConfig.tLightningEnergy));
  }
  if (server.hasArg("topicStatus") && server.arg("topicStatus").length() < sizeof(mqttConfig.topicStatus)) {
    strncpy(mqttConfig.topicStatus, server.arg("topicStatus").c_str(), sizeof(mqttConfig.topicStatus));
  }

  saveMQTTConfig();
  server.send(200, "text/html", "<html><body><h3>MQTT settings saved. Rebooting...</h3></body></html>");
  delay(1000);
  ESP.restart();
}