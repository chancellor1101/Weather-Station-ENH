#include <SPI.h>
#include <Wire.h>
#include "SparkFun_AS3935.h"
#include "RTClib.h"
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>

// --- NEW WI-FI & MQTT LIBRARIES ---
#include <WiFi.h> // For ESP32 Wi-Fi
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h> // For JSON serialization

// --- Existing Defines ---
#define INDOOR 0x12
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01
#define BUZZER_PIN 13

// --- LCD and Sensor Objects ---
LiquidCrystal_I2C lcd(0x27, 16, 2); // Address, 16 chars, 2 lines
SparkFun_AS3935 lightning;
RTC_DS3231 rtc;
Adafruit_BMP280 bmp; // I2C mode

// --- Pins ---
const int lightningInt = 26; // Example ESP32 GPIO pin for interrupt
int spiCS = 27;             // Example ESP32 GPIO pin for SPI CS

// --- Global Variables for Sensor/Display Logic ---
int intVal = 0;
int noise = 2;

unsigned long lastAlertMillis = 0;
const unsigned long alertCooldown = 10UL * 60UL * 1000UL; // 10 minutes
unsigned long lastCycle = 0;
int displayState = 0;
bool alertActive = false;
unsigned long alertStartMillis = 0;
int lastClockMinute = -1; // for tracking minute change

float currentTempF = 0.0;     // Temperature in Fahrenheit
float currentPressureInHg = 0.0; // Pressure in inches of Hg
int lastDistanceMiles = -1;   // Distance in miles
int lastEnergy = -1;          // lastEnergy declaration

unsigned long lastLightningTimeMillis = 0; // Timestamp of the last lightning event
const unsigned long lightningClearCooldown = 1UL * 60UL * 60UL * 1000UL; // 1 hour in milliseconds

// --- Custom Character Arrays ---
byte signalBarsFull[8] = {
  0b00000, 0b00001, 0b00011, 0b00111, 0b01111, 0b11111, 0b00000, 0b00000
};
byte lightningBoltDiagonal[8] = {
  0b10000, 0b11000, 0b01000, 0b00100, 0b00010, 0b00110, 0b01000, 0b10000
};
byte thermometer[8] = {B00100, B01010, B01010, B01110, B01110, B01110, B11111, B01110};
byte droplet[8] = {B00100, B00100, B01010, B01010, B10001, B10001, B10001, B01110};
byte pressure[8] = {B00000, B01110, B10001, B10101, B10001, B01110, B00000, B00000};


// --- WI-FI & MQTT GLOBAL VARIABLES ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiManager wm; // WiFiManager object
WebServer server(80); // Create a web server object on port 80 for the status page

// MQTT Custom Parameters
char mqtt_server[40];
char mqtt_port[6] = "1883"; // Default MQTT port
char mqtt_username[40] = "";
char mqtt_password[40] = "";
char mqtt_topic_temp[40] = "weather/temperature";
char mqtt_topic_pressure[40] = "weather/pressure";
char mqtt_topic_lightning_dist[40] = "weather/lightning_distance";
char mqtt_topic_lightning_energy[40] = "weather/lightning_energy";
char mqtt_topic_status[40] = "weather/status";

// Custom parameter pointers for WiFiManager
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
WiFiManagerParameter custom_mqtt_username("username", "mqtt username", mqtt_username, 40);
WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, 40);
WiFiManagerParameter custom_mqtt_topic_temp("top_t", "temp topic", mqtt_topic_temp, 40);
WiFiManagerParameter custom_mqtt_topic_pressure("top_p", "pres topic", mqtt_topic_pressure, 40);
WiFiManagerParameter custom_mqtt_topic_lightning_dist("top_ld", "dist topic", mqtt_topic_lightning_dist, 40);
WiFiManagerParameter custom_mqtt_topic_lightning_energy("top_le", "energy topic", mqtt_topic_lightning_energy, 40);
WiFiManagerParameter custom_mqtt_topic_status("top_s", "status topic", mqtt_topic_status, 40);


unsigned long lastMQTTPublishMillis = 0;
const unsigned long mqttPublishInterval = 5UL * 60UL * 1000UL; // Publish every 5 minutes

// --- Forward Declarations (best practice) ---
void playStartupChime();
void playAlertChime();
void displayClock();
void haltAndCatchFire();
void displayScrollingInfo();
void saveCustomParamsCallback();
bool testMqttConnection(const char* server_addr, int port, const char* user, const char* pass);
void reconnectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleRoot();
void handleJsonStatus();
void handleConfig(); // NEW: Forward declaration for the config handler


// --- BUZZER FUNCTIONS ---
void playStartupChime()
{
  tone(BUZZER_PIN, 523, 150);
  delay(200);
  tone(BUZZER_PIN, 659, 150);
  delay(200);
  noTone(BUZZER_PIN);
}

void playAlertChime()
{
  tone(BUZZER_PIN, 523, 200);
  delay(250);
  tone(BUZZER_PIN, 440, 200);
  delay(250);
  tone(BUZZER_PIN, 659, 250);
  delay(300);
  noTone(BUZZER_PIN);
}


// --- DISPLAY FUNCTIONS ---
void displayClock()
{
  DateTime now = rtc.now();
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", now.hour(), now.minute()); // Still using 24-hour for compactness

  int startCol = 15 - strlen(timeStr); // right-align
  lcd.setCursor(1, 0); // Start clearing from a safe spot, not column 0 where WiFi icon is
  lcd.print("               "); // Clear the rest of the line
  lcd.setCursor(startCol, 0);
  lcd.print(timeStr);
}


void haltAndCatchFire()
{
  while (1)
  {
    playAlertChime();
    delay(1500); // Wait for 1.5 seconds
  };
}

void displayScrollingInfo()
{
  lcd.setCursor(0, 1);
  switch (displayState)
  {
  case 0:
    lcd.write((uint8_t)2); // thermometer
    lcd.print(" Temp: ");
    lcd.print(currentTempF, 1); // Display Fahrenheit
    lcd.print("F   ");
    break;
  case 1:
    lcd.write((uint8_t)4); // pressure (using the new pressure icon)
    lcd.print(" Pres:");
    lcd.print(currentPressureInHg, 2); // Display inHg with 2 decimal places
    lcd.print("inHg");
    break;
  case 2:
    lcd.write((uint8_t)1); // lightning bolt
    lcd.print(" Last: ");
    if (lastDistanceMiles >= 0)
    {
      lcd.print(lastDistanceMiles);
      lcd.print("mi   "); // Change unit to miles
    }
    else
    {
      lcd.print("None     ");
    }
    break;
  case 3:
    lcd.write((uint8_t)1); // lightning bolt
    lcd.print(" Energy: ");
    if (lastEnergy >= 0)
    {
      lcd.print(lastEnergy);
      lcd.print("      ");
    }
    else
    {
      lcd.print("None     ");
    }
    break;
  default:
    displayState = 0;
    return;
  }
  displayState = (displayState + 1) % 4;
}

// --- WiFiManager Custom Parameter Save Callback ---
void saveCustomParamsCallback() {
  Serial.println("Should save config");
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_username, custom_mqtt_username.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(mqtt_topic_temp, custom_mqtt_topic_temp.getValue());
  strcpy(mqtt_topic_pressure, custom_mqtt_topic_pressure.getValue());
  strcpy(mqtt_topic_lightning_dist, custom_mqtt_topic_lightning_dist.getValue());
  strcpy(mqtt_topic_lightning_energy, custom_mqtt_topic_lightning_energy.getValue());
  strcpy(mqtt_topic_status, custom_mqtt_topic_status.getValue());

  Serial.print("MQTT Server: "); Serial.println(mqtt_server);
  Serial.print("MQTT Port: "); Serial.println(mqtt_port);
  // Do not print password for security reasons
}

// --- MQTT Test Function for WiFiManager ---
bool testMqttConnection(const char* server_addr, int port, const char* user, const char* pass) { // Renamed 'server' to 'server_addr' to avoid conflict with WebServer object
  Serial.print("Attempting MQTT test connect to "); Serial.print(server_addr); Serial.print(":"); Serial.println(port);
  lcd.setCursor(0,0);
  lcd.print("Testing MQTT...");
  lcd.setCursor(0,1);
  lcd.print("               "); // Clear line 2

  mqttClient.setServer(server_addr, port);
  // Generate a unique client ID
  String clientId = "ESP32Weather-";
  clientId += String(micros(), HEX); // Using micros() for a somewhat unique ID

  bool connected;
  if (strlen(user) > 0) {
    connected = mqttClient.connect(clientId.c_str(), user, pass);
  } else {
    connected = mqttClient.connect(clientId.c_str());
  }

  if (connected) {
    Serial.println("MQTT Test CONNECTED");
    lcd.setCursor(0,1);
    lcd.print("MQTT OK!      ");
    mqttClient.disconnect(); // Disconnect immediately after successful test
    delay(1000); // Give user time to see success
    return true;
  } else {
    Serial.print("MQTT Test FAILED, rc=");
    Serial.println(mqttClient.state());
    lcd.setCursor(0,1);
    lcd.print("MQTT Failed! rc:");
    lcd.print(mqttClient.state());
    delay(2000); // Give user time to see failure
    return false;
  }
}

// --- MQTT Callback Function (for subscribed topics, if any) ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Add logic here to act on received MQTT messages if needed
}

// --- MQTT Reconnection Logic ---
void reconnectMqtt() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    lcd.setCursor(0,1);
    lcd.print("MQTT Reconn...");
    // Generate a unique client ID based on MAC address for persistence
    String clientId = "ESP32Weather-";
    clientId += WiFi.macAddress().substring(12); // Last 2 bytes of MAC for a shorter ID

    // Attempt to connect
    bool connected;
    if (strlen(mqtt_username) > 0) {
      connected = mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password);
    } else {
      connected = mqttClient.connect(clientId.c_str());
    }

    if (connected) {
      Serial.println("MQTT Connected!");
      lcd.setCursor(0,1);
      lcd.print("MQTT Connected  ");
      // Subscribe to any topics if needed, e.g., for commands
      // mqttClient.subscribe("weather/commands");
      mqttClient.publish(mqtt_topic_status, "Weather Station Online"); // Publish online status
      lastMQTTPublishMillis = millis(); // Reset publish timer on connect
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      lcd.setCursor(0,1);
      lcd.print("MQTT Failed, ");
      lcd.print(mqttClient.state());
      delay(5000); // Wait 5 seconds before retrying
    }
  }
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
      body { font-family: Arial, Helvetica, sans-serif; text-align: center; }
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
      <p>View sensor data in JSON format: <a href="/json">/json</a></p>
      <p><b><a href="/config">Configure WiFi/MQTT Settings</a></b></p>
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
  StaticJsonDocument<512> doc;

  // Add sensor readings
  doc["temperature_F"] = currentTempF;
  doc["pressure_inHg"] = currentPressureInHg;

  // Add lightning strike info
  if (lastDistanceMiles >= 0) {
    doc["last_lightning_distance_miles"] = lastDistanceMiles;
    doc["last_lightning_energy"] = lastEnergy;
  } else {
    doc["last_lightning_distance_miles"] = "N/A";
    doc["last_lightning_energy"] = "N/A";
  }

  // Add clock time
  DateTime now = rtc.now();
  char timeStr[20];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  doc["current_time"] = timeStr;

  char dateStr[20];
  sprintf(dateStr, "%04d-%02d-%02d", now.year(), now.month(), now.day());
  doc["current_date"] = dateStr;

  // Add network status
  doc["wifi_status"] = WiFi.status() == WL_CONNECTED ? "connected" : "disconnected";
  doc["ip_address"] = WiFi.localIP().toString();
  doc["mqtt_status"] = mqttClient.connected() ? "connected" : "disconnected";

  String jsonString;
  serializeJson(doc, jsonString);

  server.send(200, "application/json", jsonString);
}

// NEW: Handler to launch the WiFiManager configuration portal
void handleConfig() {
  server.send(200, "text/html", "Launching configuration portal... You will be redirected to the AP.");
  Serial.println("Manually launching WiFiManager configuration portal...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Config Mode...");
  lcd.setCursor(0,1);
  lcd.print("Connect to AP:");
  delay(100); // Small delay to allow response to send to client before AP starts
  wm.startConfigPortal("WeatherStationSetup"); // Re-launch the config portal
  // After configuration, WiFiManager will restart the ESP32.
  // This function won't return until the portal is exited and reboot happens.
}


// --- SETUP FUNCTION ---
void setup()
{
  pinMode(lightningInt, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(115200);
  SPI.begin();
  lcd.init();
  lcd.backlight();

  // Create custom characters
  lcd.createChar(0, signalBarsFull);
  lcd.createChar(1, lightningBoltDiagonal);
  lcd.createChar(2, thermometer);
  lcd.createChar(3, droplet);
  lcd.createChar(4, pressure);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0); // Display the Wi-Fi icon during startup message
  lcd.print(" System Start");
  delay(2000); // Wait for sensors and LCD to warm up

  // --- SENSOR INITIALIZATION ---
  if (!lightning.beginSPI(spiCS))
  {
    lcd.setCursor(0, 1);
    lcd.print("Sensor Failure");
    haltAndCatchFire();
  }
  lightning.setNoiseLevel(noise);
  lightning.setIndoorOutdoor(OUTDOOR); // Assuming outdoor usage

  if (!rtc.begin())
  {
    lcd.setCursor(0, 1);
    lcd.print("Clock Failure ");
    haltAndCatchFire();
  }
  if (rtc.lostPower()) { // Check if RTC lost power (battery flat/missing)
    Serial.println("RTC lost power, setting time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set to compile time
  }

  if (!bmp.begin(0x76))
  {
    lcd.setCursor(0, 1);
    lcd.print("BMP280 Fail 0x76 ");
    if (!bmp.begin(0x77)) {
       lcd.setCursor(0, 1);
       lcd.print("BMP280 Fail 0x77 ");
       haltAndCatchFire();
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  lcd.print("                "); // Clear the "System Start" second line
  lcd.setCursor(0,1);
  lcd.print("Sensors OK!");
  delay(1500); // Brief confirmation
  lcd.clear();

  playStartupChime();

  // --- WIFI & MQTT SETUP ---
  wm.setAPCallback([](WiFiManager *myWiFiManager){ // Lambda function for AP mode callback
      Serial.println("Entered WiFi config mode");
      Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
      Serial.print("AP Name: "); Serial.println(myWiFiManager->getConfigPortalSSID());
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Connect to AP:");
      lcd.setCursor(0,1);
      lcd.print(myWiFiManager->getConfigPortalSSID());
      lcd.write((uint8_t)0); // WiFi icon next to AP name
  });

  // Add all custom parameters to WiFiManager
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_username);
  wm.addParameter(&custom_mqtt_password);
  wm.addParameter(&custom_mqtt_topic_temp);
  wm.addParameter(&custom_mqtt_topic_pressure);
  wm.addParameter(&custom_mqtt_topic_lightning_dist);
  wm.addParameter(&custom_mqtt_topic_lightning_energy);
  wm.addParameter(&custom_mqtt_topic_status);

  wm.setSaveConfigCallback(saveCustomParamsCallback); // Set callback for saving custom params
  wm.setSaveParamsCallback(saveCustomParamsCallback); // For older WiFiManager versions, good to have both

  // Try to connect to WiFi, or start AP if not configured/failed
  bool res = wm.autoConnect("WeatherStationSetup"); // No password on AP, but easier for user

  if(!res) {
      Serial.println("Failed to connect to WiFi and config portal timed out!");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("WiFi Error!");
      lcd.setCursor(0,1);
      lcd.print("Restarting...");
      delay(3000);
      ESP.restart(); // Restart if unable to connect or configure
  }
  else {
      Serial.println("WiFi connected!");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write((uint8_t)0); // Display Wi-Fi icon
      lcd.print(" WiFi Connected");
      lcd.setCursor(0,1);
      lcd.print(WiFi.localIP()); // Show local IP
      delay(2000);

      // --- START WEB SERVER AND DEFINE HANDLERS ---
      server.on("/", handleRoot); // Handle root URL
      server.on("/json", handleJsonStatus); // Handle JSON status URL
      server.on("/config", handleConfig); // NEW: Handle /config URL for re-configuration
      server.begin(); // Start the web server
      Serial.println("HTTP server started");
      // --- END NEW ---
  }

  // Once WiFi is connected (either from saved creds or new config),
  // load custom parameters if they were saved or changed.
  Serial.print("Loaded MQTT Server: "); Serial.println(mqtt_server);
  Serial.print("Loaded MQTT Port: "); Serial.println(mqtt_port);

  // Test MQTT connection with the loaded parameters
  if (strlen(mqtt_server) > 0) { // Only test if an MQTT server was provided
    if (!testMqttConnection(mqtt_server, atoi(mqtt_port), mqtt_username, mqtt_password)) {
      Serial.println("MQTT test failed after config. Re-entering AP mode.");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("MQTT Test FAIL!");
      lcd.setCursor(0,1);
      lcd.print("Reconfigure AP");
      delay(3000);
      // If MQTT test fails, re-enter AP mode to allow user to correct info
      wm.startConfigPortal("WeatherStationSetup"); // This will block until configured or timed out
      ESP.restart(); // Restart after re-configuration attempt
    }
  } else {
    Serial.println("No MQTT server configured. Skipping MQTT connection.");
    lcd.setCursor(0,0);
    lcd.print("No MQTT config");
    delay(1500);
  }


  // If we reach here, WiFi is connected and MQTT test was successful (or skipped)
  // Set MQTT server and callback for main operation
  if (strlen(mqtt_server) > 0) { // Only set up MQTT client if server is provided
    mqttClient.setServer(mqtt_server, atoi(mqtt_port));
    mqttClient.setCallback(mqttCallback); // Set a callback for incoming messages
  } else {
    Serial.println("MQTT client not initialized due to no server config.");
  }

  lcd.clear(); // Clear LCD after initial setup messages
}

// --- LOOP FUNCTION ---
void loop()
{
  unsigned long nowMillis = millis();
  DateTime now = rtc.now();

  // If WiFi is connected, handle HTTP requests
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient(); // NEW: Process incoming web requests
  }

  // Keep MQTT client connected and processed
  if (strlen(mqtt_server) > 0) {
    if (!mqttClient.connected()) {
      reconnectMqtt();
    }
    mqttClient.loop(); // Must be called frequently to process incoming messages and maintain connection
  }


  // --- Imperial Conversions ---
  float tempC = bmp.readTemperature();
  currentTempF = (tempC * 9.0 / 5.0) + 32.0; // Celsius to Fahrenheit

  float pressurePa = bmp.readPressure();
  currentPressureInHg = pressurePa / 3386.389; // Pascal to inHg

  // --- Lightning Sensor Handling ---
  if (digitalRead(lightningInt) == HIGH)
  {
    intVal = lightning.readInterruptReg();
    if (intVal == LIGHTNING_INT)
    {
      byte distanceKm = lightning.distanceToStorm();
      lastDistanceMiles = (int)round(distanceKm * 0.621371); // Convert km to miles
      lastEnergy = lightning.lightningEnergy();
      lastLightningTimeMillis = nowMillis; // Record timestamp of this strike

      if (lastDistanceMiles <= 10 && (lastAlertMillis == 0 || nowMillis - lastAlertMillis > alertCooldown))
      {
        alertStartMillis = nowMillis;
        alertActive = true;
        lastAlertMillis = nowMillis; // Update last alert time to now
        playAlertChime();
      }
    }
  }

  // --- Clear Lightning Stats after 1 Hour ---
  if (lastLightningTimeMillis > 0 && nowMillis - lastLightningTimeMillis > lightningClearCooldown) {
      lastDistanceMiles = -1; // Reset to indicate no recent lightning
      lastEnergy = -1;        // Reset energy
      lastLightningTimeMillis = 0; // Reset timestamp
  }


  // --- Display Logic ---
  if (alertActive)
  {
    if (nowMillis - alertStartMillis < 30000) // Alert lasts for 30 seconds
    {
      lcd.setCursor(0, 0);
      lcd.print("!!! LIGHTNING !!!");
      lcd.setCursor(0, 1);
      lcd.print("Within ");
      lcd.print(lastDistanceMiles); // Display distance in miles
      lcd.print(" miles!    ");
      // Optional: Flash backlight during alert
      if ( (nowMillis / 500) % 2 == 0 ) lcd.backlight();
      else lcd.noBacklight();

    }
    else // Alert period ended
    {
      alertActive = false;
      lcd.backlight(); // Ensure backlight is on after alert
      lcd.clear();     // Clear screen after alert
      // Re-initialize main display elements if needed
      lcd.setCursor(0, 0);
      lcd.write((uint8_t)0); // WiFi icon
      displayClock(); // Refresh clock
      displayScrollingInfo(); // Refresh info
    }
  }
  else // Normal operation
  {
    lcd.setCursor(0, 0);
    lcd.write((uint8_t)0); // wifi icon

    if (now.minute() != lastClockMinute)
    {
      lastClockMinute = now.minute();
      displayClock();
    }

    if (nowMillis - lastCycle > 4000) // Cycle info every 4 seconds
    {
      lastCycle = nowMillis;
      displayScrollingInfo();
    }

    // --- MQTT Publishing ---
    if (strlen(mqtt_server) > 0 && mqttClient.connected() && nowMillis - lastMQTTPublishMillis > mqttPublishInterval) {
      lastMQTTPublishMillis = nowMillis;

      char payload[32]; // Buffer for string conversions

      // Publish Temperature
      sprintf(payload, "%.1f", currentTempF);
      mqttClient.publish(mqtt_topic_temp, payload);
      Serial.print("Published Temp: "); Serial.println(payload);

      // Publish Pressure
      sprintf(payload, "%.2f", currentPressureInHg);
      mqttClient.publish(mqtt_topic_pressure, payload);
      Serial.print("Published Pressure: "); Serial.println(payload);

      // Publish Last Lightning Distance (if available)
      if (lastDistanceMiles >= 0) {
        sprintf(payload, "%d", lastDistanceMiles);
        mqttClient.publish(mqtt_topic_lightning_dist, payload);
        Serial.print("Published Last Lightning Distance: "); Serial.println(payload);
      } else {
        mqttClient.publish(mqtt_topic_lightning_dist, "none"); // Or an empty string, or specific "no strike" value
      }

      // Publish Last Lightning Energy (if available)
      if (lastEnergy >= 0) {
        sprintf(payload, "%d", lastEnergy);
        mqttClient.publish(mqtt_topic_lightning_energy, payload);
        Serial.print("Published Last Lightning Energy: "); Serial.println(payload);
      } else {
        mqttClient.publish(mqtt_topic_lightning_energy, "none");
      }
    }
  }
  delay(100); // Small delay to prevent CPU overload
}