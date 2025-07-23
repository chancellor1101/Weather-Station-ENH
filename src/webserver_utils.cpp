#include "globals.h"
#include "webserver_utils.h"
#include <ArduinoJson.h>
#include "display.h"
#include "mqtt_utils.h"


void handleRoot()
{
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
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
    <div class=\"container\">
      <h1>ESP32 Weather Station</h1>
      <p>Your weather station is up and running!</p>
      <p>View sensor data in JSON format: <a href=\"/json\">/json</a></p>
<p><b><a href="/mqtt-config">Configure MQTT Settings</a></b></p>
<p><b><a href="/config">Launch WiFiManager</a></b></p>
      <p>Current IP: <code>)rawliteral";
  html += WiFi.localIP().toString();
  html += R"rawliteral(</code></p>
    </div>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleJsonStatus()
{
  StaticJsonDocument<512> doc;
  doc["temperature_F"] = currentTempF;
  doc["pressure_inHg"] = currentPressureInHg;
  if (lastDistanceMiles >= 0)
  {
    doc["last_lightning_distance_miles"] = lastDistanceMiles;
    doc["last_lightning_energy"] = lastEnergy;
  }
  else
  {
    doc["last_lightning_distance_miles"] = "N/A";
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

void handleConfig()
{
  server.send(200, "text/html", "Launching configuration portal... You will be redirected to the AP.");
  Serial.println("Manually launching WiFiManager configuration portal...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Config Mode...");
  lcd.setCursor(0, 1);
  lcd.print("Connect to AP:");
  delay(100);
  wm.startConfigPortal("WeatherStationSetup");
}

void handleMQTTConfigForm()
{
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head><title>MQTT Config</title></head>
    <body>
      <h2>MQTT Configuration</h2>
      <form method='POST' action='/mqtt-config'>
        Host: <input name='host' value=')"rawliteral" + String(mqttConfig.host) + R"rawliteral('><br>
        Port: <input name='port' type='number' value=')"rawliteral" + String(mqttConfig.port) + R"rawliteral('><br>
        Username: <input name='username' value=')"rawliteral" + String(mqttConfig.username) + R"rawliteral('><br>
        Password: <input name='password' type='password' value=')"rawliteral" + String(mqttConfig.password) + R"rawliteral('><br>
        Topic: <input name='topic' value=')"rawliteral" + String(mqttConfig.topic) + R"rawliteral('><br>
        <input type='submit' value='Save'>
      </form>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleMQTTConfigSubmit()
{
  if (server.hasArg("host"))
    strncpy(mqttConfig.host, server.arg("host").c_str(), sizeof(mqttConfig.host));
  if (server.hasArg("port"))
    mqttConfig.port = server.arg("port").toInt();
  if (server.hasArg("username"))
    strncpy(mqttConfig.username, server.arg("username").c_str(), sizeof(mqttConfig.username));
  if (server.hasArg("password"))
    strncpy(mqttConfig.password, server.arg("password").c_str(), sizeof(mqttConfig.password));
  if (server.hasArg("topic"))
    strncpy(mqttConfig.topic, server.arg("topic").c_str(), sizeof(mqttConfig.topic));

  saveMQTTConfig();
  reconnectMQTT();

  server.send(200, "text/plain", "MQTT config saved. Reconnecting...");
}
