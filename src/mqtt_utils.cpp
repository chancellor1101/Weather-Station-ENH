#include "globals.h"
#include "mqtt_utils.h"
#include "display.h"
#include <Arduino.h>
#include <EEPROM.h>

mqtt_config_t mqttConfig = {
  "mqtt.local",     // host
  1883,             // port
  "",               // username
  "",               // password
  "weather"         // topic
};


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
}

bool testMqttConnection(const char* server_addr, int port, const char* user, const char* pass) {
  Serial.print("Attempting MQTT test connect to "); Serial.print(server_addr); Serial.print(":"); Serial.println(port);
  lcd.setCursor(0,0);
  lcd.print("Testing MQTT...");
  lcd.setCursor(0,1);
  lcd.print("               ");

  mqttClient.setServer(server_addr, port);
  String clientId = "ESP32Weather-";
  clientId += String(micros(), HEX);

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
    mqttClient.disconnect();
    delay(1000);
    return true;
  } else {
    Serial.print("MQTT Test FAILED, rc=");
    Serial.println(mqttClient.state());
    lcd.setCursor(0,1);
    lcd.print("MQTT Failed! rc:");
    lcd.print(mqttClient.state());
    delay(2000);
    return false;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    lcd.setCursor(0,1);
    lcd.print("MQTT Reconn...");
    String clientId = "ESP32Weather-";
    clientId += WiFi.macAddress().substring(12);

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
      mqttClient.publish(mqtt_topic_status, "Weather Station Online");
      lastMQTTPublishMillis = millis();
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      lcd.setCursor(0,1);
      lcd.print("MQTT Failed, ");
      lcd.print(mqttClient.state());
      delay(5000);
    }
  }
}

void saveMQTTConfig() {
  // Replace this with your actual persistence logic
  EEPROM.put(0, mqttConfig);
  EEPROM.commit();
  Serial.println("MQTT config saved to EEPROM");
}

void reconnectMQTT() {
  mqttClient.disconnect();
  delay(100);
  mqttClient.setServer(mqttConfig.host, mqttConfig.port);
  // Optionally trigger immediate reconnect here
  Serial.println("Reconnecting to MQTT with new settings...");
  // mqtt_connect();  // Uncomment if you have a reconnect function defined
}
