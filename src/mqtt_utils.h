#ifndef MQTT_UTILS_H
#define MQTT_UTILS_H

#include <Arduino.h>

void saveCustomParamsCallback();
bool testMqttConnection(const char* server_addr, int port, const char* user, const char* pass);
void reconnectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);

#endif // MQTT_UTILS_H
