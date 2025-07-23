#ifndef MQTT_UTILS_H
#define MQTT_UTILS_H

#include <Arduino.h>

typedef struct {
  char host[64];
  int port;
  char username[32];
  char password[32];
  char topic[64];
} mqtt_config_t;

extern mqtt_config_t mqttConfig;

void saveMQTTConfig();
void saveCustomParamsCallback();
bool testMqttConnection(const char* server_addr, int port, const char* user, const char* pass);
void reconnectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);

#endif // MQTT_UTILS_H
