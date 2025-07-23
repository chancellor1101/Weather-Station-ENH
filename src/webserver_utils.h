#include "mqtt_utils.h"

#ifndef WEBSERVER_UTILS_H
#define WEBSERVER_UTILS_H

extern mqtt_config_t mqttConfig;

void handleRoot();
void handleJsonStatus();
void handleConfig();
void saveMQTTConfig();
void reconnectMQTT();
void handleMQTTConfigForm();
void handleMQTTConfigSubmit();


#endif // WEBSERVER_UTILS_H
