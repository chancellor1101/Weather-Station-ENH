#ifndef GLOBALS_H
#define GLOBALS_H

#include <LiquidCrystal_I2C.h>
#include "SparkFun_AS3935.h"
#include "RTClib.h"
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#define INDOOR 0x12
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01
#define BUZZER_PIN 13

extern LiquidCrystal_I2C lcd;
extern SparkFun_AS3935 lightning;
extern RTC_DS3231 rtc;
extern Adafruit_BMP280 bmp;

extern const int lightningInt;
extern int spiCS;
extern int intVal;
extern int noise;

extern unsigned long lastAlertMillis;
extern const unsigned long alertCooldown;
extern unsigned long lastCycle;
extern int displayState;
extern bool alertActive;
extern unsigned long alertStartMillis;
extern int lastClockMinute;

extern float currentTempF;
extern float currentPressureInHg;
extern int lastDistanceMiles;
extern int lastEnergy;
extern unsigned long lastLightningTimeMillis;
extern const unsigned long lightningClearCooldown;

extern byte signalBarsFull[8];
extern byte lightningBoltDiagonal[8];
extern byte thermometer[8];
extern byte droplet[8];
extern byte pressure[8];

extern WiFiClient espClient;
extern PubSubClient mqttClient;
extern WiFiManager wm;
extern WebServer server;

extern char mqtt_server[40];
extern char mqtt_port[6];
extern char mqtt_username[40];
extern char mqtt_password[40];
extern char mqtt_topic_temp[40];
extern char mqtt_topic_pressure[40];
extern char mqtt_topic_lightning_dist[40];
extern char mqtt_topic_lightning_energy[40];
extern char mqtt_topic_status[40];

extern WiFiManagerParameter custom_mqtt_server;
extern WiFiManagerParameter custom_mqtt_port;
extern WiFiManagerParameter custom_mqtt_username;
extern WiFiManagerParameter custom_mqtt_password;
extern WiFiManagerParameter custom_mqtt_topic_temp;
extern WiFiManagerParameter custom_mqtt_topic_pressure;
extern WiFiManagerParameter custom_mqtt_topic_lightning_dist;
extern WiFiManagerParameter custom_mqtt_topic_lightning_energy;
extern WiFiManagerParameter custom_mqtt_topic_status;

extern unsigned long lastMQTTPublishMillis;
extern const unsigned long mqttPublishInterval;

#endif // GLOBALS_H
